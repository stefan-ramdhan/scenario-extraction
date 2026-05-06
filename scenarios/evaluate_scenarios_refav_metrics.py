#!/usr/bin/env python3
"""
evaluate_scenarios_refav_metrics.py — Timestamp-level and Log-level evaluation
of scenario extractions against ground-truth APs.

Supports two scenario modes (--mode):
  long_following  (default) — evaluates long_following_* variants
  cut_in                    — evaluates cut-in variants (veh_in_adj_right_lane,
                              veh_straddling_ego_lane, lead_veh_too_close,
                              cut_in_from_right) split by phase

=============================================================================
METRIC DEFINITIONS
=============================================================================

1. TIMESTAMP BALANCED ACCURACY
   Each actual AV2 frame is a binary classification unit.
     GT label   = positive  ↔  frame timestamp falls inside ≥1 GT interval
     Pred label = positive  ↔  frame timestamp falls inside ≥1 pred interval
   TP = frame is GT-positive  AND pred-positive
   TN = frame is GT-negative  AND pred-negative
   FP = frame is GT-negative  AND pred-positive
   FN = frame is GT-positive  AND pred-negative
   TPR = TP / (TP + FN)    (sensitivity / recall)
   TNR = TN / (TN + FP)    (specificity)
   Timestamp Balanced Accuracy = (TPR + TNR) / 2

2. LOG BALANCED ACCURACY
   Each log (UUID) is a binary classification unit.
     GT label   = positive  ↔  log contains ≥1 GT interval
     Pred label = positive  ↔  log contains ≥1 predicted interval
   A log contributes exactly one classification regardless of how many
   GT or predicted intervals it contains.
   TPR = TP / (TP + FN)
   TNR = TN / (TN + FP)
   Log Balanced Accuracy = (TPR + TNR) / 2

3. STANDARD INSTANCE-LEVEL METRICS (included for reference)
   Precision = TP_inst / (TP_inst + FP_inst)
   Recall    = TP_inst / (TP_inst + FN_inst)
   F1        = harmonic mean of Precision and Recall
   (Instance matching: any temporal overlap counts.)

=============================================================================
IMPLEMENTATION CHOICES
=============================================================================

Timestamp resolution
  Uses actual AV2 frame timestamps read from {uuid8}_rs2v_tracked.json
  (≈10 Hz, ~100 ms between frames). No artificial resampling is performed.
  Each entry in that file is one evaluation unit.

Interval boundary convention
  Closed intervals: a frame timestamp t (in seconds) is inside interval
  [start_s, end_s] iff  start_s ≤ t ≤ end_s.
  Both GT and predictions use this convention.

Overlapping intervals
  Before scoring, overlapping (or adjacent) intervals are merged per log.
  This ensures each timestamp is counted at most once as TP or FP.
  Merging is applied independently to GT intervals and predicted intervals.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from collections import defaultdict

# ─────────────────────────────────────────────────────────────────────────────
# Paths
# ─────────────────────────────────────────────────────────────────────────────

_SCRIPT_DIR  = Path(__file__).resolve().parent
_REPO_ROOT   = _SCRIPT_DIR.parent

MY_RESULTS_DIR = _REPO_ROOT / "SceneFlowLang" / "my_results" / "SG_tracks"  # overridden by --data
GT_APS_DIR     = _SCRIPT_DIR / "aps"
TRACKED_DIR    = _REPO_ROOT / "sg_processing" / "code" / "tracking_estimation" / "obj_positions"
MATCHED_DIR    = _REPO_ROOT / "sg_processing" / "code" / "tracking_estimation" / "obj_positions"

# ─────────────────────────────────────────────────────────────────────────────
# Variant → GT AP mappings
# ─────────────────────────────────────────────────────────────────────────────

# long_following: prefix-based matching (same as evaluate_long_following.py)
LF_VARIANT_GT_AP: dict[str, str] = {
    "long_following_visible":      "lead_vehicle_in_ego_lane_60m",
    "long_following_near":         "lead_vehicle_in_ego_lane_30m",
    "long_following_very_near":    "lead_vehicle_in_ego_lane_18m",
    "long_following_super_near":   "lead_vehicle_in_ego_lane_12m",
    "long_following_near_coll":    "lead_vehicle_in_ego_lane_6m",
    "long_following_headway_1.0s": "lead_vehicle_in_ego_lane_1s",
    "long_following_headway_2.0s": "lead_vehicle_in_ego_lane_2s",
    "long_following_headway_3.0s": "lead_vehicle_in_ego_lane_3s",
}

# cut_in: exact-name matching, results shown in phase order
CI_VARIANT_GT_AP: dict[str, str] = {
    "veh_in_adj_right_lane":   "cut_in_phase1_veh_in_adj_right_lane",
    "veh_straddling_ego_lane": "cut_in_phase2_partial_merge",
    "lead_veh_too_close":      "cut_in_phase3_aligned_and_close",
    "cut_in_from_right":       "cut_in_from_right_full_sequence",
}

CI_GT_AP_ORDER: list[str] = [
    "cut_in_phase1_veh_in_adj_right_lane",
    "cut_in_phase2_partial_merge",
    "cut_in_phase3_aligned_and_close",
    "cut_in_from_right_full_sequence",
]


def _resolve_lf(variant: str) -> str | None:
    match = max(
        (pfx for pfx in LF_VARIANT_GT_AP if variant.startswith(pfx)),
        key=len, default=None,
    )
    return LF_VARIANT_GT_AP[match] if match else None


def _resolve_ci(variant: str) -> str | None:
    return CI_VARIANT_GT_AP.get(variant)


# ─────────────────────────────────────────────────────────────────────────────
# Interval helpers
# ─────────────────────────────────────────────────────────────────────────────

def merge_intervals(intervals: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Merge overlapping/adjacent intervals. Input need not be sorted."""
    if not intervals:
        return []
    sorted_ivs = sorted(intervals)
    merged = [sorted_ivs[0]]
    for start, end in sorted_ivs[1:]:
        if start <= merged[-1][1]:           # overlapping or touching
            merged[-1] = (merged[-1][0], max(merged[-1][1], end))
        else:
            merged.append((start, end))
    return merged


def timestamp_in_intervals(t: float, merged: list[tuple[float, float]]) -> bool:
    """Binary-search test: is t inside any merged interval?"""
    lo, hi = 0, len(merged) - 1
    while lo <= hi:
        mid = (lo + hi) // 2
        s, e = merged[mid]
        if t < s:
            hi = mid - 1
        elif t > e:
            lo = mid + 1
        else:
            return True
    return False


# ─────────────────────────────────────────────────────────────────────────────
# Data loaders
# ─────────────────────────────────────────────────────────────────────────────

def load_frame_timestamps_s(uuid8: str) -> list[float]:
    """
    Return sorted list of all frame timestamps (in seconds) for this log,
    read from {uuid8}_rs2v_tracked.json.
    """
    path = TRACKED_DIR / f"{uuid8}_rs2v_tracked.json"
    if not path.exists():
        return []
    with open(path) as f:
        frames = json.load(f)
    ts = []
    for frame in frames:
        raw = frame.get("timestamp")
        if raw is not None:
            ts.append(int(raw) / 1e9)
    return sorted(ts)


def load_gt_intervals(uuid8: str, ap_name: str) -> list[tuple[float, float]]:
    """Return merged GT intervals (start_s, end_s) for ap_name in this log."""
    gt_path = GT_APS_DIR / f"{uuid8}_aps.json"
    if not gt_path.exists():
        return []
    with open(gt_path) as f:
        data = json.load(f)
    raw: list[tuple[float, float]] = []
    for track_entry in data.get("aps", {}).get(ap_name, []):
        for inst in track_entry.get("instances", []):
            s = inst.get("start_global_time_s")
            e = inst.get("end_global_time_s")
            if s is not None and e is not None:
                raw.append((s, e))
    return merge_intervals(raw)


def load_pred_intervals(uuid8: str, variant: str) -> list[tuple[float, float]]:
    """Return merged predicted intervals (start_s, end_s) for (uuid8, variant)."""
    sat_dir = MY_RESULTS_DIR / uuid8 / variant / "satisfactions"
    if not sat_dir.exists():
        return []
    raw: list[tuple[float, float]] = []
    for json_file in sat_dir.glob("*.json"):
        with open(json_file) as f:
            d = json.load(f)
        start_s = int(d["start_frame"]) / 1e9
        end_s   = int(d["end_frame"])   / 1e9
        raw.append((start_s, end_s))
    return merge_intervals(raw)


# ─────────────────────────────────────────────────────────────────────────────
# Track-aware helpers
# ─────────────────────────────────────────────────────────────────────────────

def load_track_mapping(uuid8: str) -> dict[int, str]:
    """Build track_id (int) → av2_track_uuid (str) mapping from *_matched.json."""
    path = MATCHED_DIR / f"{uuid8}_matched.json"
    if not path.exists():
        return {}
    with open(path) as f:
        frames = json.load(f)
    mapping: dict[int, str] = {}
    for frame in frames:
        for key, val in frame.items():
            if key in ("timestamp", "av2_timestamp", "ego") or not isinstance(val, dict):
                continue
            tid  = val.get("track_id")
            uuid = val.get("av2_track_uuid")
            if tid is not None and uuid is not None and tid not in mapping:
                mapping[tid] = uuid
    return mapping


def parse_track_int(label: str) -> int | None:
    """'track_car_7' / 'track_truck_7' / 'track_bus_7' → 7."""
    parts = label.split("_")
    if len(parts) >= 3 and parts[0] == "track":
        try:
            return int(parts[-1])
        except ValueError:
            pass
    return None


def load_pred_instances(uuid8: str, variant: str,
                        track_mapping: dict[int, str]) -> list[dict]:
    """
    Return satisfaction instances for (uuid8, variant) as list of dicts:
      { "av2_track_uuid": str | None, "start_s": float, "end_s": float }

    av2_track_uuid is None when the track cannot be resolved.
    """
    sat_dir = MY_RESULTS_DIR / uuid8 / variant / "satisfactions"
    if not sat_dir.exists():
        return []
    results = []
    for json_file in sorted(sat_dir.glob("*.json")):
        with open(json_file) as f:
            data = json.load(f)
        entity_mapping = data.get("entity_mapping", {})
        track_label = entity_mapping.get("yield_vehicle_1")
        if track_label is None:
            track_label = next(iter(entity_mapping.values()), None)
        if track_label is None:
            continue
        track_int = parse_track_int(track_label)
        av2_uuid  = track_mapping.get(track_int) if track_int is not None else None
        results.append({
            "av2_track_uuid": av2_uuid,
            "start_s":        int(data["start_frame"]) / 1e9,
            "end_s":          int(data["end_frame"])   / 1e9,
        })
    return results


def load_gt_track_uuids(uuid8: str, ap_name: str) -> set[str]:
    """Return the set of AV2 track UUIDs that appear in GT for ap_name."""
    gt_path = GT_APS_DIR / f"{uuid8}_aps.json"
    if not gt_path.exists():
        return set()
    with open(gt_path) as f:
        data = json.load(f)
    uuids: set[str] = set()
    for track_entry in data.get("aps", {}).get(ap_name, []):
        tid = track_entry.get("track_id")
        if tid:
            uuids.add(tid)
    return uuids


# ─────────────────────────────────────────────────────────────────────────────
# Per-log classification
# ─────────────────────────────────────────────────────────────────────────────

def classify_timestamps(
    frame_ts: list[float],
    gt_merged: list[tuple[float, float]],
    pred_merged: list[tuple[float, float]],
    tp_pred_merged: list[tuple[float, float]] | None = None,
) -> tuple[int, int, int, int]:
    """
    Return (TP, TN, FP, FN) at timestamp level for one log.

    tp_pred_merged: when track-aware, the subset of pred intervals whose track
    UUID matches a GT track UUID.  Used for TP/FN scoring only; pred_merged
    (all intervals) is still used for FP/TN so wrong-track predictions are not
    silently ignored.  When None, pred_merged is used for both.
    """
    if tp_pred_merged is None:
        tp_pred_merged = pred_merged
    tp = tn = fp = fn = 0
    for t in frame_ts:
        gt_pos   = timestamp_in_intervals(t, gt_merged)
        pred_pos = timestamp_in_intervals(t, pred_merged)
        tp_pos   = timestamp_in_intervals(t, tp_pred_merged)
        if gt_pos and tp_pos:
            tp += 1
        elif not gt_pos and not pred_pos:
            tn += 1
        elif not gt_pos and pred_pos:
            fp += 1
        else:
            fn += 1
    return tp, tn, fp, fn


# ─────────────────────────────────────────────────────────────────────────────
# Aggregation helpers
# ─────────────────────────────────────────────────────────────────────────────

def balanced_accuracy(tpr: float, tnr: float) -> float:
    return (tpr + tnr) / 2.0


def compute_rates(tp: int, tn: int, fp: int, fn: int) -> tuple[float, float, float]:
    tpr = tp / (tp + fn) if (tp + fn) > 0 else 0.0
    tnr = tn / (tn + fp) if (tn + fp) > 0 else 0.0
    ba  = balanced_accuracy(tpr, tnr)
    return tpr, tnr, ba


# ─────────────────────────────────────────────────────────────────────────────
# Printing helpers
# ─────────────────────────────────────────────────────────────────────────────

def _fmt_ts(label: str, tp: int, tn: int, fp: int, fn: int, col_w: int) -> str:
    tpr, tnr, ba = compute_rates(tp, tn, fp, fn)
    n     = tp + fn
    total = tp + tn + fp + fn
    return (f"{label:<{col_w}}  "
            f"{ba:>8.3f}  {tpr:>6.3f}  {tnr:>6.3f}  "
            f"{n:>8}  {tp:>7}  {tn:>7}  {fp:>7}  {fn:>7}  {total:>8}")


def _fmt_log(label: str, tp: int, tn: int, fp: int, fn: int, col_w: int) -> str:
    tpr, tnr, ba = compute_rates(tp, tn, fp, fn)
    n     = tp + fn
    total = tp + tn + fp + fn
    return (f"{label:<{col_w}}  "
            f"{ba:>8.3f}  {tpr:>6.3f}  {tnr:>6.3f}  "
            f"{n:>5}  {tp:>4}  {tn:>4}  {fp:>4}  {fn:>4}  {total:>6}")


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Timestamp- and log-level evaluation of scenario extractions."
    )
    parser.add_argument(
        "--data", required=True, choices=["tracks", "tracks+state", "tracks+state+map"],
        help=(
            "Which SG dataset results to evaluate. "
            "'tracks': my_results/SG_tracks/. "
            "'tracks+state': my_results/SG_tracks_kinematics/. "
            "'tracks+state+map': my_results/SG_tracks_kinematics_map/."
        ),
    )
    parser.add_argument(
        "--mode", choices=["long_following", "cut_in"], default="long_following",
        help=(
            "Scenario type to evaluate. "
            "'long_following' (default): long_following_* variants vs lead-vehicle APs. "
            "'cut_in': phase-by-phase cut-in variants vs cut-in GT APs."
        ),
    )
    parser.add_argument(
        "--track-aware", action="store_true",
        help=(
            "Require extraction and GT to share the same AV2 track UUID. "
            "Extractions with an unresolvable or non-matching track UUID are "
            "still counted as FP but cannot contribute to TP."
        ),
    )
    detail_grp = parser.add_mutually_exclusive_group()
    detail_grp.add_argument(
        "--per-uuid", action="store_true",
        help="Show per-UUID breakdown.",
    )
    detail_grp.add_argument(
        "--uuid", metavar="UUID8",
        help="Show breakdown for a specific UUID.",
    )
    args = parser.parse_args()

    global MY_RESULTS_DIR
    if args.data == "tracks+state+map":
        MY_RESULTS_DIR = _REPO_ROOT / "SceneFlowLang" / "my_results" / "SG_tracks_kinematics_map"
    elif args.data == "tracks+state":
        MY_RESULTS_DIR = _REPO_ROOT / "SceneFlowLang" / "my_results" / "SG_tracks_kinematics"
    else:
        MY_RESULTS_DIR = _REPO_ROOT / "SceneFlowLang" / "my_results" / "SG_tracks"

    cut_in_mode  = args.mode == "cut_in"
    resolve_gt_ap = _resolve_ci if cut_in_mode else _resolve_lf

    # ── Discover UUIDs and variants ──────────────────────────────────────────
    uuids = sorted(p.stem.replace("_aps", "")
                   for p in GT_APS_DIR.glob("*_aps.json"))

    variants: set[str] = set()
    for uuid8 in uuids:
        d = MY_RESULTS_DIR / uuid8
        if d.exists():
            for sub in d.iterdir():
                if cut_in_mode:
                    if sub.is_dir() and sub.name in CI_VARIANT_GT_AP:
                        variants.add(sub.name)
                else:
                    if sub.is_dir() and sub.name.startswith("long_following"):
                        variants.add(sub.name)

    unmatched = sorted(v for v in variants if resolve_gt_ap(v) is None)
    if unmatched:
        print(f"WARNING: no GT AP mapping for variants (skipped): {unmatched}\n")
    variants = {v for v in variants if resolve_gt_ap(v) is not None}

    if not variants:
        print(f"No {args.mode} extraction variants found under {MY_RESULTS_DIR}.")
        return

    groups: dict[str, list[str]] = defaultdict(list)
    for v in sorted(variants):
        groups[resolve_gt_ap(v)].append(v)

    # Preserve phase order for cut-in; use dict order (insertion) for long-following
    if cut_in_mode:
        ordered_groups = [(ap, groups[ap]) for ap in CI_GT_AP_ORDER if ap in groups]
    else:
        ordered_groups = list(groups.items())

    if args.uuid:
        if args.uuid not in uuids:
            parser.error(f"UUID '{args.uuid}' has no GT file in {GT_APS_DIR}")
        detail_uuids: set[str] = {args.uuid}
    elif args.per_uuid:
        detail_uuids = set(uuids)
    else:
        detail_uuids = set()

    # Cache track mappings (needed for track-aware mode)
    track_mappings: dict[str, dict[int, str]] = {}
    if args.track_aware:
        track_mappings = {uuid8: load_track_mapping(uuid8) for uuid8 in uuids}

    # Cache GT track UUIDs per (uuid, ap_name) for track-aware mode
    gt_uuids_cache: dict[tuple[str, str], set[str]] = {}
    def get_gt_uuids(uuid8: str, ap_name: str) -> set[str]:
        key = (uuid8, ap_name)
        if key not in gt_uuids_cache:
            gt_uuids_cache[key] = load_gt_track_uuids(uuid8, ap_name)
        return gt_uuids_cache[key]

    print(f"UUIDs with ground truth : {len(uuids)}")
    print(f"mode                    : {args.mode}")
    print(f"GT AP groups            : {[ap for ap, _ in ordered_groups]}")
    print(f"track-aware matching    : {args.track_aware}")
    print()

    col_w = max(len(v) for v in variants) + 2

    # Headers
    ts_header  = (f"{'Variant':<{col_w}}  {'TS-BalAcc':>8}  {'TPR':>6}  {'TNR':>6}  "
                  f"{'N':>8}  {'TP':>7}  {'TN':>7}  {'FP':>7}  {'FN':>7}  {'Frames':>8}")
    log_header = (f"{'Variant':<{col_w}}  {'Log-BalAcc':>8}  {'TPR':>6}  {'TNR':>6}  "
                  f"{'N':>5}  {'TP':>4}  {'TN':>4}  {'FP':>4}  {'FN':>4}  {'Logs':>6}")
    sep  = "-" * len(ts_header)
    thin = "·" * len(ts_header)

    # Accumulators for grand total
    grand_ts  = defaultdict(lambda: [0, 0, 0, 0])   # variant → [tp,tn,fp,fn]
    grand_log = defaultdict(lambda: [0, 0, 0, 0])

    first_group = True
    for ap_name, group_variants in ordered_groups:
        if not first_group:
            print()
        first_group = False
        print(f"{'='*len(sep)}")
        print(f"GT AP: {ap_name}")
        print(f"{'='*len(sep)}")

        # ── Per-UUID breakdown ────────────────────────────────────────────
        if detail_uuids:
            for uuid8 in uuids:
                if uuid8 not in detail_uuids:
                    continue
                frame_ts = load_frame_timestamps_s(uuid8)
                print(f"\n  UUID: {uuid8}  ({len(frame_ts)} frames)")

                print(f"\n  [Timestamp-level]")
                print(f"  {ts_header}")
                print(f"  {sep}")
                for variant in group_variants:
                    gt_iv   = load_gt_intervals(uuid8, ap_name)
                    if args.track_aware:
                        pred_inst = load_pred_instances(uuid8, variant, track_mappings[uuid8])
                        gt_uuids  = get_gt_uuids(uuid8, ap_name)
                        correct   = [p for p in pred_inst if p["av2_track_uuid"] in gt_uuids]
                        all_iv    = merge_intervals([(p["start_s"], p["end_s"]) for p in pred_inst])
                        corr_iv   = merge_intervals([(p["start_s"], p["end_s"]) for p in correct])
                        tp, tn, fp, fn = classify_timestamps(frame_ts, gt_iv, all_iv, tp_pred_merged=corr_iv)
                    else:
                        pred_iv = load_pred_intervals(uuid8, variant)
                        tp, tn, fp, fn = classify_timestamps(frame_ts, gt_iv, pred_iv)
                    print(f"  {_fmt_ts(variant, tp, tn, fp, fn, col_w)}")

                print(f"\n  [Log-level]")
                print(f"  {log_header}")
                print(f"  {sep}")
                for variant in group_variants:
                    gt_iv   = load_gt_intervals(uuid8, ap_name)
                    gt_pos  = len(gt_iv) > 0
                    if args.track_aware:
                        pred_inst = load_pred_instances(uuid8, variant, track_mappings[uuid8])
                        gt_uuids  = get_gt_uuids(uuid8, ap_name)
                        correct   = [p for p in pred_inst if p["av2_track_uuid"] in gt_uuids]
                        corr_pred_pos = len(correct) > 0
                        any_pred_pos  = len(pred_inst) > 0
                        pred_pos = corr_pred_pos   # TP eligibility
                        fp_pred_pos = any_pred_pos  # FP eligibility
                    else:
                        pred_iv  = load_pred_intervals(uuid8, variant)
                        pred_pos = fp_pred_pos = len(pred_iv) > 0
                    ltp = ltn = lfp = lfn = 0
                    if gt_pos and pred_pos:              ltp = 1
                    elif not gt_pos and not fp_pred_pos: ltn = 1
                    elif not gt_pos and fp_pred_pos:     lfp = 1
                    else:                                lfn = 1
                    print(f"  {_fmt_log(variant, ltp, ltn, lfp, lfn, col_w)}")

                print()

        # ── Aggregated summary ────────────────────────────────────────────
        print(f"\n[Timestamp-level — aggregated over all UUIDs]")
        print(ts_header)
        print(sep)

        for variant in group_variants:
            agg_tp = agg_tn = agg_fp = agg_fn = 0
            for uuid8 in uuids:
                frame_ts = load_frame_timestamps_s(uuid8)
                gt_iv    = load_gt_intervals(uuid8, ap_name)
                if args.track_aware:
                    pred_inst = load_pred_instances(uuid8, variant, track_mappings[uuid8])
                    gt_uuids  = get_gt_uuids(uuid8, ap_name)
                    correct   = [p for p in pred_inst if p["av2_track_uuid"] in gt_uuids]
                    all_iv    = merge_intervals([(p["start_s"], p["end_s"]) for p in pred_inst])
                    corr_iv   = merge_intervals([(p["start_s"], p["end_s"]) for p in correct])
                    tp, tn, fp, fn = classify_timestamps(frame_ts, gt_iv, all_iv, tp_pred_merged=corr_iv)
                else:
                    pred_iv = load_pred_intervals(uuid8, variant)
                    tp, tn, fp, fn = classify_timestamps(frame_ts, gt_iv, pred_iv)
                agg_tp += tp; agg_tn += tn; agg_fp += fp; agg_fn += fn
            print(_fmt_ts(variant, agg_tp, agg_tn, agg_fp, agg_fn, col_w))
            for i, v in enumerate([agg_tp, agg_tn, agg_fp, agg_fn]):
                grand_ts[variant][i] += v

        print(f"\n[Log-level — aggregated over all UUIDs]")
        print(log_header)
        print(sep)

        for variant in group_variants:
            agg_tp = agg_tn = agg_fp = agg_fn = 0
            for uuid8 in uuids:
                gt_iv   = load_gt_intervals(uuid8, ap_name)
                gt_pos  = len(gt_iv) > 0
                if args.track_aware:
                    pred_inst = load_pred_instances(uuid8, variant, track_mappings[uuid8])
                    gt_uuids  = get_gt_uuids(uuid8, ap_name)
                    correct   = [p for p in pred_inst if p["av2_track_uuid"] in gt_uuids]
                    pred_pos    = len(correct) > 0
                    fp_pred_pos = len(pred_inst) > 0
                else:
                    pred_iv     = load_pred_intervals(uuid8, variant)
                    pred_pos = fp_pred_pos = len(pred_iv) > 0
                if gt_pos and pred_pos:              agg_tp += 1
                elif not gt_pos and not fp_pred_pos: agg_tn += 1
                elif not gt_pos and fp_pred_pos:     agg_fp += 1
                else:                                agg_fn += 1
            print(_fmt_log(variant, agg_tp, agg_tn, agg_fp, agg_fn, col_w))
            for i, v in enumerate([agg_tp, agg_tn, agg_fp, agg_fn]):
                grand_log[variant][i] += v


    # ── Grand totals across all AP groups ────────────────────────────────────
    if len(ordered_groups) > 1:
        print(f"\n{'='*len(sep)}")
        print("GRAND TOTAL (all AP groups combined)")
        print(f"{'='*len(sep)}")

        print(f"\n[Timestamp-level]")
        print(ts_header)
        print(sep)
        gtot_tp = gtot_tn = gtot_fp = gtot_fn = 0
        for variant in sorted(grand_ts):
            tp, tn, fp, fn = grand_ts[variant]
            print(_fmt_ts(variant, tp, tn, fp, fn, col_w))
            gtot_tp += tp; gtot_tn += tn; gtot_fp += fp; gtot_fn += fn
        print(thin)
        print(_fmt_ts("  TOTAL", gtot_tp, gtot_tn, gtot_fp, gtot_fn, col_w))

        print(f"\n[Log-level]")
        print(log_header)
        print(sep)
        gtot_tp = gtot_tn = gtot_fp = gtot_fn = 0
        for variant in sorted(grand_log):
            tp, tn, fp, fn = grand_log[variant]
            print(_fmt_log(variant, tp, tn, fp, fn, col_w))
            gtot_tp += tp; gtot_tn += tn; gtot_fp += fp; gtot_fn += fn
        print(thin)
        print(_fmt_log("  TOTAL", gtot_tp, gtot_tn, gtot_fp, gtot_fn, col_w))



if __name__ == "__main__":
    main()
