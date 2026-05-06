#!/usr/bin/env python3
"""
evaluate_long_following.py — Compare long_following_* extractions against
ground-truth lead_vehicle_in_ego_lane APs.

Metrics are computed per long_following_* variant (aggregated over all UUIDs),
grouped by the GT AP each variant is evaluated against, with a per-group
subtotal and a grand COMBINED row.

Three evaluation strategies are available (select with --strategy):

  greedy      — Greedy one-to-one interval matching.
                An extraction counts as TP if its IoU with an unmatched GT instance
                >= --min-overlap.  Each GT instance is claimable at most once.
                TP/FP/FN are counts of intervals.

  timestamp   — Timestamp-based coverage evaluation.
                The GT active duration (union of all GT intervals) defines the
                positive signal.  TP/FP/FN are durations in seconds:
                  TP = total duration covered by both GT and extractions
                  FP = total extraction duration not covered by GT
                  FN = total GT duration not covered by extractions
                --min-overlap is not applicable to this strategy.

  many_to_one — Forgiving instance-level coverage matching.
                For each GT interval, all overlapping predictions are collected,
                their union within the GT interval is computed, and
                coverage = covered_duration / GT_duration.  A GT interval is
                detected (TP) when coverage > --min-overlap; otherwise FN.
                A prediction is useful (not FP) if it contributes positive
                overlap to at least one detected GT interval.
                TP/FP/FN are counts of intervals / predictions.

Variant → GT AP mapping
-----------------------
Controlled by VARIANT_GT_AP below.  Each key is a variant *prefix*; any
long_following_* variant whose name starts with that prefix is evaluated
against the corresponding GT AP.  Variants that match no prefix are skipped
with a warning.

Assumptions
-----------
- The vehicle of interest in every satisfaction file is keyed as 'yield_vehicle_1'
  in entity_mapping.  (All checked files use this key.)
- 'track_car_X' in entity_mapping → the integer X is the RS2V track_id; the
  AV2 UUID is found by scanning *_matched.json for the first frame where any
  car entry has track_id == X and a non-null av2_track_uuid.
- Extractions whose track cannot be resolved to an AV2 UUID (no match in the
  matched JSON) are counted as False Positives.
- GT intervals are taken from start_global_time_s / end_global_time_s (absolute
  seconds) present in all generated *_aps.json files.
- Extraction timestamps (start_frame / end_frame) are nanoseconds; dividing by
  1e9 converts them to the same absolute seconds as the GT.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from collections import defaultdict

# ─────────────────────────────────────────────────────────────────────────────
# Paths
# ─────────────────────────────────────────────────────────────────────────────

_SCRIPT_DIR  = Path(__file__).resolve().parent
_REPO_ROOT   = _SCRIPT_DIR.parent

MY_RESULTS_DIR = _REPO_ROOT / "SceneFlowLang" / "my_results" / "SG_tracks"  # overridden by --data
GT_APS_DIR     = _SCRIPT_DIR / "aps"
MATCHED_DIR    = _REPO_ROOT / "sg_processing" / "code" / "tracking_estimation" / "obj_positions"

# ─────────────────────────────────────────────────────────────────────────────
# Variant → GT AP mapping
#
# Key   : variant name prefix (longest matching prefix wins)
# Value : AP name in the *_aps.json files (as registered in extract_aps.py)
# ─────────────────────────────────────────────────────────────────────────────

VARIANT_GT_AP: dict[str, str] = {
    # Distance-based variants
    "long_following_visible":    "lead_vehicle_in_ego_lane_60m",  # ≤ 60 m
    "long_following_near":       "lead_vehicle_in_ego_lane_30m",  # ≤ 30 m
    "long_following_very_near":  "lead_vehicle_in_ego_lane_18m",  # ≤ 18 m
    "long_following_super_near": "lead_vehicle_in_ego_lane_12m",  # ≤ 12 m
    "long_following_near_coll":  "lead_vehicle_in_ego_lane_6m",   # ≤  6 m
    # Headway-based variants
    "long_following_headway_1.0s": "lead_vehicle_in_ego_lane_1s", # ≤ 1 s
    "long_following_headway_2.0s": "lead_vehicle_in_ego_lane_2s", # ≤ 2 s
    "long_following_headway_3.0s": "lead_vehicle_in_ego_lane_3s", # ≤ 3 s
}


def resolve_gt_ap(variant: str) -> str | None:
    """Return the GT AP name for a variant, or None if no prefix matches."""
    # Longest matching prefix wins, so more specific prefixes take priority.
    match = max(
        (pfx for pfx in VARIANT_GT_AP if variant.startswith(pfx)),
        key=len,
        default=None,
    )
    return VARIANT_GT_AP[match] if match else None


# ─────────────────────────────────────────────────────────────────────────────
# Track ID resolution
# ─────────────────────────────────────────────────────────────────────────────

def load_track_mapping(uuid8: str) -> dict[int, str]:
    """
    Build track_id (int) → av2_track_uuid (str) mapping for a scenario.

    Scans every frame in *_matched.json; uses the first frame where a given
    track_id has a non-null av2_track_uuid.  Track IDs are not reused within
    a scenario, so one occurrence is sufficient.
    """
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
    """'track_car_7' / 'track_truck_7' / 'track_bus_7' → 7.  Returns None if format is unexpected."""
    parts = label.split("_")
    if len(parts) >= 3 and parts[0] == "track":
        try:
            return int(parts[-1])
        except ValueError:
            pass
    return None


# ─────────────────────────────────────────────────────────────────────────────
# Load extractions
# ─────────────────────────────────────────────────────────────────────────────

def load_extractions(uuid8: str, variant: str,
                     track_mapping: dict[int, str]) -> list[dict]:
    """
    Return all satisfaction instances for (uuid8, variant) as a list of dicts:
      { "av2_track_uuid": str | None, "start_s": float, "end_s": float }

    av2_track_uuid is None when the track cannot be resolved; those instances
    are still returned so they are counted as FP.
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
            # Fallback: take the first value in entity_mapping
            track_label = next(iter(entity_mapping.values()), None)
        if track_label is None:
            continue

        track_int  = parse_track_int(track_label)
        av2_uuid   = track_mapping.get(track_int) if track_int is not None else None

        start_ns = int(data["start_frame"])
        end_ns   = int(data["end_frame"])

        results.append({
            "av2_track_uuid": av2_uuid,
            "start_s":        start_ns / 1e9,
            "end_s":          end_ns   / 1e9,
        })
    return results


# ─────────────────────────────────────────────────────────────────────────────
# Load ground truth
# ─────────────────────────────────────────────────────────────────────────────

def load_gt_instances(uuid8: str, ap_name: str) -> list[dict]:
    """
    Return all instances of ap_name for uuid8 as a list of dicts:
      { "av2_track_uuid": str, "start_s": float, "end_s": float }
    """
    gt_path = GT_APS_DIR / f"{uuid8}_aps.json"
    if not gt_path.exists():
        return []
    with open(gt_path) as f:
        data = json.load(f)

    instances = []
    for track_entry in data.get("aps", {}).get(ap_name, []):
        av2_uuid = track_entry["track_id"]
        for inst in track_entry.get("instances", []):
            start_s = inst.get("start_global_time_s")
            end_s   = inst.get("end_global_time_s")
            if start_s is None or end_s is None:
                continue
            instances.append({
                "av2_track_uuid": av2_uuid,
                "start_s":        start_s,
                "end_s":          end_s,
            })
    return instances


# ─────────────────────────────────────────────────────────────────────────────
# Matching & metrics
# ─────────────────────────────────────────────────────────────────────────────

def _iou(a_start: float, a_end: float,
         b_start: float, b_end: float) -> float:
    """Return IoU (intersection / union) of two intervals, in [0, 1]."""
    intersection = max(0.0, min(a_end, b_end) - max(a_start, b_start))
    if intersection == 0.0:
        return 0.0
    union = (a_end - a_start) + (b_end - b_start) - intersection
    return intersection / union if union > 0.0 else 0.0


def compute_metrics_greedy(extractions: list[dict],
                           gt_instances: list[dict],
                           min_iou: float = 0.0,
                           track_aware: bool = False) -> dict:
    """
    Greedy one-to-one matching of extractions to GT instances.

    An extraction matches a GT instance when IoU >= min_iou.
    When track_aware=True, track UUIDs must also match (both non-None).
    Each GT instance is claimable at most once.

    Returns dict with keys: tp, fp, fn, precision, recall, f1.
    tp/fp/fn are integer interval counts.
    """
    gt_matched = [False] * len(gt_instances)
    tp = fp = 0

    for ext in extractions:
        matched = False
        for i, gt in enumerate(gt_instances):
            if gt_matched[i]:
                continue
            if track_aware:
                ext_uuid = ext.get("av2_track_uuid")
                gt_uuid  = gt.get("av2_track_uuid")
                if ext_uuid is None or gt_uuid is None or ext_uuid != gt_uuid:
                    continue
            if _iou(ext["start_s"], ext["end_s"],
                    gt["start_s"], gt["end_s"]) > min_iou:
                gt_matched[i] = True
                matched = True
                break

        if matched:
            tp += 1
        else:
            fp += 1

    fn = sum(1 for m in gt_matched if not m)

    precision = tp / (tp + fp)           if (tp + fp) > 0 else 0.0
    recall    = tp / (tp + fn)           if (tp + fn) > 0 else 0.0
    f1        = (2 * precision * recall
                 / (precision + recall)) if (precision + recall) > 0 else 0.0

    return {"tp": tp, "fp": fp, "fn": fn, "n": len(gt_instances),
            "precision": precision, "recall": recall, "f1": f1}


# ─── Timestamp-based helpers ─────────────────────────────────────────────────

def _merge_intervals(intervals: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """Return sorted, non-overlapping union of the given intervals."""
    if not intervals:
        return []
    result = [list(sorted(intervals, key=lambda x: x[0])[0])]
    for start, end in sorted(intervals, key=lambda x: x[0])[1:]:
        if start <= result[-1][1]:
            result[-1][1] = max(result[-1][1], end)
        else:
            result.append([start, end])
    return [(s, e) for s, e in result]


def _total_duration(intervals: list[tuple[float, float]]) -> float:
    return sum(e - s for s, e in intervals)


def _intersection_duration(a: list[tuple[float, float]],
                           b: list[tuple[float, float]]) -> float:
    """Total duration of the intersection of two sets of merged intervals."""
    i = j = 0
    total = 0.0
    while i < len(a) and j < len(b):
        overlap = max(0.0, min(a[i][1], b[j][1]) - max(a[i][0], b[j][0]))
        total += overlap
        if a[i][1] < b[j][1]:
            i += 1
        else:
            j += 1
    return total


def compute_metrics_many_to_one(extractions: list[dict],
                                gt_instances: list[dict],
                                min_coverage: float = 0.0,
                                track_aware: bool = False) -> dict:
    """
    Many-to-one coverage matching.

    For each GT interval, collect all overlapping predictions, take their union
    within the GT interval, and compute coverage = covered / GT_duration.
    A GT interval is TP when coverage > min_coverage, else FN.

    A prediction counts as useful (not FP) if it overlaps with at least one TP
    GT interval (and, when track_aware=True, shares the same av2_track_uuid).
    Predictions with an unresolvable track UUID are always FP in track_aware mode.

    Returns dict with keys: tp, fp, fn, precision, recall, f1.
    tp/fp/fn are integer counts.
    """
    useful_ext_indices: set[int] = set()
    tp = fn = 0

    for gt in gt_instances:
        gt_start = gt["start_s"]
        gt_end   = gt["end_s"]
        gt_dur   = gt_end - gt_start
        if gt_dur <= 0.0:
            fn += 1
            continue

        overlapping_idx: list[int] = []
        clipped: list[tuple[float, float]] = []
        for j, ext in enumerate(extractions):
            if track_aware:
                ext_uuid = ext.get("av2_track_uuid")
                gt_uuid  = gt.get("av2_track_uuid")
                if ext_uuid is None or gt_uuid is None or ext_uuid != gt_uuid:
                    continue
            clip_s = max(ext["start_s"], gt_start)
            clip_e = min(ext["end_s"],   gt_end)
            if clip_e > clip_s:
                overlapping_idx.append(j)
                clipped.append((clip_s, clip_e))

        covered  = _total_duration(_merge_intervals(clipped))
        coverage = covered / gt_dur

        if coverage > min_coverage:
            tp += 1
            useful_ext_indices.update(overlapping_idx)
        else:
            fn += 1

    fp = sum(1 for j in range(len(extractions)) if j not in useful_ext_indices)

    precision = tp / (tp + fp)           if (tp + fp) > 0 else 0.0
    recall    = tp / (tp + fn)           if (tp + fn) > 0 else 0.0
    f1        = (2 * precision * recall
                 / (precision + recall)) if (precision + recall) > 0 else 0.0

    return {"tp": tp, "fp": fp, "fn": fn, "n": len(gt_instances),
            "precision": precision, "recall": recall, "f1": f1}


def compute_metrics_timestamp(extractions: list[dict],
                               gt_instances: list[dict],
                               track_aware: bool = False) -> dict:
    """
    Timestamp-based evaluation.

    TP (seconds) = duration covered by both GT and extractions
    FP (seconds) = extraction duration not covered by GT
    FN (seconds) = GT duration not covered by extractions

    When track_aware=True, extractions whose av2_track_uuid does not appear in
    any GT instance are always FP for their full duration and cannot contribute
    to TP.

    Returns dict with keys: tp, fp, fn, precision, recall, f1.
    tp/fp/fn are durations in seconds (floats).
    """
    if track_aware:
        gt_uuids = {g["av2_track_uuid"] for g in gt_instances}
        correct_track = [e for e in extractions
                         if e.get("av2_track_uuid") is not None
                         and e["av2_track_uuid"] in gt_uuids]
        wrong_track   = [e for e in extractions
                         if e.get("av2_track_uuid") is None
                         or e["av2_track_uuid"] not in gt_uuids]
    else:
        correct_track = extractions
        wrong_track   = []

    gt_merged    = _merge_intervals([(g["start_s"], g["end_s"]) for g in gt_instances])
    ext_merged   = _merge_intervals([(e["start_s"], e["end_s"]) for e in correct_track])
    wrong_merged = _merge_intervals([(e["start_s"], e["end_s"]) for e in wrong_track])

    gt_dur  = _total_duration(gt_merged)
    ext_dur = _total_duration(ext_merged)
    tp      = _intersection_duration(gt_merged, ext_merged)
    fp      = (ext_dur - tp) + _total_duration(wrong_merged)
    fn      = gt_dur  - tp

    precision = tp / (tp + fp)           if (tp + fp) > 0 else 0.0
    recall    = tp / (tp + fn)           if (tp + fn) > 0 else 0.0
    f1        = (2 * precision * recall
                 / (precision + recall)) if (precision + recall) > 0 else 0.0

    return {"tp": tp, "fp": fp, "fn": fn, "n": len(gt_instances),
            "precision": precision, "recall": recall, "f1": f1}


def _fmt_row(label: str, m: dict, col_w: int, timestamp: bool = False) -> str:
    n = m.get("n", 0)
    if timestamp:
        return (f"{label:<{col_w}}  "
                f"{n:>5}  "
                f"{m['precision']:>9.3f}  "
                f"{m['recall']:>6.3f}  "
                f"{m['f1']:>6.3f}  "
                f"{m['tp']:>8.1f}s"
                f"{m['fp']:>8.1f}s"
                f"{m['fn']:>8.1f}s")
    return (f"{label:<{col_w}}  "
            f"{n:>5}  "
            f"{m['precision']:>9.3f}  "
            f"{m['recall']:>6.3f}  "
            f"{m['f1']:>6.3f}  "
            f"{m['tp']:>5}  "
            f"{m['fp']:>5}  "
            f"{m['fn']:>5}")


def _aggregate(counts: list[tuple]) -> dict:
    tp = sum(c[0] for c in counts)
    fp = sum(c[1] for c in counts)
    fn = sum(c[2] for c in counts)
    n  = sum(c[3] for c in counts)
    precision = tp / (tp + fp)           if (tp + fp) > 0 else 0.0
    recall    = tp / (tp + fn)           if (tp + fn) > 0 else 0.0
    f1        = (2 * precision * recall
                 / (precision + recall)) if (precision + recall) > 0 else 0.0
    return {"tp": tp, "fp": fp, "fn": fn, "n": n,
            "precision": precision, "recall": recall, "f1": f1}


# ─────────────────────────────────────────────────────────────────────────────
# Main
# ─────────────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="Evaluate long_following extractions against GT APs.")
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
        "--strategy", required=True, choices=["greedy", "timestamp", "many_to_one"],
        help=(
            "Evaluation strategy. "
            "'greedy': greedy 1-1 interval matching; TP/FP/FN are interval counts. "
            "'timestamp': coverage-based; TP/FP/FN are durations in seconds. "
            "'many_to_one': forgiving coverage matching; multiple predictions may "
            "cover one GT interval; TP/FP/FN are interval/prediction counts."
        ),
    )
    parser.add_argument(
        "--min-overlap", type=float, default=0.0, metavar="PERCENT",
        help=(
            "Minimum overlap required for a TP (0–100, default: 0). "
            "For 'greedy': minimum IoU %%. "
            "For 'many_to_one': minimum coverage %% of the GT interval that must "
            "be covered by the union of overlapping predictions."
        ),
    )
    parser.add_argument(
        "--track-aware", action="store_true",
        help=(
            "Require extraction and GT to share the same AV2 track UUID. "
            "For greedy: track UUIDs must match in addition to the IoU threshold. "
            "For timestamp: extractions whose track UUID does not appear in any GT "
            "instance are counted as FP for their full duration. "
            "For many_to_one: predictions are only matched to GT intervals on the "
            "same track; predictions with unresolvable track IDs are always FP. "
            "Extractions with unresolvable track IDs are always counted as FP."
        ),
    )
    detail_grp = parser.add_mutually_exclusive_group()
    detail_grp.add_argument(
        "--per-uuid", action="store_true",
        help="Show per-UUID breakdown for every UUID, then the summary.",
    )
    detail_grp.add_argument(
        "--uuid", metavar="UUID8",
        help="Show per-UUID breakdown for a specific UUID, then the summary.",
    )
    args = parser.parse_args()
    if not 0.0 <= args.min_overlap <= 100.0:
        parser.error("--min-overlap must be between 0 and 100")
    min_iou = args.min_overlap / 100.0
    use_timestamp   = args.strategy == "timestamp"
    use_many_to_one = args.strategy == "many_to_one"

    global MY_RESULTS_DIR
    if args.data == "tracks+state+map":
        MY_RESULTS_DIR = _REPO_ROOT / "SceneFlowLang" / "my_results" / "SG_tracks_kinematics_map"
    elif args.data == "tracks+state":
        MY_RESULTS_DIR = _REPO_ROOT / "SceneFlowLang" / "my_results" / "SG_tracks_kinematics"
    else:
        MY_RESULTS_DIR = _REPO_ROOT / "SceneFlowLang" / "my_results" / "SG_tracks"

    # Discover all UUIDs that have a ground-truth file
    uuids = sorted(p.stem.replace("_aps", "")
                   for p in GT_APS_DIR.glob("*_aps.json"))

    # Discover all long_following_* variant names present in my_results
    variants: set[str] = set()
    for uuid8 in uuids:
        scenario_dir = MY_RESULTS_DIR / uuid8
        if scenario_dir.exists():
            for d in scenario_dir.iterdir():
                if d.is_dir() and d.name.startswith("long_following"):
                    variants.add(d.name)

    # Warn about and discard variants that have no GT AP mapping
    unmatched = sorted(v for v in variants if resolve_gt_ap(v) is None)
    if unmatched:
        print(f"WARNING: no GT AP mapping for variants (skipped): {unmatched}")
        print()
    variants = {v for v in variants if resolve_gt_ap(v) is not None}

    if not variants:
        print(f"No long_following extraction variants found under {MY_RESULTS_DIR}.")
        return

    # Group variants by GT AP (preserving alphabetical order within each group)
    groups: dict[str, list[str]] = defaultdict(list)
    for v in sorted(variants):
        groups[resolve_gt_ap(v)].append(v)

    print(f"UUIDs with ground truth : {len(uuids)}")
    print(f"GT AP groups            : {list(groups.keys())}")
    print(f"evaluation strategy     : {args.strategy}")
    if args.strategy == "greedy":
        print(f"min IoU overlap required: {args.min_overlap:.1f}%")
    elif args.strategy == "many_to_one":
        print(f"min coverage threshold  : {args.min_overlap:.1f}%")
    print(f"track-aware matching    : {args.track_aware}")
    print()

    # Cache track mappings — one per UUID (shared across all variants)
    track_mappings: dict[str, dict[int, str]] = {
        uuid8: load_track_mapping(uuid8) for uuid8 in uuids
    }

    # Cache GT instances per (uuid, ap_name) — shared across variants in a group
    gt_cache: dict[tuple[str, str], list[dict]] = {}
    def get_gt(uuid8: str, ap_name: str) -> list[dict]:
        key = (uuid8, ap_name)
        if key not in gt_cache:
            gt_cache[key] = load_gt_instances(uuid8, ap_name)
        return gt_cache[key]

    # Which UUIDs to expand in the per-UUID breakdown
    if args.uuid:
        if args.uuid not in uuids:
            parser.error(f"UUID '{args.uuid}' has no GT file in {GT_APS_DIR}")
        detail_uuids: set[str] = {args.uuid}
    elif args.per_uuid:
        detail_uuids = set(uuids)
    else:
        detail_uuids = set()

    col_w = max(len(v) for v in variants) + 2
    if use_timestamp:
        header = (f"{'Variant':<{col_w}}  {'N':>5}  {'Precision':>9}  {'Recall':>6}  "
                  f"{'F1':>6}  {'TP (s)':>9}  {'FP (s)':>8}  {'FN (s)':>8}")
    elif use_many_to_one:
        header = (f"{'Variant':<{col_w}}  {'N':>5}  {'Precision':>9}  {'Recall':>6}  "
                  f"{'F1':>6}  {'TP(GT)':>6}  {'FP(pred)':>8}  {'FN(GT)':>6}")
    else:
        header = (f"{'Variant':<{col_w}}  {'N':>5}  {'Precision':>9}  {'Recall':>6}  "
                  f"{'F1':>6}  {'TP':>5}  {'FP':>5}  {'FN':>5}")
    sep    = "-" * len(header)
    thin   = "·" * len(header)

    grand_counts: list[tuple[int, int, int, int]] = []

    first_group = True
    for ap_name, group_variants in groups.items():
        if not first_group:
            print()
        first_group = False

        print(f"GT AP: {ap_name}")

        # ── Per-UUID breakdown ────────────────────────────────────────────
        if detail_uuids:
            for uuid8 in uuids:
                if uuid8 not in detail_uuids:
                    continue
                print(f"  UUID: {uuid8}")
                print(f"  {header}")
                print(f"  {sep}")
                for variant in group_variants:
                    ext = load_extractions(uuid8, variant, track_mappings[uuid8])
                    gt  = get_gt(uuid8, ap_name)
                    if use_timestamp:
                        m = compute_metrics_timestamp(ext, gt, track_aware=args.track_aware)
                    elif use_many_to_one:
                        m = compute_metrics_many_to_one(ext, gt, min_coverage=min_iou,
                                                        track_aware=args.track_aware)
                    else:
                        m = compute_metrics_greedy(ext, gt, min_iou=min_iou,
                                                   track_aware=args.track_aware)
                    print(f"  {_fmt_row(variant, m, col_w, timestamp=use_timestamp)}")
                print()

        # ── Aggregated summary rows ───────────────────────────────────────
        print(header)
        print(sep)

        group_counts: list[tuple[int, int, int, int]] = []

        for variant in group_variants:
            all_ext: list[dict] = []
            all_gt:  list[dict] = []

            for uuid8 in uuids:
                all_ext.extend(load_extractions(uuid8, variant, track_mappings[uuid8]))
                all_gt.extend(get_gt(uuid8, ap_name))

            if use_timestamp:
                m = compute_metrics_timestamp(all_ext, all_gt, track_aware=args.track_aware)
            elif use_many_to_one:
                m = compute_metrics_many_to_one(all_ext, all_gt, min_coverage=min_iou,
                                                track_aware=args.track_aware)
            else:
                m = compute_metrics_greedy(all_ext, all_gt, min_iou=min_iou,
                                           track_aware=args.track_aware)
            group_counts.append((m["tp"], m["fp"], m["fn"], m["n"]))
            print(_fmt_row(variant, m, col_w, timestamp=use_timestamp))

        # Per-group subtotal
        gm = _aggregate(group_counts)
        grand_counts.extend(group_counts)
        print(thin)
        print(_fmt_row(f"  subtotal ({ap_name})", gm, col_w, timestamp=use_timestamp))

    # Grand combined
    print()
    print(sep)
    print(_fmt_row("COMBINED", _aggregate(grand_counts), col_w, timestamp=use_timestamp))


if __name__ == "__main__":
    main()
