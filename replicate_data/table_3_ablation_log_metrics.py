#!/usr/bin/env python3
"""
table_3_ablation_log_metrics.py — Reproduce Table 3 (log-level scenario extraction
performance) from the paper.

Evaluates cut-in and long-following extractions across three SG levels using
log-level balanced accuracy, TP rate, and TN rate. Evaluation is not track-aware
(a log is predicted-positive if any extraction interval is present).
N = 848 logs across all SG levels (850 in paper; 2 logs absent from results).

Run from the repo root:
    python replicate_data/table_3_ablation_log_metrics.py
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent

DATA_CONFIGS = [
    ("Level I: Tracking-only",            "SG_tracks"),
    ("Level II: Tracking & State",        "SG_tracks_kinematics"),
    ("Level III: Tracking, State, & Map", "SG_tracks_kinematics_map"),
]

SCENARIOS = [
    ("φ_long_following (30 m)", "long_following_near_3",    "lead_vehicle_in_ego_lane_30m"),
    ("φ_long_following (60 m)", "long_following_visible_3", "lead_vehicle_in_ego_lane_60m"),
    ("φ_cut_in",                "cut_in_from_right",        "cut_in_from_right_full_sequence"),
]

GT_APS_DIR    = _REPO_ROOT / "scenarios" / "aps"
SUMMARIES_DIR = Path(__file__).resolve().parent


def has_gt(uuid8: str, ap_name: str) -> bool:
    gt_path = GT_APS_DIR / f"{uuid8}_aps.json"
    if not gt_path.exists():
        return False
    with open(gt_path) as f:
        data = json.load(f)
    for track_entry in data.get("aps", {}).get(ap_name, []):
        for inst in track_entry.get("instances", []):
            if inst.get("start_global_time_s") is not None:
                return True
    return False


def compute_rates(tp: int, tn: int, fp: int, fn: int) -> tuple[float, float, float]:
    tpr = tp / (tp + fn) if (tp + fn) > 0 else 0.0
    tnr = tn / (tn + fp) if (tn + fp) > 0 else 0.0
    return tpr, tnr, 0.5 * (tpr + tnr)


def all_uuids() -> list[str]:
    return sorted(p.stem.replace("_aps", "") for p in GT_APS_DIR.glob("*_aps.json"))


def main() -> None:
    uuids = all_uuids()
    if not uuids:
        print(f"No ground-truth APs found in {GT_APS_DIR}")
        sys.exit(1)

    with open(SUMMARIES_DIR / "extractions.json") as f:
        all_extractions = json.load(f)

    rows: list[tuple[str, str, int, float, float, float]] = []

    for level_label, sg_dir in DATA_CONFIGS:
        level_extractions = all_extractions.get(sg_dir, {})
        for scenario_label, variant, ap_name in SCENARIOS:
            tp = tn = fp = fn = 0
            for uuid8 in uuids:
                gt_pos   = has_gt(uuid8, ap_name)
                pred_pos = bool(level_extractions.get(uuid8, {}).get(variant))
                if gt_pos and pred_pos:
                    tp += 1
                elif not gt_pos and not pred_pos:
                    tn += 1
                elif not gt_pos and pred_pos:
                    fp += 1
                else:
                    fn += 1
            tpr, tnr, ba = compute_rates(tp, tn, fp, fn)
            rows.append((level_label, scenario_label, tp + tn + fp + fn, ba, tpr, tnr))

    col_level    = 42
    col_scenario = 26
    header = (f"{'SG Level':<{col_level}}  {'Scenario':<{col_scenario}}  "
              f"{'N':>5}  {'Log-bal. Acc':>12}  {'TP Rate':>7}  {'TN Rate':>7}")
    sep = "-" * len(header)

    print()
    print("Table 3: Log-level scenario extraction performance (non-track-aware)")
    print(sep)
    print(header)
    print(sep)

    prev_level = None
    for level_label, scenario_label, n, ba, tpr, tnr in rows:
        level_col = level_label if level_label != prev_level else ""
        prev_level = level_label
        print(f"{level_col:<{col_level}}  {scenario_label:<{col_scenario}}  "
              f"{n:>5}  {ba:>12.3f}  {tpr:>7.3f}  {tnr:>7.3f}")
        if scenario_label == "φ_cut_in":
            print(sep)

    print()


if __name__ == "__main__":
    main()
