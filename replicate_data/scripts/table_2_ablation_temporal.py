#!/usr/bin/env python3
"""
table_2_ablation_temporal.py — Reproduce Table 2 (temporal localization ablation) from the paper.

Evaluates cut-in and long-following scenario extractions across three SG levels
(tracks, tracks+state, tracks+state+map) plus a non-track-aware baseline, using
timestamp-based (duration-overlap) evaluation.

Run from the repo root:
    python replicate_data/scripts/table_2_ablation_temporal.py
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(_REPO_ROOT / "scenarios"))

import evaluate_cut_in as ec


DATA_CONFIGS = [
    ("Level I: Tracking-only",                   "SG_tracks",                True),
    ("Level II: Tracking & State",               "SG_tracks_kinematics",     True),
    ("Level III: Tracking, State, & Map",        "SG_tracks_kinematics_map", True),
    ("Tracking, State, & Map (non-track-aware)", "SG_tracks_kinematics_map", False),
]

GT_APS_DIR    = _REPO_ROOT / "scenarios" / "aps"
SUMMARIES_DIR = Path(__file__).resolve().parent.parent / "precomputed"


def load_gt(uuid8: str, ap_name: str) -> list[dict]:
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
            if start_s is not None and end_s is not None:
                instances.append({"av2_track_uuid": av2_uuid, "start_s": start_s, "end_s": end_s})
    return instances


def all_uuids() -> list[str]:
    return sorted(p.stem.replace("_aps", "") for p in GT_APS_DIR.glob("*_aps.json"))


def evaluate_scenario(level_extractions: dict, uuids: list[str],
                      variant: str, ap_name: str, track_aware: bool) -> dict:
    all_ext: list[dict] = []
    all_gt:  list[dict] = []
    for uuid8 in uuids:
        all_ext.extend(level_extractions.get(uuid8, {}).get(variant, []))
        all_gt.extend(load_gt(uuid8, ap_name))
    return ec.compute_metrics_timestamp(all_ext, all_gt, track_aware=track_aware)


def main() -> None:
    uuids = all_uuids()
    if not uuids:
        print(f"No ground-truth APs found in {GT_APS_DIR}")
        sys.exit(1)

    with open(SUMMARIES_DIR / "extractions.json") as f:
        all_extractions = json.load(f)

    rows: list[tuple[str, str, float, float, float]] = []

    for level_label, sg_dir, track_aware in DATA_CONFIGS:
        level_extractions = all_extractions.get(sg_dir, {})

        m30 = evaluate_scenario(level_extractions, uuids,
                                "long_following_near_3", "lead_vehicle_in_ego_lane_30m", track_aware)
        rows.append((level_label, "φ_long_following (30 m)", m30["precision"], m30["recall"], m30["f1"]))

        m60 = evaluate_scenario(level_extractions, uuids,
                                "long_following_visible_3", "lead_vehicle_in_ego_lane_60m", track_aware)
        rows.append((level_label, "φ_long_following (60 m)", m60["precision"], m60["recall"], m60["f1"]))

        mci = evaluate_scenario(level_extractions, uuids,
                                "cut_in_from_right", "cut_in_from_right_full_sequence", track_aware)
        rows.append((level_label, "φ_cut_in", mci["precision"], mci["recall"], mci["f1"]))

    col_level    = 42
    col_scenario = 26
    header = (f"{'SG Level':<{col_level}}  {'Scenario':<{col_scenario}}  "
              f"{'Precision':>9}  {'Recall':>6}  {'F1':>6}")
    sep = "-" * len(header)

    print()
    print("Table 2: Ablation — timestamp-based temporal localization, track-aware (rows 1-9) / non-track-aware (rows 10-12)")
    print(sep)
    print(header)
    print(sep)

    prev_level = None
    for level_label, scenario_label, precision, recall, f1 in rows:
        level_col = level_label if level_label != prev_level else ""
        prev_level = level_label
        print(f"{level_col:<{col_level}}  {scenario_label:<{col_scenario}}  "
              f"{precision:>9.3f}  {recall:>6.3f}  {f1:>6.3f}")
        if scenario_label == "φ_cut_in":
            print(sep)

    print()


if __name__ == "__main__":
    main()
