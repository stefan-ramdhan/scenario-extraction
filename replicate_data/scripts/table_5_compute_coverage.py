#!/usr/bin/env python3
"""
table_5_compute_coverage.py — Reproduce Table 5 (scenario coverage of the full
Argoverse 2 training and validation dataset) from the paper.

Uses track+state+map scene graph results (SG_tracks_kinematics_map).

Run from the repo root:
    python replicate_data/scripts/table_5_compute_coverage.py
"""

from __future__ import annotations

import json
import statistics
import sys
from collections import defaultdict
from pathlib import Path

_REPO_ROOT    = Path(__file__).resolve().parent.parent.parent
SUMMARIES_DIR = Path(__file__).resolve().parent.parent / "precomputed"

TOTAL_SCENES = 848

SCENARIOS = [
    ("φ_long_following,30m",                "long_following_near_3"),
    ("φ_long_following,60m",                "long_following_visible_3"),
    ("φ_long_following,very_near,LV_decel", "long_following_very_near_veh_decel_3.0"),
    ("φ_cut_in_from_right",                 "cut_in_from_right"),
    ("φ_cut_in_from_right,snowy",           "cut_in_from_right_snowy"),
    ("φ_cut_in_from_right,LV_decel",        "cut_in_from_right_veh_decel"),
]


def main() -> None:
    ext_path = SUMMARIES_DIR / "extractions.json"
    if not ext_path.exists():
        print(f"extractions.json not found at {ext_path}")
        sys.exit(1)

    with open(ext_path) as f:
        all_extractions = json.load(f)

    level_data = all_extractions.get("SG_tracks_kinematics_map", {})
    if not level_data:
        print("No SG_tracks_kinematics_map data found in extractions.json")
        sys.exit(1)

    instances:     dict[str, int]         = defaultdict(int)
    durations:     dict[str, list[float]] = defaultdict(list)
    unique_scenes: dict[str, set[str]]    = defaultdict(set)

    for uuid8, variants_data in level_data.items():
        for _, variant in SCENARIOS:
            for ext in variants_data.get(variant, []):
                dur = ext["end_s"] - ext["start_s"]
                instances[variant] += 1
                durations[variant].append(dur)
                unique_scenes[variant].add(uuid8)

    col_sc = 42
    header = (f"{'Scenario':<{col_sc}}  {'Unique # Instances':>18}  "
              f"{'Median Duration (s)':>19}  {'Unique Scenes':>13}  "
              f"{'% of Full Dataset':>17}")
    sep = "-" * len(header)

    print()
    print("Table 5: Scenario coverage of the Argoverse 2 dataset (track + HD map SG)")
    print(sep)
    print(header)
    print(sep)

    for label, variant in SCENARIOS:
        n_inst  = instances[variant]
        n_scene = len(unique_scenes[variant])
        med_dur = statistics.median(durations[variant]) if durations[variant] else 0.0
        pct     = n_scene / TOTAL_SCENES * 100
        print(f"{label:<{col_sc}}  {n_inst:>18}  {med_dur:>19.2f}  {n_scene:>13}  {pct:>17.2f}")

    print(sep)
    print()


if __name__ == "__main__":
    main()
