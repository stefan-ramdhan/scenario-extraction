#!/usr/bin/env python3
"""
Summarizes scenario extraction results across UUID drives.

Directory structure:
  scenarios/
    <uuid>/
      <scenario_name>/
        satisfactions/
          <instance>.json   <- each JSON = one instance

For each UUID drive, reports how many instances of each scenario existed,
the start/end timestamps, elapsed time since the tracking sequence start,
and the track of interest.
"""

import json
from pathlib import Path
from collections import defaultdict
from typing import List, Dict, Optional

_MY_RESULTS_DIR = Path(__file__).parent
_OUTPUT_BASE    = Path(
    "sg_processing/argo2/output/binary"
)

# Overridden in main() based on --data flag
SCENARIOS_DIR = _MY_RESULTS_DIR / "SG_tracks"
TRACKING_BASE = _OUTPUT_BASE / "SG_tracks"

NS_PER_SEC = 1_000_000_000


# ── Helpers ──────────────────────────────────────────────────────────────────

def parse_duration(start_frame: str, end_frame: str) -> float:
    """Return duration in seconds from nanosecond timestamp strings."""
    try:
        return (int(end_frame) - int(start_frame)) / NS_PER_SEC
    except (ValueError, TypeError):
        return float("nan")


def get_sequence_start_ns(uuid: str) -> Optional[int]:
    """
    Return the lowest timestamp (nanoseconds, int) found among .pkl filenames
    in <TRACKING_BASE>/<uuid>_tracking/.  Returns None if the directory does
    not exist or contains no .pkl files.
    """
    tracking_dir = TRACKING_BASE / f"{uuid}_tracking"
    if not tracking_dir.exists():
        return None

    timestamps = []
    for pkl in tracking_dir.glob("*.pkl"):
        try:
            timestamps.append(int(pkl.stem))
        except ValueError:
            pass

    return min(timestamps) if timestamps else None


def time_since_sequence_start(frame_ts: str, seq_start_ns: Optional[int]) -> str:
    """
    Return elapsed seconds from seq_start_ns to frame_ts as a formatted string,
    or 'N/A' if either value is unavailable.
    """
    if seq_start_ns is None:
        return "N/A"
    try:
        elapsed = (int(frame_ts) - seq_start_ns) / NS_PER_SEC
        return f"{elapsed:.3f}"
    except (ValueError, TypeError):
        return "N/A"


# ── Data collection ──────────────────────────────────────────────────────────

def collect_instances(scenario_dir: Path) -> List[Dict]:
    """Return list of instance dicts from the satisfactions/ subdirectory."""
    satisfactions_dir = scenario_dir / "satisfactions"
    if not satisfactions_dir.exists():
        return []

    instances = []
    for json_file in sorted(satisfactions_dir.glob("*.json")):
        try:
            with open(json_file) as f:
                data = json.load(f)
            entity_mapping = data.get("entity_mapping", {})
            # Collect all tracks of interest (usually just one)
            tracks = ", ".join(f"{k}={v}" for k, v in entity_mapping.items()) or "N/A"
            instances.append({
                "file": json_file.name,
                "start_frame": data.get("start_frame", "N/A"),
                "end_frame": data.get("end_frame", "N/A"),
                "duration_s": parse_duration(
                    data.get("start_frame"), data.get("end_frame")
                ),
                "tracks": tracks,
            })
        except (json.JSONDecodeError, OSError) as e:
            print(f"  [WARNING] Could not read {json_file}: {e}")
    return instances


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    import argparse
    parser = argparse.ArgumentParser(description="Summarize scenario extraction results.")
    parser.add_argument(
        "--data", choices=["tracks", "tracks+state", "tracks+state+map"],
        help=(
            "Which SG dataset results to summarize. "
            "'tracks': my_results/SG_tracks/. "
            "'tracks+state': my_results/SG_tracks_kinematics/. "
            "'tracks+state+map': my_results/SG_tracks_kinematics_map/."
        ),
    )
    parser.add_argument(
        "--rss", action="store_true",
        help="Summarize RSS d_min violation results from my_results/rss/.",
    )
    args = parser.parse_args()

    if not args.rss and not args.data:
        parser.error("one of --data or --rss is required")

    global SCENARIOS_DIR, TRACKING_BASE
    if args.rss:
        SCENARIOS_DIR = _MY_RESULTS_DIR / "rss"
        TRACKING_BASE = _OUTPUT_BASE / "SG_tracks_kinematics_map"
    elif args.data == "tracks+state+map":
        SCENARIOS_DIR = _MY_RESULTS_DIR / "SG_tracks_kinematics_map"
        TRACKING_BASE = _OUTPUT_BASE / "SG_tracks_kinematics_map"
    elif args.data == "tracks+state":
        SCENARIOS_DIR = _MY_RESULTS_DIR / "SG_tracks_kinematics"
        TRACKING_BASE = _OUTPUT_BASE / "SG_tracks_kinematics"
    else:
        SCENARIOS_DIR = _MY_RESULTS_DIR / "SG_tracks"
        TRACKING_BASE = _OUTPUT_BASE / "SG_tracks"

    if not SCENARIOS_DIR.exists():
        print(f"ERROR: Directory not found: {SCENARIOS_DIR}")
        return

    uuid_dirs = sorted(
        p for p in SCENARIOS_DIR.iterdir()
        if p.is_dir() and not p.name.startswith(".")
    )
    if not uuid_dirs:
        print("No UUID directories found.")
        return

    # Pre-compute sequence start timestamp for each UUID
    seq_starts: Dict[str, Optional[int]] = {}
    for uuid_dir in uuid_dirs:
        uuid = uuid_dir.name
        seq_start = get_sequence_start_ns(uuid)
        seq_starts[uuid] = seq_start
        if seq_start is None:
            print(f"  [WARNING] No tracking directory / pkl files found for UUID {uuid}")

    # ── Per-UUID data collection ──────────────────────────────────────────────
    # uuid -> scenario_name -> list of instances
    uuid_data: Dict[str, Dict[str, List[Dict]]] = {}
    all_scenario_names = set()

    for uuid_dir in uuid_dirs:
        uuid = uuid_dir.name
        uuid_data[uuid] = {}
        scenario_dirs = sorted(p for p in uuid_dir.iterdir() if p.is_dir())
        for scenario_dir in scenario_dirs:
            scenario_name = scenario_dir.name
            all_scenario_names.add(scenario_name)
            instances = collect_instances(scenario_dir)
            if instances:
                uuid_data[uuid][scenario_name] = instances

    # ── Summary table: UUID x scenario instance counts ────────────────────────
    sorted_scenarios = sorted(all_scenario_names)
    col_w = max(len(s) for s in sorted_scenarios) + 2 if sorted_scenarios else 20
    uuid_col = 12

    W = uuid_col + 2 + len(sorted_scenarios) * (col_w + 1) + 10
    W = max(W, 80)

    print("=" * W)
    print("SCENARIO INSTANCE COUNT SUMMARY  (rows = UUID drives, cols = scenarios)")
    print("=" * W)

    header = f"  {'UUID':<{uuid_col}}"
    for s in sorted_scenarios:
        header += f"  {s:>{col_w}}"
    header += f"  {'TOTAL':>7}"
    print(header)
    print(f"  {'-'*uuid_col}" + "".join(f"  {'-'*col_w}" for _ in sorted_scenarios) + f"  {'-'*7}")

    total_per_scenario: Dict[str, int] = defaultdict(int)
    grand_total = 0

    for uuid in sorted(uuid_data.keys()):
        row = f"  {uuid:<{uuid_col}}"
        row_total = 0
        for s in sorted_scenarios:
            count = len(uuid_data[uuid].get(s, []))
            total_per_scenario[s] += count
            row_total += count
            row += f"  {count:>{col_w}}"
        grand_total += row_total
        row += f"  {row_total:>7}"
        print(row)

    print(f"  {'-'*uuid_col}" + "".join(f"  {'-'*col_w}" for _ in sorted_scenarios) + f"  {'-'*7}")
    totals_row = f"  {'TOTAL':<{uuid_col}}"
    for s in sorted_scenarios:
        totals_row += f"  {total_per_scenario[s]:>{col_w}}"
    totals_row += f"  {grand_total:>7}"
    print(totals_row)
    print("=" * W)
    print()

    # ── Detailed breakdown: per UUID, per scenario ────────────────────────────
    DW = 115
    print("=" * DW)
    print("DETAILED INSTANCE BREAKDOWN  (by UUID drive)")
    print("=" * DW)

    for uuid in sorted(uuid_data.keys()):
        scenarios = uuid_data[uuid]
        if not scenarios:
            continue

        seq_start = seq_starts.get(uuid)
        total_instances = sum(len(v) for v in scenarios.values())

        print(f"\n{'═' * DW}")
        print(f"  UUID : {uuid}   |   Total instances: {total_instances}")
        print(f"{'═' * DW}")

        for scenario_name in sorted(scenarios.keys()):
            instances = scenarios[scenario_name]
            print(f"\n  Scenario : {scenario_name}  ({len(instances)} instance{'s' if len(instances) != 1 else ''})")
            print(
                f"  {'#':<4} {'START FRAME':<22} {'END FRAME':<22} "
                f"{'START @ (s)':<13} {'DURATION (s)':<14} {'TRACK(S) OF INTEREST'}"
            )
            print(
                f"  {'-'*4} {'-'*22} {'-'*22} "
                f"{'-'*13} {'-'*14} {'-'*20}"
            )
            for idx, inst in enumerate(instances, start=1):
                dur = (
                    f"{inst['duration_s']:.3f}"
                    if inst["duration_s"] == inst["duration_s"]
                    else "N/A"
                )
                start_at = time_since_sequence_start(inst["start_frame"], seq_start)
                print(
                    f"  {idx:<4} {inst['start_frame']:<22} {inst['end_frame']:<22} "
                    f"{start_at:<13} {dur:<14} {inst['tracks']}"
                )

    print(f"\n{'=' * DW}")
    print(f"Total UUID drives with any scenario: "
          f"{sum(1 for v in uuid_data.values() if v)}")
    print(f"Total scenario types found: {len(all_scenario_names)}")
    print(f"Total instances across all UUIDs and scenarios: {grand_total}")
    print("=" * DW)


if __name__ == "__main__":
    main()
