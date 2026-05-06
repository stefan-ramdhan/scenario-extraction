#!/usr/bin/env python3
"""
fragmentation_metrics.py
========================
Compute SG track fragmentation metrics relative to AV2 ground-truth tracks.

For each log the script:
  1. Loads *_matched.json (SG↔AV2 association output) and *_av2.json.
  2. Applies the same forward-camera FOV filter used in
     av2_rs2v_imagespace_association.py to decide which AV2 tracks/frames
     are eligible association candidates.
  3. For each eligible AV2 track g, collects the SG track IDs that are
     associated with it across eligible frames (only fragments with
     C(s,g) >= --min-overlap are counted to suppress one-frame noise).
  4. Reports per-track metrics and aggregate summaries.

Definitions
-----------
  T_g          – number of frames where AV2 track g is inside the forward
                 camera FOV (eligible frames).
  C(s, g)      – number of eligible frames where SG track s is associated
                 with AV2 track g.
  S(g)         – set of SG tracks with C(s,g) >= min_overlap.
  fragments_per_track         = mean |S(g)|  (tracks with |S(g)| >= 1)
  fragmented_track_rate       = fraction of eligible AV2 tracks with |S(g)| > 1
  dominant_fragment_coverage  = mean  max_s C(s,g) / T_g

Output
------
  --output-csv      per-track CSV for all eligible AV2 tracks
  --output-csv-gt   same, restricted to AV2 tracks in GT scenario APs

CSV columns:
  log_id, av2_track_id, num_sg_fragments, is_fragmented,
  dominant_fragment_coverage, eligible_av2_track_duration, dominant_sg_track_id

Usage
-----
  python fragmentation_metrics.py
  python fragmentation_metrics.py --obj-dir /path/to/obj_positions \
                                  --aps-dir /path/to/aps \
                                  --output-csv out.csv \
                                  --min-overlap 2
"""

import json
import math
import csv
import os
import sys
import argparse
from collections import defaultdict
from typing import Dict, List, Optional, Set, Tuple


# ---------------------------------------------------------------------------
# FOV constants — must match av2_rs2v_imagespace_association.py
# ---------------------------------------------------------------------------

CAMERA_FOV_DEG: float = 54.0   # total horizontal FoV (±27° half-angle)

HALF_LENGTH_M: Dict[str, float] = {
    "car":   2.25,
    "truck": 3.50,
    "bus":   6.00,
    "ped":   0.45,
}

FOV_EDGE_FRAC: float = 0.75   # fraction of half-length used for edge check


# ---------------------------------------------------------------------------
# FOV helpers (copied verbatim from av2_rs2v_imagespace_association.py)
# ---------------------------------------------------------------------------

def _fov_edge_azimuth(
    dx: float,
    dy: float,
    ego_yaw: float,
    obj_yaw: Optional[float],
    obj_type: str,
    frac: float = FOV_EDGE_FRAC,
) -> float:
    """
    Return the minimum-magnitude azimuth among the object centre and its two
    length-axis edge points.  Allows a partially-visible vehicle whose centre
    is just outside the FoV boundary to still be treated as eligible.
    """
    c_fwd = math.cos(ego_yaw) * dx + math.sin(ego_yaw) * dy
    c_lat = math.sin(ego_yaw) * dx - math.cos(ego_yaw) * dy
    az_center = math.atan2(c_lat, c_fwd)

    half_len = HALF_LENGTH_M.get(obj_type, 0.0)
    if half_len == 0.0 or obj_yaw is None:
        return az_center

    offset = frac * half_len
    best_az = az_center
    for sign in (1.0, -1.0):
        edx = dx + sign * offset * math.cos(obj_yaw)
        edy = dy + sign * offset * math.sin(obj_yaw)
        e_fwd = math.cos(ego_yaw) * edx + math.sin(ego_yaw) * edy
        if e_fwd <= 0:
            continue
        e_lat = math.sin(ego_yaw) * edx - math.cos(ego_yaw) * edy
        az_edge = math.atan2(e_lat, e_fwd)
        if abs(az_edge) < abs(best_az):
            best_az = az_edge
    return best_az


def fov_eligible_uuids(av2_frame: Dict, half_fov_rad: float) -> Set[str]:
    """
    Return AV2 track UUIDs that pass the forward-camera FOV filter for this
    frame.  Mirrors av2_objects() in av2_rs2v_imagespace_association.py:
      • forward > 0        (object in front of ego)
      • |edge_azimuth| <= half_fov_rad
    """
    ego = av2_frame["ego_vehicle"]
    ego_x   = ego["position_x"]
    ego_y   = ego["position_y"]
    ego_yaw = ego["yaw_rad"]
    cos_yaw = math.cos(ego_yaw)
    sin_yaw = math.sin(ego_yaw)

    eligible: Set[str] = set()
    for k, v in av2_frame.items():
        if k in ("timestamp", "timestamp_s", "ego_vehicle") or not isinstance(v, dict):
            continue
        dx      = v["position_x"] - ego_x
        dy      = v["position_y"] - ego_y
        forward = cos_yaw * dx + sin_yaw * dy
        if forward <= 0:
            continue
        edge_az = _fov_edge_azimuth(
            dx, dy, ego_yaw,
            v.get("yaw"),
            v.get("object_type", ""),
        )
        if abs(edge_az) <= half_fov_rad:
            eligible.add(k)
    return eligible


# ---------------------------------------------------------------------------
# I/O helpers
# ---------------------------------------------------------------------------

def load_json(path: str):
    with open(path) as fh:
        return json.load(fh)


def gt_scenario_uuids(aps_path: str) -> Set[str]:
    """
    Return all AV2 track UUIDs that appear as actors in any GT scenario AP.
    """
    data = load_json(aps_path)
    uuids: Set[str] = set()
    for instances in data.get("aps", {}).values():
        for inst in instances:
            tid = inst.get("track_id")
            if tid:
                uuids.add(str(tid))
    return uuids


# ---------------------------------------------------------------------------
# Core computation
# ---------------------------------------------------------------------------

def compute_log_metrics(
    log_id: str,
    matched_frames: List[Dict],
    av2_frames: List[Dict],
    half_fov_rad: float,
    min_overlap: int,
) -> List[Dict]:
    """
    Compute per-AV2-track fragmentation metrics for one log.

    Returns one row dict per eligible AV2 track that has at least one
    non-trivial SG fragment (C(s,g) >= min_overlap).
    """
    # T_g: eligible frame count per AV2 UUID
    track_eligible_frames: Dict[str, int] = defaultdict(int)

    # C(s, g): association frame count per (av2_uuid, sg_track_id)
    pair_frames: Dict[str, Dict[int, int]] = defaultdict(lambda: defaultdict(int))

    for matched_frame, av2_frame in zip(matched_frames, av2_frames):
        eligible = fov_eligible_uuids(av2_frame, half_fov_rad)

        for uuid in eligible:
            track_eligible_frames[uuid] += 1

        # Record associations only for FOV-eligible AV2 tracks
        for key, val in matched_frame.items():
            if key in ("timestamp", "av2_timestamp", "ego") or not isinstance(val, dict):
                continue
            av2_uuid = val.get("av2_track_uuid")
            sg_tid   = val.get("track_id")
            if av2_uuid is None or sg_tid is None:
                continue
            if av2_uuid not in eligible:
                continue
            pair_frames[av2_uuid][sg_tid] += 1

    rows: List[Dict] = []
    for uuid, T_g in track_eligible_frames.items():
        sg_counts = pair_frames.get(uuid, {})

        # Keep only fragments with nontrivial overlap
        filtered = {s: c for s, c in sg_counts.items() if c >= min_overlap}
        if not filtered:
            continue   # no SG association → not included in fragmentation stats

        num_frags      = len(filtered)
        dominant_sg    = max(filtered, key=lambda s: filtered[s])
        dominant_c     = filtered[dominant_sg]
        dominant_cov   = dominant_c / T_g if T_g > 0 else 0.0

        rows.append({
            "log_id":                      log_id,
            "av2_track_id":                uuid,
            "num_sg_fragments":            num_frags,
            "is_fragmented":               int(num_frags > 1),
            "dominant_fragment_coverage":  round(dominant_cov, 6),
            "eligible_av2_track_duration": T_g,
            "dominant_sg_track_id":        dominant_sg,
        })
    return rows


def aggregate_summary(rows: List[Dict], label: str) -> None:
    print(f"\n── {label} ──")
    if not rows:
        print("  (no data)")
        return
    n = len(rows)
    mean_frags   = sum(r["num_sg_fragments"] for r in rows) / n
    frag_rate    = sum(r["is_fragmented"]    for r in rows) / n
    mean_dom_cov = sum(r["dominant_fragment_coverage"] for r in rows) / n
    print(f"  num_tracks                      : {n}")
    print(f"  mean_fragments_per_track        : {mean_frags:.4f}")
    print(f"  fragmented_track_rate           : {frag_rate:.4f}")
    print(f"  mean_dominant_fragment_coverage : {mean_dom_cov:.4f}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_OBJ_DIR = os.path.normpath(
    os.path.join(_SCRIPT_DIR, "..", "sg_processing", "code",
                 "tracking_estimation", "obj_positions")
)
_DEFAULT_APS_DIR = os.path.join(_SCRIPT_DIR, "aps")

CSV_FIELDS = [
    "log_id", "av2_track_id", "num_sg_fragments", "is_fragmented",
    "dominant_fragment_coverage", "eligible_av2_track_duration",
    "dominant_sg_track_id",
]


def main() -> None:
    parser = argparse.ArgumentParser(
        description="SG track fragmentation metrics relative to AV2 GT tracks."
    )
    parser.add_argument(
        "--obj-dir", default=_DEFAULT_OBJ_DIR,
        help="Directory containing *_matched.json and *_av2.json files.",
    )
    parser.add_argument(
        "--aps-dir", default=_DEFAULT_APS_DIR,
        help="Directory containing *_aps.json ground-truth scenario files.",
    )
    parser.add_argument(
        "--output-csv", default="fragmentation_metrics.csv",
        help="Output CSV for all eligible AV2 tracks.",
    )
    parser.add_argument(
        "--output-csv-gt", default="fragmentation_metrics_gt_scenarios.csv",
        help="Output CSV restricted to GT scenario AV2 tracks.",
    )
    parser.add_argument(
        "--min-overlap", type=int, default=2,
        help="Minimum C(s,g) frames for a fragment to be counted (default: 2).",
    )
    parser.add_argument(
        "--camera-fov", type=float, default=CAMERA_FOV_DEG,
        help="Total horizontal camera FoV in degrees (default: 54.0).",
    )
    args = parser.parse_args()

    half_fov_rad = math.radians(args.camera_fov / 2.0)

    log_ids = sorted(
        fn[: -len("_matched.json")]
        for fn in os.listdir(args.obj_dir)
        if fn.endswith("_matched.json")
    )
    if not log_ids:
        sys.exit(f"No *_matched.json files found in {args.obj_dir}")

    all_rows: List[Dict] = []
    gt_rows:  List[Dict] = []

    for log_id in log_ids:
        matched_path = os.path.join(args.obj_dir, f"{log_id}_matched.json")
        av2_path     = os.path.join(args.obj_dir, f"{log_id}_av2.json")
        aps_path     = os.path.join(args.aps_dir,  f"{log_id}_aps.json")

        if not os.path.exists(av2_path):
            print(f"[skip] {log_id}: missing _av2.json", file=sys.stderr)
            continue

        matched_frames: List[Dict] = load_json(matched_path)
        av2_frames:     List[Dict] = load_json(av2_path)

        # Align frames by timestamp when lengths differ (should be rare)
        if len(matched_frames) != len(av2_frames):
            print(
                f"[warn] {log_id}: frame count mismatch "
                f"({len(matched_frames)} matched vs {len(av2_frames)} av2) — aligning by timestamp",
                file=sys.stderr,
            )
            av2_by_ts = {f["timestamp_s"]: f for f in av2_frames}
            pairs: List[Tuple[Dict, Dict]] = []
            for mf in matched_frames:
                af = av2_by_ts.get(mf["av2_timestamp"])
                if af is not None:
                    pairs.append((mf, af))
            matched_frames = [p[0] for p in pairs]
            av2_frames     = [p[1] for p in pairs]

        rows = compute_log_metrics(
            log_id, matched_frames, av2_frames, half_fov_rad, args.min_overlap
        )
        all_rows.extend(rows)

        if os.path.exists(aps_path):
            gt_uuids = gt_scenario_uuids(aps_path)
            gt_rows.extend(r for r in rows if r["av2_track_id"] in gt_uuids)

    # Write CSVs
    for out_path, rows, label in [
        (args.output_csv,    all_rows, "all tracks"),
        (args.output_csv_gt, gt_rows,  "GT scenario tracks"),
    ]:
        with open(out_path, "w", newline="") as fh:
            writer = csv.DictWriter(fh, fieldnames=CSV_FIELDS)
            writer.writeheader()
            writer.writerows(rows)
        print(f"Wrote {len(rows):,} rows ({label}) → {out_path}")

    # Print aggregate summaries
    # aggregate_summary(all_rows,  "ALL eligible AV2 tracks")
    aggregate_summary(gt_rows,   "GT scenario tracks only")


if __name__ == "__main__":
    main()
