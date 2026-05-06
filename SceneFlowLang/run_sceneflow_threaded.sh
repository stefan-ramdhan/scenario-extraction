#!/bin/bash
# =============================================================================
# run_sceneflow_threaded.sh
# -------------------------
# Runs SceneFlowLang property checking on a SG dataset (threaded).
#
# A mandatory --data flag selects which dataset to process:
#
#   --data tracks
#       Input  : first_half_data/SG_tracks/  (or second_half_data/...)
#       Output : my_results/SG_tracks/
#
#   --data tracks+state
#       Input  : first_half_data/SG_tracks_kinematics/
#       Output : my_results/SG_tracks_kinematics/
#
#   --data tracks+state+map
#       Input  : first_half_data/SG_tracks_kinematics_map/
#       Output : my_results/SG_tracks_kinematics_map/
#
#   --data rss
#       Input  : first_half_data/SG_tracks_kinematics_map/
#       Output : my_results/rss/
#       Props  : symbolic_properties_rss.py
#
# A mandatory --half flag selects which data split to use: first or second.
#
# Usage
# -----
#   bash run_sceneflow_threaded.sh --data tracks --half first
#   bash run_sceneflow_threaded.sh --data tracks+state --half second
#   bash run_sceneflow_threaded.sh --data rss --half first
# =============================================================================

set -e

# ---------------------------------------------------------------------------
# Parse arguments
# ---------------------------------------------------------------------------

DATA=""
HALF=""

while [[ $# -gt 0 ]]; do
    case "$1" in
        --data)
            DATA="$2"
            shift 2
            ;;
        --half)
            HALF="$2"
            shift 2
            ;;
        *)
            echo "ERROR: Unknown argument: $1"
            echo "Usage: bash run_sceneflow_threaded.sh --data tracks|tracks+state|tracks+state+map|rss --half first|second"
            exit 1
            ;;
    esac
done

if [[ -z "$DATA" ]]; then
    echo "ERROR: --data is required."
    echo "Usage: bash run_sceneflow_threaded.sh --data tracks|tracks+state|tracks+state+map|rss --half first|second"
    exit 1
fi

if [[ "$DATA" != "tracks" && "$DATA" != "tracks+state" && "$DATA" != "tracks+state+map" && "$DATA" != "rss" ]]; then
    echo "ERROR: Invalid --data '$DATA'. Must be 'tracks', 'tracks+state', 'tracks+state+map', or 'rss'."
    exit 1
fi

if [[ -z "$HALF" ]]; then
    echo "ERROR: --half is required."
    echo "Usage: bash run_sceneflow_threaded.sh --data tracks|tracks+state|tracks+state+map|rss --half first|second"
    exit 1
fi

if [[ "$HALF" != "first" && "$HALF" != "second" ]]; then
    echo "ERROR: Invalid --half '$HALF'. Must be 'first' or 'second'."
    exit 1
fi

DATA_DIR="./${HALF}_half_data"

# ---------------------------------------------------------------------------
# Select mode-specific directories and props
# ---------------------------------------------------------------------------

if [[ "$DATA" == "tracks" ]]; then
    INPUT_DIR="${DATA_DIR}/SG_tracks/"
    OUTPUT_DIR="my_results/SG_tracks/"
    PROPS_DIR="./my_props/tracks"
elif [[ "$DATA" == "tracks+state" ]]; then
    INPUT_DIR="${DATA_DIR}/SG_tracks_kinematics/"
    OUTPUT_DIR="my_results/SG_tracks_kinematics/"
    PROPS_DIR="./my_props/tracks_state"
elif [[ "$DATA" == "rss" ]]; then
    INPUT_DIR="${DATA_DIR}/SG_tracks_kinematics_map/"
    OUTPUT_DIR="my_results/rss/"
    PROPS_DIR="./my_props/tracks_state_map_rss"
else
    INPUT_DIR="${DATA_DIR}/SG_tracks_kinematics_map/"
    OUTPUT_DIR="my_results/SG_tracks_kinematics_map/"
    PROPS_DIR="./my_props/tracks_state_map"
fi

# ---------------------------------------------------------------------------
# Run
# ---------------------------------------------------------------------------

# rm -rf "$OUTPUT_DIR"

N_THREADS=$(( $(nproc) - 6 ))
if [[ $N_THREADS -lt 1 ]]; then N_THREADS=1; fi

find "$INPUT_DIR" -mindepth 1 -maxdepth 1 -type d \
  | xargs -P "$N_THREADS" -I{} \
      python3 run_check.py --props-dir "$PROPS_DIR" -f {} -s "$OUTPUT_DIR" --ego_only --no_iter

if [[ "$DATA" == "rss" ]]; then
    python3 my_results/summarize_results.py --rss
else
    python3 my_results/summarize_results.py --data "$DATA"
fi
