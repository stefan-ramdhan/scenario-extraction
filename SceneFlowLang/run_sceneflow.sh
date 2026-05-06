#!/bin/bash
# =============================================================================
# run_sceneflow.sh
# ----------------
# Runs SceneFlowLang property checking on a SG dataset.
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
# A mandatory --half flag selects which data split to use: first or second.
#
# Usage
# -----
#   bash run_sceneflow.sh --data tracks --half first
#   bash run_sceneflow.sh --data tracks+state --half second
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
            echo "Usage: bash run_sceneflow.sh --data tracks|tracks+state|tracks+state+map --half first|second"
            exit 1
            ;;
    esac
done

if [[ -z "$DATA" ]]; then
    echo "ERROR: --data is required."
    echo "Usage: bash run_sceneflow.sh --data tracks|tracks+state|tracks+state+map --half first|second"
    exit 1
fi

if [[ "$DATA" != "tracks" && "$DATA" != "tracks+state" && "$DATA" != "tracks+state+map" ]]; then
    echo "ERROR: Invalid --data '$DATA'. Must be 'tracks', 'tracks+state', or 'tracks+state+map'."
    exit 1
fi

if [[ -z "$HALF" ]]; then
    echo "ERROR: --half is required."
    echo "Usage: bash run_sceneflow.sh --data tracks|tracks+state|tracks+state+map --half first|second"
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
else
    INPUT_DIR="${DATA_DIR}/SG_tracks_kinematics_map/"
    OUTPUT_DIR="my_results/SG_tracks_kinematics_map/"
    PROPS_DIR="./my_props/tracks_state_map"
fi

# ---------------------------------------------------------------------------
# Run
# ---------------------------------------------------------------------------

# rm -rf "$OUTPUT_DIR"

python3 run_check.py --props-dir "$PROPS_DIR" -f "$INPUT_DIR" -s "$OUTPUT_DIR" --ego_only

python3 my_results/summarize_results.py --data "$DATA"
