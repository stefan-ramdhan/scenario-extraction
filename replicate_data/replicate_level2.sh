#!/bin/bash
# =============================================================================
# replicate_level2.sh
# --------------------
# Replication Level 2: re-run SceneFlowLang on the provided scene graphs,
# regenerate extractions.json, then print all paper tables.
#
# Prerequisites
# -------------
#   1. Conda — tcp_env activated automatically if it exists; if not, create it first:
#        conda env create -f SceneFlowLang/tcp_environment.yml
#   2. Mona model checker — installed automatically if not present
#   3. Scene graph data in SceneFlowLang/ (script will offer to download if absent):
#        SceneFlowLang/first_half_data/   (SG_tracks/, SG_tracks_kinematics/, SG_tracks_kinematics_map/)
#        SceneFlowLang/second_half_data/  (same structure)
#
# Usage
# -----
#   cd /path/to/scenario-extraction
#   bash replicate_data/replicate_level2.sh
#
# Expected runtime: 1–3 hours depending on available CPU cores.
# =============================================================================

set -e

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
SCENEFLOW_DIR="$REPO_ROOT/SceneFlowLang"

# ---------------------------------------------------------------------------
# Conda environment — activate tcp_env if not already active
# ---------------------------------------------------------------------------

if [[ "$CONDA_DEFAULT_ENV" != "tcp_env" ]]; then
    if command -v conda &>/dev/null; then
        CONDA_BASE="$(conda info --base)"
        # shellcheck source=/dev/null
        source "$CONDA_BASE/etc/profile.d/conda.sh"
        if conda env list | grep -q "^tcp_env[[:space:]]"; then
            echo "Activating conda environment tcp_env ..."
            conda activate tcp_env
        else
            echo "ERROR: conda environment 'tcp_env' not found."
            echo "       Create it first, then re-run this script:"
            echo "         conda env create -f SceneFlowLang/tcp_environment.yml"
            echo "         conda activate tcp_env"
            exit 1
        fi
    else
        echo "ERROR: conda not found and tcp_env is not active."
        echo "       Install conda, create the environment, and activate it before running:"
        echo "         conda env create -f SceneFlowLang/tcp_environment.yml"
        echo "         conda activate tcp_env"
        exit 1
    fi
fi

# ---------------------------------------------------------------------------
# Helper: download a file using curl or wget
# ---------------------------------------------------------------------------

download() {
    local url="$1"
    local dest="$2"
    if command -v curl &>/dev/null; then
        curl -L --progress-bar -o "$dest" "$url"
    elif command -v wget &>/dev/null; then
        wget -O "$dest" "$url"
    else
        echo "ERROR: Neither curl nor wget found. Install one and re-run, or download manually."
        exit 1
    fi
}

# ---------------------------------------------------------------------------
# Check whether my_results/ already has all scenario data needed for the tables.
#
# For each of the three SG levels we sample three UUID dirs (first, middle,
# last) and verify that the scenario variant subdirs required by
# build_extractions.py are present in every sampled dir.
#
# Variants checked:
#   SG_tracks / SG_tracks_kinematics:
#     long_following_near_3, long_following_visible_3, cut_in_from_right
#   SG_tracks_kinematics_map (adds map-only variants):
#     + cut_in_from_right_snowy, cut_in_from_right_veh_decel,
#       long_following_very_near_veh_decel_3.0
# ---------------------------------------------------------------------------

results_complete() {
    local TRACKS_VARIANTS=(
        "long_following_near_3"
        "long_following_visible_3"
        "cut_in_from_right"
    )
    local MAP_VARIANTS=(
        "long_following_near_3"
        "long_following_visible_3"
        "cut_in_from_right"
        "cut_in_from_right_snowy"
        "cut_in_from_right_veh_decel"
        "long_following_very_near_veh_decel_3.0"
    )

    local levels=("SG_tracks" "SG_tracks_kinematics" "SG_tracks_kinematics_map")
    local level uuid_dirs total sample_uuids variants v u dir

    for level in "${levels[@]}"; do
        dir="$SCENEFLOW_DIR/my_results/$level"
        if [[ ! -d "$dir" ]]; then
            echo "  my_results/$level/ not found — will run SceneFlowLang."
            return 1
        fi

        mapfile -t uuid_dirs < <(find "$dir" -maxdepth 1 -mindepth 1 -type d -printf '%f\n' 2>/dev/null | sort)
        total=${#uuid_dirs[@]}
        if [[ $total -eq 0 ]]; then
            echo "  my_results/$level/ is empty — will run SceneFlowLang."
            return 1
        fi

        # Sample first, middle, and last UUID dirs
        sample_uuids=("${uuid_dirs[0]}")
        [[ $total -gt 2 ]] && sample_uuids+=("${uuid_dirs[$((total / 2))]}")
        [[ $total -gt 1 ]] && sample_uuids+=("${uuid_dirs[$((total - 1))]}")

        if [[ "$level" == "SG_tracks_kinematics_map" ]]; then
            variants=("${MAP_VARIANTS[@]}")
        else
            variants=("${TRACKS_VARIANTS[@]}")
        fi

        for u in "${sample_uuids[@]}"; do
            for v in "${variants[@]}"; do
                if [[ ! -d "$dir/$u/$v" ]]; then
                    echo "  my_results/$level/$u/$v/ not found — will run SceneFlowLang."
                    return 1
                fi
            done
        done
    done

    return 0
}

# ---------------------------------------------------------------------------
# Decide whether to run the full pipeline or go straight to table generation
# ---------------------------------------------------------------------------

echo "Checking my_results/ for existing scenario extraction data ..."

if results_complete; then
    echo "  All required scenario data found — skipping SceneFlowLang run."
    echo
else
    echo

    # ---------------------------------------------------------------------------
    # Scene graph data — offer to download from Zenodo if absent
    # ---------------------------------------------------------------------------

    DATA_MISSING=false
    for HALF_DIR in first_half_data second_half_data; do
        if [[ ! -d "$SCENEFLOW_DIR/$HALF_DIR" ]]; then
            DATA_MISSING=true
            break
        fi
    done

    if [[ "$DATA_MISSING" == "true" ]]; then
        echo "Scene graph data not found in SceneFlowLang/."
        echo "  first_half_data.zip  — 663 MB"
        echo "  second_half_data.zip — 660 MB"
        echo "  Source: https://zenodo.org/records/20057765"
        echo
        read -r -p "Download now? [y/N] " ANSWER
        if [[ "$ANSWER" =~ ^[Yy]$ ]]; then
            echo
            echo "Downloading first_half_data.zip ..."
            download \
                "https://zenodo.org/records/20057765/files/first_half_data.zip?download=1" \
                "$SCENEFLOW_DIR/first_half_data.zip"
            echo "Extracting first_half_data.zip ..."
            unzip -q "$SCENEFLOW_DIR/first_half_data.zip" -d "$SCENEFLOW_DIR"
            rm "$SCENEFLOW_DIR/first_half_data.zip"

            echo
            echo "Downloading second_half_data.zip ..."
            download \
                "https://zenodo.org/records/20057765/files/second_half_data.zip?download=1" \
                "$SCENEFLOW_DIR/second_half_data.zip"
            echo "Extracting second_half_data.zip ..."
            unzip -q "$SCENEFLOW_DIR/second_half_data.zip" -d "$SCENEFLOW_DIR"
            rm "$SCENEFLOW_DIR/second_half_data.zip"
            echo
        else
            echo
            echo "ERROR: Scene graph data is required."
            echo "       Place first_half_data/ and second_half_data/ inside SceneFlowLang/ and re-run."
            exit 1
        fi
    fi

    # ---------------------------------------------------------------------------
    # Remaining prerequisite checks
    # ---------------------------------------------------------------------------

    echo "Checking prerequisites ..."

    if ! command -v mona &>/dev/null; then
        echo "mona not found — installing via SceneFlowLang/install_mona.sh ..."
        (cd "$SCENEFLOW_DIR" && bash install_mona.sh)
    fi

    echo "All prerequisites satisfied."
    echo

    # ---------------------------------------------------------------------------
    # Run SceneFlowLang on all three SG levels x both data halves
    # ---------------------------------------------------------------------------

    DATA_LEVELS="tracks tracks+state tracks+state+map"

    for DATA in $DATA_LEVELS; do
        for HALF in first second; do
            echo "============================================================"
            echo "  Running SceneFlowLang: --data $DATA --half $HALF"
            echo "============================================================"
            (cd "$SCENEFLOW_DIR" && bash run_sceneflow_threaded.sh --data "$DATA" --half "$HALF")
            echo
        done
    done

fi

# ---------------------------------------------------------------------------
# Build extractions.json from the new results
# ---------------------------------------------------------------------------

echo "============================================================"
echo "  Building extractions.json"
echo "============================================================"
python3 "$REPO_ROOT/replicate_data/scripts/build_extractions.py"
echo

# ---------------------------------------------------------------------------
# Print all paper tables
# ---------------------------------------------------------------------------

echo "============================================================"
echo "  Table 1 — Ablation study (many-to-one matching, 50% overlap)"
echo "============================================================"
python3 "$REPO_ROOT/replicate_data/scripts/table_1_ablation.py"

echo "============================================================"
echo "  Table 2 — Ablation study (timestamp-based temporal localization)"
echo "============================================================"
python3 "$REPO_ROOT/replicate_data/scripts/table_2_ablation_temporal.py"

echo "============================================================"
echo "  Table 3 — Log-level scenario extraction performance"
echo "============================================================"
python3 "$REPO_ROOT/replicate_data/scripts/table_3_ablation_log_metrics.py"

echo "============================================================"
echo "  Table 4 — SG track fragmentation (pre-computed, not regenerated)"
echo "============================================================"
python3 "$REPO_ROOT/replicate_data/scripts/table_4_track_fragmentation.py"

echo "============================================================"
echo "  Table 5 — Scenario coverage of the full Argoverse 2 dataset"
echo "============================================================"
python3 "$REPO_ROOT/replicate_data/scripts/table_5_compute_coverage.py"

echo
echo "Replication Level 2 complete. All tables printed above."
