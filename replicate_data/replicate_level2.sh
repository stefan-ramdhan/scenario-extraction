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
#   4. Track mapping file — downloaded automatically if not present
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

if [[ ! -f "$REPO_ROOT/replicate_data/matched/track_mappings.json" ]]; then
    echo "Downloading track_mappings.json (~3 MB) ..."
    mkdir -p "$REPO_ROOT/replicate_data/matched"
    download \
        "https://zenodo.org/records/20057765/files/track_mappings.json?download=1" \
        "$REPO_ROOT/replicate_data/matched/track_mappings.json"
    echo
fi

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
