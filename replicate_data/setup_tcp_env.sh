#!/bin/bash
# =============================================================================
# setup_tcp_env.sh
# ----------------
# Creates the tcp_env conda environment from SceneFlowLang/tcp_environment.yml.
# Run this once before running replicate_level2.sh.
#
# Usage
# -----
#   bash replicate_data/setup_tcp_env.sh
# =============================================================================

set -e

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ENV_FILE="$REPO_ROOT/SceneFlowLang/tcp_environment.yml"

# ---------------------------------------------------------------------------
# Check conda is available
# ---------------------------------------------------------------------------

if ! command -v conda &>/dev/null; then
    echo "ERROR: conda not found."
    echo
    echo "Install Miniconda (recommended) from:"
    echo "  https://docs.conda.io/en/latest/miniconda.html"
    echo
    echo "Quick install on Linux:"
    echo "  wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh"
    echo "  bash Miniconda3-latest-Linux-x86_64.sh"
    echo "  source ~/.bashrc   # or restart your terminal"
    echo
    echo "Then re-run this script."
    exit 1
fi

# ---------------------------------------------------------------------------
# Create the environment
# ---------------------------------------------------------------------------

if conda env list | grep -q "^tcp_env[[:space:]]"; then
    echo "conda environment 'tcp_env' already exists — skipping creation."
    echo "To recreate it from scratch:"
    echo "  conda env remove -n tcp_env"
    echo "  bash replicate_data/setup_tcp_env.sh"
else
    echo "Creating conda environment 'tcp_env' from $ENV_FILE ..."
    echo "(This may take several minutes.)"
    echo
    conda env create -f "$ENV_FILE"
    echo
    echo "Environment created successfully."
fi

echo
echo "You can now run Replication Level 2:"
echo "  bash replicate_data/replicate_level2.sh"
