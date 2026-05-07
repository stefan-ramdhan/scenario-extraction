#!/bin/bash
# =============================================================================
# replicate_level1.sh
# --------------------
# Replication Level 1: reproduce all paper tables from pre-computed results.
#
# No re-running of SceneFlowLang or any other pipeline step is needed.
# All required data is included in the repository.
#
# Usage
# -----
#   cd /path/to/scenario-extraction
#   bash replicate_data/replicate_level1.sh
# =============================================================================

set -e

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"

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
echo "  Table 4 — SG track fragmentation"
echo "============================================================"
python3 "$REPO_ROOT/replicate_data/scripts/table_4_track_fragmentation.py"

echo "============================================================"
echo "  Table 5 — Scenario coverage of the full Argoverse 2 dataset"
echo "============================================================"
python3 "$REPO_ROOT/replicate_data/scripts/table_5_compute_coverage.py"

echo
echo "Replication Level 1 complete. All tables printed above."
