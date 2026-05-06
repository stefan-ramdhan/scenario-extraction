"""
Wrapper that injects a props directory at sys.path[0] before any SceneFlowLang
imports occur, allowing a one-line shim named symbolic_properties_ego_only.py
to shadow the real file and redirect to the appropriate properties module.

Usage (from run_sceneflow.sh):
    python3 run_check.py --props-dir ./my_props/tracks [check_symbolic_properties args...]
"""
import sys
import os

for i, a in enumerate(sys.argv[1:], 1):
    if a == '--props-dir':
        sys.path.insert(0, os.path.abspath(sys.argv[i + 1]))
        del sys.argv[i:i + 2]
        break

import check_symbolic_properties
check_symbolic_properties.main()
