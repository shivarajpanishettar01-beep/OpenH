#!/usr/bin/env bash
# Unit tests (no robot, no scanner, no camera needed).
set -eo pipefail
cd "$(dirname "$0")/.."
source scripts/env.sh
# ROS puts its launch_testing pytest plugin on PYTHONPATH; it is incompatible with pytest 9.
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
# Test the source tree, not the installed copies (a non-symlink build would otherwise hide edits).
for _p in src/usrig_common src/usrig_robot src/usrig_teleop src/usrig_episode src/usrig_qc src/usrig_dashboard; do
  PYTHONPATH="$PWD/$_p:$PYTHONPATH"
done
export PYTHONPATH
python -m pytest -q src/usrig_common/test src/usrig_robot/test src/usrig_teleop/test src/usrig_episode/test src/usrig_qc/test src/usrig_dashboard/test export/test_export_fields.py "$@"
