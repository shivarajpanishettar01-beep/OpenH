#!/usr/bin/env bash
# Unit tests (no robot, no scanner, no camera needed).
set -eo pipefail
cd "$(dirname "$0")/.."
source scripts/env.sh
# ROS puts its launch_testing pytest plugin on PYTHONPATH; it is incompatible with pytest 9.
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
python -m pytest -q src/usrig_common/test src/usrig_robot/test src/usrig_teleop/test src/usrig_episode/test src/usrig_qc/test "$@"
