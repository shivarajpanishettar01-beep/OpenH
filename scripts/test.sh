#!/usr/bin/env bash
# Unit tests (no robot, no scanner, no camera needed).
set -eo pipefail
cd "$(dirname "$0")/.."
source scripts/env.sh
# Isolation: the sim tests publish /teleop/twist and /operator/command. A random domain (never the rig's 0)
# and localhost only, so a test run cannot reach a running rig or another test run.
export ROS_DOMAIN_ID="${USRIG_TEST_DOMAIN_ID:-$((20 + RANDOM % 80))}" ROS_LOCALHOST_ONLY=1
# ROS puts its launch_testing pytest plugin on PYTHONPATH; it is incompatible with pytest 9.
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
# Tests never depend on this PC's machine settings (~/.config/usrig/machine.yaml).
export USRIG_MACHINE_CONFIG=/nonexistent/usrig_machine.yaml
# Test the source tree, not the installed copies (a non-symlink build would otherwise hide edits).
for _p in src/usrig_common src/usrig_robot src/usrig_teleop src/usrig_presets src/usrig_freedrive src/usrig_episode src/usrig_qc src/usrig_dashboard src/usrig_demo_gui; do
  PYTHONPATH="$PWD/$_p:$PYTHONPATH"
done
export PYTHONPATH
python -m pytest -q src/usrig_common/test src/usrig_robot/test src/usrig_teleop/test src/usrig_presets/test src/usrig_freedrive/test src/usrig_episode/test src/usrig_qc/test src/usrig_dashboard/test src/usrig_demo_gui/test export/test_export_fields.py "$@"
