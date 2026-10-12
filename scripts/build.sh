#!/usr/bin/env bash
# Build the ROS 2 workspace with the openh conda Python.
set -eo pipefail
cd "$(dirname "$0")/.."
source scripts/env.sh
colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE="$(which python)" -DPYTHON_EXECUTABLE="$(which python)" \
  "$@"
echo "built. In each new terminal: source scripts/env.sh"
