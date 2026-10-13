#!/usr/bin/env bash
# Start URSim e-series in Docker for testing (no real robot involved).
#   scripts/ursim.sh [MODEL]      MODEL default UR7e; if this URSim version does not know UR7e,
#                                 fall back to UR5e (read-only RTDE tests do not depend on it).
# Ports on localhost: 30004 RTDE (robot_state, path B), 29999 dashboard, 30001-30003 primary/
# secondary/realtime, 51680 reachable from the container for the path A URScript client.
# Teach pendant in a browser: http://localhost:6080/vnc.html
# Stop: docker stop ursim
set -euo pipefail
MODEL="${1:-UR7e}"
IMAGE="${URSIM_IMAGE:-universalrobots/ursim_e-series}"
docker run --rm -d --name ursim \
  -e ROBOT_MODEL="$MODEL" \
  -p 6080:6080 -p 29999:29999 -p 30001-30004:30001-30004 \
  --add-host=host.docker.internal:host-gateway \
  "$IMAGE"
echo "URSim ($MODEL) starting. Pendant: http://localhost:6080/vnc.html"
echo "Power on and release brakes in the pendant, then: ros2 launch usrig_bringup robot.launch.py sim:=true"
