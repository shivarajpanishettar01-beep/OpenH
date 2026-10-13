# Source this in every terminal:  source scripts/env.sh
# Order: conda env (Python 3.10) -> ROS 2 Humble -> this workspace.
_usrig_repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
eval "$(conda shell.bash hook)"
conda activate openh
source /opt/ros/humble/setup.bash
[ -f "$_usrig_repo/install/setup.bash" ] && source "$_usrig_repo/install/setup.bash"
export USRIG_REPO="$_usrig_repo"
export BAG_TO_LEROBOT="${BAG_TO_LEROBOT:-$HOME/US SDK/micrus_capture/tools/bag_to_lerobot/bag_to_lerobot.py}"
# Every stamp is the system clock: never use sim time on this rig.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
unset _usrig_repo
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
# Optional Fast DDS profile (larger shared-memory segments). Measured 13 Oct 2026: no benefit on the
# dev PC (docs/data_integrity.md), so it is opt-in: USRIG_FASTDDS_PROFILE=1 source scripts/env.sh
if [ "${USRIG_FASTDDS_PROFILE:-0}" = 1 ]; then
  export FASTRTPS_DEFAULT_PROFILES_FILE="$USRIG_REPO/config/fastdds_rig.xml"
fi
