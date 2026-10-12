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
