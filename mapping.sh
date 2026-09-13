#!/usr/bin/env bash
set -euo pipefail

WS_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
LOCAL_SETUP="${WS_ROOT}/install/setup.bash"
SKIP_BUILD="${SKIP_BUILD:-0}"
LAUNCH_DELAY="${LAUNCH_DELAY:-0.5}"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "ROS setup file not found: ${ROS_SETUP}" >&2
  echo "Set ROS_DISTRO or source the correct ROS 2 installation before running this script." >&2
  exit 1
fi

source "${ROS_SETUP}"

if [[ "${SKIP_BUILD}" != "1" ]] && ! command -v colcon >/dev/null 2>&1; then
  echo "colcon was not found after sourcing ${ROS_SETUP}." >&2
  exit 1
fi

if [[ "${SKIP_BUILD}" != "1" ]]; then
  colcon build --symlink-install ${COLCON_BUILD_ARGS:-}
fi

if [[ ! -f "${LOCAL_SETUP}" ]]; then
  echo "Workspace setup file not found: ${LOCAL_SETUP}" >&2
  echo "Run colcon build successfully first, or run with SKIP_BUILD=0." >&2
  exit 1
fi

if ! command -v gnome-terminal >/dev/null 2>&1; then
  echo "gnome-terminal is required to launch this stack in separate terminals." >&2
  exit 1
fi

launch_in_terminal() {
  local cmd="$1"
  echo "Current CMD: ${cmd}"
  gnome-terminal --working-directory="${WS_ROOT}" -- bash -lc \
    "source '${LOCAL_SETUP}'; ${cmd}; exec bash"
  sleep "${LAUNCH_DELAY}"
}

cmds=(
  "ros2 launch livox_ros_driver2 msg_MID360_launch.py"
  "ros2 launch imu_complementary_filter complementary_filter.launch.py"
  "ros2 launch fast_lio mapping.launch.py"
  "ros2 launch fast_lio tf.launch.py"
  "ros2 launch linefit_ground_segmentation_ros segmentation.launch.py"
  "ros2 launch pointcloud_to_laserscan pointcloud_to_laserscan_launch.py"
  "ros2 launch nav2_bringup online_async_launch.py use_sim_time:=false"
  "ros2 launch nav2_bringup bringup_launch.py"
)

for cmd in "${cmds[@]}"; do
  launch_in_terminal "${cmd}"
done
