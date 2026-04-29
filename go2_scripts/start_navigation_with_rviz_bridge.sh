#!/usr/bin/env bash
set -eo pipefail

cd /home/unitree/go2_navigation_project
source /opt/ros/foxy/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
source install/setup.bash
set -u

PROJECT_DIR="/home/unitree/go2_navigation_project"
PROJECT_INSTALL="${PROJECT_DIR}/install"

prepend_path_front() {
  local var_name="$1"
  local path_value="$2"
  local current_value="${!var_name:-}"
  local filtered_value=""
  local entry
  local old_ifs="${IFS}"

  IFS=:
  for entry in ${current_value}; do
    [ -n "${entry}" ] || continue
    [ "${entry}" = "${path_value}" ] && continue
    filtered_value="${filtered_value}${filtered_value:+:}${entry}"
  done
  IFS="${old_ifs}"

  export "${var_name}=${path_value}${filtered_value:+:${filtered_value}}"
}

# The stock GO2 workspace also provides a go2_navigation package. Keep the
# copied project in front so ros2 launch/run resolves this workspace first.
for prefix in "${PROJECT_INSTALL}"/*; do
  [ -d "${prefix}" ] || continue
  prepend_path_front AMENT_PREFIX_PATH "${prefix}"
  prepend_path_front CMAKE_PREFIX_PATH "${prefix}"
  prepend_path_front COLCON_PREFIX_PATH "${prefix}"
done

: "${GO2_RVIZ_SEND_HOST:=127.0.0.1}"
: "${GO2_RVIZ_SEND_PORT:=16000}"
: "${GO2_RVIZ_RECV_HOST:=127.0.0.1}"
: "${GO2_RVIZ_RECV_PORT:=16001}"
: "${GO2_RVIZ_GOAL_MODE:=action}"
: "${GO2_RVIZ_IMAGE_MAX_HZ:=5.0}"
: "${GO2_RVIZ_POINTCLOUD_MAX_HZ:=2.0}"
: "${GO2_VIDEO_ENABLE:=true}"
: "${GO2_VIDEO_FPS:=10}"

ros2 launch go2_navigation go2_navigation_stack.launch.py \
  video_enable:="${GO2_VIDEO_ENABLE}" \
  target_fps:="${GO2_VIDEO_FPS}" \
  enable_rviz_bridge:=true \
  rviz_bridge_send_host:="${GO2_RVIZ_SEND_HOST}" \
  rviz_bridge_send_port:="${GO2_RVIZ_SEND_PORT}" \
  rviz_bridge_recv_host:="${GO2_RVIZ_RECV_HOST}" \
  rviz_bridge_recv_port:="${GO2_RVIZ_RECV_PORT}" \
  rviz_bridge_goal_mode:="${GO2_RVIZ_GOAL_MODE}" \
  rviz_bridge_image_max_hz:="${GO2_RVIZ_IMAGE_MAX_HZ}" \
  rviz_bridge_pointcloud_max_hz:="${GO2_RVIZ_POINTCLOUD_MAX_HZ}" \
  "$@"
