#!/usr/bin/env bash
set -e

IMAGE="${HAW_IMAGE:-haw-poultry-ros2:humble}"

docker run --rm -it \
  --network host \
  -v /tmp:/tmp \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-75}" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" \
  --name haw_dil_fake_robot \
  "$IMAGE" \
  ros2 launch high_level_mission_planer dil_fake_robot.launch.py "$@"
