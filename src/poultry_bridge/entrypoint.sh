#!/usr/bin/env bash
set -e

# Source ROS 2 Humble base
source /opt/ros/humble/setup.bash

# Source the built workspace overlay (populated by colcon build in Dockerfile)
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

export ROS_DOMAIN_ID=75
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# Uncomment when multiple ethernet port and set the right one for ROS2 communication
#export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="enp129s0"/></Interfaces></General></Domain></CycloneDDS>'
exec "$@"
