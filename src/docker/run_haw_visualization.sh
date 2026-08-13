#!/usr/bin/env bash
set -e

IMAGE="${HAW_IMAGE:-haw-poultry-ros2:humble}"
XAUTH="${XAUTHORITY:-$HOME/.Xauthority}"

DOCKER_ARGS=(
  --rm -it
  --network host \
  -v /tmp:/tmp \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY="${DISPLAY}" \
  -e QT_X11_NO_MITSHM=1 \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-75}" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}" \
  --name haw_visualization
)

if [ -f "$XAUTH" ]; then
  DOCKER_ARGS+=(
    -v "$XAUTH:/tmp/.docker.xauth:ro"
    -e XAUTHORITY="/tmp/.docker.xauth"
  )
else
  echo "Warning: Xauthority file not found at '$XAUTH'."
  echo "If RViz cannot open, run: xhost +si:localuser:root"
fi

if [ -d /dev/dri ]; then
  DOCKER_ARGS+=(--device /dev/dri)
fi

docker run "${DOCKER_ARGS[@]}" \
  "$IMAGE" \
  ros2 launch high_level_mission_planer visualization.launch.py "$@"
