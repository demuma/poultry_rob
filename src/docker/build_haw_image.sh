#!/usr/bin/env bash
set -e

IMAGE="${HAW_IMAGE:-haw-poultry-ros2:humble}"

docker build \
  -f docker/Dockerfile.haw \
  -t "$IMAGE" \
  .
