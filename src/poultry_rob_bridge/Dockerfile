FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# ── System basics ─────────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release ca-certificates \
    python3 python3-pip \
    && rm -rf /var/lib/apt/lists/*

# ── ROS 2 Humble apt repo ─────────────────────────────────────────────────────
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  | gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
  > /etc/apt/sources.list.d/ros2.list

# ── ROS 2 Humble + build/runtime tools ───────────────────────────────────────
RUN apt-get update && apt-get install -y \
    ros-humble-ros-base \
    ros-humble-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions \
    python3-rosdep \
    protobuf-compiler \
    libprotobuf-dev \
    iputils-ping \
    net-tools \
    && rm -rf /var/lib/apt/lists/*

# ── Python protobuf runtime (pinned for stability) ───────────────────────────
# grpcio-tools bundles a compatible protoc for regenerating pb2 files
RUN pip3 install --no-cache-dir "protobuf>=3.20,<4" grpcio-tools

# ── Copy package sources into a colcon workspace ─────────────────────────────
WORKDIR /ros2_ws
COPY poultry_rob_bridge/ /ros2_ws/src/poultry_rob_bridge/
COPY poultry_rob_bridge_msgs/ /ros2_ws/src/poultry_rob_bridge_msgs/

# ── Regenerate protobuf Python bindings from source ─────────────────────────
RUN python3 -m grpc_tools.protoc \
      -I /ros2_ws/src/poultry_rob_bridge/proto \
      --python_out=/ros2_ws/src/poultry_rob_bridge/poultry_rob_bridge \
      dil_frame.proto

# ── Build the workspace (msgs first, then bridge) ────────────────────────────
RUN source /opt/ros/humble/setup.bash && \
    colcon build \
      --packages-select poultry_rob_bridge_msgs poultry_rob_bridge \
      --cmake-args -DCMAKE_BUILD_TYPE=Release

# ── Entrypoint ────────────────────────────────────────────────────────────────
COPY poultry_rob_bridge/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
# Default: launch the UDS → ROS 2 bridge node
CMD ["ros2", "launch", "poultry_rob_bridge", "bridge.launch.py"]
