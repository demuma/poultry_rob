# DIL/HAW Poultry Robot Mission System

ROS 2 Humble packages for the DIL/HAW poultry robot interface and high-level
mission planning. The system receives hen detections from DIL, manages stable
mission targets, selects targets by detector priority and distance, and sends
navigation goals to Nav2.

## Architecture

```text
DIL or Fake-DIL
  -> UDS protobuf socket (/tmp/farm.sock)
  -> poultry_rob_bridge/uds_bridge_node
  -> /dil/frame
  -> high_level_mission_planer/target_manager
  -> /mission/tracked_targets
  -> high_level_mission_planer/mission_executor
  -> Nav2 navigate_to_pose
```

RViz visualization runs in parallel and also prefers `/mission/tracked_targets`.
Raw `/dil/frame` is kept as fallback/debug input.

## Packages

| Package | Role |
| --- | --- |
| `poultry_rob_bridge_msgs` | ROS message definitions for DIL frames and tracked mission targets. |
| `poultry_rob_bridge` | UDS/protobuf bridge plus minimal and scenario-based fake DIL servers. |
| `high_level_mission_planer` | Target management, mission execution, fake Nav2 simulation, RViz visualization, launch files. |

## Build

```bash
cd poultry_rob
source /opt/ros/humble/setup.bash
rosdep install --from-paths src -y --ignore-src
colcon build
source install/setup.bash
```

## Simulation

All-in-one simulation with fake DIL, bridge, fake Nav2, mission nodes, robot
description, and optional RViz:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch high_level_mission_planer simulation.launch.py \
  scenario:=new_near_hen \
  use_robot_description:=true \
  use_rviz:=true
```

Useful scenarios:

```text
basic
new_near_hen
priority_ramp
new_high_priority_far
hen_disappears_before_arrival
hen_moves
many_hens_uniform
many_hens_clusters
many_hens_hotspot
```

## Live Robot/DIL Mode

Use this when the real DIL UDS server and the real robot/Nav2 stack run
separately:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

ROS_DOMAIN_ID=75 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  ros2 launch high_level_mission_planer robot_mission.launch.py
```

This launch does not start fake Nav2 and does not start a fake DIL server.
It also does not start RViz or the visualization node.

Start visualization separately when needed:

```bash
ROS_DOMAIN_ID=75 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  ros2 launch high_level_mission_planer visualization.launch.py \
    use_robot_description:=true \
    use_rviz:=true
```

Enable `use_robot_description:=true` only if the real robot stack does not
already start its own `robot_state_publisher`.

If DIL starts later, the system waits without moving. Empty tracked-target
messages such as `targets: []` mean the target manager is alive but currently
has no hens.

## HAW Docker Container

The real DIL system can run natively on Ubuntu 24.04 and provide the UDS socket.
The HAW stack runs in one ROS Humble container and consumes `/tmp/farm.sock`.

Build the image from `poultry_rob/src`:

```bash
cd poultry_rob/src
./docker/build_haw_image.sh
```

Run the headless mission stack:

```bash
./docker/run_haw_mission.sh
```

Run visualization separately:

```bash
./docker/run_haw_visualization.sh use_robot_description:=true use_rviz:=true
```

If RViz cannot connect to `DISPLAY`, allow local root X11 access once on the
host and restart the visualization container:

```bash
xhost +si:localuser:root
```

Run the full simulation profile:

```bash
./docker/run_haw_simulation.sh scenario:=new_near_hen use_robot_description:=true
```

## Target Handling

The detector is responsible for hen detection, detector IDs, positions, and
priority. The HAW side does not try to replace DIL tracking; it only manages
mission targets for navigation.

For dense barn conditions, target association is conservative:

```yaml
association_radius_m: 0.35
same_source_max_jump_m: 0.8
```

Stable DIL IDs are trusted first. If an ID changes or disappears, spatial
association only merges detections that are very close. This favors duplicate
targets over accidentally merging two different nearby hens.

Visited hens are suppressed while they remain at the same position. If a hen
with the same target ID moves farther than `arrival_radius_m`, it can become
active again.

## Important Topics

```text
/dil/frame
/mission/tracked_targets
/mission/current_goal
/mission/planned_target_sequence
/mission/visited_target_ids
/mission/visualization_markers
/mission/robot_path
/odom
/tf
```

## Using the Protobuf Message

```bash
cd poultry_rob/src/poultry_rob_bridge/proto
cp dil_frame.proto custom.proto
protoc --python_out=. custom.proto
```

```python
import custom_pb2 as pb
```

## Tests

```bash
cd poultry_rob/src
PYTHONPATH=high_level_mission_planer:poultry_rob_bridge python3 -m pytest -q \
  high_level_mission_planer/test/test_mission_logic.py \
  poultry_rob_bridge/test/test_uds_server_scenarios.py
```

More detailed startup and debugging notes are in `src/TESTING.md`.
