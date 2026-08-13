# Mission Executor Integration Test

## Build

```bash
cd /home/maxdemu/Documents/ros2-ws
source /opt/ros/humble/setup.bash
colcon build --packages-select poultry_rob_bridge high_level_mission_planer
source install/setup.bash
```

## Start detector/bridge simulation

Recommended all-in-one simulation launch:

```bash
source /opt/ros/humble/setup.bash
source /home/maxdemu/Documents/ros2-ws/install/setup.bash

ros2 launch high_level_mission_planer simulation.launch.py \
  scenario:=new_near_hen \
  use_robot_description:=true
```

With RViz:

```bash
ros2 launch high_level_mission_planer simulation.launch.py \
  scenario:=new_near_hen \
  use_robot_description:=true \
  use_rviz:=true
```

Manual startup remains useful when debugging individual components:

Terminal 1:

```bash
docker run --rm -it --network host -v /tmp:/tmp --name dil dil-ros2-humble:latest
```

Terminal 2:

```bash
source /opt/ros/humble/setup.bash
source /home/maxdemu/Documents/ros2-ws/install/setup.bash
UDS_SCENARIO=new_near_hen ros2 run poultry_rob_bridge scenario_uds_server
```

Expected scenario UDS server logs:

```text
[server] listening on /tmp/farm.sock scenario=new_near_hen
[server] seq=1 t=0.0s objects=2
[server] seq=20 t=3.8s objects=3
```

Optional quick topic check:

```bash
ROS_DOMAIN_ID=75 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  ros2 topic echo /dil/frame --once
```

The default `uds_server` is intentionally minimal and mimics the real DIL side:
it only sends tracked hens with detector-owned `id`, `position`, and `priority`.
Use `scenario_uds_server` for deterministic HAW simulation cases:

| Scenario | Purpose |
| --- | --- |
| `basic` | Two fixed hens, one with higher priority. |
| `new_near_hen` | Two fixed hens at start; after roughly 3 seconds a new low-priority hen appears near the expected route. |
| `new_near_low_priority` | Alias for `new_near_hen`, kept for more explicit test naming. |
| `priority_ramp` | One seated hen increases priority every 15 seconds from `0` to `3`; another low-priority hen stays nearby. |
| `new_high_priority_far` | A close low-priority hen is present first; after roughly 3 seconds a far priority-3 hen appears. Useful for later preemption tests. |
| `hen_disappears_before_arrival` | A priority-2 hen vanishes after roughly 4 seconds while another target remains. Tests stale filtering and arrival validation. |
| `hen_moves` | The same hen ID moves in steps after roughly 3 and 6 seconds. Tests target position updates. |
| `many_hens_uniform` | 120 deterministic hens spread across a larger field. |
| `many_hens_clusters` | 160 deterministic hens in several clusters. |
| `many_hens_hotspot` | 180 deterministic hens with one denser, higher-priority hotspot-like region. |

All scenarios intentionally publish only `HEN` objects; robot pose should come from TF `map -> base_link`.

Large scenarios can be tuned with environment variables:

```bash
UDS_SCENARIO=many_hens_clusters UDS_HEN_COUNT=250 UDS_RANDOM_SEED=7 \
UDS_FIELD_MIN_X=-5 UDS_FIELD_MAX_X=35 UDS_FIELD_MIN_Y=-12 UDS_FIELD_MAX_Y=12 \
  ros2 run poultry_rob_bridge scenario_uds_server
```

The fake robot writes reached goal positions to `/tmp/poultry_robot_visits.jsonl`. The scenario UDS server can read new visit events and remove hens within `UDS_VISIT_CLEAR_RADIUS_M` from following frames, simulating that hens are scared away after the robot arrives.

```bash
UDS_ENABLE_VISIT_EFFECTS=true UDS_VISIT_CLEAR_RADIUS_M=0.8 \
  ros2 run poultry_rob_bridge scenario_uds_server
```

By default, old visit events from previous runs are ignored when the scenario UDS server starts. Set `UDS_REPLAY_VISIT_EVENTS=true` to replay them.

## Current scoring setup

The detector already encodes dwell time into `Object.priority` with priorities from `0` to `3`. For the first tuning round, the mission executor therefore weighs detector priority against distance and does not add its own dwell-time bonus:

```yaml
priority_weight: 2.5
dwell_weight: 0.0
distance_weight: 1.0
stale_weight: 0.5
max_priority: 3.0
```

This keeps the first experiments focused: priority comes from the detector, distance comes from the robot pose via TF.

## Start robot and mission executor simulation manually

Terminal 3:

```bash
source /opt/ros/humble/setup.bash
source /home/maxdemu/Documents/ros2-ws/install/setup.bash
ROS_DOMAIN_ID=75 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  ros2 run high_level_mission_planer fake_nav2_server
```

Terminal 4:

```bash
source /opt/ros/humble/setup.bash
source /home/maxdemu/Documents/ros2-ws/install/setup.bash
ROS_DOMAIN_ID=75 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  ros2 launch high_level_mission_planer mission_executor.launch.py
```

Visualization can be started separately:

```bash
source /opt/ros/humble/setup.bash
source /home/maxdemu/Documents/ros2-ws/install/setup.bash
ROS_DOMAIN_ID=75 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  ros2 launch high_level_mission_planer visualization.launch.py \
    use_robot_description:=true \
    use_rviz:=true
```

## Start real robot mission side

When the real DIL UDS server and robot/Nav2 are running separately, start the HAW side with:

```bash
source /opt/ros/humble/setup.bash
source /home/maxdemu/Documents/ros2-ws/install/setup.bash

ros2 launch high_level_mission_planer robot_mission.launch.py
```

Optional visualization:

```bash
ros2 launch high_level_mission_planer visualization.launch.py \
  use_robot_description:=true \
  use_rviz:=true
```

Only enable `use_robot_description:=true` if the real robot does not already start its own `robot_state_publisher`.

By default this uses the lightweight visualization model installed with
`high_level_mission_planer`. The robot repository's original Xacro can be used
later through its own `robot_description` launch once its hardware dependencies
and `xacro` are installed.

The mission launch expects a ready URDF file via `robot_description_file`.

Expected mission executor logs:

```text
MissionExecutor started.
Incoming position: id=... type=HEN prio=...
Selected target id=... type=HEN prio=... score=... distance=...
Goal accepted.
Waypoint reached.
Target id=... marked visited for ...
```

Expected fake Nav2 logs:

```text
FakeNav2Server started
Received goal: x=... y=...
Moving toward goal
Goal reached
```

## RViz visualization

Start RViz with the prepared display setup:

```bash
source /opt/ros/humble/setup.bash
source /home/maxdemu/Documents/ros2-ws/install/setup.bash
ROS_DOMAIN_ID=75 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  rviz2 -d /home/maxdemu/Documents/ros2-ws/install/high_level_mission_planer/share/high_level_mission_planer/rviz/mission_visualization.rviz
```

Visualization topics:

```text
/visualization_marker_array     # RViz MarkerArray display: field, hens, labels, robot, current goal
/mission/visualization_markers  # same MarkerArray on a project-specific debug topic
/mission/tracked_targets        # TargetManager output: stable HAW target table from /dil/frame
/mission/robot_path             # accumulated robot trajectory
/mission/planned_target_sequence # current target-sequence preview, not a guaranteed full route
/mission/current_goal           # current mission goal from MissionExecutor
/mission/visited_target_ids     # visited hens hidden from RViz and excluded from target selection
/odom                           # fake Nav2 odometry, or real robot odometry later
/joint_states                   # fake wheel joint positions for the RViz RobotModel
/tf                             # map -> base_link and camera transform
/robot_description              # URDF model when use_robot_description:=true
```

The mission executor and visualizer prefer `/mission/tracked_targets`. They only
fall back to raw `/dil/frame` if tracked targets are enabled but no tracked
target message is received for `tracked_targets_timeout_sec`.

The target manager is intentionally conservative for dense barns:

```yaml
association_radius_m: 0.35
same_source_max_jump_m: 0.8
```

It trusts stable DIL IDs first. If an ID is missing or changes, detections are
only merged spatially when they are very close. This favors harmless duplicate
targets over incorrectly merging two nearby hens.

Visited hens stay suppressed by default while the detector keeps reporting them at the same position. If a hen with the same ID moves farther than `arrival_radius_m`, it becomes active again.

Marker colors by priority:

```text
P0 green, P1 yellow, P2 orange, P3 red
```

## HAW Docker container

Build the HAW ROS Humble image from the repository `src` directory:

```bash
cd /home/maxdemu/Documents/ros2-ws/src/poultry_rob/src
./docker/build_haw_image.sh
```

Run the live mission stack headless:

```bash
./docker/run_haw_mission.sh
```

Run real DIL with the fake robot/Nav2 simulation:

```bash
./docker/run_haw_dil_fake_robot.sh
```

Run fake DIL with the real robot/Nav2 stack:

```bash
./docker/run_haw_fake_dil_robot.sh scenario:=basic
```

Run visualization separately:

```bash
./docker/run_haw_visualization.sh use_robot_description:=true use_rviz:=true
```

If RViz prints `Authorization required` or `could not connect to display`, allow
the root user inside the container to access the local X server:

```bash
xhost +si:localuser:root
```

Then start the visualization container again.

Run the full simulation profile:

```bash
./docker/run_haw_simulation.sh scenario:=new_near_hen use_robot_description:=true
```

## Known DIL integration issues

- HAW filters detections with `type == "HEN"`. In the DIL system, object types
  currently come from string enums and `HEN` corresponds to enum value `4`.
  During the current integration test, DIL maps objects with value `4` to the
  outgoing type string `"HEN"`. The final interface should document this type
  convention explicitly.
- If the HAW UDS client stops or crashes, the DIL UDS server can currently fail
  with `BrokenPipeError: Datenuebergabe unterbrochen` in
  `send_msg(docker_server, payload)`. DIL should handle disconnected clients,
  close the broken connection, and wait for a new HAW client.

## Nav2 restart recovery

The mission executor has a Nav2 watchdog for robot power-loss or battery-swap scenarios:

```yaml
enable_nav_watchdog: true
nav_watchdog_period_sec: 1.0
nav_recovery_grace_sec: 2.0
```

If the `navigate_to_pose` action server disappears during an active goal, the executor waits for the grace period, releases the current in-progress target back to `active`, and waits for Nav2 to return. When Nav2 is available again, it replans from the current `map -> base_link` pose and sends a fresh goal. Stale action callbacks from the old server are ignored.

## Local scoring tests

```bash
cd /home/maxdemu/Documents/ros2-ws/src/poultry_rob/src
PYTHONPATH=high_level_mission_planer:poultry_rob_bridge python3 -m pytest -q \
  high_level_mission_planer/test/test_mission_logic.py \
  poultry_rob_bridge/test/test_uds_server_scenarios.py
```
