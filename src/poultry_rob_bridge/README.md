# Poultry Rob Bridge

Bridge package for the DIL UDS/protobuf interface.

## Roles

| Executable | Role |
| --- | --- |
| `uds_bridge_node` | Connects to `/tmp/farm.sock`, decodes protobuf frames, and publishes ROS `/dil/frame`. |
| `uds_server` | Minimal fake DIL server. Sends detector-like hen IDs, positions, and priorities only. |
| `scenario_uds_server` | HAW simulation server with deterministic test scenarios. |

The default `uds_server` should stay close to the real DIL side. It should not
contain HAW target-management or mission-planning logic. Those parts live in
`high_level_mission_planer`, mainly in `target_manager` and `mission_executor`.

## Minimal Fake DIL

```bash
source /opt/ros/humble/setup.bash
source /home/maxdemu/Documents/ros2-ws/install/setup.bash
ros2 run poultry_rob_bridge uds_server
```

## Scenario Server

```bash
source /opt/ros/humble/setup.bash
source /home/maxdemu/Documents/ros2-ws/install/setup.bash
UDS_SCENARIO=new_near_hen ros2 run poultry_rob_bridge scenario_uds_server
```

Available scenarios include `basic`, `new_near_hen`, `priority_ramp`,
`new_high_priority_far`, `hen_disappears_before_arrival`, `hen_moves`,
`many_hens_uniform`, `many_hens_clusters`, and `many_hens_hotspot`.

The scenario server can simulate hens being scared away after robot visits by
reading `/tmp/poultry_robot_visits.jsonl`:

```bash
UDS_ENABLE_VISIT_EFFECTS=true UDS_VISIT_CLEAR_RADIUS_M=0.8 \
  ros2 run poultry_rob_bridge scenario_uds_server
```
