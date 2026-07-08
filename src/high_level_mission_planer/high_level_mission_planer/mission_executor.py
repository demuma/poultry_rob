#!/usr/bin/env python3

import math
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from tf2_ros import TransformListener, Buffer

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from std_msgs.msg import UInt32MultiArray

from poultry_rob_bridge_msgs.msg import Frame

from high_level_mission_planer.mission_logic import (
    MissionScoringConfig,
    Target,
    available_targets,
    score_target,
    select_next_target,
)


class MissionExecutor(Node):

    def __init__(self):
        super().__init__("mission_executor")

        self.declare_parameter("travel_strategy", "nearest_neighbor")
        self.declare_parameter("approach_strategy", "direct")
        self.declare_parameter("target_stale_timeout_sec", 2.0)
        self.declare_parameter("visited_cooldown_sec", 30.0)
        self.declare_parameter("revisit_visited_after_cooldown", False)
        self.declare_parameter("arrival_radius_m", 0.75)
        self.declare_parameter("priority_weight", 2.5)
        self.declare_parameter("dwell_weight", 0.0)
        self.declare_parameter("distance_weight", 1.0)
        self.declare_parameter("stale_weight", 0.5)
        self.declare_parameter("max_priority", 3.0)
        self.declare_parameter("max_relevant_distance_m", 10.0)
        self.declare_parameter("max_dwell_time_sec", 60.0)
        self.declare_parameter("plan_preview_length", 10)
        self.declare_parameter("enable_nav_watchdog", True)
        self.declare_parameter("nav_watchdog_period_sec", 1.0)
        self.declare_parameter("nav_recovery_grace_sec", 2.0)

        # Permanent storage (history capable)
        # Columns: id | type | priority | x | y | timestamp
        self.positions = np.empty((0, 6), dtype=object)
        self.targets: Dict[int, Target] = {}
        self.robot_position: Optional[Tuple[float, float]] = None

        self.mission_active = False
        self.travel_plan: List[PoseStamped] = []
        self.current_goal_index = 0
        self.current_target_id: Optional[int] = None
        self.current_goal_position: Optional[Tuple[float, float]] = None
        self._pending_msg = None
        self._start_timer = None
        self.nav_was_available = False
        self.nav_outage_started_at: Optional[float] = None
        self.waiting_for_nav_recovery = False
        self.goal_sequence = 0

        self.create_subscription(
            Frame,
            "/dil/frame",
            self.new_positions_callback,
            10
        )
        self.current_goal_pub = self.create_publisher(
            PoseStamped,
            "/mission/current_goal",
            10
        )
        self.plan_preview_pub = self.create_publisher(
            Path,
            "/mission/planned_target_sequence",
            10
        )
        self.visited_targets_pub = self.create_publisher(
            UInt32MultiArray,
            "/mission/visited_target_ids",
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose"
        )
        watchdog_period = max(
            self.get_parameter("nav_watchdog_period_sec").get_parameter_value().double_value,
            0.1,
        )
        self.create_timer(watchdog_period, self._nav_watchdog_callback)

        self.get_logger().info("MissionExecutor started.")

    # ==========================================================
    # DATA INPUT
    # ==========================================================

    def _stamp_to_seconds(self, stamp) -> float:
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) / 1e9
        if stamp_sec > 0.0:
            return stamp_sec
        return self.get_clock().now().nanoseconds / 1e9

    def _update_target(self, obj, source_frame: str, stamp_sec: float):
        tx, ty = self._transform_point(obj.position.x, obj.position.y, source_frame)
        target_id = int(obj.id)

        existing = self.targets.get(target_id)
        if existing is None:
            self.targets[target_id] = Target(
                id=target_id,
                type=obj.type,
                priority=int(obj.priority),
                x=tx,
                y=ty,
                first_seen=stamp_sec,
                last_seen=stamp_sec,
            )
            return

        moved_distance = math.hypot(tx - existing.x, ty - existing.y)
        existing.type = obj.type
        existing.priority = int(obj.priority)
        existing.x = tx
        existing.y = ty
        existing.last_seen = stamp_sec
        existing.seen_count += 1

        if existing.status == "visited" and moved_distance > self._arrival_radius():
            existing.status = "active"
            existing.visited_at = None
        elif existing.status == "stale":
            existing.status = "active"

    def _start_mission_once(self):
        if self._start_timer is not None:
            self._start_timer.cancel()
            self._start_timer = None

        self.execute_mission()

    def new_positions_callback(self, msg: Frame):
        source_frame = msg.header.frame_id or "camera_optical_frame"
        stamp_sec = self._stamp_to_seconds(msg.header.stamp)

        new_rows = []

        for obj in msg.objects:
            new_rows.append([
                obj.id,
                obj.type,
                obj.priority,
                obj.position.x,
                obj.position.y,
                stamp_sec
            ])

        if new_rows:
            self.positions = np.vstack([self.positions, new_rows])

        for obj in msg.objects:
            self.get_logger().info(
                f"Incoming position: id={obj.id} type={obj.type} prio={obj.priority} "
                f"x={obj.position.x:.3f} y={obj.position.y:.3f}"
            )

            if obj.type == "ROBOT":
                self.robot_position = self._transform_point(
                    obj.position.x,
                    obj.position.y,
                    source_frame
                )
            else:
                self._update_target(obj, source_frame, stamp_sec)

        self._publish_visited_targets()

        if self._has_available_target():
            current_target = self.targets.get(self.current_target_id) if self.current_target_id is not None else None
            self._publish_plan_preview(current_target)

        if not self.mission_active and self._has_available_target():
            self.mission_active = True

            if self._start_timer is None:
                self._start_timer = self.create_timer(0.01, self._start_mission_once)

    # ==========================================================
    # STRATEGY SELECTION
    # ==========================================================

    def choose_travel_strategy(self) -> str:
        return self.get_parameter(
            "travel_strategy"
        ).get_parameter_value().string_value

    def choose_approach_strategy(self) -> str:
        return self.get_parameter(
            "approach_strategy"
        ).get_parameter_value().string_value

    # ==========================================================
    # TRAVEL STRATEGIES
    # Input: positions as list of (id, type, x, y)
    # Output: List[PoseStamped]
    # ==========================================================

    def compute_travel_plan(self, current_positions, robot_position=None) -> List[PoseStamped]:
        strategy = self.choose_travel_strategy()

        if strategy == "nearest_neighbor":
            return self.nearest_neighbor_strategy(current_positions, robot_position)

        if strategy == "sequential":
            return self.sequential_strategy(current_positions)

        return self.nearest_neighbor_strategy(current_positions, robot_position)

    def sequential_strategy(self, current_positions):
        ordered = sorted(current_positions, key=lambda p: p[0])
        return [self.create_pose(p[2], p[3]) for p in ordered]

    def nearest_neighbor_strategy(self, current_positions, robot_position=None):

        remaining = current_positions.copy()
        poses = []

        # Try TF first, fall back to robot position from message
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            current_x = transform.transform.translation.x
            current_y = transform.transform.translation.y
        except Exception:
            if robot_position is not None:
                current_x, current_y = robot_position
                self.get_logger().warn("TF unavailable, using ROBOT position from message.")
            else:
                self.get_logger().warn(
                    "TF unavailable and no ROBOT position. Falling back to sequential."
                )
                return self.sequential_strategy(current_positions)

        while remaining:
            nearest = min(
                remaining,
                key=lambda p: math.hypot(
                    p[2] - current_x,
                    p[3] - current_y
                )
            )

            poses.append(self.create_pose(nearest[2], nearest[3]))

            current_x = nearest[2]
            current_y = nearest[3]

            remaining.remove(nearest)

        return poses

    # ==========================================================
    # APPROACH STRATEGY
    # ==========================================================

    def apply_approach_strategy(self, pose: PoseStamped) -> PoseStamped:
        strategy = self.choose_approach_strategy()

        if strategy == "direct":
            return pose

        return pose

    # ==========================================================
    # MISSION EXECUTION (Asynchronous)
    # ==========================================================

    def execute_mission(self, _msg: Optional[Frame] = None):
        self.current_goal_index = 0

        if not self._has_available_target():
            self.get_logger().info("No navigation targets available.")
            self.mission_active = False
            return

        self.get_logger().info("Mission started.")

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 server unavailable.")
            self.mission_active = False
            return

        self.nav_was_available = True
        self.waiting_for_nav_recovery = False
        self.nav_outage_started_at = None
        self.send_next_goal()

    def _nav_server_ready_callback(self, future):
        try:
            server_available = future.result()
        except Exception as exc:
            self.get_logger().error(f"Error while waiting for Nav2 server: {exc}")
            self.mission_active = False
            return

        if not server_available:
            self.get_logger().error("Nav2 server unavailable.")
            self.mission_active = False
            return

        self.send_next_goal()

    def send_next_goal(self):
        selected = self._select_next_target()

        if selected is None:
            self.get_logger().info("Mission completed.")
            self.mission_active = False
            self.current_target_id = None
            return

        target, score, distance = selected
        pose = self.create_pose(target.x, target.y)
        self._publish_plan_preview(target)
        self.current_target_id = target.id
        self.current_goal_position = (target.x, target.y)
        target.status = "in_progress"

        goal = NavigateToPose.Goal()
        goal.pose = self.apply_approach_strategy(pose)
        self.current_goal_pub.publish(goal.pose)
        

        self.get_logger().info(
            f"Selected target id={target.id} type={target.type} prio={target.priority} "
            f"score={score:.3f} distance={distance:.3f}"
        )

        self.get_logger().info(
            f"Sending goal [{self.current_goal_index + 1}]: "
            f"x={goal.pose.pose.position.x:.3f} y={goal.pose.pose.position.y:.3f}"
        )

        self.goal_sequence += 1
        goal_sequence = self.goal_sequence
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(
            lambda done_future, sequence=goal_sequence: self.goal_response_callback(
                done_future,
                sequence,
            )
        )

    def goal_response_callback(self, future, goal_sequence: int):
        if goal_sequence != self.goal_sequence:
            self.get_logger().warn("Ignoring stale Nav2 goal response.")
            return

        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to send goal: {exc}")
            self._release_current_target()
            self.send_next_goal()
            return

        if goal_handle is None:
            self.get_logger().error("Goal handle is None.")
            self._release_current_target()
            self.send_next_goal()
            return

        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected.")
            self._release_current_target()
            self.send_next_goal()
            return

        self.get_logger().info("Goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done_future, sequence=goal_sequence: self.goal_result_callback(
                done_future,
                sequence,
            )
        )

    def goal_result_callback(self, future, goal_sequence: int):
        if goal_sequence != self.goal_sequence:
            self.get_logger().warn("Ignoring stale Nav2 goal result.")
            return

        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to get navigation result: {exc}")
            self._release_current_target()
            self.send_next_goal()
            return

        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Waypoint reached.")
            self._mark_current_target_visited()
        else:
            self.get_logger().warn(f"Navigation failed: {result.status}")
            self._release_current_target()

        self.current_goal_index += 1
        self.send_next_goal()

    # ==========================================================
    # UTILITIES
    # ==========================================================

    def _param_float(self, name: str) -> float:
        return self.get_parameter(name).get_parameter_value().double_value

    def _target_stale_timeout(self) -> float:
        return self._param_float("target_stale_timeout_sec")

    def _visited_cooldown(self) -> float:
        return self._param_float("visited_cooldown_sec")

    def _nav_recovery_grace(self) -> float:
        return self._param_float("nav_recovery_grace_sec")

    def _nav_watchdog_enabled(self) -> bool:
        return self.get_parameter("enable_nav_watchdog").get_parameter_value().bool_value

    def _revisit_visited_after_cooldown(self) -> bool:
        return self.get_parameter("revisit_visited_after_cooldown").get_parameter_value().bool_value

    def _arrival_radius(self) -> float:
        return self._param_float("arrival_radius_m")

    def _scoring_config(self) -> MissionScoringConfig:
        return MissionScoringConfig(
            priority_weight=self._param_float("priority_weight"),
            dwell_weight=self._param_float("dwell_weight"),
            distance_weight=self._param_float("distance_weight"),
            stale_weight=self._param_float("stale_weight"),
            max_priority=self._param_float("max_priority"),
            max_relevant_distance_m=self._param_float("max_relevant_distance_m"),
            max_dwell_time_sec=self._param_float("max_dwell_time_sec"),
        )

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _get_robot_position(self) -> Optional[Tuple[float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            return (
                transform.transform.translation.x,
                transform.transform.translation.y,
            )
        except Exception:
            return self.robot_position

    def _active_targets(self):
        now = self._now_seconds()
        return available_targets(
            self.targets.values(),
            now,
            self._target_stale_timeout(),
            self._visited_cooldown(),
            self._revisit_visited_after_cooldown(),
        )

    def _has_available_target(self) -> bool:
        return len(self._active_targets()) > 0

    def _score_target(self, target: Target, robot_position: Optional[Tuple[float, float]]):
        now = self._now_seconds()
        return score_target(
            target,
            robot_position,
            now,
            self._target_stale_timeout(),
            self._scoring_config(),
        )

    def _select_next_target(self):
        robot_position = self._get_robot_position()
        return select_next_target(
            self.targets.values(),
            robot_position,
            self._now_seconds(),
            self._target_stale_timeout(),
            self._visited_cooldown(),
            self._scoring_config(),
            self._revisit_visited_after_cooldown(),
        )

    def _rank_targets(self):
        robot_position = self._get_robot_position()
        now = self._now_seconds()
        ranked = []

        for target in self._active_targets():
            score, distance = score_target(
                target,
                robot_position,
                now,
                self._target_stale_timeout(),
                self._scoring_config(),
            )
            ranked.append((target, score, distance))

        return sorted(ranked, key=lambda item: item[1], reverse=True)

    def _publish_plan_preview(self, first_target: Optional[Target] = None):
        max_targets = max(int(self.get_parameter("plan_preview_length").value), 1)
        ranked = self._rank_targets()

        ordered_targets: List[Target] = []
        if first_target is not None:
            ordered_targets.append(first_target)

        for target, _score, _distance in ranked:
            if first_target is not None and target.id == first_target.id:
                continue
            ordered_targets.append(target)
            if len(ordered_targets) >= max_targets:
                break

        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()

        points = []
        robot_position = self._get_robot_position()
        if robot_position is not None:
            points.append(robot_position)

        for target in ordered_targets[:max_targets]:
            points.append((target.x, target.y))

        for index, point in enumerate(points):
            if index + 1 < len(points):
                next_point = points[index + 1]
                yaw = math.atan2(next_point[1] - point[1], next_point[0] - point[0])
            elif index > 0:
                previous_point = points[index - 1]
                yaw = math.atan2(point[1] - previous_point[1], point[0] - previous_point[0])
            else:
                yaw = 0.0
            path.poses.append(self.create_pose(point[0], point[1], yaw))

        self.plan_preview_pub.publish(path)

    def _publish_visited_targets(self):
        msg = UInt32MultiArray()
        msg.data = [
            int(target.id)
            for target in self.targets.values()
            if target.status == "visited"
        ]
        self.visited_targets_pub.publish(msg)

    def _nav_watchdog_callback(self):
        if not self._nav_watchdog_enabled():
            return

        now = self._now_seconds()
        nav_available = self.nav_client.server_is_ready()

        if nav_available:
            if not self.nav_was_available:
                self.get_logger().info("Nav2 action server available.")
            self.nav_was_available = True
            self.nav_outage_started_at = None

            if self.waiting_for_nav_recovery:
                self.get_logger().warn("Nav2 recovered. Replanning from current robot pose.")
                self.waiting_for_nav_recovery = False

                if not self.mission_active and self._has_available_target():
                    self.mission_active = True
                    self.execute_mission()
            elif not self.mission_active and self._has_available_target():
                self.mission_active = True
                self.execute_mission()
            return

        if self.nav_outage_started_at is None:
            self.nav_outage_started_at = now
            if self.nav_was_available or self.mission_active or self.current_target_id is not None:
                self.get_logger().warn("Nav2 action server unavailable. Starting recovery grace period.")
            self.nav_was_available = False
            return

        outage_duration = now - self.nav_outage_started_at
        if outage_duration < self._nav_recovery_grace():
            return

        if self.mission_active or self.current_target_id is not None:
            self.get_logger().warn(
                "Nav2 outage exceeded grace period. Releasing current goal and waiting for recovery."
            )
            self._release_current_target()
            self.mission_active = False
            self.waiting_for_nav_recovery = True

    def _release_current_target(self):
        if self.current_target_id is None:
            return

        target = self.targets.get(self.current_target_id)
        if target is not None and target.status == "in_progress":
            target.status = "active"
        self.current_target_id = None
        self.current_goal_position = None
        self.goal_sequence += 1

    def _mark_current_target_visited(self):
        if self.current_target_id is None:
            return

        target = self.targets.get(self.current_target_id)
        if target is None:
            self.current_target_id = None
            self.current_goal_position = None
            return

        now = self._now_seconds()
        age = max(0.0, now - target.last_seen)
        if self.current_goal_position is None:
            current_distance = 0.0
        else:
            current_distance = math.hypot(
                target.x - self.current_goal_position[0],
                target.y - self.current_goal_position[1],
            )

        if age <= self._target_stale_timeout() and current_distance <= self._arrival_radius():
            target.status = "visited"
            target.visited_at = now
            self.get_logger().info(
                f"Target id={target.id} marked visited for {self._visited_cooldown():.1f}s."
            )
        else:
            target.status = "active"
            self.get_logger().warn(
                f"Target id={target.id} not marked visited: age={age:.2f}s "
                f"distance={current_distance:.2f}m."
            )

        self.current_target_id = None
        self.current_goal_position = None
        self._publish_visited_targets()

    def _transform_point(self, x: float, y: float, source_frame: str):
        """Transform a 2D point from source_frame to map via TF."""
        try:
            t = self.tf_buffer.lookup_transform("map", source_frame, rclpy.time.Time())
            tx = x + t.transform.translation.x
            ty = y + t.transform.translation.y
            return tx, ty
        except Exception as exc:
            self.get_logger().warn(
                f"TF transform {source_frame} -> map failed: {exc}. Using raw coordinates."
            )
            return x, y

    def create_pose(self, x, y, yaw: float = 0.0) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.z = math.sin(yaw * 0.5)
        pose.pose.orientation.w = math.cos(yaw * 0.5)
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = MissionExecutor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
