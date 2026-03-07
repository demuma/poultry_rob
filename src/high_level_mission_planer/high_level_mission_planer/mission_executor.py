#!/usr/bin/env python3

import math
from typing import List

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from tf2_ros import TransformListener, Buffer

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from poultry_bridge_msgs.msg import TestFrame


class MissionExecutor(Node):

    def __init__(self):
        super().__init__("mission_executor")

        self.declare_parameter("travel_strategy", "nearest_neighbor")
        self.declare_parameter("approach_strategy", "direct")

        # Permanent storage (history capable)
        # Columns: id | type | x | y | timestamp
        self.positions = np.empty((0, 5), dtype=object)

        self.mission_active = False

        self.create_subscription(
            TestFrame,
            "/farm/test_frame",
            self.new_positions_callback,
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            "navigate_to_pose"
        )

        self.get_logger().info("MissionExecutor started.")

    # ==========================================================
    # DATA INPUT
    # ==========================================================

    def new_positions_callback(self, msg: TestFrame):

        new_rows = []

        for obj in msg.objects:
            new_rows.append([
                obj.id,
                obj.type,
                obj.position.x,
                obj.position.y,
                msg.header.stamp
            ])

        if new_rows:
            self.positions = np.vstack([self.positions, new_rows])

        # Filter out ROBOT objects — only navigate to actual targets
        targets = [obj for obj in msg.objects if obj.type != "ROBOT"]
        if not self.mission_active and len(targets) > 0:
            self.execute_mission(msg)

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

        elif strategy == "sequential":
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
                self.get_logger().warn("TF unavailable and no ROBOT position. Falling back to sequential.")
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
    # MISSION EXECUTION (Synchronous)
    # ==========================================================

    def execute_mission(self, msg: TestFrame):

        self.mission_active = True

        # Extract ROBOT position as current location
        robot_position = None
        for obj in msg.objects:
            if obj.type == "ROBOT":
                robot_position = (obj.position.x, obj.position.y)
                break

        # Extract only non-ROBOT objects as navigation targets
        current_positions = [
            (obj.id, obj.type, obj.position.x, obj.position.y)
            for obj in msg.objects if obj.type != "ROBOT"
        ]

        travel_plan = self.compute_travel_plan(current_positions, robot_position)

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Nav2 server unavailable.")
            self.mission_active = False
            return

        for pose in travel_plan:

            goal = NavigateToPose.Goal()
            goal.pose = self.apply_approach_strategy(pose)

            self.get_logger().info(
                f"Navigating to x={pose.pose.position.x:.2f}, "
                f"y={pose.pose.position.y:.2f}"
            )

            send_goal_future = self.nav_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_goal_future)

            goal_handle = send_goal_future.result()

            if not goal_handle.accepted:
                self.get_logger().warn("Goal rejected.")
                continue

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

            result = result_future.result()

            if result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info("Waypoint reached.")
            else:
                self.get_logger().warn(
                    f"Navigation failed: {result.status}"
                )

        self.get_logger().info("Mission completed.")
        self.mission_active = False

    # ==========================================================
    # UTILITIES
    # ==========================================================

    def create_pose(self, x, y) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
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
