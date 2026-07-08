#!/usr/bin/env python3

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse
from rclpy.duration import Duration
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from tf2_ros import TransformBroadcaster


class FakeNavServer(Node):

    def __init__(self):
        super().__init__("fake_nav2_server")

        self.declare_parameter("robot_speed_mps", 1.0)
        self.declare_parameter("robot_angular_speed_radps", 0.8)
        self.declare_parameter("rotate_in_place", True)
        self.declare_parameter("feedback_period_sec", 0.2)
        self.declare_parameter("goal_tolerance_m", 0.05)
        self.declare_parameter("yaw_tolerance_rad", 0.03)
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("visit_event_path", "/tmp/poultry_robot_visits.jsonl")
        self.declare_parameter("publish_visit_events", True)

        self.robot_x = self.get_parameter("initial_x").value
        self.robot_y = self.get_parameter("initial_y").value
        self.robot_yaw = 0.0
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0
        self.last_robot_x = self.robot_x
        self.last_robot_y = self.robot_y
        self.last_motion_time = self.get_clock().now()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.create_timer(0.1, self._publish_robot_state)

        self._action_server = ActionServer(
            self,
            NavigateToPose,
            "navigate_to_pose",
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info(
            "Fake NavigateToPose server ready. "
            f"start=({self.robot_x:.2f}, {self.robot_y:.2f}) "
            f"speed={self._robot_speed():.2f}m/s"
        )

    def _publish_robot_state(self):
        now = self.get_clock().now()
        dt = max((now - self.last_motion_time).nanoseconds / 1e9, 0.001)
        vx = (self.robot_x - self.last_robot_x) / dt
        vy = (self.robot_y - self.last_robot_y) / dt

        transform = TransformStamped()
        transform.header.stamp = now.to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = float(self.robot_x)
        transform.transform.translation.y = float(self.robot_y)
        transform.transform.rotation.z = math.sin(self.robot_yaw * 0.5)
        transform.transform.rotation.w = math.cos(self.robot_yaw * 0.5)
        self.tf_broadcaster.sendTransform(transform)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = float(self.robot_x)
        odom.pose.pose.position.y = float(self.robot_y)
        odom.pose.pose.orientation.z = math.sin(self.robot_yaw * 0.5)
        odom.pose.pose.orientation.w = math.cos(self.robot_yaw * 0.5)
        odom.twist.twist.linear.x = float(self.current_linear_velocity)
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = float(self.current_angular_velocity)
        self.odom_pub.publish(odom)

        self.last_robot_x = self.robot_x
        self.last_robot_y = self.robot_y
        self.last_motion_time = now

    def _robot_speed(self) -> float:
        return max(float(self.get_parameter("robot_speed_mps").value), 0.01)

    def _robot_angular_speed(self) -> float:
        return max(float(self.get_parameter("robot_angular_speed_radps").value), 0.01)

    def _rotate_in_place(self) -> bool:
        return bool(self.get_parameter("rotate_in_place").value)

    def _feedback_period(self) -> float:
        return max(float(self.get_parameter("feedback_period_sec").value), 0.05)

    def _goal_tolerance(self) -> float:
        return max(float(self.get_parameter("goal_tolerance_m").value), 0.0)

    def _yaw_tolerance(self) -> float:
        return max(float(self.get_parameter("yaw_tolerance_rad").value), 0.0)

    def _normalize_angle(self, angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    def _publish_visit_event(self, x: float, y: float) -> None:
        if not bool(self.get_parameter("publish_visit_events").value):
            return

        event = {
            "stamp_unix_ms": int(time.time() * 1000),
            "frame_id": "map",
            "x": float(x),
            "y": float(y),
        }
        path = str(self.get_parameter("visit_event_path").value)
        try:
            with open(path, "a", encoding="utf-8") as event_file:
                event_file.write(json.dumps(event) + "\n")
        except OSError as exc:
            self.get_logger().warn(f"Failed to write visit event to {path}: {exc}")

    def cancel_callback(self, _goal_handle):
        self.get_logger().info("Cancel requested.")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        pose = goal_handle.request.pose.pose.position
        target_x = float(pose.x)
        target_y = float(pose.y)

        self.get_logger().info(
            f"Goal received: x={target_x:.2f}, y={target_y:.2f} "
            f"from=({self.robot_x:.2f}, {self.robot_y:.2f})"
        )

        speed = self._robot_speed()
        angular_speed = self._robot_angular_speed()
        period = self._feedback_period()
        tolerance = self._goal_tolerance()
        start_x = self.robot_x
        start_y = self.robot_y
        total_distance = math.hypot(target_x - start_x, target_y - start_y)
        target_yaw = self.robot_yaw
        if total_distance > tolerance:
            target_yaw = math.atan2(target_y - start_y, target_x - start_x)

        if total_distance <= tolerance:
            self.robot_x = target_x
            self.robot_y = target_y
            self.current_linear_velocity = 0.0
            self.current_angular_velocity = 0.0
            self._publish_robot_state()
            goal_handle.succeed()
            self._publish_visit_event(target_x, target_y)
            self.get_logger().info("Goal already reached.")
            return NavigateToPose.Result()

        if self._rotate_in_place():
            remaining_yaw = self._normalize_angle(target_yaw - self.robot_yaw)
            rotate_elapsed = 0.0
            while abs(remaining_yaw) > self._yaw_tolerance():
                if goal_handle.is_cancel_requested:
                    self.current_linear_velocity = 0.0
                    self.current_angular_velocity = 0.0
                    goal_handle.canceled()
                    self.get_logger().info(
                        f"Goal canceled while rotating at yaw={self.robot_yaw:.2f}rad"
                    )
                    return NavigateToPose.Result()

                step_time = min(period, abs(remaining_yaw) / angular_speed)
                direction = 1.0 if remaining_yaw >= 0.0 else -1.0
                self.current_linear_velocity = 0.0
                self.current_angular_velocity = direction * angular_speed
                time.sleep(step_time)
                self.robot_yaw = self._normalize_angle(
                    self.robot_yaw + direction * angular_speed * step_time
                )
                self._publish_robot_state()
                rotate_elapsed += step_time
                remaining_yaw = self._normalize_angle(target_yaw - self.robot_yaw)

                feedback = NavigateToPose.Feedback()
                feedback.current_pose.header.frame_id = "map"
                feedback.current_pose.header.stamp = self.get_clock().now().to_msg()
                feedback.current_pose.pose.position.x = self.robot_x
                feedback.current_pose.pose.position.y = self.robot_y
                feedback.current_pose.pose.orientation.z = math.sin(self.robot_yaw * 0.5)
                feedback.current_pose.pose.orientation.w = math.cos(self.robot_yaw * 0.5)
                feedback.distance_remaining = float(total_distance)
                feedback.navigation_time = self._duration_msg(rotate_elapsed)
                feedback.estimated_time_remaining = self._duration_msg(abs(remaining_yaw) / angular_speed)
                goal_handle.publish_feedback(feedback)

                self.get_logger().info(
                    f"Rotating: yaw={self.robot_yaw:.2f}rad, "
                    f"remaining_yaw={remaining_yaw:.2f}rad"
                )

            self.robot_yaw = target_yaw
            self.current_angular_velocity = 0.0
            self._publish_robot_state()
        else:
            self.robot_yaw = target_yaw

        travel_time = total_distance / speed
        elapsed = 0.0

        while elapsed < travel_time:
            if goal_handle.is_cancel_requested:
                self.current_linear_velocity = 0.0
                self.current_angular_velocity = 0.0
                goal_handle.canceled()
                self.get_logger().info(
                    f"Goal canceled at x={self.robot_x:.2f}, y={self.robot_y:.2f}"
                )
                return NavigateToPose.Result()

            time.sleep(min(period, travel_time - elapsed))
            elapsed = min(elapsed + period, travel_time)
            progress = min(elapsed / travel_time, 1.0)

            self.robot_x = start_x + (target_x - start_x) * progress
            self.robot_y = start_y + (target_y - start_y) * progress
            self.current_linear_velocity = speed
            self.current_angular_velocity = 0.0
            self._publish_robot_state()
            remaining = math.hypot(target_x - self.robot_x, target_y - self.robot_y)

            feedback = NavigateToPose.Feedback()
            feedback.current_pose.header.frame_id = "map"
            feedback.current_pose.header.stamp = self.get_clock().now().to_msg()
            feedback.current_pose.pose.position.x = self.robot_x
            feedback.current_pose.pose.position.y = self.robot_y
            feedback.current_pose.pose.orientation.z = math.sin(self.robot_yaw * 0.5)
            feedback.current_pose.pose.orientation.w = math.cos(self.robot_yaw * 0.5)
            feedback.distance_remaining = float(remaining)
            feedback.navigation_time = self._duration_msg(elapsed)
            feedback.estimated_time_remaining = self._duration_msg(remaining / speed)
            goal_handle.publish_feedback(feedback)

            self.get_logger().info(
                f"Moving: x={self.robot_x:.2f}, y={self.robot_y:.2f}, "
                f"remaining={remaining:.2f}m"
            )

        self.robot_x = target_x
        self.robot_y = target_y
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0
        self._publish_robot_state()
        goal_handle.succeed()
        self._publish_visit_event(target_x, target_y)

        result = NavigateToPose.Result()

        self.get_logger().info(
            f"Goal reached: x={self.robot_x:.2f}, y={self.robot_y:.2f}"
        )

        return result

    def _duration_msg(self, seconds: float):
        msg = Duration(seconds=max(seconds, 0.0)).to_msg()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = FakeNavServer()

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
