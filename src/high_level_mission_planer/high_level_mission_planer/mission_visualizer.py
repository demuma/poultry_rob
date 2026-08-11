#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Dict, Optional, Set, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import ColorRGBA, UInt32MultiArray
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from poultry_rob_bridge_msgs.msg import Frame
from poultry_rob_bridge_msgs.msg import TrackedTargetArray


@dataclass
class VisualTarget:
    id: int
    type: str
    priority: int
    x: float
    y: float
    last_seen: float
    status: str = "active"


class MissionVisualizer(Node):

    def __init__(self):
        super().__init__("mission_visualizer")

        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_frame", "base_link")
        self.declare_parameter("target_stale_timeout_sec", 2.0)
        self.declare_parameter("publish_period_sec", 0.2)
        self.declare_parameter("field_min_x", -5.0)
        self.declare_parameter("field_max_x", 30.0)
        self.declare_parameter("field_min_y", -10.0)
        self.declare_parameter("field_max_y", 10.0)
        self.declare_parameter("path_min_step_m", 0.05)
        self.declare_parameter("max_path_points", 1000)
        self.declare_parameter("use_tracked_targets", True)
        self.declare_parameter("tracked_targets_topic", "/mission/tracked_targets")
        self.declare_parameter("dil_frame_topic", "/dil/frame")
        self.declare_parameter("enable_dil_frame_fallback", True)
        self.declare_parameter("tracked_targets_timeout_sec", 2.0)

        self.targets: Dict[int, VisualTarget] = {}
        self.current_goal: Optional[PoseStamped] = None
        self.latest_odom: Optional[Odometry] = None
        self.last_tracked_targets_at: Optional[float] = None
        self.visible_target_ids: Set[int] = set()
        self.visited_target_ids: Set[int] = set()
        self.robot_path = Path()
        self.robot_path.header.frame_id = self._global_frame()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/mission/visualization_markers",
            10
        )
        self.default_marker_pub = self.create_publisher(
            MarkerArray,
            "/visualization_marker_array",
            10
        )
        self.path_pub = self.create_publisher(Path, "/mission/robot_path", 10)

        self.create_subscription(
            Frame,
            self.get_parameter("dil_frame_topic").get_parameter_value().string_value,
            self.frame_callback,
            10,
        )
        self.create_subscription(
            TrackedTargetArray,
            self.get_parameter("tracked_targets_topic").get_parameter_value().string_value,
            self.tracked_targets_callback,
            10,
        )
        self.create_subscription(PoseStamped, "/mission/current_goal", self.goal_callback, 10)
        self.create_subscription(UInt32MultiArray, "/mission/visited_target_ids", self.visited_callback, 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        period = max(float(self.get_parameter("publish_period_sec").value), 0.05)
        self.create_timer(period, self.publish_visualization)

        self.get_logger().info("MissionVisualizer started.")

    def _global_frame(self) -> str:
        return self.get_parameter("global_frame").value

    def _robot_frame(self) -> str:
        return self.get_parameter("robot_frame").value

    def _now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _use_tracked_targets(self) -> bool:
        return self.get_parameter("use_tracked_targets").get_parameter_value().bool_value

    def _dil_frame_fallback_enabled(self) -> bool:
        return self.get_parameter("enable_dil_frame_fallback").get_parameter_value().bool_value

    def _tracked_targets_timeout(self) -> float:
        return max(
            self.get_parameter("tracked_targets_timeout_sec").get_parameter_value().double_value,
            0.1,
        )

    def _dil_frame_fallback_active(self) -> bool:
        if not self._use_tracked_targets():
            return True
        if not self._dil_frame_fallback_enabled():
            return False
        if self.last_tracked_targets_at is None:
            return True
        return self._now_seconds() - self.last_tracked_targets_at > self._tracked_targets_timeout()

    def _stamp_to_seconds(self, stamp) -> float:
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) / 1e9
        if stamp_sec > 0.0:
            return stamp_sec
        return self._now_seconds()

    def _transform_point(self, x: float, y: float, source_frame: str) -> Tuple[float, float]:
        if source_frame == self._global_frame():
            return x, y

        try:
            transform = self.tf_buffer.lookup_transform(
                self._global_frame(),
                source_frame,
                rclpy.time.Time()
            )
            return (
                x + transform.transform.translation.x,
                y + transform.transform.translation.y,
            )
        except Exception as exc:
            self.get_logger().warn(
                f"TF transform {source_frame} -> {self._global_frame()} failed: {exc}. "
                "Using raw coordinates."
            )
            return x, y

    def _yaw_from_quaternion(self, q) -> float:
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

    def _apply_yaw(self, pose, yaw: float):
        pose.orientation.z = math.sin(yaw * 0.5)
        pose.orientation.w = math.cos(yaw * 0.5)

    def _robot_pose(self) -> Optional[Tuple[float, float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self._global_frame(),
                self._robot_frame(),
                rclpy.time.Time()
            )
            yaw = self._yaw_from_quaternion(transform.transform.rotation)
            return (
                transform.transform.translation.x,
                transform.transform.translation.y,
                yaw,
            )
        except Exception:
            if self.latest_odom is None:
                return None
            pose = self.latest_odom.pose.pose.position
            yaw = self._yaw_from_quaternion(self.latest_odom.pose.pose.orientation)
            return pose.x, pose.y, yaw

    def frame_callback(self, msg: Frame):
        if not self._dil_frame_fallback_active():
            return

        source_frame = msg.header.frame_id or self._global_frame()
        stamp_sec = self._stamp_to_seconds(msg.header.stamp)

        for obj in msg.objects:
            if obj.type != "HEN":
                continue

            x, y = self._transform_point(obj.position.x, obj.position.y, source_frame)
            self.targets[int(obj.id)] = VisualTarget(
                id=int(obj.id),
                type=obj.type,
                priority=int(obj.priority),
                x=x,
                y=y,
                last_seen=stamp_sec,
                status="active",
            )

    def tracked_targets_callback(self, msg: TrackedTargetArray):
        if not self._use_tracked_targets():
            return

        self.last_tracked_targets_at = self._now_seconds()
        source_frame = msg.header.frame_id or self._global_frame()

        targets: Dict[int, VisualTarget] = {}
        for tracked in msg.targets:
            if tracked.type != "HEN":
                continue

            x, y = self._transform_point(
                tracked.position.x,
                tracked.position.y,
                source_frame,
            )
            targets[int(tracked.target_id)] = VisualTarget(
                id=int(tracked.target_id),
                type=tracked.type,
                priority=int(tracked.priority),
                x=x,
                y=y,
                last_seen=self._stamp_to_seconds(tracked.last_seen),
                status=tracked.status or "active",
            )

        self.targets = targets

    def goal_callback(self, msg: PoseStamped):
        self.current_goal = msg

    def visited_callback(self, msg: UInt32MultiArray):
        self.visited_target_ids = {int(target_id) for target_id in msg.data}

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def publish_visualization(self):
        now = self._now_seconds()
        markers = MarkerArray()
        markers.markers.extend(self._field_markers())

        active_targets = [
            target
            for target in self.targets.values()
            if now - target.last_seen <= float(self.get_parameter("target_stale_timeout_sec").value)
            and target.status != "stale"
            and target.id not in self.visited_target_ids
        ]
        active_target_ids = {target.id for target in active_targets}

        for deleted_id in self.visible_target_ids - active_target_ids:
            markers.markers.append(self._delete_marker("hens", deleted_id))
            markers.markers.append(self._delete_marker("hen_labels", deleted_id))
        self.visible_target_ids = active_target_ids

        for target in active_targets:
            markers.markers.extend(self._target_markers(target))

        robot_pose = self._robot_pose()
        if robot_pose is not None:
            markers.markers.extend(self._robot_markers(robot_pose))
            self._append_path_pose(robot_pose)

        if self.current_goal is not None:
            markers.markers.extend(self._goal_markers(self.current_goal))

        self.marker_pub.publish(markers)
        self.default_marker_pub.publish(markers)
        self.robot_path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.robot_path)

    def _append_path_pose(self, robot_pose: Tuple[float, float, float]):
        if self.robot_path.poses:
            last = self.robot_path.poses[-1].pose.position
            if math.hypot(robot_pose[0] - last.x, robot_pose[1] - last.y) < float(
                self.get_parameter("path_min_step_m").value
            ):
                return

        pose = PoseStamped()
        pose.header.frame_id = self._global_frame()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(robot_pose[0])
        pose.pose.position.y = float(robot_pose[1])
        self._apply_yaw(pose.pose, robot_pose[2])
        self.robot_path.poses.append(pose)

        max_points = max(int(self.get_parameter("max_path_points").value), 1)
        if len(self.robot_path.poses) > max_points:
            self.robot_path.poses = self.robot_path.poses[-max_points:]

    def _delete_marker(self, namespace: str, marker_id: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._global_frame()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.DELETE
        return marker

    def _base_marker(self, namespace: str, marker_id: int, marker_type: int) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._global_frame()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _color(self, r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
        color = ColorRGBA()
        color.r = float(r)
        color.g = float(g)
        color.b = float(b)
        color.a = float(a)
        return color

    def _priority_color(self, priority: int) -> ColorRGBA:
        if priority <= 0:
            return self._color(0.2, 0.8, 0.25)
        if priority == 1:
            return self._color(1.0, 0.9, 0.1)
        if priority == 2:
            return self._color(1.0, 0.45, 0.05)
        return self._color(0.9, 0.05, 0.05)

    def _target_markers(self, target: VisualTarget):
        sphere = self._base_marker("hens", target.id, Marker.SPHERE)
        sphere.pose.position.x = target.x
        sphere.pose.position.y = target.y
        sphere.pose.position.z = 0.18
        sphere.scale.x = 0.35
        sphere.scale.y = 0.35
        sphere.scale.z = 0.35
        sphere.color = self._priority_color(target.priority)

        label = self._base_marker("hen_labels", target.id, Marker.TEXT_VIEW_FACING)
        label.pose.position.x = target.x
        label.pose.position.y = target.y - 0.025
        label.pose.position.z = 0.75
        label.scale.z = 0.15
        label.text = f"H{target.id}\nP{target.priority}"
        label.color = self._color(1.0, 1.0, 1.0)

        return [sphere, label]

    def _robot_markers(self, robot_pose: Tuple[float, float, float]):
        robot = self._base_marker("robot", 1, Marker.ARROW)
        robot.pose.position.x = robot_pose[0]
        robot.pose.position.y = robot_pose[1]
        robot.pose.position.z = 0.08
        self._apply_yaw(robot.pose, robot_pose[2])
        robot.scale.x = 0.28
        robot.scale.y = 0.045
        robot.scale.z = 0.06
        robot.color = self._color(0.1, 0.45, 1.0, 0.35)

        label = self._base_marker("robot_label", 1, Marker.TEXT_VIEW_FACING)
        label.pose.position.x = robot_pose[0]
        label.pose.position.y = robot_pose[1]
        label.pose.position.z = 0.75
        label.scale.z = 0.16
        label.text = "base_link"
        label.color = self._color(0.65, 0.85, 1.0, 0.55)

        return [robot, label]

    def _goal_markers(self, goal: PoseStamped):
        goal_x, goal_y = self._transform_point(
            goal.pose.position.x,
            goal.pose.position.y,
            goal.header.frame_id or self._global_frame(),
        )

        marker = self._base_marker("current_goal", 1, Marker.CYLINDER)
        marker.pose.position.x = goal_x
        marker.pose.position.y = goal_y
        marker.pose.position.z = 0.04
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.08
        marker.color = self._color(0.1, 0.55, 1.0, 0.55)

        label = self._base_marker("current_goal_label", 1, Marker.TEXT_VIEW_FACING)
        label.pose.position.x = goal_x - 0.17
        label.pose.position.y = goal_y
        label.pose.position.z = 0.95
        label.scale.z = 0.20
        label.text = "CURRENT GOAL"
        label.color = self._color(0.2, 0.7, 1.0)

        return [marker, label]

    def _field_markers(self):
        min_x = float(self.get_parameter("field_min_x").value)
        max_x = float(self.get_parameter("field_max_x").value)
        min_y = float(self.get_parameter("field_min_y").value)
        max_y = float(self.get_parameter("field_max_y").value)

        boundary = self._base_marker("field", 1, Marker.LINE_STRIP)
        boundary.scale.x = 0.04
        boundary.color = self._color(0.75, 0.75, 0.75, 0.9)
        for x, y in [
            (min_x, min_y),
            (max_x, min_y),
            (max_x, max_y),
            (min_x, max_y),
            (min_x, min_y),
        ]:
            point = Point()
            point.x = x
            point.y = y
            boundary.points.append(point)

        return [boundary]


def main(args=None):
    rclpy.init(args=args)
    node = MissionVisualizer()

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
