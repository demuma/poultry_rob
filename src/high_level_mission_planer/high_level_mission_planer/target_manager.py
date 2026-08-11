#!/usr/bin/env python3

import math
from dataclasses import dataclass
from typing import Dict, Optional, Set, Tuple

import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from poultry_rob_bridge_msgs.msg import Frame
from poultry_rob_bridge_msgs.msg import TrackedTarget
from poultry_rob_bridge_msgs.msg import TrackedTargetArray


@dataclass
class ManagedTarget:
    target_id: int
    source_id: int
    type: str
    priority: int
    x: float
    y: float
    first_seen: float
    last_seen: float
    seen_count: int = 1
    status: str = "active"


class TargetManager(Node):
    def __init__(self):
        super().__init__("target_manager")

        self.declare_parameter("global_frame", "map")
        self.declare_parameter("association_radius_m", 0.75)
        self.declare_parameter("same_source_max_jump_m", 1.5)
        self.declare_parameter("target_stale_timeout_sec", 2.0)
        self.declare_parameter("target_prune_timeout_sec", 30.0)
        self.declare_parameter("publish_period_sec", 0.2)
        self.declare_parameter("enable_debug_logs", True)

        self.targets: Dict[int, ManagedTarget] = {}
        self.next_target_id = 1
        self.last_frame_stamp_sec = 0.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(Frame, "/dil/frame", self.frame_callback, 10)
        self.publisher = self.create_publisher(
            TrackedTargetArray,
            "/mission/tracked_targets",
            10,
        )

        period = max(float(self.get_parameter("publish_period_sec").value), 0.05)
        self.create_timer(period, self.publish_targets)

        self.get_logger().info("TargetManager started.")

    def _global_frame(self) -> str:
        return str(self.get_parameter("global_frame").value)

    def _association_radius(self) -> float:
        return max(float(self.get_parameter("association_radius_m").value), 0.0)

    def _same_source_max_jump(self) -> float:
        return max(float(self.get_parameter("same_source_max_jump_m").value), 0.0)

    def _stale_timeout(self) -> float:
        return max(float(self.get_parameter("target_stale_timeout_sec").value), 0.001)

    def _prune_timeout(self) -> float:
        return max(float(self.get_parameter("target_prune_timeout_sec").value), self._stale_timeout())

    def _debug_enabled(self) -> bool:
        return bool(self.get_parameter("enable_debug_logs").value)

    def _stamp_to_seconds(self, stamp) -> float:
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) / 1e9
        if stamp_sec > 0.0:
            return stamp_sec
        return self.get_clock().now().nanoseconds / 1e9

    def _seconds_to_stamp(self, seconds: float):
        sec = int(seconds)
        nanosec = int((seconds - sec) * 1e9)
        stamp = self.get_clock().now().to_msg()
        stamp.sec = sec
        stamp.nanosec = nanosec
        return stamp

    def _transform_point(self, x: float, y: float, source_frame: str) -> Tuple[float, float]:
        target_frame = self._global_frame()
        if source_frame == target_frame:
            return x, y

        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
            )
            return (
                x + transform.transform.translation.x,
                y + transform.transform.translation.y,
            )
        except Exception as exc:
            self.get_logger().warn(
                f"TF transform {source_frame} -> {target_frame} failed: {exc}. "
                "Using raw coordinates."
            )
            return x, y

    def _distance_to_target(self, target: ManagedTarget, x: float, y: float) -> float:
        return math.hypot(target.x - x, target.y - y)

    def _find_same_source_target(
        self,
        source_id: int,
        x: float,
        y: float,
        updated_target_ids: Set[int],
    ) -> Optional[ManagedTarget]:
        candidates = [
            target for target in self.targets.values()
            if target.source_id == source_id and target.target_id not in updated_target_ids
        ]
        if not candidates:
            return None

        target = min(candidates, key=lambda item: self._distance_to_target(item, x, y))
        if self._distance_to_target(target, x, y) <= self._same_source_max_jump():
            return target

        self.get_logger().warn(
            f"Source id={source_id} jumped from target_id={target.target_id} "
            f"by {self._distance_to_target(target, x, y):.2f}m. Creating new target."
        )
        return None

    def _find_nearest_target(
        self,
        x: float,
        y: float,
        now_sec: float,
        updated_target_ids: Set[int],
    ) -> Optional[ManagedTarget]:
        candidates = []
        for target in self.targets.values():
            if target.target_id in updated_target_ids:
                continue
            if now_sec - target.last_seen > self._stale_timeout():
                continue
            distance = self._distance_to_target(target, x, y)
            if distance <= self._association_radius():
                candidates.append((distance, target))

        if not candidates:
            return None
        return min(candidates, key=lambda item: item[0])[1]

    def _create_target(self, source_id: int, obj_type: str, priority: int, x: float, y: float, stamp_sec: float) -> ManagedTarget:
        target = ManagedTarget(
            target_id=self.next_target_id,
            source_id=source_id,
            type=obj_type,
            priority=priority,
            x=x,
            y=y,
            first_seen=stamp_sec,
            last_seen=stamp_sec,
        )
        self.targets[target.target_id] = target
        self.next_target_id += 1

        if self._debug_enabled():
            self.get_logger().info(
                f"Created target_id={target.target_id} from source_id={source_id} "
                f"prio={priority} pos=({x:.2f}, {y:.2f})"
            )
        return target

    def _update_target(self, target: ManagedTarget, source_id: int, obj_type: str, priority: int, x: float, y: float, stamp_sec: float) -> None:
        target.source_id = source_id
        target.type = obj_type
        target.priority = priority
        target.x = x
        target.y = y
        target.last_seen = stamp_sec
        target.seen_count += 1
        if target.status == "stale":
            target.status = "active"

    def _refresh_lifecycle(self, now_sec: float) -> None:
        stale_timeout = self._stale_timeout()
        prune_timeout = self._prune_timeout()
        stale_ids = []
        prune_ids = []

        for target_id, target in self.targets.items():
            age = now_sec - target.last_seen
            if age > prune_timeout:
                prune_ids.append(target_id)
            elif age > stale_timeout and target.status != "stale":
                target.status = "stale"
                stale_ids.append(target_id)

        for target_id in prune_ids:
            del self.targets[target_id]

        if self._debug_enabled():
            for target_id in stale_ids:
                self.get_logger().info(f"Target id={target_id} marked stale.")
            if prune_ids:
                self.get_logger().info(f"Pruned {len(prune_ids)} stale target(s).")

    def frame_callback(self, msg: Frame) -> None:
        source_frame = msg.header.frame_id or self._global_frame()
        stamp_sec = self._stamp_to_seconds(msg.header.stamp)
        self.last_frame_stamp_sec = stamp_sec

        updated_target_ids: Set[int] = set()

        for obj in msg.objects:
            if obj.type != "HEN":
                continue

            x, y = self._transform_point(obj.position.x, obj.position.y, source_frame)
            source_id = int(obj.id)
            priority = int(obj.priority)

            target = self._find_same_source_target(source_id, x, y, updated_target_ids)
            if target is None:
                target = self._find_nearest_target(x, y, stamp_sec, updated_target_ids)
            if target is None:
                target = self._create_target(source_id, obj.type, priority, x, y, stamp_sec)
            else:
                self._update_target(target, source_id, obj.type, priority, x, y, stamp_sec)

            updated_target_ids.add(target.target_id)

        self._refresh_lifecycle(stamp_sec)
        self.publish_targets()

    def publish_targets(self) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if self.last_frame_stamp_sec <= 0.0:
            self._refresh_lifecycle(now_sec)
        else:
            self._refresh_lifecycle(max(now_sec, self.last_frame_stamp_sec))

        msg = TrackedTargetArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._global_frame()

        for target in sorted(self.targets.values(), key=lambda item: item.target_id):
            tracked = TrackedTarget()
            tracked.target_id = int(target.target_id)
            tracked.source_id = int(target.source_id)
            tracked.type = target.type
            tracked.priority = int(target.priority)
            tracked.position = Point(x=float(target.x), y=float(target.y), z=0.0)
            tracked.first_seen = self._seconds_to_stamp(target.first_seen)
            tracked.last_seen = self._seconds_to_stamp(target.last_seen)
            tracked.seen_count = int(target.seen_count)
            tracked.status = target.status
            msg.targets.append(tracked)

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TargetManager()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
