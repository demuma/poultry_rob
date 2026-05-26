#!/usr/bin/env python3
import socket
import struct
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Point

from poultry_rob_bridge import dil_frame_pb2 as pb

from poultry_rob_bridge_msgs.msg import Frame
from poultry_rob_bridge_msgs.msg import Object as ObjectMsg


SOCK_PATH = "/tmp/farm.sock"


def recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("socket closed")
        buf += chunk
    return buf


def recv_msg(sock: socket.socket) -> bytes:
    header = recv_exact(sock, 4)
    (length,) = struct.unpack("!I", header)
    if length <= 0 or length > 50_000_000:
        raise ValueError(f"invalid message length: {length}")
    return recv_exact(sock, length)


class UdsProtoToRosBridge(Node):
    def __init__(self, context: Optional[Context] = None):
        super().__init__("uds_proto_bridge", context=context)

        self.publisher = self.create_publisher(Frame, "/dil/frame", 10)

        self._lock = threading.Lock()
        self._latest: Optional[pb.Frame] = None
        self._last_published_seq = 0

        self._stop_event = threading.Event()
        self._reader = threading.Thread(target=self._socket_reader, daemon=True)
        self._reader.start()

        self.timer = self.create_timer(0.1, self._publish_if_new)

        self.get_logger().info("UDS → ROS2 bridge started")

    def _socket_reader(self):
        while not self._stop_event.is_set():
            try:
                s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                s.connect(SOCK_PATH)
                self.get_logger().info("Connected to UDS")

                while not self._stop_event.is_set():
                    payload = recv_msg(s)
                    frame = pb.Frame()
                    frame.ParseFromString(payload)

                    with self._lock:
                        self._latest = frame

            except Exception as e:
                self.get_logger().warn(f"UDS error: {e}, reconnecting...")
            finally:
                try:
                    s.close()
                except Exception:
                    pass

    def _publish_if_new(self) -> None:
        """Publish latest frame if seq changed since last publish."""
        with self._lock:
            frame = self._latest

        if frame is None:
            return

        seq = int(frame.header.seq)
        if seq == self._last_published_seq:
            return

        msg = Frame()

        # std_msgs/Header
        stamp_s = frame.header.stamp_unix_ms / 1000.0
        msg.header.stamp.sec = int(stamp_s)
        msg.header.stamp.nanosec = int((stamp_s % 1) * 1e9)
        msg.header.frame_id = frame.header.frame_id

        # top-level seq field
        msg.seq = seq

        # Object[]
        for proto_obj in frame.objects:
            ros_obj = ObjectMsg()
            ros_obj.id = proto_obj.id
            ros_obj.type = proto_obj.type
            ros_obj.priority = proto_obj.priority
            ros_obj.position = Point(
                x=float(proto_obj.position.x),
                y=float(proto_obj.position.y),
                z=0.0,
            )
            msg.objects.append(ros_obj)

        self.publisher.publish(msg)
        self._last_published_seq = seq

    def stop(self):
        self._stop_event.set()

    def destroy_node(self):
        self.stop()
        if self._reader.is_alive():
            self._reader.join(timeout=1.0)
        super().destroy_node()


def main():
    ctx = Context()
    rclpy.init(context=ctx)

    node = UdsProtoToRosBridge(context=ctx)

    executor = SingleThreadedExecutor(context=ctx)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown(context=ctx)


if __name__ == "__main__":
    main()
