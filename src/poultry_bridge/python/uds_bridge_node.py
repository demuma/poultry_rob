#!/usr/bin/env python3
import socket
import struct
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String

import test_frame_pb2 as pb


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

        self.publisher = self.create_publisher(String, "/farm/test_frame", 10)

        self._lock = threading.Lock()
        self._latest: Optional[pb.TestFrame] = None
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
                    frame = pb.TestFrame()
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

        # Create a compact debug string
        parts = [
            f"seq={seq}",
            f"t_ms={int(frame.header.stamp_unix_ms)}",
            f"frame={frame.header.frame_id}",
            f"n={len(frame.objects)}",
        ]
        for obj in frame.objects[:5]:
            parts.append(
                f"{obj.type}#{obj.id}@({obj.position.x:.2f},{obj.position.y:.2f})"
            )

        msg = String()
        msg.data = " | ".join(parts)
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