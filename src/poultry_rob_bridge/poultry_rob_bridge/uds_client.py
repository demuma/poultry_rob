#!/usr/bin/env python3
import socket
import struct
from typing import Optional

from poultry_rob_bridge import dil_frame_pb2 as pb


SOCK_PATH = "/tmp/farm.sock"


def recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("socket closed while receiving")
        buf += chunk
    return buf


def recv_msg(sock: socket.socket) -> bytes:
    # Read 4-byte big-endian length prefix
    header = recv_exact(sock, 4)
    (length,) = struct.unpack("!I", header)
    if length <= 0 or length > 50_000_000:
        raise ValueError(f"invalid message length: {length}")
    return recv_exact(sock, length)


def main() -> None:
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    print(f"[client] connecting to {SOCK_PATH} ...")
    s.connect(SOCK_PATH)
    print("[client] connected")

    try:
        while True:
            payload = recv_msg(s)
            frame = pb.Frame()
            frame.ParseFromString(payload)

            print(
                f"[client] got frame seq={frame.header.seq} "
                f"t={frame.header.stamp_unix_ms} "
                f"frame_id={frame.header.frame_id} "
                f"objects={len(frame.objects)}"
            )

            # Print first few objects
            for obj in frame.objects[:5]:
                print(f"  - id={obj.id} type={obj.type} pos=({obj.position.x:.2f},{obj.position.y:.2f})")

    except KeyboardInterrupt:
        print("\n[client] bye")
    finally:
        s.close()


if __name__ == "__main__":
    main()
