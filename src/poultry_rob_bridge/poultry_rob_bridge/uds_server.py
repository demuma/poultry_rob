#!/usr/bin/env python3
import os
import signal
import socket
import struct
import threading
import time

from poultry_rob_bridge import dil_frame_pb2 as pb


SOCK_PATH = "/tmp/farm.sock"
FRAME_PERIOD_SEC = 0.2


def send_msg(conn: socket.socket, payload: bytes) -> None:
    header = struct.pack("!I", len(payload))
    conn.sendall(header + payload)


def add_object(frame: pb.Frame, obj_id: int, obj_type: str, priority: int, x: float, y: float) -> None:
    obj = frame.objects.add()
    obj.id = obj_id
    obj.type = obj_type
    obj.priority = priority
    obj.position.x = x
    obj.position.y = y


def build_frame(seq: int) -> pb.Frame:
    frame = pb.Frame()
    frame.header.seq = seq
    frame.header.stamp_unix_ms = int(time.time() * 1000)
    frame.header.frame_id = "camera_optical_frame"

    # Minimal fake-DIL output: tracked hens with detector-owned IDs,
    # detector-owned priorities, and detector coordinates.
    add_object(frame, 1, "HEN", 1, 6.0, -1.0)
    add_object(frame, 2, "HEN", 0, 3.0, 2.0)

    return frame


def prepare_socket_path(path: str) -> None:
    if not os.path.exists(path):
        return

    probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        probe.settimeout(0.2)
        probe.connect(path)
        raise RuntimeError(
            f"{path} is already in use. Stop the running UDS server first."
        )
    except (ConnectionRefusedError, FileNotFoundError):
        pass
    finally:
        probe.close()

    try:
        os.remove(path)
    except PermissionError as exc:
        raise RuntimeError(
            f"Cannot remove stale socket {path}. It is probably owned by a "
            f"container/root process. Remove it once with: sudo rm {path}"
        ) from exc


def main() -> None:
    stop = threading.Event()

    def _handle_sig(_sig, _frm):
        stop.set()

    signal.signal(signal.SIGINT, _handle_sig)
    signal.signal(signal.SIGTERM, _handle_sig)

    prepare_socket_path(SOCK_PATH)

    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(SOCK_PATH)
    os.chmod(SOCK_PATH, 0o666)
    srv.listen(1)
    srv.settimeout(1.0)

    print(
        f"[server] listening on {SOCK_PATH} minimal_fake_dil=true "
        "(Ctrl+C to stop)"
    )

    conn = None
    try:
        while not stop.is_set():
            try:
                conn, _ = srv.accept()
                print("[server] client connected")
                conn.settimeout(1.0)
            except socket.timeout:
                continue

            seq = 1
            while not stop.is_set():
                try:
                    frame = build_frame(seq)
                    send_msg(conn, frame.SerializeToString())
                    if seq == 1 or seq % 10 == 0:
                        print(f"[server] seq={seq} objects={len(frame.objects)}")
                    seq += 1

                    if stop.wait(FRAME_PERIOD_SEC):
                        break

                except (BrokenPipeError, ConnectionResetError, socket.timeout):
                    print("[server] client disconnected")
                    break

            try:
                conn.close()
            except Exception:
                pass
            conn = None

    finally:
        if conn is not None:
            try:
                conn.close()
            except Exception:
                pass
        try:
            srv.close()
        except Exception:
            pass
        try:
            prepare_socket_path(SOCK_PATH)
        except RuntimeError as exc:
            print(f"[server] warning: {exc}")
        print("[server] stopped cleanly")


if __name__ == "__main__":
    main()
