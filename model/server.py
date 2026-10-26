"""Serve the motion policy to Unreal over a local TCP socket.

Every physics tick Unreal sends the avatar's observation and waits for the joint actions, so the
loop is lockstep. Messages are a 4-byte big-endian length followed by UTF-8 JSON.

    python model/server.py        # listens on 127.0.0.1:41911
"""
import argparse
import json
import os
import socket
import struct
import time

import torch

from policy import OBS_SIZE, MotionPolicy

DEFAULT_WEIGHTS = os.path.join(os.path.dirname(os.path.abspath(__file__)), "weights", "policy.pt")
# Below Windows' dynamic port range, which Hyper-V and WSL reserve blocks of.
DEFAULT_PORT = 41911
MAX_PAYLOAD = 4 << 20
# Windows does not interrupt blocking socket calls on Ctrl+C, so they wake this often to let it in.
WAKE_SECONDS = 0.5


def recv_exact(conn: socket.socket, size: int) -> bytes | None:
    data = b""
    while len(data) < size:
        try:
            chunk = conn.recv(size - len(data))
        except TimeoutError:
            continue
        if not chunk:
            return None
        data += chunk
    return data


def recv_message(conn: socket.socket) -> dict | None:
    header = recv_exact(conn, 4)
    if header is None:
        return None
    (size,) = struct.unpack("!I", header)
    if size == 0 or size > MAX_PAYLOAD:
        raise ValueError("bad frame length %d" % size)
    body = recv_exact(conn, size)
    return None if body is None else json.loads(body.decode("utf-8"))


def send_message(conn: socket.socket, message: dict) -> None:
    payload = json.dumps(message).encode("utf-8")
    conn.sendall(struct.pack("!I", len(payload)) + payload)


def serve_session(conn: socket.socket, policy: MotionPolicy) -> int:
    """Drive one Play session until Unreal hangs up. Returns the number of actions served."""
    policy.reset()
    served = 0
    try:
        while True:
            message = recv_message(conn)
            if message is None:
                break
            if "observations" not in message:
                continue
            obs = torch.tensor(message["observations"], dtype=torch.float32, device=policy.device)
            if obs.shape[-1] != OBS_SIZE:
                raise ValueError("observation has %d values, the policy expects %d"
                                 % (obs.shape[-1], OBS_SIZE))
            actions = policy.act(obs)
            send_message(conn, {
                "timestamp": time.time(),
                "step": message["step"],
                "env_ids": message.get("env_ids", [0]),
                "actions": actions.cpu().tolist(),
            })
            served += 1
    except ConnectionError:
        pass  # Unreal closed without stopping Play first
    except ValueError as error:
        print("Session ended: %s" % error, flush=True)
    return served


def main() -> None:
    parser = argparse.ArgumentParser(description="Serve the motion policy to Unreal.")
    parser.add_argument("--host", default="127.0.0.1",
                        help="address to listen on; 0.0.0.0 to accept UE from another machine")
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--weights", default=DEFAULT_WEIGHTS)
    parser.add_argument("--device", default="cpu",
                        help="cpu is plenty: one inference takes a few ms; 'cuda' also works")
    args = parser.parse_args()

    policy = MotionPolicy(args.weights, device=args.device)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        server.bind((args.host, args.port))
    except OSError as error:
        raise SystemExit(
            "Cannot listen on %s:%d (%s).\nIf another server is already running, reuse it; "
            "otherwise pick a free port with --port and set the same one in Unreal "
            "(see README)." % (args.host, args.port, error))
    server.listen(1)
    server.settimeout(WAKE_SECONDS)
    print("Motion model ready on %s:%d (%s). Press Play in Unreal; Ctrl+C to stop."
          % (args.host, args.port, args.device), flush=True)

    try:
        while True:
            try:
                conn, address = server.accept()
            except TimeoutError:
                continue
            conn.settimeout(WAKE_SECONDS)
            conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print("Unreal connected from %s:%d" % address, flush=True)
            started = time.time()
            try:
                served = serve_session(conn, policy)
            finally:
                conn.close()
            elapsed = max(time.time() - started, 1e-9)
            print("Unreal disconnected after %d steps (%.1f Hz). Waiting for the next Play."
                  % (served, served / elapsed), flush=True)
    except KeyboardInterrupt:
        print("Stopped.", flush=True)
    finally:
        server.close()


if __name__ == "__main__":
    main()
