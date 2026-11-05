"""PiPER raw SocketCAN capture. Only explicitly whitelisted query frames can be sent.
No SDK connection side effects, no enable/motion/reset/configuration commands.
Raw frames are authoritative; decoded fields follow piper_sdk 0.6.2 protocol_v2.
Current is signed as in the parser; no firmware-dependent torque scaling is guessed.
"""

#!/usr/bin/env python3
import argparse
import collections
import datetime
import json
import socket
import struct
import time
from pathlib import Path

from piper_dynamics.paths import RUNS
from piper_dynamics.real_robot.protocol.feedback import decode


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--channel", default="can0")
    ap.add_argument("--seconds", type=float, default=10)
    ap.add_argument(
        "--query", action="store_true", help="Also send firmware/limit/protection read queries"
    )
    ap.add_argument("--output", type=Path, default=RUNS / "captures")
    a = ap.parse_args()
    if not 1 <= a.seconds <= 3600:
        ap.error("seconds must be 1..3600")
    root = a.output / datetime.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    root.mkdir(parents=True)
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024 * 1024)
    sock.bind((a.channel,))
    sock.settimeout(0.1)
    count = collections.Counter()
    latest = {}
    samples = collections.defaultdict(list)
    fw = []
    replies = {}
    tx = []
    t0 = time.monotonic()
    log = (root / "raw_frames.jsonl").open("w")
    _passive = []

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            try:
                raw = sock.recv(16)
            except socket.timeout:
                continue
            if len(raw) != 16:
                continue
            cid, n, d = struct.unpack("=IB3x8s", raw)
            now = time.monotonic()
            d = d[:n]
            item = {
                "t_monotonic_s": now,
                "t_host_unix_ns": time.time_ns(),
                "can_id": hex(cid),
                "data_hex": d.hex(),
            }
            log.write(json.dumps(item) + "\n")
            count[cid] += 1
            samples[cid].append(now)
            value = decode(cid, d)
            if value:
                latest[hex(cid)] = {"t_monotonic_s": now, **value}
            if cid == 0x4AF:
                fw.append(d)
            if cid in (0x473, 0x47C) and value:
                replies[(cid, d[0])] = {"t_monotonic_s": now, **value}

    allowed = {(0x4AF, bytes([1, 0, 0, 0, 0, 0, 0, 0])), (0x477, bytes([2, 0, 0, 0, 3, 0, 0, 0]))}
    allowed.update((0x472, bytes([j, k, 0, 0, 0, 0, 0, 0])) for j in range(1, 7) for k in (1, 2))

    def send(cid, d):
        if (cid, d) not in allowed:
            raise RuntimeError("non-query frame forbidden")
        now = time.monotonic()
        sock.send(struct.pack("=IB3x8s", cid, 8, d))
        tx.append({"t_monotonic_s": now, "can_id": hex(cid), "data_hex": d.hex()})
        return now

    try:
        pump(a.seconds)
        passive_count = dict(count)
        passive_elapsed = time.monotonic() - t0
        if a.query:
            send(0x4AF, bytes([1, 0, 0, 0, 0, 0, 0, 0]))
            pump(1)
            for j in range(1, 7):
                for kind, replyid in [(1, 0x473), (2, 0x47C)]:
                    sent = send(0x472, bytes([j, kind, 0, 0, 0, 0, 0, 0]))
                    pump(0.25)
                    reply = replies.get((replyid, j))
                    if reply is None or reply["t_monotonic_s"] < sent:
                        replies.pop((replyid, j), None)
            send(0x477, bytes([2, 0, 0, 0, 3, 0, 0, 0]))
            pump(0.5)
    finally:
        log.close()
        sock.close()
    stats = {
        hex(cid): {
            "count": n,
            "passive_hz": passive_count.get(cid, 0) / passive_elapsed,
            "max_host_gap_ms": max(
                [0] + [(y - x) * 1000 for x, y in zip(samples[cid], samples[cid][1:])]
            ),
        }
        for cid, n in sorted(count.items())
    }
    result = {
        "note": "No enable, mode, motion or parameter-write commands sent. Host receive timestamps, not hardware timestamps. No calibrated torque yet.",
        "channel": a.channel,
        "duration_s": time.monotonic() - t0,
        "firmware_ascii": b"".join(fw).decode("ascii", errors="replace").strip("\x00"),
        "frame_stats": stats,
        "latest": latest,
        "joint_limits": {str(j): replies.get((0x473, j)) for j in range(1, 7)},
        "joint_acc_limits": {str(j): replies.get((0x47C, j)) for j in range(1, 7)},
        "transmitted_queries": tx,
    }
    (root / "summary.json").write_text(json.dumps(result, ensure_ascii=False, indent=2))
    print(root)
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()

__all__ = ["decode", "main"]
