"""Export actual asynchronous measurements. Does not invent synchronized samples or torque."""

#!/usr/bin/env python3
import argparse
import csv
import json
from pathlib import Path

from piper_dynamics.real_robot.protocol.feedback import decode


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("raw", type=Path)
    a = p.parse_args()
    streams = {}
    writers = {}
    counts = {}
    specs = {
        "position": ["t_monotonic_s", "joint", "q_deg"],
        "drive": [
            "t_monotonic_s",
            "joint",
            "qd_sdk_rad_s",
            "current_signed_A",
            "motor_position_raw",
        ],
        "status": ["t_monotonic_s", "control_mode", "arm_status", "error_code"],
    }
    try:
        for name, fields in specs.items():
            streams[name] = (a.raw.parent / (name + ".csv")).open("w", newline="")
            writers[name] = csv.writer(streams[name])
            writers[name].writerow(fields)
            counts[name] = 0
        for line in a.raw.open():
            r = json.loads(line)
            cid = int(r["can_id"], 16)
            d = decode(cid, bytes.fromhex(r["data_hex"]))
            t = r["t_monotonic_s"]
            if 0x2A5 <= cid <= 0x2A7 and d:
                for j, q in zip(d["joint_pair"], d["position_deg"]):
                    writers["position"].writerow([t, j, q])
                    counts["position"] += 1
            elif 0x251 <= cid <= 0x256 and d:
                writers["drive"].writerow(
                    [
                        t,
                        d["joint"],
                        d["velocity_sdk_rad_s"],
                        d["current_signed_A"],
                        d["motor_position_raw"],
                    ]
                )
                counts["drive"] += 1
            elif cid == 0x2A1 and d:
                writers["status"].writerow([t, d["control_mode"], d["arm_status"], d["error_code"]])
                counts["status"] += 1
    finally:
        for f in streams.values():
            f.close()
    print(json.dumps(counts))


if __name__ == "__main__":
    main()
