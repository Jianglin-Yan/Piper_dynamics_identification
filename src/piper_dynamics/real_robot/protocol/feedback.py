"""Pure PiPER feedback decoding, with no sockets or runtime side effects."""

import struct


def decode(cid, d):
    if len(d) != 8:
        return {}
    if cid == 0x2A1:
        return dict(
            zip(
                [
                    "control_mode",
                    "arm_status",
                    "motion_mode",
                    "teach_status",
                    "motion_status",
                    "trajectory_index",
                    "error_code",
                ],
                [*d[:6], int.from_bytes(d[6:8], "big")],
            )
        )
    if 0x2A5 <= cid <= 0x2A7:
        return {
            "joint_pair": [2 * (cid - 0x2A5) + 1, 2 * (cid - 0x2A5) + 2],
            "position_deg": [v / 1000 for v in struct.unpack(">ii", d)],
        }
    if 0x251 <= cid <= 0x256:
        v, i, p = struct.unpack(">hhi", d)
        return {
            "joint": cid - 0x250,
            "velocity_sdk_rad_s": v / 1000,
            "current_signed_A": i / 1000,
            "motor_position_raw": p,
        }
    if 0x261 <= cid <= 0x266:
        v, t, mt, s, i = struct.unpack(">HhbBH", d)
        return {
            "joint": cid - 0x260,
            "voltage_V": v / 10,
            "driver_temp_C": t,
            "motor_temp_C": mt,
            "status_raw": s,
            "enabled": bool(s & 64),
            "fault_bits_excluding_enable": s & 191,
            "bus_current_A": i / 1000,
        }
    if cid == 0x473:
        return {
            "joint": d[0],
            "max_deg": int.from_bytes(d[1:3], "big", signed=True) / 10,
            "min_deg": int.from_bytes(d[3:5], "big", signed=True) / 10,
            "max_speed_raw": int.from_bytes(d[5:7], "big"),
        }
    if cid == 0x47C:
        return {"joint": d[0], "max_acc_raw": int.from_bytes(d[1:3], "big")}
    if cid == 0x47B:
        return {"collision_levels": list(d[:6])}
    if cid == 0x2A8:
        return {
            "gripper_position_raw": int.from_bytes(d[:4], "big", signed=True),
            "gripper_effort_raw": int.from_bytes(d[4:6], "big", signed=True),
            "status": d[6],
        }
    return {}
