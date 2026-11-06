"""Regression: dropped CAN packets must never kill disarm feedback reception."""

import io
import json
import queue
import socket
import struct
import threading
import time
import unittest

import numpy as np

from piper_dynamics.real_robot.backends.hardware import Hardware
from piper_dynamics.real_robot.shutdown import Shutdown


def packet(cid=0x261, status=0):
    return struct.pack("=IB3x8s", cid, 8, struct.pack(">HhbBH", 240, 30, 30, status, 0))


def ancillary(count):
    return [(socket.SOL_SOCKET, 40, struct.pack("=I", count))]


class TestCanReceive(unittest.TestCase):
    def run_receiver(self, events, capacity=32):
        h = Hardware.__new__(Hardware)
        h.done = threading.Event()
        h.log_done = h.done
        h.rx_lock = threading.Lock()
        h.latest = {}
        h.limits = {}
        h.acc = {}
        h.firmware = bytearray()
        h.error = None
        h.rx_dropped = 0
        h.raw_log_dropped = 0
        h.raw_queue = queue.Queue(capacity)
        it = iter(events)

        class Socket:
            def recvmsg(self, *args):
                try:
                    return next(it)
                except StopIteration:
                    h.done.set()
                    raise socket.timeout()

        h.sock = Socket()
        h._receive()
        return h

    def test_overflow_counts_delta_and_continues_receiving_all_six_disable_feedback(self):
        events = [
            (packet(0x261 + i), ancillary(n), 0, None) for i, n in enumerate([7, 7, 9, 9, 9, 9])
        ]
        start = time.monotonic()
        h = self.run_receiver(events)
        self.assertIsNone(h.error)
        self.assertEqual(h.rx_dropped, 9)
        self.assertEqual(len(h.latest), 6)
        self.assertEqual([r["rx_dropped_delta"] for r in h.raw_queue.queue], [7, 0, 2, 0, 0, 0])
        observation = {
            "low_times": np.array([h.latest[cid]["time"] for cid in range(0x261, 0x267)]),
            "enabled": np.array([h.latest[cid]["enabled"] for cid in range(0x261, 0x267)]),
        }
        self.assertTrue(
            Shutdown.fresh_disabled(
                type("Policy", (), {"c": {"status_timeout_s": 0.15}})(),
                observation,
                time.monotonic(),
                start,
            )
        )

    def test_bus_error_remains_fatal_but_receiver_survives_to_observe_disable(self):
        h = self.run_receiver([(packet(0x20000000), [], 0, None), (packet(0x266), [], 0, None)])
        self.assertEqual(h.error, "CAN error frame")
        self.assertFalse(h.latest[0x266]["enabled"])

    def test_recording_queue_full_does_not_prevent_fresh_disable_feedback(self):
        h = self.run_receiver([(packet(0x261 + i), [], 0, None) for i in range(6)], capacity=1)
        self.assertEqual(h.error, "CAN raw logging queue full")
        self.assertEqual(h.raw_log_dropped, 5)
        self.assertEqual(len(h.latest), 6)

    def test_writer_drains_queue_on_close(self):
        h = self.run_receiver([(packet(), ancillary(2), 0, None)])
        h.raw = io.StringIO()
        h._write_raw()
        self.assertEqual(json.loads(h.raw.getvalue())["rx_dropped_delta"], 2)
        self.assertTrue(h.raw_queue.empty())
        self.assertIsNone(h.error)

    def test_missing_overflow_ancillary_does_not_reset_cumulative_counter(self):
        h = self.run_receiver(
            [
                (packet(), ancillary(5), 0, None),
                (packet(), [], 0, None),
                (packet(), ancillary(5), 0, None),
            ]
        )
        self.assertEqual([r["rx_dropped_delta"] for r in h.raw_queue.queue], [5, 0, 0])


if __name__ == "__main__":
    unittest.main()
