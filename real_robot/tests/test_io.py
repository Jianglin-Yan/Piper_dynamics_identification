import json, threading, time, types, unittest
from pathlib import Path
import numpy as np
from real_robot.gamepad import Gamepad, Chords
from real_robot.hardware import Hardware

class FakeDevice:
    def __init__(self,events,keys=()):self.events=events;self.keys=keys
    def read(self):return iter(self.events)
    def active_keys(self):return self.keys

class TestQueuedButtons(unittest.TestCase):
    def make(self,events):
        p=Gamepad.__new__(Gamepad);p.keys=set();p.evdev=types.SimpleNamespace(ecodes=types.SimpleNamespace(EV_KEY=1,EV_SYN=0,SYN_DROPPED=3))
        p.chords=Chords({'LB':310,'RB':311,'X':308,'Y':307});p.chords.update(set(),0)
        p.dev=FakeDevice([types.SimpleNamespace(type=1,code=c,value=v) for c,v in events]);return p
    def test_short_press_and_release_in_one_read(self):
        p=self.make([(310,1),(308,1),(308,0),(310,0)])
        self.assertEqual(p.poll(1),'PREPARE')
    def test_abort_beats_start_in_same_batch(self):
        p=self.make([(310,1),(307,1),(311,1),(311,0),(307,0),(310,0)])
        self.assertEqual(p.poll(1),'ABORT')
    def test_disconnect_propagates(self):
        p=self.make([])
        def removed():raise OSError('device disconnected')
        p.dev.active_keys=removed
        with self.assertRaises(OSError):p.poll(1)

class TestHardwarePackets(unittest.TestCase):
    def setUp(self):
        self.h=Hardware.__new__(Hardware);self.h.tx_lock=threading.Lock();self.h.stopped=False;self.h.active=False;self.h.c={'move_speed_percent':10};self.frames=[]
        self.h._send_locked=lambda cid,data:self.frames.append((cid,bytes(data)))
    def test_position_mode_enable_and_stop_match_official_parser(self):
        import can
        from piper_sdk.protocol.protocol_v2.piper_protocol_v2 import C_PiperParserV2
        from piper_sdk.piper_msgs.msg_v2 import PiperMessage, ArmMsgType
        parser=C_PiperParserV2();q=np.deg2rad([1.234,45,-60,-3.2,20,6.7]);self.h.arm(q)
        expected=[]
        for msgtype in ('PiperMsgJointCtrl_12','PiperMsgJointCtrl_34','PiperMsgJointCtrl_56'):
            msg=PiperMessage();msg.type_=getattr(ArmMsgType,msgtype)
            for j,value in enumerate(np.rint(np.rad2deg(q)*1000).astype(int),1):setattr(msg.arm_joint_ctrl,'joint_'+str(j),int(value))
            frame=can.Message();parser.EncodeMessage(msg,frame);expected.append((frame.arbitration_id,bytes(frame.data)))
        msg=PiperMessage();msg.type_=ArmMsgType.PiperMsgMotionCtrl_2
        msg.arm_motion_ctrl_2.ctrl_mode=1;msg.arm_motion_ctrl_2.move_mode=1;msg.arm_motion_ctrl_2.move_spd_rate_ctrl=10;msg.arm_motion_ctrl_2.mit_mode=0;msg.arm_motion_ctrl_2.residence_time=0;msg.arm_motion_ctrl_2.installation_pos=0
        frame=can.Message();parser.EncodeMessage(msg,frame);self.assertEqual(self.frames[0],(frame.arbitration_id,bytes(frame.data)))
        self.assertEqual(self.frames[1:4],expected);self.assertEqual(self.frames[4:7],expected)
        for j in range(1,7):
            msg=PiperMessage();msg.type_=ArmMsgType.PiperMsgMotorEnableDisableConfig;msg.arm_motor_enable.motor_num=j;msg.arm_motor_enable.enable_flag=2
            frame=can.Message();parser.EncodeMessage(msg,frame);self.assertEqual(self.frames[6+j],(frame.arbitration_id,bytes(frame.data)))
        self.h.quick_stop('test');msg=PiperMessage();msg.type_=ArmMsgType.PiperMsgMotionCtrl_1
        msg.arm_motion_ctrl_1.emergency_stop=1;msg.arm_motion_ctrl_1.track_ctrl=0;msg.arm_motion_ctrl_1.grag_teach_ctrl=0
        frame=can.Message();parser.EncodeMessage(msg,frame);self.assertEqual(self.frames[-1],(frame.arbitration_id,bytes(frame.data)))
    def test_each_position_target_refreshes_mode_and_speed(self):
        self.h.command(np.zeros(6))
        self.assertEqual([cid for cid,data in self.frames],[0x151,0x155,0x156,0x157])
        self.h.begin_native_move(np.zeros(6));self.assertEqual(self.frames[-4][1][2],2)
        self.h.finish_native_move();self.h.command(np.zeros(6));self.assertEqual(self.frames[-4][1][2],10)
    def test_latched_stop_suppresses_future_position_and_enable(self):
        self.h.quick_stop('test');n=len(self.frames)
        for fn in (self.h.command,self.h.arm):
            with self.assertRaises(RuntimeError):fn(np.zeros(6))
        self.assertEqual(len(self.frames),n)
    def test_watchdog_stops_even_without_control_loop(self):
        self.h.c['loop_watchdog_s']=.02;self.h.done=threading.Event();self.h.error=None;self.h.active=True;self.h.heartbeat=time.monotonic()-.1
        t=threading.Thread(target=self.h._watchdog);t.start();time.sleep(.04);self.h.done.set();t.join()
        self.assertTrue(self.h.stopped);self.assertEqual(self.frames[-1][0],0x150)
    def test_read_query_whitelist_rejects_motion(self):
        with self.assertRaises(ValueError):self.h.query(0x471,[1,2,0,0,0,0,0,0])
        self.assertEqual(self.frames,[])

if __name__=='__main__':unittest.main()
