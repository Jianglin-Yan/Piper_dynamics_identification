import contextlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from real_robot.controller import Controller
from real_robot.standing import STANDING_DEG
from real_robot.session_history import previous_controller_session,require_recovered
from real_robot.tests.test_pilot import Fake,config,tick


class ClampingArm(Fake):
    """Firmware-like target clipping, bounded following, and persistent state across clients."""
    def __init__(self,c):
        super().__init__(c);self.lower=np.deg2rad(c['lower_deg']);self.upper=np.deg2rad(c['upper_deg']);self.native_active=False
    def command(self,q):
        self.commands.append(np.array(q).copy());goal=np.clip(q,self.lower,self.upper)
        step=np.clip(goal-self.q,-.12*.01,.12*.01);self.v=step/.01;self.q+=step
    def begin_native_move(self,q):self.native_active=True;self.command(q)
    def finish_native_move(self):self.native_active=False


class TestLegalReturnAndRestart(unittest.TestCase):
    def make(self):
        c=config();c.update(standing_start=True,current_pose_only=False,start_deg=STANDING_DEG,simulation_initial_deg=[2.614,-2.618,2.798,5.159,19.415,7.411])
        b=ClampingArm(c);messages=[];k=Controller(b,c,b.snapshot(),0,messages.append)
        return c,b,k,messages
    def run_session(self,b,k,abort=False):
        started=False;returned=[];aborted=False
        for n in range(9000):
            t=n*.01;action='PREPARE' if n==1 else None
            if k.state=='READY' and not started:action='START';started=True
            if abort and k.state=='TESTING' and t-k.path_t>2 and not aborted:action='ABORT';aborted=True
            tick(k,t,b.snapshot(),action)
            if k.state in ('RETURNING','SETTLING_HOME'):returned.append(k.last_target.copy())
            if k.state in ('DONE','FAULT','HELD'):break
        self.assertTrue(started);self.assertEqual(k.state,'DONE');self.assertEqual(b.stops,[])
        self.assertFalse(b.enabled);self.assertFalse(b.active);self.assertTrue(k.disarm_confirmed)
        self.assertTrue(returned)
        self.assertTrue(np.all(np.array(returned)>=b.lower-1e-9));self.assertTrue(np.all(np.array(returned)<=b.upper+1e-9))
    def test_real_clipped_return_reaches_done_without_electronic_stop(self):
        c,b,k,m=self.make();home=k.home.copy();self.run_session(b,k)
        np.testing.assert_allclose(k.home,home);np.testing.assert_allclose(b.q,k.return_home,atol=1e-8)
        np.testing.assert_allclose(np.rad2deg(k.return_home-home),[0,2.618,-2.798,0,0,0],atol=1e-8)
        self.assertTrue(any('RETURN_HOME' in x and '差' in x for x in m))
    def test_two_sessions_rearm_disabled_arm_without_reset(self):
        c,b,k,m=self.make();self.run_session(b,k)
        next_session=Controller(b,c,b.snapshot(),0,m.append)
        np.testing.assert_allclose(next_session.home,k.return_home,atol=1e-8)
        self.run_session(b,next_session)
    def test_abort_return_uses_same_legal_goal(self):
        c,b,k,m=self.make();self.run_session(b,k,abort=True);self.assertTrue(k.aborted)


class TestHistory(unittest.TestCase):
    def setUp(self):self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name);self.index=0
    def tearDown(self):self.tmp.cleanup()
    def session(self,state,frames,mode='hardware',channel='can0'):
        self.index+=1;p=self.root/f'20261029_0900{self.index:02d}_000000';p.mkdir()
        (p/'config.json').write_text(json.dumps({'channel':channel}));(p/'tx_can.jsonl').write_text(''.join(json.dumps({'can_id':cid,'data_hex':data})+'\n' for cid,data in frames))
        if state is not None:(p/'result.json').write_text(json.dumps({'mode':mode,'state':state}))
        return p
    def history(self):return previous_controller_session([self.root],'can0')
    def test_electronic_stop_blocks_restart(self):
        p=self.session('FAULT',[('0x471','0102000000000000'),('0x150','0100000000000000')])
        h=self.history();self.assertEqual(h['run'],str(p));self.assertTrue(h['requires_recovery'])
        with self.assertRaisesRegex(RuntimeError,'RECOVERY_REQUIRED'):require_recovered(h)
    def test_successful_explicit_recovery_clears_prior_stop(self):
        self.session('FAULT',[('0x150','0100000000000000')]);self.session('RECOVERED',[('0x150','0200000000000000')],mode='recovery')
        self.assertFalse(self.history()['requires_recovery']);require_recovered(self.history())
    def test_normal_and_held_handoffs_do_not_require_reset(self):
        for state in ('DONE','HELD'):
            self.session(state,[('0x471','0102000000000000'),('0x157','0000000000000000')])
            self.assertFalse(self.history()['requires_recovery'])
    def test_confirmed_disabled_fault_needs_no_reset_unless_electronic_stop_was_sent(self):
        for stopped in (False,True):
            frames=[('0x471','0101000000000000')]
            if stopped:frames.insert(0,('0x150','0100000000000000'))
            p=self.session('FAULT',frames)
            (p/'result.json').write_text(json.dumps({'mode':'hardware','state':'FAULT','motors_disabled_confirmed':True}))
            self.assertEqual(self.history()['requires_recovery'],stopped)
    def test_readonly_preflight_and_other_channel_cannot_hide_stop(self):
        p=self.session('FAULT',[('0x150','0100000000000000')]);self.session('PREFLIGHT_ONLY',[('0x472','0101000000000000')]);self.session('DONE',[('0x157','0000000000000000')],channel='can1')
        self.assertEqual(self.history()['run'],str(p));self.assertTrue(self.history()['requires_recovery'])
    def test_unfinished_or_failed_recovery_cannot_clear_stop(self):
        self.session('FAULT',[('0x150','0100000000000000')]);self.session(None,[('0x150','0200000000000000')],mode='recovery')
        self.assertTrue(self.history()['requires_recovery'])
    def test_firmware_stop_record_overrides_incorrect_done_label(self):
        self.session('DONE',[('0x150','0100000000000000')]);self.assertTrue(self.history()['requires_recovery'])
    def test_excluded_current_session_does_not_mask_previous(self):
        old=self.session('FAULT',[('0x150','0100000000000000')]);new=self.session('RECOVERED',[('0x150','0200000000000000')])
        self.assertEqual(previous_controller_session([self.root],'can0',exclude=new)['run'],str(old))
    def test_guard_runs_before_can_or_pad_is_opened(self):
        self.session('FAULT',[('0x150','0100000000000000')],channel='test-can0')
        c=config();c['channel']='test-can0';config_path=self.root/'custom.json';config_path.write_text(json.dumps(c))
        from real_robot.run import main
        with patch('sys.argv',['pilot','--hardware','--standing','--config',str(config_path),'--output',str(self.root)]),patch('real_robot.run.BASE',self.root/'real_robot'),patch('real_robot.hardware.Hardware') as hardware,patch('real_robot.run.Gamepad') as pad,contextlib.redirect_stdout(io.StringIO()):
            with self.assertRaises(SystemExit) as e:main()
        self.assertEqual(e.exception.code,2);hardware.assert_not_called();pad.assert_not_called()


if __name__=='__main__':unittest.main()
