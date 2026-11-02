"""Signals must provide an operator-owned safe exit, including fault cleanup."""
import threading,unittest
from unittest.mock import Mock,patch
import numpy as np
from real_robot.operator_exit import OperatorExit
from real_robot.controller import Controller
from real_robot.hardware import Hardware
from real_robot.tests.test_pilot import Fake,config,tick

class LostAck(Fake):
 def disable(self):self.disables+=1;self.active=False

class OperatorExitTests(unittest.TestCase):
 def make(self,lost_ack=False):
  c=config();b=(LostAck if lost_ack else Fake)(c);messages=[];k=Controller(b,c,b.snapshot(),0,messages.append)
  return c,b,k,messages,OperatorExit(messages.append)
 def fault_far(self,b,k):
  b.enabled=True;b.active=True;k.active=True;k.armed=True;b.q[1]+=.4;k.fault('injected fault')
 def test_first_signal_requests_normal_stop(self):
  c,b,k,m,o=self.make();o.signal(2,None);o.update(k,b.snapshot(),.1)
  self.assertEqual(k.shutdown_phase,'VERIFY_PARK');self.assertFalse(k.operator_supported_disarm)
 def test_fault_confirmed_support_disables_at_current_pose_without_return(self):
  c,b,k,m,o=self.make();self.fault_far(b,k);before=b.q.copy();commands=len(b.commands)
  o.signal(2,None);o.update(k,b.snapshot(),.1)
  self.assertEqual(b.disables,0);self.assertTrue(any('SUPPORTED_EXIT' in x for x in m))
  o.signal(2,None);o.update(k,b.snapshot(),.2)
  self.assertEqual(b.disables,1);self.assertTrue(k.operator_supported_disarm);np.testing.assert_array_equal(b.q,before)
  for n in range(21,45):tick(k,n*.01,b.snapshot())
  self.assertTrue(k.finished);self.assertTrue(k.disarm_confirmed);self.assertFalse(b.enabled)
  self.assertEqual(len(b.commands),commands);self.assertEqual(k.state,'FAULT')
 def test_burst_before_prompt_cannot_claim_operator_support(self):
  c,b,k,m,o=self.make();self.fault_far(b,k)
  for _ in range(30):o.signal(2,None)
  o.update(k,b.snapshot(),.1);o.update(k,b.snapshot(),.11)
  self.assertEqual(b.disables,0)
  o.signal(2,None);o.update(k,b.snapshot(),.2);self.assertEqual(b.disables,1)
 def test_communication_loss_needs_typed_power_cut_confirmation_not_more_interrupts(self):
  c,b,k,m,o=self.make(True);self.fault_far(b,k)
  o.signal(2,None);o.update(k,b.snapshot(),.1);o.signal(2,None);o.update(k,b.snapshot(),.2)
  for n in range(21,260):tick(k,n*.01,b.snapshot())
  self.assertEqual(k.shutdown_phase,'DISABLE_UNCONFIRMED');self.assertFalse(k.finished)
  for n in range(5):o.signal(2,None);o.update(k,b.snapshot(),2.7+n*.01)
  self.assertFalse(k.finished);self.assertFalse(k.operator_power_off_confirmed)
  with patch.object(o,'_command',return_value='POWER_OFF'):o.update(k,b.snapshot(),2.9)
  self.assertTrue(k.finished);self.assertTrue(k.operator_power_off_confirmed);self.assertFalse(k.disarm_confirmed)
  self.assertEqual(k.state,'FAULT');self.assertTrue(b.enabled) # Cached enabled bit must not be forged.
 def test_power_cut_keyword_is_not_honored_before_disable_timeout(self):
  c,b,k,m,o=self.make();self.fault_far(b,k)
  with patch.object(o,'_command',return_value='POWER_OFF'):o.update(k,b.snapshot(),.1)
  self.assertFalse(k.finished);self.assertFalse(k.operator_power_off_confirmed);self.assertEqual(b.disables,0)
 def test_adapter_only_permits_unconfirmed_close_on_explicit_escape(self):
  h=Hardware.__new__(Hardware);h.active=False;h.motion_started=True;h.disable_acknowledged=False
  h.done=threading.Event();h.log_done=threading.Event();h.reader=Mock();h.guard=Mock();h.writer=Mock();h.sock=Mock();h.raw=Mock();h.tx_log=Mock()
  with self.assertRaisesRegex(RuntimeError,'Refusing adapter close'):h.close()
  h.close(allow_unconfirmed_power_off=True);self.assertTrue(h.done.is_set());self.assertFalse(h.disable_acknowledged)

if __name__=='__main__':unittest.main()
