"""Operator-owned exit: graceful stop, supported disable, explicit power-cut escape."""
import select
import sys


class OperatorExit:
    def __init__(self,emit,stdin=None):
        self.emit=emit;self.stdin=sys.stdin if stdin is None else stdin
        self.serial=0;self.seen=0;self.support_prompted=False

    @property
    def requested(self):return self.serial>0

    def signal(self,signum,frame):
        # Never touch CAN, call input(), or throw inside a signal handler.
        self.serial+=1

    def _command(self):
        try:
            if not self.stdin.isatty() or not select.select([self.stdin],[],[],0)[0]:return None
            return self.stdin.readline().strip()
        except (OSError,ValueError,AttributeError):return None

    def update(self,ctrl,s,now):
        if ctrl.finished:return
        if ctrl.shutdown_phase=='DISABLE_UNCONFIRMED' and self._command()=='POWER_OFF':
            ctrl.confirm_physical_power_off();return
        if self.serial==self.seen:return
        # Coalesce interrupts already queued before showing a confirmation prompt.
        self.seen=self.serial
        if ctrl.shutdown_phase in ('DISABLING','DISABLE_UNCONFIRMED'):
            self.emit('EXIT：正在请求失能并检查反馈。若通信完全中断，先支撑并切断机械臂电源；确认已失电后输入 POWER_OFF 回车，可保存未确认状态并退出。')
            return
        stopping=bool(ctrl.shutdown_phase or ctrl.state in ('BRAKING','RETURNING','SETTLING_HOME'))
        if stopping:
            if not self.support_prompted:
                self.support_prompted=True
                self.emit('SUPPORTED_EXIT：先用可靠支撑承住机械臂，再按一次 Ctrl+C，确认立即撤去驱动力矩并退出；不再要求回到起点。')
            else:ctrl.supported_disarm(s,now)
            return
        ctrl.request_stop(s,now,'Ctrl+C/SIGTERM')
