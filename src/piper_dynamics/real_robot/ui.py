"""终端输出格式：header、状态、每秒进度、警告、故障块。

所有输出仍交给调用方的 emit，由它同时写 events.jsonl，保证终端和记录一致。
"""
import time

STATES={'WAITING','ARMING','BOUNDARY_ENTRY','PREPARING','SETTLING_READY','READY',
        'TESTING','BRAKING','RETURNING','SETTLING_HOME','RELEASING','DISABLING','DONE','FAULT',
        'FF_RAMP_IN','FF_RAMP_OUT','RECOVERING'}


class Printer:
    def __init__(self,emit):
        self.emit=emit

    @staticmethod
    def _stamp():
        return time.strftime('%H:%M:%S')

    def header(self,title,lines=()):
        self.emit(f'=== {title} ===')
        for line in lines:self.emit(line)

    def event(self,message):
        message=str(message)
        self.emit(f'[{self._stamp()}] {message}')

    def status(self,text):
        self.emit(f'[{self._stamp()}] {text}')

    def warn(self,text):
        self.emit(f'[{self._stamp()}] 警告  {text}')

    def fault(self,reason,part='未细分',measured='见原因',action='',next_step=''):
        # 故障必须能对照现场：原因、部位、实测、当前处理、下一步。
        self.emit(f'[{self._stamp()}] 故障  {reason}')
        self.emit(f'  部位  {part}')
        self.emit(f'  实测  {measured}')
        self.emit(f'  处理  {action}')
        self.emit(f'  下一步  {next_step}')
