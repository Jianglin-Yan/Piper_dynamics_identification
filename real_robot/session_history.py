"""Explain persistent controller recovery requirements from actual prior TX logs.

This reads session records, never sends CAN or assumes mode/enable feedback
proves that a preceding electronic stop has been cleared.
"""
import json
import re
from pathlib import Path


def previous_controller_session(roots,channel,exclude=None):
    candidates=set()
    for root in roots:
        root=Path(root)
        if not root.is_dir():continue
        for path in root.iterdir():
            if path.is_dir() and re.fullmatch(r'\d{8}_\d{6}_\d{6}',path.name):
                if exclude is None or path.resolve()!=Path(exclude).resolve():candidates.add(path.resolve())
    for path in sorted(candidates,key=lambda p:p.name,reverse=True):
        if not (path/'tx_can.jsonl').is_file():continue
        try:
            cfg=json.loads((path/'config.json').read_text())
            if cfg.get('channel')!=channel:continue
            result=json.loads((path/'result.json').read_text()) if (path/'result.json').is_file() else {}
        except (OSError,ValueError):continue
        controls=[]
        with (path/'tx_can.jsonl').open() as stream:
            for line in stream:
                try:
                    event=json.loads(line);cid=int(event['can_id'],16)
                    if cid in (0x150,0x151,0x155,0x156,0x157,0x471):controls.append(event)
                except (ValueError,KeyError,TypeError):continue
        if not controls:continue # Read-only and preflight sessions do not clear recovery history.
        last_stop=next((e for e in reversed(controls) if int(e['can_id'],16)==0x150),None)
        info={'run':str(path),'state':result.get('state','UNFINISHED'),'requires_recovery':False}
        if last_stop and last_stop.get('data_hex','').startswith('01'):
            info.update(requires_recovery=True,reason='上次已发送电子急停，未记录到后续成功恢复。')
        elif result.get('state')=='RECOVERED' and last_stop and last_stop.get('data_hex','').startswith('02'):
            info['reason']='上次显式恢复已确认成功。'
        elif result.get('state')=='DONE' and result.get('motors_disabled_confirmed') is True:
            info['reason']='上次回位后已确认六轴失能，未发送电子急停；无需例行重置。'
        elif result.get('state') in ('DONE','HELD'):
            info['reason']='旧版会话记录为保持交接，未发送电子急停；新版会在结束时回位失能。'
        else:
            info.update(requires_recovery=True,reason='上次控制会话没有记录完整的正常交接或成功恢复。')
        return info
    return None


def require_recovered(info):
    if info and info['requires_recovery']:
        raise RuntimeError('RECOVERY_REQUIRED：'+info['reason']+' 记录：'+info['run']+
                           '。先让机械臂蜷缩并获得支撑，再单独运行 bash real_robot/pilot.sh recover --supported；'
                           '本次不会使能、移动或自动重置。历史日志无法识别外部断电重启。')
