"""Linux evdev input only. Physical button calibration is required for hardware use."""
import json, time
from pathlib import Path


class Chords:
    def __init__(self, mapping, dwell=0.):
        self.mapping=mapping; self.dwell=dwell; self.ready=False; self.pending=None; self.since=0.; self.fired=False
    def update(self, keys, now):
        b={name for name,code in self.mapping.items() if code in keys}
        # Startup-held keys never start motion. Abort is always highest priority.
        if {'LB','RB'}<=b:
            if self.pending!='ABORT' or not self.fired:
                self.pending='ABORT';self.fired=True;return 'ABORT'
            return None
        if not b:
            self.ready=True;self.pending=None;self.fired=False;return None
        if not self.ready:return None
        wanted='PREPARE' if b=={'LB','X'} else 'START' if b=={'LB','Y'} else None
        if wanted is None:self.pending=None;self.fired=False;return None
        if wanted!=self.pending:self.pending=wanted;self.since=now;self.fired=False
        if not self.fired and now-self.since>=self.dwell:
            self.fired=True;self.ready=False;return wanted
        return None


def devices():
    import evdev
    found=[]
    for path in evdev.list_devices():
        dev=evdev.InputDevice(path)
        keys=dev.capabilities().get(evdev.ecodes.EV_KEY,[])
        if evdev.ecodes.BTN_GAMEPAD in keys and evdev.ecodes.BTN_TL in keys:
            found.append(dev)
        else:dev.close()
    return found


def identity(dev):
    return {'name':dev.name,'vendor':dev.info.vendor,'product':dev.info.product,'uniq':dev.uniq}


class Gamepad:
    def __init__(self, config_path):
        import evdev
        self.evdev=evdev; c=json.loads(Path(config_path).read_text()); ds=devices()
        matches=[d for d in ds if identity(d)==c['identity']]
        for d in ds:
            if d not in matches:d.close()
        if len(matches)!=1:
            for d in matches:d.close()
            raise RuntimeError('Calibrated G8 is not uniquely connected; run --calibrate-pad')
        self.dev=matches[0]; self.chords=Chords(c['buttons']); self.keys=set(self.dev.active_keys())
    def poll(self,now):
        # Preserve short presses from the evdev queue, not just the final key bitmap.
        actions=[]
        try:
            for event in self.dev.read():
                if event.type==self.evdev.ecodes.EV_SYN and event.code==self.evdev.ecodes.SYN_DROPPED:
                    raise RuntimeError('gamepad input overflow')
                if event.type==self.evdev.ecodes.EV_KEY and event.value in (0,1):
                    if event.value:self.keys.add(event.code)
                    else:self.keys.discard(event.code)
                    action=self.chords.update(self.keys,now)
                    if action:actions.append(action)
        except BlockingIOError:pass
        actual=set(self.dev.active_keys()) # detects device removal without an idle-key timeout
        self.keys=actual
        action=self.chords.update(actual,now)
        if action:actions.append(action)
        # If more than one event arrived in one cycle, abort wins before any command.
        if 'ABORT' in actions:return 'ABORT'
        return actions[0] if actions else None
    def close(self):self.dev.close()


def calibrate(path, timeout=120, initial=None):
    ds=devices()
    if len(ds)!=1:
        for d in ds:d.close()
        raise RuntimeError('Expected one gamepad; disconnect other controllers')
    dev=ds[0];mapping=dict(initial or {});start=time.monotonic()
    try:
        print('Controller:',identity(dev),flush=True)
        for label in ('LB','RB','X','Y'):
            if label in mapping: continue
            print('请先松开全部按键，然后单独按下 '+label,flush=True)
            released=False;release_since=None
            while time.monotonic()-start<timeout:
                keys=set(dev.active_keys())
                if not keys:
                    if release_since is None: release_since=time.monotonic()
                    if time.monotonic()-release_since>.3: released=True
                else: release_since=None
                if released and len(keys)==1:
                    code=keys.pop()
                    if code in mapping.values():
                        released=False;time.sleep(.01);continue
                    mapping[label]=code;print(label+' = '+str(code),flush=True);break
                time.sleep(.01)
            else:raise TimeoutError('calibration timeout')
        Path(path).write_text(json.dumps({'identity':identity(dev),'buttons':mapping,'calibrated':True},ensure_ascii=False,indent=2))
        print('Saved '+str(path),flush=True)
    finally:dev.close()
