"""SocketCAN adapter for the observed V1.7-3 position interface. Never writes calibration or protection settings."""
import copy, json, queue, socket, struct, threading, time
import numpy as np
from .protocol_read import decode


class Hardware:
    def __init__(self,cfg,run_dir):
        self.c=cfg;self.latest={};self.limits={};self.acc={};self.firmware=bytearray();self.error=None
        self.rx_lock=threading.Lock();self.tx_lock=threading.Lock();self.stopped=False;self.active=False;self.motion_closed=False;self.disable_acknowledged=False;self.motion_started=False
        self.heartbeat=time.monotonic();self.done=threading.Event();self.raw=(run_dir/'raw_can.jsonl').open('w',buffering=65536)
        self.rx_dropped=0;self.raw_log_dropped=0;self.log_done=threading.Event();self.raw_queue=queue.Queue(maxsize=32768)
        self.tx_log=(run_dir/'tx_can.jsonl').open('w',buffering=65536)
        self.sock=socket.socket(socket.AF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
        try:
            self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,4*1024*1024)
            self.sock.setsockopt(socket.SOL_SOCKET,40,1) # SO_RXQ_OVFL
            self.sock.bind((cfg['channel'],));self.sock.settimeout(.02)
            self.receive_buffer_bytes=self.sock.getsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF)
            self.writer=threading.Thread(target=self._write_raw,daemon=True);self.writer.start()
            self.reader=threading.Thread(target=self._receive,daemon=True);self.reader.start()
            self.guard=threading.Thread(target=self._watchdog,daemon=True);self.guard.start()
        except Exception:
            self.done.set()
            if hasattr(self,'reader'):self.reader.join()
            self.log_done.set()
            if hasattr(self,'writer'):self.writer.join()
            self.sock.close();self.raw.close();self.tx_log.close();raise
    def _write_raw(self):
        # Disk/JSON work must not block the receiver that also confirms shutdown.
        while not self.log_done.is_set() or not self.raw_queue.empty():
            try:row=self.raw_queue.get(timeout=.02)
            except queue.Empty:continue
            try:self.raw.write(json.dumps(row)+'\n')
            except Exception as e:
                self.raw_log_dropped+=1;self.error='CAN raw logging failed: '+str(e)
            finally:self.raw_queue.task_done()

    def _receive(self):
        try:
            while not self.done.is_set():
                try: raw,anc,flags,addr=self.sock.recvmsg(16,128)
                except socket.timeout:continue
                if len(raw)!=16:continue
                cid,n,data=struct.unpack('=IB3x8s',raw);data=data[:n];now=time.monotonic();dropped=0
                # SO_RXQ_OVFL is a cumulative uint32 counter, not a fatal socket error.
                for level,kind,value in anc:
                    if level==socket.SOL_SOCKET and kind==40 and len(value)>=4:
                        total=struct.unpack('=I',value[:4])[0]
                        dropped=(total-self.rx_dropped)&0xffffffff;self.rx_dropped=total
                row={'t_monotonic_s':now,'t_host_unix_ns':time.time_ns(),'can_id':hex(cid),'data_hex':data.hex(),
                     'rx_dropped_total':self.rx_dropped,'rx_dropped_delta':dropped}
                if cid&0x20000000:
                    self.error='CAN error frame';decoded={}
                else:
                    try:decoded=decode(cid,data)
                    except Exception as e:
                        self.error='CAN decode failed: '+str(e);decoded={}
                with self.rx_lock:
                    if decoded:self.latest[cid]={'time':now,**decoded}
                    if cid==0x473 and decoded:self.limits[data[0]]={'time':now,**decoded}
                    if cid==0x47c and decoded:self.acc[data[0]]={'time':now,**decoded}
                    if cid==0x4af:self.firmware.extend(data)
                try:self.raw_queue.put_nowait(row)
                except queue.Full:
                    self.raw_log_dropped+=1;self.error='CAN raw logging queue full'
                    # Preserve live feedback for stop/disarm, even when data recording fails.
        except Exception as e:
            if not self.done.is_set():self.error='CAN receiver failed: '+str(e)
    def _send_locked(self,cid,data):
        if len(data)!=8:raise ValueError('CAN payload must be 8 bytes')
        self.sock.send(struct.pack('=IB3x8s',cid,8,bytes(data)))
        self.tx_log.write(json.dumps({'t_monotonic_s':time.monotonic(),'can_id':hex(cid),'data_hex':bytes(data).hex()})+'\n')
    def query(self,cid,data):
        allowed=(cid==0x4af and data==[1,0,0,0,0,0,0,0]) or (cid==0x472 and data[0] in range(1,7) and data[1] in (1,2) and data[2:]==[0]*6) or (cid==0x477 and data==[2,0,0,0,3,0,0,0])
        if not allowed:raise ValueError('query not whitelisted')
        with self.tx_lock:self._send_locked(cid,data)
    def discover(self):
        self.query(0x4af,[1,0,0,0,0,0,0,0]);time.sleep(.5)
        for j in range(1,7):
            for kind,store in ((1,self.limits),(2,self.acc)):
                sent=time.monotonic();self.query(0x472,[j,kind,0,0,0,0,0,0]);deadline=sent+.5
                while time.monotonic()<deadline:
                    with self.rx_lock:reply=copy.deepcopy(store.get(j))
                    if reply and reply['time']>=sent:break
                    time.sleep(.005)
                else:raise RuntimeError('No fresh limit reply for joint '+str(j))
        sent=time.monotonic();self.query(0x477,[2,0,0,0,3,0,0,0]);time.sleep(.1)
        with self.rx_lock:
            result={'firmware':self.firmware.decode('ascii',errors='replace'),'limits':copy.deepcopy(self.limits),'acceleration_limits':copy.deepcopy(self.acc),'collision':copy.deepcopy(self.latest.get(0x47b))}
        if not result['collision'] or result['collision']['time']<sent:raise RuntimeError('No fresh collision policy reply')
        return result
    def preflight(self,info):
        if self.c['firmware_prefix']!='S-V1.7-3' or 'S-V1.7-3250618' not in info['firmware']:
            raise ValueError('Firmware not the reviewed S-V1.7-3250618 profile; no guessed torque scaling')
        if self.c['model_confirmed'] is not True:
            raise ValueError('Confirm the actual PiPER model before motion')
        current_pose = self.c.get('current_pose_only',False)
        standing = self.c.get('standing_start',False)
        # The operator explicitly selects this bounded stationary-body wrist trial.
        # Keep workspace/protection metadata truthful; never mark unchecked facts as confirmed.
        multisine=self.c.get('trajectory')=='multisine'
        if multisine and (not standing or current_pose):raise ValueError('Multisine requires the standing preparation session')
        friction_current = self.c.get('trajectory') == 'friction' and self.c.get('friction_start') == 'current'
        if friction_current:
            # One joint at a time, a few degrees around the measured park; there is no
            # separate start pose, so the general workspace assertion does not apply.
            pass
        elif current_pose or standing:
            if not multisine and (self.c['joint']!=6 or (self.c['start_deg'] is not None and not standing) or self.c['amplitude_deg']>5 or self.c['frequency_hz']>.1):
                raise ValueError('Invalid current-pose J6 pilot profile')
        else:
            if self.c['workspace_checked'] is not True:
                raise ValueError('Confirm the workspace for a trial with a separate start pose')
            if any(x==0 for x in info['collision']['collision_levels']) and self.c['collision_policy_reviewed'] is not True:
                raise ValueError('Collision detection disabled; review protection policy for a general trial')
        self.c['reported_motor_max_velocity_rad_s']=[info['limits'][j]['max_speed_raw']*.01 for j in range(1,7)]
        self.c['reported_motor_max_acceleration_rad_s2']=[info['acceleration_limits'][j]['max_acc_raw']*.001 for j in range(1,7)]
        for j in range(1,7):
            lim=info['limits'][j];acc=info['acceleration_limits'][j]['max_acc_raw']
            if lim['max_deg']<=lim['min_deg'] or not 0<lim['max_speed_raw']<32767 or not 0<acc<32767:raise ValueError('Invalid hardware limits')
            self.c['lower_deg'][j-1]=max(self.c['lower_deg'][j-1],lim['min_deg'])
            self.c['upper_deg'][j-1]=min(self.c['upper_deg'][j-1],lim['max_deg'])
            if self.c['max_velocity_rad_s']>lim['max_speed_raw']*.01 or self.c['max_acceleration_rad_s2']>acc*.001:
                raise ValueError('Configured pilot speed/acceleration exceeds hardware settings')
    def snapshot(self):
        with self.rx_lock:latest=copy.deepcopy(self.latest)
        q=[];qt=[]
        for cid in range(0x2a5,0x2a8):
            x=latest.get(cid,{});q.extend(x.get('position_deg',[float('nan')]*2));qt.extend([x.get('time',0)]*2)
        high=[latest.get(0x251+j,{}) for j in range(6)];low=[latest.get(0x261+j,{}) for j in range(6)]
        current=np.array([x.get('current_signed_A',float('nan')) for x in high])
        return {'time':time.monotonic(),'q':np.deg2rad(q),'qd':np.array([x.get('velocity_sdk_rad_s',float('nan')) for x in high]),'current_A':current,'tau_candidate_Nm':current*np.asarray(self.c['torque_candidate_coefficients_Nm_per_A'],float),
                'q_times':np.array(qt),'drive_times':np.array([x.get('time',0) for x in high]),'low_times':np.array([x.get('time',0) for x in low]),
                'motor_position_raw':[x.get('motor_position_raw') for x in high],
                'driver_temp_C':[x.get('driver_temp_C') for x in low],'motor_temp_C':[x.get('motor_temp_C') for x in low],
                'bus_current_A':[x.get('bus_current_A') for x in low],
                'enabled':np.array([x.get('enabled',False) for x in low]),'faults':[x.get('fault_bits_excluding_enable',255) for x in low],
                'temp_C':[max(x.get('driver_temp_C',999),x.get('motor_temp_C',999)) for x in low], 'voltage_V':[x.get('voltage_V',0) for x in low],
                'status':latest.get(0x2a1,{}),'gripper':latest.get(0x2a8,{}),
                'can_rx_dropped_total':self.rx_dropped,'can_raw_log_dropped_total':self.raw_log_dropped}
    def check(self,s,active):
        if self.error:raise RuntimeError(self.error)
        if self.stopped:raise RuntimeError('Quick-stop latch is set')
        for key in ('q','qd','current_A'):
            if not np.all(np.isfinite(s[key])):raise RuntimeError('Missing '+key)
        now=time.monotonic()
        if max(now-np.min(s['q_times']),now-np.min(s['drive_times']))>self.c['feedback_timeout_s']:raise RuntimeError('Joint feedback stale')
        if now-min(np.min(s['low_times']),s['status'].get('time',0))>self.c['status_timeout_s']:raise RuntimeError('Status feedback stale')
        if s['status'].get('arm_status',255)!=0 or s['status'].get('error_code',65535)!=0 or any(s['faults']):raise RuntimeError(f'Arm/driver fault: arm_status={s["status"].get("arm_status")}, error_code={s["status"].get("error_code")}, driver_bits={s["faults"]}; if current-pose target was rejected, no automatic retries')
        if s['status'].get('control_mode') not in (0,1):raise RuntimeError('Arm is under another control mode')
        if active and s['status'].get('control_mode')!=1 and now-self.armed_at>.3:raise RuntimeError('Lost CAN position control')
        # Current levels are timed by Controller/monitoring; only firmware fault bits are immediate.
        if max(s['temp_C'])>self.c['max_temperature_C']:raise RuntimeError('Over-temperature threshold')
        if min(s['voltage_V'])<20 or max(s['voltage_V'])>26:raise RuntimeError('Supply voltage outside pilot range')
        self.heartbeat=now
    def _pose_locked(self,q):
        values=np.rint(np.rad2deg(q)*1000).astype(int)
        for i in range(3):self._send_locked(0x155+i,list(struct.pack('>ii',*values[2*i:2*i+2])))
    def arm(self,q):
        with self.tx_lock:
            if self.stopped or getattr(self,'motion_closed',False):raise RuntimeError('Stop/disarm is latched')
            self.heartbeat=time.monotonic();self.armed_at=self.heartbeat;self.active=True;self.motion_started=True
            if self.c.get('passive_wrist',False):
                self._send_locked(0x471,[6,1,0,0,0,0,0,0])
            if self.c.get('release_gripper',False):
                with self.rx_lock:g=self.latest.get(0x2a8,{}).get('gripper_position_raw',0)
                self._send_locked(0x159,list(struct.pack('>iHBB',g,0,0,0))) # gripper disable only
            # Passive wrist is released before any position target can reach it.
            # Select MOVE J first: on S-V1.7 the arm may still be in MOVE_MIT from the
            # previous run, and a position target sent before the mode switch can leave
            # joints in a state where the enable is ignored (arm_status=5).
            self._send_locked(0x151,[1,1,self.c['move_speed_percent'],0,0,0,0,0])
            status={}
            deadline=time.monotonic()+float(self.c.get('movej_mode_wait_s',.5))
            while time.monotonic()<deadline:
                latest=getattr(self,'latest',None);rx_lock=getattr(self,'rx_lock',None)
                if latest is None or rx_lock is None:break
                with rx_lock:status=dict(latest.get(0x2a1,{}))
                if not status:break
                if status.get('motion_mode')==1 and status.get('arm_status',0)==0 and status.get('error_code',0)==0:break
                self.heartbeat=time.monotonic();time.sleep(.01)
            if status and (status.get('motion_mode')!=1 or status.get('arm_status',0)!=0 or status.get('error_code',0)!=0):
                raise RuntimeError('Arm did not enter MOVE J cleanly before enable: motion_mode=%s, arm_status=%s, error_code=%s; if it was left in MIT mode, power-cycle the arm or switch mode in the vendor app' % (status.get('motion_mode'),status.get('arm_status'),status.get('error_code')))
            # Seed position only after the arm reports MOVE J.
            self._pose_locked(q)
            self._pose_locked(q)
            for j in range(1,6 if self.c.get('passive_wrist',False) else 7):self._send_locked(0x471,[j,2,0,0,0,0,0,0])
    def begin_native_move(self,q):
        self.native_speed_percent=2
        self.command(q)
    def finish_native_move(self):self.native_speed_percent=None
    def reset_once(self):
        """Send one recovery/reset frame; the session may arm afterward.

        Only called after the operator explicitly confirms support and no e-stop
        at the startup prompt. Unlike reset_supported() this does not latch the
        instance, so the normal arm/enable sequence can follow.
        """
        with self.tx_lock:
            if self.active or self.stopped:raise RuntimeError('Cannot reset an active or stopped session')
            self._send_locked(0x150,[2,0,0,0,0,0,0,0])

    def reset_supported(self):
        """Explicit recovery CLI only; never called by arming or fault recovery."""
        with self.tx_lock:
            if self.active or self.stopped:raise RuntimeError('Cannot reset an active or already reset/stopped session')
            self._send_locked(0x150,[2,0,0,0,0,0,0,0])
            # This instance may only observe afterward; no subsequent arm/position command.
            self.stopped=True
    def command(self,q):
        with self.tx_lock:
            if self.stopped or getattr(self,'motion_closed',False):raise RuntimeError('Stop/disarm is latched; command suppressed')
            # Match the official V2 joint-control demo: keep mode/speed alive with each target.
            speed=getattr(self,'native_speed_percent',None) or self.c['move_speed_percent']
            self._send_locked(0x151,[1,1,speed,0,0,0,0,0])
            self._pose_locked(q)
    def quick_stop(self,reason):
        with self.tx_lock:
            if self.stopped:return
            self.stopped=True;self.active=False
            try:self._send_locked(0x150,[1,0,0,0,0,0,0,0])
            except Exception as e:self.error='Quick stop could not be delivered: '+str(e)
        # No position/enable/reset after this; a supported park may still be disabled.
    def _watchdog(self):
        while not self.done.wait(.01):
            if self.active and (self.error or time.monotonic()-self.heartbeat>self.c['loop_watchdog_s']):
                self.quick_stop('loop/receiver watchdog')
    def disable(self):
        """Explicitly disable arm joints 1..6; no gripper/enable/reset command.

        MOVE J has no reviewed torque ramp. Removing enable removes drive torque;
        do not insert 0x150=01 (damping stop with a persistent stop state) here.
        """
        with self.tx_lock:
            self.motion_closed=True;self.active=False
            for j in range(1,7):self._send_locked(0x471,[j,1,0,0,0,0,0,0])
    def confirm_disabled(self):
        # Called only after fresh six-axis feedback has remained disabled.
        self.disable_acknowledged=True;self.active=False
    def finish(self):
        if getattr(self,'motion_started',False) and not getattr(self,'disable_acknowledged',False):
            raise RuntimeError('Cannot finish: six-axis disable not confirmed')
        self.active=False
    def close(self,allow_unconfirmed_power_off=False):
        if self.active:self.quick_stop('closing while active')
        if getattr(self,'motion_started',False) and not getattr(self,'disable_acknowledged',False) and not allow_unconfirmed_power_off:
            raise RuntimeError('Refusing adapter close before confirmed disable')
        self.done.set();self.reader.join();self.guard.join();self.log_done.set();self.writer.join();self.sock.close();self.raw.close();self.tx_log.close()
