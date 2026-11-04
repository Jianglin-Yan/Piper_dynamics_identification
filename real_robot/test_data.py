"""TESTING-only experimental data; session logs remain separate diagnostics.

All timestamps are host receive/control times, not device-synchronized time.
Reference acceleration is analytic; measured acceleration is estimated offline.
"""
import argparse
import csv
import json
import subprocess
from pathlib import Path
import numpy as np


def encode(x):
    if isinstance(x,np.ndarray):return x.tolist()
    if isinstance(x,np.generic):return x.item()
    raise TypeError(type(x).__name__)


def code_version():
    try:
        root=Path(__file__).resolve().parents[1]
        def git(*args):return subprocess.check_output(['git',*args],cwd=root,stderr=subprocess.DEVNULL,text=True).strip()
        return {'commit':git('rev-parse','HEAD'),'dirty':bool(git('status','--porcelain'))}
    except (OSError,subprocess.SubprocessError):return {'commit':None,'dirty':None}


class TestRecorder:
    def __init__(self,path,cfg,mode,version=None):
        self.path=Path(path);self.c=cfg;self.mode=mode;self.active=False;self.closed=False
        self.rows=0;self.good=0;self.previous_sample=None;self.max_control_gap=0.
        self.stream=(self.path/'test_samples.jsonl').open('w',buffering=65536)
        self.manifest={'schema_version':1,'mode':mode,'code':code_version() if version is None else version,
            'scope':'TESTING only; prepare, settle, brake, return, disable excluded',
            'units':{'q':'rad','qd':'rad/s','qdd':'rad/s^2','current':'A','torque_candidate':'Nm','time':'s'},
            'time_source':'host receive monotonic time; asynchronous streams, not hardware synchronized',
            'torque_calibrated':False,'acceleration_source':'offline derivative of measured velocity; never reference acceleration',
            'payload_description':cfg.get('payload_description','gripper present; extra payload not specified'),
            'configuration':cfg,'windows':[],'sample_count':0}
        if cfg.get('passive_wrist',False):self.manifest.update(controlled_joints=[1,2,3,4,5],passive_joints=[6],gripper_policy='released; manually adjustable')
        if cfg.get('release_gripper'):self.manifest.update(gripper_policy='released; manually adjustable')
        if cfg.get('hold_start_wrist'):self.manifest.update(fixed_at_startup_joints=[6],controlled_joints=[1,2,3,4,5,6],passive_joints=[])
        self.save_manifest()

    def save_manifest(self):
        (self.path/'test_manifest.json').write_text(json.dumps(self.manifest,default=encode,ensure_ascii=False,indent=2)+'\n')

    def update(self,ctrl,now,s,command_time,action=None):
        if self.closed:return
        if ctrl.state=='TESTING' and not self.active:
            self.active=True;self.start=ctrl.path_t
            self.previous_rx_dropped=s.get('can_rx_dropped_total',0);self.previous_log_dropped=s.get('can_raw_log_dropped_total',0)
            self.manifest['windows'].append({'start_monotonic_s':self.start,'end_monotonic_s':None,
                'planned_duration_s':None if self.c.get('trajectory')=='payload' else ctrl.test.duration,'trajectory_scale':self.c.get('trajectory_scale',1.),
                'trajectory_speed_scale':self.c.get('trajectory_speed_scale',1.),
                'anchor_rad':ctrl.start.tolist(),'nominal_amplitude_deg':self.c['amplitude_deg'] if self.c.get('trajectory','pilot')=='pilot' else None,
                'trajectory':self.c.get('trajectory','pilot'),'reference_plan':getattr(ctrl,'reference_plan',None)})
            self.save_manifest()
            return # Snapshot precedes the transition; it still belongs to READY/ARMING.
        if self.active and ctrl.state!='TESTING':
            self.end(now,ctrl.stop_reason or ('normal_test_end' if ctrl.state=='RETURNING' else ctrl.state));return
        if not self.active:return
        t=float(s['time']);q,v,a=ctrl.test.at(t-self.start)
        if self.c.get('passive_wrist',False):
            q=q.copy();v=v.copy();a=a.copy();q[5]=s['q'][5];v[5]=0.;a[5]=0.
        finite=all(np.shape(s.get(k))==(6,) and np.all(np.isfinite(s[k])) for k in ('q','qd','current_A'))
        fresh=True;ages={}
        for key,limit in (('q_times',self.c['feedback_timeout_s']),('drive_times',self.c['feedback_timeout_s']),('low_times',self.c['status_timeout_s'])):
            ts=np.asarray(s.get(key,[]));ok=(ts.shape==(6,) and np.all(np.isfinite(ts)))
            ages[key]=None if not ok else (t-ts).tolist()
            fresh=bool(fresh and ok and np.all(ts>=self.start) and np.all(ts<=t+1e-6) and np.max(t-ts)<=limit)
        status=s.get('status',{})
        expected=np.array([True]*5+[not self.c.get('passive_wrist',False)])
        enable=np.asarray(s.get('enabled',[]),bool)
        enable_ok=enable.shape==(6,) and bool(np.all(enable==expected))
        healthy=bool(enable_ok and not any(s.get('faults',[255])) and
                     status.get('arm_status',255)==0 and status.get('error_code',65535)==0)
        rx_total=s.get('can_rx_dropped_total',0);log_total=s.get('can_raw_log_dropped_total',0)
        lost=bool(rx_total!=self.previous_rx_dropped or log_total!=self.previous_log_dropped)
        self.previous_rx_dropped=rx_total;self.previous_log_dropped=log_total
        good=bool(finite and fresh and healthy and not lost)
        row={'state':'TESTING','t_monotonic_s':t,'t_test_s':t-self.start,
             't_command_monotonic_s':command_time,'trajectory_scale':self.c.get('trajectory_scale',1.),
             'trajectory_speed_scale':self.c.get('trajectory_speed_scale',1.),
             'q_ref':q,'qd_ref':v,'qdd_ref':a,'q_command_sent':ctrl.last_target.copy(),
             'feedback':s,'signal_age_s':ages,'quality':{'finite':finite,'fresh_within_test':fresh,
             'healthy':healthy,'can_loss_since_previous_sample':lost,'valid_measurement':good},'action':action}
        if getattr(ctrl,'trial',None):row['trial']=ctrl.trial.metadata(s,now)
        if getattr(ctrl,'gravity_telemetry',None) is not None:row['gravity']=ctrl.gravity_telemetry
        if getattr(ctrl,'mit_telemetry',None) is not None:row['mit']=ctrl.mit_telemetry
        self.stream.write(json.dumps(row,default=encode)+'\n');self.rows+=1;self.good+=int(good)
        if self.previous_sample is not None:self.max_control_gap=max(self.max_control_gap,t-self.previous_sample)
        self.previous_sample=t

    def end(self,now,reason):
        if self.active:
            self.manifest['windows'][-1].update(end_monotonic_s=now,end_reason=reason)
            self.active=False;self.stream.flush();self.save_manifest()

    def close(self,now,reason):
        if self.closed:return
        self.end(now,reason);self.stream.close();self.closed=True
        self.manifest.update(sample_count=self.rows,valid_measurement_count=self.good,
            max_control_gap_s=self.max_control_gap,
            suitable_for_identification=False,
            identification_note='Torque calibration and excitation adequacy must be verified separately; partial runs are retained and labelled.')
        self.save_manifest()


def extract_test_frames(path,manifest):
    """Use receive timestamps, not cached snapshots, to isolate full-rate raw test frames."""
    path=Path(path);count=0;stats={};windows=[w for w in manifest['windows'] if w['end_monotonic_s'] is not None]
    if not (path/'raw_can.jsonl').is_file():return 0,{}
    with (path/'test_raw_can.jsonl').open('w') as dst:
        if True:
            with (path/'raw_can.jsonl').open() as src:
                for line in src:
                    r=json.loads(line);t=r['t_monotonic_s']
                    if any(w['start_monotonic_s']<=t<w['end_monotonic_s'] for w in windows):
                        dst.write(line);count+=1
                        item=stats.setdefault(r['can_id'],{'count':0,'first':t,'last':t,'max_gap_s':0.})
                        item['max_gap_s']=max(item['max_gap_s'],t-item['last']);item['last']=t;item['count']+=1
    for item in stats.values():
        item['observed_hz']=(item['count']-1)/(item['last']-item['first']) if item['last']>item['first'] else None
    return count,stats


def export_test_data(path):
    path=Path(path);manifest=json.loads((path/'test_manifest.json').read_text())
    rows=[json.loads(x) for x in (path/'test_samples.jsonl').read_text().splitlines()]
    def matrix(get):return np.asarray([get(r) for r in rows],float).reshape((-1,6))
    fields={'q':matrix(lambda r:r['feedback']['q']),'qd':matrix(lambda r:r['feedback']['qd']),
            'current_A':matrix(lambda r:r['feedback']['current_A']),
            'tau_candidate_Nm':matrix(lambda r:r['feedback'].get('tau_candidate_Nm',[np.nan]*6)),
            'q_ref':matrix(lambda r:r['q_ref']),'qd_ref':matrix(lambda r:r['qd_ref']),
            'qdd_ref':matrix(lambda r:r['qdd_ref']),'q_command_sent':matrix(lambda r:r['q_command_sent']),
            'q_times':matrix(lambda r:r['feedback']['q_times']),
            'drive_times':matrix(lambda r:r['feedback']['drive_times']),
            'low_times':matrix(lambda r:r['feedback']['low_times'])}
    for key in ('temp_C','driver_temp_C','motor_temp_C','voltage_V','bus_current_A','motor_position_raw','enabled','faults'):
        fields[key]=matrix(lambda r:r['feedback'].get(key,[np.nan]*6))
    t=np.array([r['t_monotonic_s'] for r in rows]);elapsed=np.array([r['t_test_s'] for r in rows])
    valid=np.array([r['quality']['valid_measurement'] for r in rows],bool)
    # A labelled preview derivative on a uniform grid, using actual velocity receive times.
    # Missing frames/gaps and filter edge samples never acquire a valid acceleration flag.
    grid=np.array([]);qd_grid=np.empty((0,6));qdd=np.empty((0,6));acc_valid=np.empty((0,6),bool)
    if len(t)>=11:
        from scipy.signal import savgol_filter
        dt=1/manifest['configuration']['command_hz'];grid=np.arange(t[0],t[-1]+dt*.01,dt)
        qd_grid=np.full((len(grid),6),np.nan);qdd=qd_grid.copy();acc_valid=np.zeros_like(qdd,bool)
        for j in range(6):
            x=fields['drive_times'][:,j];y=fields['qd'][:,j]
            take=valid & np.isfinite(x) & np.isfinite(y)
            x,index=np.unique(x[take],return_index=True);y=y[take][index]
            if len(x)<11:continue
            inside=(grid>=x[0])&(grid<=x[-1]);p=np.clip(np.searchsorted(x,grid),1,len(x)-1)
            trustworthy=inside & (x[p]-x[p-1]<=3*dt)
            values=np.interp(grid,x,y);qd_grid[:,j]=np.where(inside,values,np.nan)
            if len(values)<11:continue
            qdd[:,j]=savgol_filter(values,11,3,deriv=1,delta=dt)
            acc_valid[:,j]=np.convolve(trustworthy.astype(int),np.ones(11,dtype=int),mode='same')==11
            qdd[~acc_valid[:,j],j]=np.nan
    labels={}
    if manifest['configuration'].get('trajectory') in ('static','reciprocal','payload','friction'):
        tags=[r.get('trial',{}) for r in rows]
        labels={
            'trial_phase':np.array([r.get('phase','') for r in tags],dtype=str),
            'trial_pose_id':np.array([r.get('pose_id',0) for r in tags],int),
            'trial_direction':np.array([r.get('direction','') for r in tags],dtype=str),
            'trial_mass_kg':np.array([np.nan if r.get('mass_kg') is None else r['mass_kg'] for r in tags]),
            'trial_speed_round':np.array([0 if r.get('speed_round') is None else r['speed_round'] for r in tags],int),
            'trial_speed_bound_rad_s':np.array([r.get('speed_bound_rad_s',np.nan) for r in tags]),
            'trial_capture_id':np.array([r.get('capture_id',0) for r in tags],int),
            'trial_steady_candidate':np.array([r.get('steady_candidate',False) for r in tags],bool),
            'trial_pair_time_s':np.array([np.nan if r.get('pair_time_s') is None else r['pair_time_s'] for r in tags]),
            'trial_joint':np.array([r.get('joint',0) for r in tags],int),
            'trial_repeat':np.array([r.get('repeat',0) for r in tags],int),
            'gripper_position_raw':np.array([r.get('feedback',{}).get('gripper',{}).get('gripper_position_raw',np.nan) for r in rows]),
        }
    if manifest['configuration'].get('trajectory')=='gravity':
        tags=[r.get('gravity',{}) for r in rows]
        for key in ('g_Nm','support_Nm','pd_Nm','requested_Nm','sent_Nm','wire_t_ref','quantized_Nm','kp_Nm_rad','kd_Nm_s_rad'):
            fields['gravity_'+key]=np.array([r.get(key,[np.nan]*6) for r in tags],float).reshape(-1,6)
        labels.update(gravity_phase=np.array([r.get('phase','') for r in tags],str),gravity_torque_limited=np.array([r.get('limited',False) for r in tags],bool))
    if manifest['configuration'].get('trajectory')=='mit':
        tags=[r.get('mit',{}) for r in rows]
        for key in ('kp_protocol','kd_protocol','g_Nm','ff_requested_Nm','wire_t_ref','quantized_ff_Nm','pd_estimate_Nm','total_estimate_Nm','v_ref'):
            fields['mit_'+key]=np.array([r.get(key,[np.nan]*6) for r in tags],float).reshape(-1,6)
        labels.update(mit_phase=np.array([r.get('phase','') for r in tags],str),mit_ff_ratio=np.array([r.get('ff_ratio',np.nan) for r in tags]))
    np.savez_compressed(path/'test_signals.npz',t_monotonic_s=t,t_test_s=elapsed,
        t_command_monotonic_s=np.array([r['t_command_monotonic_s'] for r in rows]),
        valid_measurement=valid,**fields,**labels,t_uniform_monotonic_s=grid,
        qd_uniform=qd_grid,qdd_est=qdd,qdd_est_valid=acc_valid)
    cfg=manifest['configuration']
    if cfg.get('export_csv',True):
        with (path/'test_signals.csv').open('w',newline='') as f:
            writer=csv.writer(f);keys=list(fields)
            writer.writerow(['t_monotonic_s','t_test_s','valid_measurement']+[f'{key}_J{j}' for key in keys for j in range(1,7)]+list(labels))
            for i in range(len(rows)):writer.writerow([t[i],elapsed[i],bool(valid[i])]+[x for key in keys for x in fields[key][i]]+[labels[key][i] for key in labels])
        with (path/'test_acceleration.csv').open('w',newline='') as f:
            writer=csv.writer(f);writer.writerow(['t_monotonic_s']+[f'qdd_est_J{j}' for j in range(1,7)]+[f'qdd_est_valid_J{j}' for j in range(1,7)])
            for i,tt in enumerate(grid):writer.writerow([tt,*qdd[i],*acc_valid[i]])
    raw_count,frame_stats=extract_test_frames(path,manifest)
    test_dropped=0
    if (path/'test_raw_can.jsonl').is_file():
        with (path/'test_raw_can.jsonl').open() as f:
            test_dropped=sum(json.loads(line).get('rx_dropped_delta',0) for line in f)
    loss_rows=sum(r['quality'].get('can_loss_since_previous_sample',False) for r in rows)
    summary={'can_rx_dropped_frames_detected_during_test':test_dropped,'can_loss_affected_snapshot_count':loss_rows,
        'can_loss_detected':bool(test_dropped or loss_rows),
        'sample_count':len(rows),'valid_measurement_count':int(valid.sum()),'raw_test_frame_count':raw_count,'raw_frame_stats':frame_stats,
        'max_control_gap_s':manifest.get('max_control_gap_s'),'valid_measurement_ratio':float(valid.mean()) if len(valid) else None,
        'test_duration_s':sum(w['end_monotonic_s']-w['start_monotonic_s'] for w in manifest['windows'] if w['end_monotonic_s'] is not None),
        'trajectory_scale':manifest['configuration'].get('trajectory_scale',1.),
        'trajectory_speed_scale':manifest['configuration'].get('trajectory_speed_scale',1.),'torque_calibrated':False,
        'qdd_est_method':'11-sample cubic Savitzky-Golay derivative of velocity interpolated by per-axis receive timestamps; preview only; edges and gaps masked',
        'qdd_est_valid_per_joint':acc_valid.sum(axis=0).tolist(),'units':manifest['units'],
        'note':'Snapshot vectors are asynchronous; receive timestamps retained. qdd_est is estimated, qdd_ref is planned. No measured inertia matrix or calibrated torque is invented.'}
    (path/'test_quality.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    return summary


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path)
    print(json.dumps(export_test_data(p.parse_args().run),ensure_ascii=False,indent=2))
