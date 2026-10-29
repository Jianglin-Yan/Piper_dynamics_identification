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
        self.save_manifest()

    def save_manifest(self):
        (self.path/'test_manifest.json').write_text(json.dumps(self.manifest,default=encode,ensure_ascii=False,indent=2)+'\n')

    def update(self,ctrl,now,s,command_time,action=None):
        if self.closed:return
        if ctrl.state=='TESTING' and not self.active:
            self.active=True;self.start=ctrl.path_t
            self.manifest['windows'].append({'start_monotonic_s':self.start,'end_monotonic_s':None,
                'planned_duration_s':ctrl.test.duration,'trajectory_scale':self.c.get('trajectory_scale',1.),
                'anchor_rad':ctrl.start.tolist(),'nominal_amplitude_deg':self.c['amplitude_deg'] if self.c.get('trajectory','pilot')=='pilot' else None,
                'trajectory':self.c.get('trajectory','pilot'),'reference_plan':getattr(ctrl,'reference_plan',None)})
            self.save_manifest()
            return # Snapshot precedes the transition; it still belongs to READY/ARMING.
        if self.active and ctrl.state!='TESTING':
            self.end(now,ctrl.stop_reason or ('normal_test_end' if ctrl.state=='RETURNING' else ctrl.state));return
        if not self.active:return
        t=float(s['time']);q,v,a=ctrl.test.at(t-self.start)
        finite=all(np.shape(s.get(k))==(6,) and np.all(np.isfinite(s[k])) for k in ('q','qd','current_A'))
        fresh=True;ages={}
        for key,limit in (('q_times',self.c['feedback_timeout_s']),('drive_times',self.c['feedback_timeout_s']),('low_times',self.c['status_timeout_s'])):
            ts=np.asarray(s.get(key,[]));ok=(ts.shape==(6,) and np.all(np.isfinite(ts)))
            ages[key]=None if not ok else (t-ts).tolist()
            fresh=bool(fresh and ok and np.all(ts>=self.start) and np.all(ts<=t+1e-6) and np.max(t-ts)<=limit)
        status=s.get('status',{})
        healthy=bool(np.all(s.get('enabled',False)) and not any(s.get('faults',[255])) and
                     status.get('arm_status',255)==0 and status.get('error_code',65535)==0)
        good=bool(finite and fresh and healthy)
        row={'state':'TESTING','t_monotonic_s':t,'t_test_s':t-self.start,
             't_command_monotonic_s':command_time,'trajectory_scale':self.c.get('trajectory_scale',1.),
             'q_ref':q,'qd_ref':v,'qdd_ref':a,'q_command_sent':ctrl.last_target.copy(),
             'feedback':s,'signal_age_s':ages,'quality':{'finite':finite,'fresh_within_test':fresh,
             'healthy':healthy,'valid_measurement':good},'action':action}
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
    with (path/'test_raw_can.jsonl').open('w') as dst:
        if (path/'raw_can.jsonl').is_file():
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
    np.savez_compressed(path/'test_signals.npz',t_monotonic_s=t,t_test_s=elapsed,
        t_command_monotonic_s=np.array([r['t_command_monotonic_s'] for r in rows]),
        valid_measurement=valid,**fields,t_uniform_monotonic_s=grid,
        qd_uniform=qd_grid,qdd_est=qdd,qdd_est_valid=acc_valid)
    with (path/'test_signals.csv').open('w',newline='') as f:
        writer=csv.writer(f);keys=list(fields)
        writer.writerow(['t_monotonic_s','t_test_s','valid_measurement']+[f'{key}_J{j}' for key in keys for j in range(1,7)])
        for i in range(len(rows)):writer.writerow([t[i],elapsed[i],bool(valid[i])]+[x for key in keys for x in fields[key][i]])
    with (path/'test_acceleration.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(['t_monotonic_s']+[f'qdd_est_J{j}' for j in range(1,7)]+[f'qdd_est_valid_J{j}' for j in range(1,7)])
        for i,tt in enumerate(grid):writer.writerow([tt,*qdd[i],*acc_valid[i]])
    raw_count,frame_stats=extract_test_frames(path,manifest)
    summary={'sample_count':len(rows),'valid_measurement_count':int(valid.sum()),'raw_test_frame_count':raw_count,'raw_frame_stats':frame_stats,
        'max_control_gap_s':manifest.get('max_control_gap_s'),'valid_measurement_ratio':float(valid.mean()) if len(valid) else None,
        'test_duration_s':sum(w['end_monotonic_s']-w['start_monotonic_s'] for w in manifest['windows'] if w['end_monotonic_s'] is not None),
        'trajectory_scale':manifest['configuration'].get('trajectory_scale',1.),'torque_calibrated':False,
        'qdd_est_method':'11-sample cubic Savitzky-Golay derivative of velocity interpolated by per-axis receive timestamps; preview only; edges and gaps masked',
        'qdd_est_valid_per_joint':acc_valid.sum(axis=0).tolist(),'units':manifest['units'],
        'note':'Snapshot vectors are asynchronous; receive timestamps retained. qdd_est is estimated, qdd_ref is planned. No measured inertia matrix or calibrated torque is invented.'}
    (path/'test_quality.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
    return summary


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path)
    print(json.dumps(export_test_data(p.parse_args().run),ensure_ascii=False,indent=2))
