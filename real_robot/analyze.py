"""Summarize a pilot run and plot measured vs commanded angle; never fits inertial parameters."""
import argparse, json
from pathlib import Path
import numpy as np

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);a=p.parse_args()
    cfg=json.loads((a.run/'config.json').read_text());rows=[json.loads(s) for s in (a.run/'samples.jsonl').read_text().splitlines()]
    rows=[r for r in rows if 't_control' in r]
    if not rows:raise SystemExit('Only read-only snapshot available; no trajectory samples to analyze.')
    t=np.array([r['t_control'] for r in rows]);t-=t[0]
    q=np.array([r['feedback']['q'] for r in rows]);qd=np.array([r['feedback']['qd'] for r in rows]);target=np.array([r.get('q_target',[float('nan')]*6) for r in rows]);j=cfg['joint']-1
    result=json.loads((a.run/'result.json').read_text());home=np.array(result.get('home_rad',q[0]));return_home=np.array(result.get('return_home_rad',home));events={}
    for r,tt in zip(rows,t):
        if r.get('action'):events.setdefault(r['action'],[]).append(float(tt))
    tracking=np.abs(q-target);valid=np.all(np.isfinite(target),axis=1)
    out={'mode':result['mode'],'state':result.get('state'),'joint':j+1,'duration_s':float(t[-1]),'events_seconds':events,'peak_to_peak_deg':np.rad2deg(np.ptp(q,axis=0)).tolist(),'max_tracking_error_deg':np.rad2deg(np.max(tracking[valid],axis=0)).tolist() if np.any(valid) else None,'post_stop_observation_samples':sum(r.get('state')=='FAULT_OBSERVATION' for r in rows),'last_home_error_deg':np.rad2deg(q[-1]-home).tolist(),'note':'Simulation/controller validation only unless mode explicitly equals hardware. Current-to-torque candidate is not calibrated. No target is invented for fault-observation samples.'}
    out.update(return_home_deg=np.rad2deg(return_home).tolist(),return_home_delta_deg=np.rad2deg(return_home-home).tolist(),last_return_error_deg=np.rad2deg(q[-1]-return_home).tolist())
    out.update(motors_disabled_confirmed=result.get('motors_disabled_confirmed'),final_enabled=result.get('final_enabled'),release_home_error_deg=None if result.get('release_q_rad') is None else np.rad2deg(np.array(result['release_q_rad'])-home).tolist())
    (a.run/'analysis.json').write_text(json.dumps(out,ensure_ascii=False,indent=2));print(json.dumps(out,ensure_ascii=False,indent=2))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(3,1,figsize=(10,7),sharex=True)
    axes[0].plot(t,np.rad2deg(target[:,j]),label='Target');axes[0].plot(t,np.rad2deg(q[:,j]),label='Measured');axes[0].set_ylabel(f'J{j+1} angle (deg)');axes[0].legend()
    axes[1].plot(t,qd[:,j]);axes[1].set_ylabel('Velocity (rad/s)')
    axes[2].plot(t,np.rad2deg(q[:,j]-target[:,j]));axes[2].set_ylabel('Tracking error (deg)');axes[2].set_xlabel('Elapsed time (s)')
    for ax in axes:
        ax.grid(alpha=.2)
        for label,times in events.items():
            for tt in times:ax.axvline(tt,color='r' if label=='ABORT' else 'gray',alpha=.4,linestyle='--')
    fig.suptitle(f'{result["mode"].upper()} single-joint pilot | {result.get("state")} | red line: LB+RB')
    fig.tight_layout();fig.savefig(a.run/'trial.png',dpi=140);plt.close(fig)
if __name__=='__main__':main()
