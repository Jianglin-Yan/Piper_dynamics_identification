#!/usr/bin/env python3
"""PiPER raw SocketCAN capture. Only explicitly whitelisted query frames can be sent.
No SDK connection side effects, no enable/motion/reset/configuration commands.
Raw frames are authoritative; decoded fields follow piper_sdk 0.6.2 protocol_v2.
Current is signed as in the parser; no firmware-dependent torque scaling is guessed.
"""
import argparse, collections, datetime, json, socket, struct, time
from pathlib import Path

def decode(cid, d):
    if len(d) != 8: return {}
    if cid == 0x2a1:
        return dict(zip(['control_mode','arm_status','motion_mode','teach_status','motion_status','trajectory_index','error_code'], [*d[:6],int.from_bytes(d[6:8],'big')]))
    if 0x2a5 <= cid <= 0x2a7:
        return {'joint_pair': [2*(cid-0x2a5)+1,2*(cid-0x2a5)+2], 'position_deg':[v/1000 for v in struct.unpack('>ii', d)]}
    if 0x251 <= cid <= 0x256:
        v,i,p=struct.unpack('>hhi',d)
        return {'joint':cid-0x250,'velocity_sdk_rad_s':v/1000,'current_signed_A':i/1000,'motor_position_raw':p}
    if 0x261 <= cid <= 0x266:
        v,t,mt,s,i=struct.unpack('>HhbBH',d)
        return {'joint':cid-0x260,'voltage_V':v/10,'driver_temp_C':t,'motor_temp_C':mt,'status_raw':s,'enabled':bool(s&64),'fault_bits_excluding_enable':s&191,'bus_current_A':i/1000}
    if cid == 0x473:
        return {'joint':d[0],'max_deg':int.from_bytes(d[1:3],'big',signed=True)/10,'min_deg':int.from_bytes(d[3:5],'big',signed=True)/10,'max_speed_raw':int.from_bytes(d[5:7],'big')}
    if cid == 0x47c: return {'joint':d[0],'max_acc_raw':int.from_bytes(d[1:3],'big')}
    if cid == 0x47b: return {'collision_levels':list(d[:6])}
    if cid == 0x2a8: return {'gripper_position_raw':int.from_bytes(d[:4],'big',signed=True),'gripper_effort_raw':int.from_bytes(d[4:6],'big',signed=True),'status':d[6]}
    return {}

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--channel',default='can0'); ap.add_argument('--seconds',type=float,default=10)
    ap.add_argument('--query',action='store_true',help='Also send firmware/limit/protection read queries')
    ap.add_argument('--output',type=Path,default=Path(__file__).parent/'captures')
    a=ap.parse_args()
    if not 1<=a.seconds<=3600: ap.error('seconds must be 1..3600')
    root=a.output/datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f'); root.mkdir(parents=True)
    sock=socket.socket(socket.AF_CAN,socket.SOCK_RAW,socket.CAN_RAW)
    sock.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,1024*1024)
    sock.bind((a.channel,)); sock.settimeout(.1)
    count=collections.Counter(); latest={}; samples=collections.defaultdict(list); fw=[]; replies={}; tx=[]
    t0=time.monotonic(); log=(root/'raw_frames.jsonl').open('w'); passive=[]
    def pump(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            try: raw=sock.recv(16)
            except socket.timeout: continue
            if len(raw)!=16: continue
            cid,n,d=struct.unpack('=IB3x8s',raw); now=time.monotonic(); d=d[:n]
            item={'t_monotonic_s':now,'t_host_unix_ns':time.time_ns(),'can_id':hex(cid),'data_hex':d.hex()}
            log.write(json.dumps(item)+'\n'); count[cid]+=1; samples[cid].append(now)
            value=decode(cid,d)
            if value: latest[hex(cid)]={'t_monotonic_s':now,**value}
            if cid==0x4af: fw.append(d)
            if cid in (0x473,0x47c) and value: replies[(cid,d[0])]={'t_monotonic_s':now,**value}
    allowed={(0x4af,bytes([1,0,0,0,0,0,0,0])),(0x477,bytes([2,0,0,0,3,0,0,0]))}
    allowed.update((0x472,bytes([j,k,0,0,0,0,0,0])) for j in range(1,7) for k in (1,2))
    def send(cid,d):
        if (cid,d) not in allowed: raise RuntimeError('non-query frame forbidden')
        now=time.monotonic(); sock.send(struct.pack('=IB3x8s',cid,8,d)); tx.append({'t_monotonic_s':now,'can_id':hex(cid),'data_hex':d.hex()}); return now
    try:
        pump(a.seconds)
        passive_count=dict(count); passive_elapsed=time.monotonic()-t0
        if a.query:
            send(0x4af,bytes([1,0,0,0,0,0,0,0])); pump(1)
            for j in range(1,7):
                for kind,replyid in [(1,0x473),(2,0x47c)]:
                    sent=send(0x472,bytes([j,kind,0,0,0,0,0,0])); pump(.25)
                    reply=replies.get((replyid,j))
                    if reply is None or reply['t_monotonic_s']<sent:
                        replies.pop((replyid,j),None)
            send(0x477,bytes([2,0,0,0,3,0,0,0])); pump(.5)
    finally:
        log.close(); sock.close()
    stats={hex(cid):{'count':n,'passive_hz':passive_count.get(cid,0)/passive_elapsed,'max_host_gap_ms':max([0]+[(y-x)*1000 for x,y in zip(samples[cid],samples[cid][1:])])} for cid,n in sorted(count.items())}
    result={'note':'No enable, mode, motion or parameter-write commands sent. Host receive timestamps, not hardware timestamps. No calibrated torque yet.', 'channel':a.channel,'duration_s':time.monotonic()-t0,'firmware_ascii':b''.join(fw).decode('ascii',errors='replace').strip('\x00'),'frame_stats':stats,'latest':latest,'joint_limits':{str(j):replies.get((0x473,j)) for j in range(1,7)},'joint_acc_limits':{str(j):replies.get((0x47c,j)) for j in range(1,7)},'transmitted_queries':tx}
    (root/'summary.json').write_text(json.dumps(result,ensure_ascii=False,indent=2))
    print(root); print(json.dumps(result,ensure_ascii=False,indent=2))
if __name__=='__main__': main()
