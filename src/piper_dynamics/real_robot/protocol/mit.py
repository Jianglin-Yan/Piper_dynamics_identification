"""Reviewed DEFAULT MIT wire encoding and quantization."""
import numpy as np
from piper_dynamics.core.torque import WIRE_TO_JOINT

def uint(x,lo,hi,bits):
    if not np.isfinite(x) or not lo<=x<=hi:raise ValueError('MIT value outside reviewed encoding range')
    return int((x-lo)*((1<<bits)-1)/(hi-lo))

def encode_mit(joint,q,torque,vel_ref=0.,kp=0.,kd=0.):
    """Match pinned official DEFAULT driver/parser; gains are protocol values."""
    if type(joint) is not int or not 1<=joint<=6:raise ValueError('Invalid MIT joint')
    raw=float(torque)/WIRE_TO_JOINT[joint-1]
    p=uint(float(q),-12.5,12.5,16);v=uint(float(vel_ref),-45.,45.,12)
    kp=uint(float(kp),0.,500.,12);kd=uint(float(kd),-5.,5.,12);t=uint(raw,-8.,8.,8)
    frame=bytearray([p>>8,p&255,v>>4,((v&15)<<4)|(kp>>8),kp&255,kd>>4,((kd&15)<<4)|(t>>4)])
    frame.append(((t&15)<<4)|(int(np.bitwise_xor.reduce(frame))&15))
    actual=(t*16./255.-8.)*WIRE_TO_JOINT[joint-1]
    return bytes(frame),raw,float(actual)

def decode_command(q,v,kp,kd,ff):
    """Decoded protocol floats, including unavoidable old 8-bit zero-FF residue."""
    def quant(x,lo,hi,bits):return uint(float(x),lo,hi,bits)*(hi-lo)/((1<<bits)-1)+lo
    return tuple(np.array([quant(x,lo,hi,bits) for x in values]) for values,lo,hi,bits in
        [(q,-12.5,12.5,16),(v,-45.,45.,12),(kp,0.,500.,12),(kd,-5.,5.,12),(ff/WIRE_TO_JOINT,-8.,8.,8)])
