"""Reference handoff equations in protocol PD units."""
import numpy as np

def bumpless_reference(q,qd,old_q,old_v,old_kp,old_kd,old_ff_wire,new_kp,new_kd=None):
    # In protocol units, new v_ref=measured velocity removes the new D term.
    # New P term carries the prior total request while the FF field becomes 0.
    old_sum=old_kp*(old_q-q)+old_kd*(old_v-qd)+old_ff_wire
    velocity=qd.copy() if new_kd is None else np.zeros(6)
    correction=0. if new_kd is None else new_kd*qd
    return q+(old_sum+correction)/new_kp,velocity
