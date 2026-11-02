from dataclasses import dataclass
import numpy as np


def smooth(x):
    x = np.clip(x, 0., 1.)
    return 10*x**3-15*x**4+6*x**5, 30*x**2-60*x**3+30*x**4, 60*x-180*x**2+120*x**3


class Segment:
    """Quintic interpolation with continuous position, velocity and acceleration."""
    def __init__(self, q0, q1, duration, v0=None, a0=None):
        self.duration = float(duration)
        if not np.isfinite(duration) or duration <= 0: raise ValueError('invalid duration')
        q0, q1 = np.asarray(q0, float), np.asarray(q1, float)
        v0 = np.zeros_like(q0) if v0 is None else np.asarray(v0, float)
        a0 = np.zeros_like(q0) if a0 is None else np.asarray(a0, float)
        T = self.duration
        c0, c1, c2 = q0, v0*T, a0*T*T/2
        delta=q1-c0
        c3,c4,c5=10*delta-6*c1-3*c2,-15*delta+8*c1+3*c2,6*delta-3*c1-c2
        self.c = np.array([c0,c1,c2,c3,c4,c5])
        if self.c.shape!=(6,6) or not np.all(np.isfinite(self.c)):raise ValueError('invalid segment coefficients')
        self.c.setflags(write=False)
        if not np.any(v0) and not np.any(a0):
            # Exact extrema of the rest-to-rest quintic, independent of duration/sample count.
            self._limits=(np.minimum(q0,q1),np.maximum(q0,q1),1.875*np.abs(delta)/T,(10*np.sqrt(3)/3)*np.abs(delta)/T**2)
        else:
            self._limits=self._polynomial_limits()

    def _polynomial_limits(self):
        def extrema(coeff):
            deriv=np.arange(1,len(coeff))*coeff[1:]
            roots=np.polynomial.polynomial.polyroots(deriv) if np.any(deriv) else []
            inside=[float(np.clip(z.real,0.,1.)) for z in roots if abs(z.imag)<1e-7 and -1e-9<=z.real<=1+1e-9]
            return np.polynomial.polynomial.polyval(np.array([0.,1.,*inside]),coeff)
        lo=[];hi=[];vmax=[];amax=[]
        for j in range(6):
            q=self.c[:,j];v=np.arange(1,6)*q[1:]/self.duration;a=np.arange(1,5)*v[1:]/self.duration
            values=extrema(q);lo.append(np.min(values));hi.append(np.max(values))
            vmax.append(np.max(np.abs(extrema(v))));amax.append(np.max(np.abs(extrema(a))))
        return tuple(np.array(x) for x in (lo,hi,vmax,amax))

    def check_limits(self,lower,upper,vmax,amax):
        lo,hi,v,a=self._limits
        if np.any(lo<lower-1e-9) or np.any(hi>upper+1e-9):raise ValueError('trajectory crosses joint safety bounds')
        if np.max(v)>vmax+1e-8:raise ValueError('trajectory exceeds velocity bound')
        if np.max(a)>amax+1e-8:raise ValueError('trajectory exceeds acceleration bound')

    def at(self, t):
        x = np.clip(t/self.duration, 0, 1)
        q = sum(self.c[k]*x**k for k in range(6))
        v = sum(k*self.c[k]*x**(k-1) for k in range(1,6))/self.duration
        a = sum(k*(k-1)*self.c[k]*x**(k-2) for k in range(2,6))/self.duration**2
        return q,v,a


def transit(q0, q1, vmax, amax, minimum=2.):
    distance = float(np.max(np.abs(np.asarray(q1)-q0)))
    T = max(minimum, 1.9*distance/vmax, np.sqrt(6*distance/amax))
    return Segment(q0,q1,T)


def brake(q, v, vmax, amax):
    T = max(.5, 2.0*float(np.max(np.abs(v)))/amax)
    return Segment(q, np.asarray(q)+np.asarray(v)*T/2, T, v0=v)


class Excitation:
    def __init__(self, center, joint, amplitude, frequency, duration, ramp):
        self.center=np.asarray(center,float); self.joint=joint; self.amplitude=amplitude
        self.frequency=frequency; self.duration=duration; self.ramp=ramp
        if not 0<=joint<6 or amplitude<=0 or frequency<=0 or ramp<=0 or duration<2*ramp:
            raise ValueError('invalid excitation')
    def at(self,t):
        t=np.clip(t,0,self.duration); r=self.ramp
        if t<r: e,de,dde=smooth(t/r); de/=r; dde/=r*r
        elif t>self.duration-r: e,de,dde=smooth((self.duration-t)/r); de/=-r; dde/=r*r
        else: e,de,dde=1.,0.,0.
        w=2*np.pi*self.frequency; s,c=np.sin(w*t),np.cos(w*t)
        q=self.center.copy(); v=np.zeros(6); a=np.zeros(6); j=self.joint; A=self.amplitude
        q[j]+=A*e*s; v[j]=A*(de*s+e*w*c); a[j]=A*(dde*s+2*de*w*c-e*w*w*s)
        return q,v,a


class ScaledTrajectory:
    """Independent displacement and playback-speed fractions around a fixed anchor."""
    def __init__(self,path,anchor,scale,speed_scale=1.):
        if not np.isfinite(scale) or not 0<scale<=1:raise ValueError('scale must be in (0, 1]')
        if not np.isfinite(speed_scale) or not 0<speed_scale<=1:raise ValueError('speed_scale must be in (0, 1]')
        self.path=path;self.anchor=np.asarray(anchor,float).copy();self.scale=float(scale)
        self.speed_scale=float(speed_scale)
        self.duration=path.duration/self.speed_scale;self.supports_batch=getattr(path,'supports_batch',False)
    def at(self,t):
        q,v,a=self.path.at(np.asarray(t)*self.speed_scale)
        return self.anchor+self.scale*(q-self.anchor),self.scale*self.speed_scale*v,self.scale*self.speed_scale**2*a


def check_path(path, lower, upper, vmax, amax):
    # Polynomial/composite paths prove bounds without dense sampling in the control loop.
    if hasattr(path,'check_limits'):
        path.check_limits(np.asarray(lower),np.asarray(upper),vmax,amax)
        return
    if getattr(path,'supports_batch',False):
        q,v,a=path.at(np.linspace(0,path.duration,max(1001,int(path.duration/.002)+1)))
        if not all(np.all(np.isfinite(x)) for x in (q,v,a)):raise ValueError('nonfinite trajectory')
        if np.any(q<lower-1e-9) or np.any(q>upper+1e-9):raise ValueError('trajectory crosses joint safety bounds')
        if np.max(np.abs(v))>vmax+1e-8:raise ValueError('trajectory exceeds velocity bound')
        if np.max(np.abs(a))>amax+1e-8:raise ValueError('trajectory exceeds acceleration bound')
        return
    # Preflight sampled more densely than the command period, with conservative bounds.
    for t in np.linspace(0,path.duration,max(1001,int(path.duration/.002)+1)):
        q,v,a=path.at(t)
        if not all(np.all(np.isfinite(x)) for x in (q,v,a)): raise ValueError('nonfinite trajectory')
        if np.any(q<lower-1e-9) or np.any(q>upper+1e-9): raise ValueError('trajectory crosses joint safety bounds')
        if np.max(np.abs(v))>vmax+1e-8: raise ValueError('trajectory exceeds velocity bound')
        if np.max(np.abs(a))>amax+1e-8: raise ValueError('trajectory exceeds acceleration bound')
