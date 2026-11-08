"""Arc-length speed envelope; online RPP applies final command rate limits."""
import numpy as np
from path_geometry import wrap


def profile(points,cfg):
    p=np.asarray(points,float);n=len(p)
    if n<2:return np.zeros(n),np.zeros(n)
    ds=np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1)
    curvature=np.abs(wrap(np.diff(p[:,2])))/np.maximum(ds,1e-6)
    k=np.r_[curvature,curvature[-1]];k=np.maximum(k,np.r_[k[0],k[:-1]])
    cap=np.full(n,float(cfg['max_v']))
    cap=np.minimum(cap,float(cfg['max_w'])/np.maximum(k,1e-8))
    cap=np.minimum(cap,np.sqrt(float(cfg.get('max_lateral_acceleration',.30))/np.maximum(k,1e-8)))
    r=float(cfg.get('wheel_radius_m',.08));track=float(cfg.get('wheel_track_m',.4769668332689978))
    cap=np.minimum(cap,float(cfg.get('wheel_speed_limit_rad_s',12.))*r/(1+k*track/2))
    arc=np.r_[0,np.cumsum(ds)]
    dk=np.abs(np.gradient(k,arc)) if min(ds)>1e-8 and n>2 else np.zeros(n)
    cap=np.minimum(cap,np.sqrt(.5*float(cfg.get('max_angular_accel',.8))/np.maximum(dk,1e-8)))
    cap[0]=cap[-1]=0.
    for i in range(n-2,-1,-1):cap[i]=min(cap[i],np.sqrt(cap[i+1]**2+2*cfg['deceleration']*ds[i]))
    for i in range(1,n):cap[i]=min(cap[i],np.sqrt(cap[i-1]**2+2*cfg['acceleration']*ds[i-1]))
    return cap,k
