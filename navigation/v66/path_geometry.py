"""Differential-drive curve candidates, not Reeds-Shepp or a dynamics proof."""
import math
import numpy as np


def wrap(a):return (a+np.pi)%(2*np.pi)-np.pi


def checked_curve(grid, poses, curvature=None):
    poses=np.asarray(poses,float)
    if len(poses)<2 or not np.isfinite(poses).all():return False
    ds=np.linalg.norm(np.diff(poses[:,:2],axis=0),axis=1)
    da=np.abs(wrap(np.diff(poses[:,2])))
    if np.any(ds<1e-7) or max(ds)>.021 or max(da)>.035:return False
    if curvature is None:curvature=da/np.maximum(ds,1e-9)
    if np.max(np.abs(curvature))>grid.cfg.max_path_curvature+1e-6:return False
    # Geometry is sampled at <=2cm and <=2deg, not continuous CCD.
    return not any(grid.collision(p) for p in poses)


def as_samples(poses, gear, cfg):
    poses=np.asarray(poses,float);rows=[]
    ds=np.linalg.norm(np.diff(poses[:,:2],axis=0),axis=1)
    dy=wrap(np.diff(poses[:,2]))
    for i in range(1,len(poses)):
        duration=max(ds[i-1]/cfg.nominal_v,abs(dy[i-1])/cfg.nominal_w,1e-6)
        rows.append((poses[i].copy(),gear*float(ds[i-1])/duration,float(dy[i-1])/duration,float(duration)))
    return rows


def quintic_connectors(grid,start,goal,gear):
    """Zero endpoint curvature; preserve full XY/yaw at both ends."""
    distance=float(np.linalg.norm(goal[:2]-start[:2]));out=[]
    if distance<.04:return out
    for scale in (.65,1.,1.5,2.1):
        a=gear*distance*scale*np.array([math.cos(start[2]),math.sin(start[2])])
        b=gear*distance*scale*np.array([math.cos(goal[2]),math.sin(goal[2])])
        D=goal[:2]-start[:2]
        c3=10*D-6*a-4*b;c4=-15*D+8*a+7*b;c5=6*D-3*a-3*b
        pilot=np.linspace(0,1,81)[:,None]
        speed_bound=max(np.linalg.norm(a+3*c3*pilot**2+4*c4*pilot**3+5*c5*pilot**4,axis=1))*1.05
        n=max(50,int(math.ceil(speed_bound/.012))+1)
        if n>12000:continue
        u=np.linspace(0,1,n)[:,None]
        xy=start[:2]+a*u+c3*u**3+c4*u**4+c5*u**5
        der=a+3*c3*u**2+4*c4*u**3+5*c5*u**4
        dd=6*c3*u+12*c4*u**2+20*c5*u**3
        speed=np.linalg.norm(der,axis=1)
        if min(speed)<distance*.06:continue
        curvature=(der[:,0]*dd[:,1]-der[:,1]*dd[:,0])/speed**3
        if max(abs(curvature))>grid.cfg.max_path_curvature*.85:continue
        yaw=wrap(np.arctan2(der[:,1],der[:,0])+(np.pi if gear<0 else 0))
        poses=np.c_[xy,yaw];poses[0]=start;poses[-1]=goal
        if checked_curve(grid,poses,curvature):out.append(as_samples(poses,gear,grid.cfg))
    return out


def path_metrics(samples):
    groups=[]
    for _,v,w,_ in samples[1:]:
        kind='spin' if abs(v)<1e-8 and abs(w)>1e-8 else 'forward' if v>0 else 'reverse' if v<0 else 'idle'
        if kind!='idle' and (not groups or groups[-1]!=kind):groups.append(kind)
    p=np.array([r[0] for r in samples]);ds=np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1)
    return dict(length_m=float(sum(ds)),motion_groups=groups,spin_groups=groups.count('spin'),
                direction_switches=sum(a!=b for a,b in zip([x for x in groups if x!='spin'],[x for x in groups if x!='spin'][1:])))
