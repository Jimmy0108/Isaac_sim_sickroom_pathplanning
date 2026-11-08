"""Collision-checked, gear-preserving cubic path postprocessing, not dynamics."""
import math
import numpy as np
from scipy.interpolate import CubicSpline
from slam_core import wrap


def smooth_samples(grid,samples):
    """Never merge through a spin/cusp; reject unsafe candidates per move group."""
    out=[samples[0]];accepted=0;rejected=0;i=1
    while i<len(samples):
        if abs(samples[i][1])<1e-8:
            out.append(samples[i]);i+=1;continue
        sign=1 if samples[i][1]>0 else -1;j=i
        while j<len(samples) and samples[j][1]*sign>1e-8:j+=1
        original=samples[i:j]
        pts=np.array([samples[i-1][0]]+[x[0] for x in original],float)
        arc=np.r_[0.,np.cumsum(np.linalg.norm(np.diff(pts[:,:2],axis=0),axis=1))]
        candidate=None
        if len(pts)>=5 and arc[-1]>.35 and np.all(np.diff(arc)>1e-8):
            for spacing in (.45,.30,.20):
                knots=np.linspace(0.,arc[-1],max(3,int(math.ceil(arc[-1]/spacing))+1))
                xy=np.column_stack([np.interp(knots,arc,pts[:,k]) for k in (0,1)])
                # Tangents preserve vehicle heading and gear at both boundaries.
                tangent=lambda p:sign*np.array([math.cos(p[2]),math.sin(p[2])])
                spline=CubicSpline(knots,xy,bc_type=((1,tangent(pts[0])),(1,tangent(pts[-1]))))
                u=np.linspace(0.,arc[-1],max(3,int(math.ceil(arc[-1]/.01))+1))
                curve=spline(u);der=spline(u,1);dd=spline(u,2)
                norms=np.linalg.norm(der,axis=1)
                if np.any(norms<.2):continue
                yaw=wrap(np.arctan2(der[:,1],der[:,0])+(math.pi if sign<0 else 0))
                poses=np.column_stack([curve,yaw]);poses[0]=pts[0];poses[-1]=pts[-1]
                curvature=(der[:,0]*dd[:,1]-der[:,1]*dd[:,0])/norms**3
                if max(abs(curvature))>2.5:continue
                if any(grid.collision(p) for p in poses):continue
                # Reject any candidate that doubles back or makes a sharp yaw
                # jump between the densely sampled footprint checks.
                if max(abs(wrap(np.diff(poses[:,2]))))>.04:continue
                ds=np.linalg.norm(np.diff(curve,axis=0),axis=1)
                if max(ds)>.015:continue
                candidate=[]
                for k in range(1,len(poses)):
                    dyaw=float(wrap(poses[k,2]-poses[k-1,2]))
                    duration=max(float(ds[k-1])/grid.cfg.nominal_v,abs(dyaw)/grid.cfg.nominal_w,1e-6)
                    candidate.append((poses[k].copy(),sign*float(ds[k-1])/duration,dyaw/duration,duration))
                break
        if candidate is None:out.extend(original);rejected+=1
        else:out.extend(candidate);accepted+=1
        i=j
    return out,dict(move_groups_smoothed=accepted,move_groups_kept_original=rejected,
        policy='Gear/spin/end poses preserved; sampled full-footprint check at ~1 cm. Unsafe spline falls back to original segment; no continuous collision or dynamics guarantee.')
