"""Paper-inspired two-stage numerical CG, adapted to differential drive.

Not an exact Dolgov reproduction. Final candidates need footprint validation.
"""
import math
import time
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.ndimage import map_coordinates
from scipy.optimize import minimize
from path_geometry import wrap,checked_curve,as_samples,path_metrics


def _field(array,xy,grid,cval):
    ij=(xy-grid.origin)/grid.res-.5
    return map_coordinates(array,[ij[...,1].ravel(),ij[...,0].ravel()],order=1,
                           mode='constant',cval=cval,prefilter=False).reshape(xy.shape[:-1])


def _objective(xy,seed,grid):
    delta=np.diff(xy,axis=0);length=np.maximum(np.linalg.norm(delta,axis=1),1e-5)
    tangent=delta/length[:,None]
    mid=np.vstack([tangent[0],tangent[:-1]+tangent[1:],tangent[-1]])
    mid/=np.maximum(np.linalg.norm(mid,axis=1),1e-6)[:,None]
    normal=np.c_[-mid[:,1],mid[:,0]]
    hl=grid.cfg.length/2+grid.cfg.margin;hw=grid.cfg.width/2+grid.cfg.margin
    offsets=np.array([[-hl,-hw],[-hl,hw],[hl,-hw],[hl,hw],[0,-hw],[0,hw],[-hl,0],[hl,0]])
    body=xy[:,None,:]+offsets[None,:,0,None]*mid[:,None,:]+offsets[None,:,1,None]*normal[:,None,:]
    clearance=_field(grid.distance,body,grid,0.)
    voronoi=_field(grid.field,body[:,:4],grid,1.).max(axis=1)
    angle=np.arctan2(tangent[:-1,0]*tangent[1:,1]-tangent[:-1,1]*tangent[1:,0],np.sum(tangent[:-1]*tangent[1:],axis=1))
    curvature=angle/np.maximum(.5*(length[:-1]+length[1:]),1e-5)
    h=max(float(np.mean(length)),.03)
    smooth=np.sum(np.diff(xy,n=2,axis=0)**2)/h**3
    return float(.15*sum(length)+.6*np.mean(voronoi)+80*np.mean(np.maximum(.07-clearance,0)**2)
                 +12*np.mean(np.maximum(abs(curvature)-grid.cfg.max_path_curvature*.85,0)**2)
                 +.35*smooth+.25*np.mean(np.sum((xy-seed)**2,axis=1)))


def _cg(seed,fixed,grid,deadline,iterations):
    free=np.flatnonzero(~fixed);shape=(len(free),2);initial=_objective(seed,seed,grid);best=[seed.copy(),initial]
    count=0
    class Budget(Exception):pass
    def fun(x):
        nonlocal count
        if time.perf_counter()>deadline:raise Budget()
        p=seed.copy();p[free]=x.reshape(shape);value=_objective(p,seed,grid);count+=1
        if value<best[1]:best[:]=[p.copy(),value]
        return value
    if not len(free):return seed,dict(evaluations=0,improved=False)
    reason='iteration_limit_or_tolerance'
    try:
        res=minimize(fun,seed[free].ravel(),method='CG',options=dict(maxiter=iterations,gtol=1e-3,eps=1e-4))
        reason=str(res.message)
    except Budget:reason='wall_time_budget'
    return best[0],dict(evaluations=count,initial_objective=initial,final_objective=best[1],reason=reason,improved=bool(best[1]<initial-1e-8))


def _densify(xy,start,end,gear,grid):
    arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(xy,axis=0),axis=1))]
    if min(np.diff(arc))<1e-6:return None
    tangent=lambda p:gear*np.array([math.cos(p[2]),math.sin(p[2])])
    spline=CubicSpline(arc,xy,bc_type=((1,tangent(start)),(1,tangent(end))))
    pilot=np.linspace(0,arc[-1],max(50,len(xy)*8));rate=max(np.linalg.norm(spline(pilot,1),axis=1))
    u=np.linspace(0,arc[-1],max(3,int(math.ceil(arc[-1]*max(1.,rate)*1.1/.01))+1))
    curve=spline(u);der=spline(u,1);dd=spline(u,2);norm=np.linalg.norm(der,axis=1)
    if min(norm)<.15:return None
    curvature=(der[:,0]*dd[:,1]-der[:,1]*dd[:,0])/norm**3
    poses=np.c_[curve,wrap(np.arctan2(der[:,1],der[:,0])+(np.pi if gear<0 else 0))]
    poses[0]=start;poses[-1]=end
    if not checked_curve(grid,poses,curvature):return None
    return as_samples(poses,gear,grid.cfg)


def smooth_samples(grid,samples):
    deadline=time.perf_counter()+grid.cfg.optimizer_seconds
    out=[samples[0]];reports=[];i=1
    while i<len(samples):
        if abs(samples[i][1])<1e-8:out.append(samples[i]);i+=1;continue
        gear=1 if samples[i][1]>0 else -1;j=i
        while j<len(samples) and samples[j][1]*gear>1e-8:j+=1
        original=samples[i:j];poses=np.array([samples[i-1][0]]+[x[0] for x in original])
        arc=np.r_[0,np.cumsum(np.linalg.norm(np.diff(poses[:,:2],axis=0),axis=1))]
        candidate=None;report=dict(gear=gear,accepted=False)
        if arc[-1]>.25 and np.all(np.diff(arc)>1e-8) and time.perf_counter()<deadline:
            knots=np.linspace(0,arc[-1],min(48,max(6,int(math.ceil(arc[-1]/.30))+1)))
            seed=np.c_[np.interp(knots,arc,poses[:,0]),np.interp(knots,arc,poses[:,1])]
            fixed=np.zeros(len(seed),bool);fixed[[0,1,-2,-1]]=True
            coarse,cg=_cg(seed,fixed,grid,min(deadline,time.perf_counter()+grid.cfg.optimizer_seconds*.65),45)
            fine=np.empty((2*len(coarse)-1,2));fine[::2]=coarse;fine[1::2]=(coarse[:-1]+coarse[1:])/2
            fixed_fine=np.zeros(len(fine),bool);fixed_fine[::2]=True
            fine,cg2=_cg(fine,fixed_fine,grid,deadline,25)
            raw_u=np.linspace(0,arc[-1],max(6,int(math.ceil(arc[-1]/.08))+1))
            raw_xy=np.c_[np.interp(raw_u,arc,poses[:,0]),np.interp(raw_u,arc,poses[:,1])]
            for kind,xy in [('cg_fine',fine),('cg_coarse',coarse),('cg_blend',.5*coarse+.5*seed),('original_interpolation',raw_xy)]:
                candidate=_densify(xy,poses[0],poses[-1],gear,grid)
                if candidate is not None:report.update(accepted=True,selected=kind);break
            report.update(coarse_cg=cg,fine_cg=cg2)
        if candidate is None:out.extend(original);report['selected']='original_safe_search_segment'
        else:out.extend(candidate)
        reports.append(report);i=j
    return out,dict(method='Two-stage numerical CG + checked interpolation; differential variant, not exact reproduction',
                    groups=reports,move_groups_smoothed=sum(r['accepted'] for r in reports),
                    move_groups_kept_original=sum(not r['accepted'] for r in reports),
                    before=path_metrics(samples),after=path_metrics(out),
                    safety='Sampled full footprint and curvature checks; no dynamics/continuous-CCD guarantee')
