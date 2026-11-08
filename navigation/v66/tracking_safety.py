"""Check a reaction + braking trajectory, not constant-speed extrapolation.

Engineering stopping model, not a verified PhysX/hardware deceleration model.
Full rectangular grid + live 2D hits are checked at each sampled pose.
"""
import math
import numpy as np
from slam_core import inverse,transform


def braking_rollout(grid,pose,v,w,measured_v,obstacles,cfg):
    from amr_planner_core import integrate
    speed=max(abs(v),abs(measured_v));sign=math.copysign(1.,v if abs(v)>.001 else measured_v if measured_v else 1.)
    reaction=float(cfg.get('reaction_time_s',.15));decel=float(cfg['deceleration'])
    angular_decel=float(cfg.get('max_angular_accel',.8))
    horizon=reaction+max(speed/decel,abs(w)/angular_decel)
    # Adaptive steps bound both body translation and corner rotation.
    radius=math.hypot(grid.cfg.length/2+grid.cfg.margin,grid.cfg.width/2+grid.cfg.margin)
    dt=min(.04,.015/max(speed+radius*abs(w),.02))
    points=[np.asarray(pose,float).copy()];q=points[0].copy();elapsed=0.
    if grid.collision(q):return False,np.asarray(points),'current_footprint'
    if len(obstacles):
        local=transform(obstacles,inverse(q))
        if np.any((abs(local[:,0])<grid.cfg.length/2+grid.cfg.margin)&(abs(local[:,1])<grid.cfg.width/2+grid.cfg.margin)):
            return False,np.asarray(points),'current_lidar'
    while elapsed<horizon-1e-9:
        h=min(dt,horizon-elapsed);mid=elapsed+h/2
        after=max(0.,mid-reaction)
        vv=sign*max(0.,speed-decel*after)
        # Preserve commanded curvature while braking when moving, and brake
        # the spin independently if no translational command is present.
        ww=w*abs(vv)/speed if speed>.02 and abs(v)>.001 else math.copysign(max(0.,abs(w)-angular_decel*after),w)
        q=integrate(q,vv,ww,h)
        if grid.collision(q):return False,np.array(points+[q]),'costmap'
        if len(obstacles):
            local=transform(obstacles,inverse(q))
            if np.any((abs(local[:,0])<grid.cfg.length/2+grid.cfg.margin)&(abs(local[:,1])<grid.cfg.width/2+grid.cfg.margin)):
                return False,np.array(points+[q]),'lidar'
        points.append(q.copy());elapsed+=h
    return True,np.asarray(points),'clear'
