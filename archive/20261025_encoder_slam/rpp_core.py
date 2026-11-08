"""RPP-style differential-drive feedback controller, not a Nav2 plugin port."""
import math
import numpy as np
from slam_core import wrap,transform,inverse


def segments_from_samples(samples):
    groups=[]
    for p,v,w,dt in samples[1:]:
        kind=('spin',1 if w>=0 else -1) if abs(v)<1e-8 else ('move',1 if v>0 else -1)
        if groups and groups[-1]['kind']==kind:groups[-1]['points'].append(p)
        else:groups.append(dict(kind=kind,points=[p]))
    for g in groups:g['points']=np.asarray(g['points'])
    return groups


class RPP:
    def __init__(self,cfg):self.cfg=cfg;self.groups=[];self.index=0;self.cursor=0;self.last_v=0.;self.transition=False;self.local_path=np.empty((0,3))
    def set_path(self,samples):
        self.groups=segments_from_samples(samples);self.index=self.cursor=0;self.last_v=0.;self.transition=False
        if self.groups and self.groups[-1]['kind'][0]=='move':
            self.groups.append(dict(kind=('spin',1),points=np.array([samples[-1][0].copy()])))
    def command(self,pose,measured_v,grid,obstacles,dt):
        c=self.cfg
        if self.transition:
            if abs(measured_v)>.03:return 0.,0.,'stopping at direction/rotation boundary'
            self.index+=1;self.cursor=0;self.transition=False
        if self.index>=len(self.groups):return 0.,0.,'done'
        seg=self.groups[self.index];pts=seg['points'];goal=pts[-1]
        if seg['kind'][0]=='spin':
            error=wrap(goal[2]-pose[2])
            if abs(error)<c['yaw_tolerance']:
                self.transition=True;return 0.,0.,'segment complete'
            v=0.;w=float(np.clip(1.5*error,-c['max_w'],c['max_w']))
        else:
            if np.linalg.norm(goal[:2]-pose[:2])<c['position_tolerance']:
                self.transition=True;self.last_v=0.;return 0.,0.,'segment complete'
            end=min(len(pts),self.cursor+35)
            distance=np.linalg.norm(pts[self.cursor:end,:2]-pose[:2],axis=1)
            self.cursor+=int(np.argmin(distance))
            if float(distance.min())>.5:return 0.,0.,'replan: path deviation'
            look=float(np.clip(c['lookahead_time']*abs(measured_v),c['min_lookahead'],c['max_lookahead']))
            index=self.cursor
            while index+1<len(pts) and np.linalg.norm(pts[index,:2]-pose[:2])<look:index+=1
            target=pts[index,:2];base=transform(target[None,:],inverse(pose))[0]
            sign=seg['kind'][1];bearing=math.atan2(target[1]-pose[1],target[0]-pose[0])
            heading_error=wrap(bearing-pose[2]-(math.pi if sign<0 else 0))
            if abs(heading_error)>math.radians(45):
                v=0.;w=float(np.clip(1.5*heading_error,-c['max_w'],c['max_w']))
            else:
                curvature=2*base[1]/max(.01,float(base@base))
                remaining=float(np.linalg.norm(goal[:2]-pose[:2]))
                speed=c['max_v']
                speed=min(speed,c['max_v']/max(1.,c['min_turn_radius']*abs(curvature)))
                speed=min(speed,c['max_v']*remaining/c['approach_distance'])
                speed=min(speed,math.sqrt(2*c['deceleration']*max(0.,remaining-c['position_tolerance'])))
                if len(obstacles):
                    local=transform(obstacles,inverse(pose))
                    clearance=float(np.min(np.linalg.norm(np.maximum(np.abs(local)-[grid.cfg.length/2,grid.cfg.width/2],0),axis=1)))
                    speed=min(speed,c['max_v']*max(0.,clearance-grid.cfg.margin)/c['cost_scaling_distance'])
                speed=min(speed,abs(self.last_v)+c['acceleration']*min(dt,.1))
                if abs(curvature)>1e-6:speed=min(speed,c['max_w']/abs(curvature))
                v=sign*speed;w=v*curvature
        horizon=max(c['collision_time'],.20+max(abs(v),abs(measured_v))/c['deceleration'])
        # Collision projection covers measured wheel speed during deceleration,
        # rather than assuming a reduced command is achieved instantly.
        if abs(measured_v)>.03:
            safety_v=math.copysign(max(abs(v),abs(measured_v)),measured_v)
        else:safety_v=v
        rollout=grid.edge(pose,safety_v,w,horizon)
        if rollout is None:self.last_v=0.;return 0.,0.,'replan: predicted costmap collision'
        self.local_path=np.array([pose]+[row[0] for row in rollout])
        if len(obstacles):
            for q in self.local_path:
                local=transform(obstacles,inverse(q))
                hit=(abs(local[:,0])<grid.cfg.length/2+grid.cfg.margin)&(abs(local[:,1])<grid.cfg.width/2+grid.cfg.margin)
                if hit.any():self.last_v=0.;return 0.,0.,'replan: local obstacle'
        self.last_v=v
        return float(v),float(w),'tracking'
