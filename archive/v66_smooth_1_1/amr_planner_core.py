"""Offline differential-drive Hybrid A* + raster generalized Voronoi field.
No Isaac Sim imports, motor IO, SLAM, or dynamic obstacle avoidance.
"""
from __future__ import annotations
import argparse
import csv
import heapq
import itertools
import json
import math
import time
from dataclasses import dataclass, asdict
from pathlib import Path
import numpy as np
from scipy.ndimage import distance_transform_edt, binary_dilation


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class Config:
    # Current modeled geometry, not factory runtime limits.
    length: float = 0.898
    width: float = 0.5698
    track: float = 0.4769668332689978
    wheel_radius: float = 0.08
    margin: float = 0.04
    xy_bin: float = 0.10
    yaw_bins: int = 72
    primitive_seconds: float = 0.65
    nominal_v: float = 0.35
    nominal_w: float = 0.80
    collision_step: float = 0.025
    field_weight: float = 3.0
    field_alpha: float = 0.4
    field_range: float = 1.0
    rotation_cost: float = 0.20
    reverse_multiplier: float = 1.3
    switch_cost: float = 0.15
    control_change_cost: float = 0.03
    heuristic_weight: float = 1.2
    max_expansions: int = 60000
    timeout_seconds: float = 45.0
    connector_distance: float = 0.55

    def validate(self):
        for k in ('length','width','track','wheel_radius','xy_bin','primitive_seconds',
                  'nominal_v','nominal_w','collision_step','field_alpha','field_range'):
            if not math.isfinite(getattr(self,k)) or getattr(self,k) <= 0:
                raise ValueError(f'{k} must be finite and positive')
        for k in ('margin','field_weight','rotation_cost','switch_cost','control_change_cost'):
            if not math.isfinite(getattr(self,k)) or getattr(self,k) < 0:
                raise ValueError(f'{k} must be finite and nonnegative')
        if self.reverse_multiplier < 1 or self.heuristic_weight < 1 or self.yaw_bins < 8:
            raise ValueError('Invalid search parameters')


def wheels(v, w, cfg):
    """Logical LEFT/RIGHT viewed toward the lidar/front, positive means forward.
    Do NOT assign these directly to the vendor +X joint names/signs.
    """
    return (v-w*cfg.track/2)/cfg.wheel_radius, (v+w*cfg.track/2)/cfg.wheel_radius


def integrate(p, v, w, dt):
    x,y,a = p
    if abs(w) < 1e-12:
        return np.array([x+v*dt*math.cos(a), y+v*dt*math.sin(a), wrap(a)])
    b = a+w*dt
    return np.array([x+v/w*(math.sin(b)-math.sin(a)),
                     y-v/w*(math.cos(b)-math.cos(a)), wrap(b)])


class Grid:
    def __init__(self, occupied, resolution, origin, cfg):
        self.occupied = np.asarray(occupied, dtype=bool).copy()
        self.res = float(resolution)
        self.origin = np.asarray(origin, dtype=float)
        self.cfg = cfg
        if self.occupied.ndim != 2 or min(self.occupied.shape)<3 or self.res<=0:
            raise ValueError('Invalid grid shape/resolution')
        if self.origin.shape != (2,) or not np.all(np.isfinite(self.origin)):
            raise ValueError('Invalid origin')
        self.h,self.w = self.occupied.shape
        # Treat map edges as occupied. Row 0 is the LOWEST world Y.
        self.occupied[[0,-1],:] = True
        self.occupied[:,[0,-1]] = True
        self.distance,self.nearest = distance_transform_edt(~self.occupied,
                                        sampling=self.res, return_indices=True)
        self.voronoi = self._voronoi_edges()
        if not np.any(self.voronoi):
            raise ValueError('No raster Voronoi edges found; inspect map')
        self.dv = distance_transform_edt(~self.voronoi, sampling=self.res)
        d = self.distance
        self.field = (cfg.field_alpha/(cfg.field_alpha+d) * self.dv/(d+self.dv+1e-12)
                      * (np.maximum(0,cfg.field_range-d)/cfg.field_range)**2)
        self.field[self.occupied] = 1.0
        self.obstacle_y,self.obstacle_x = np.nonzero(self.occupied)

    def _voronoi_edges(self):
        # Raster GVD approximation: adjacent free cells have well-separated
        # nearest obstacle sites on different sides of the query location.
        # Not merely a local maximum filter or inverse obstacle distance.
        edge = np.zeros_like(self.occupied)
        yy,xx = np.indices(self.occupied.shape)
        for dy,dx in ((0,1),(1,0),(1,1),(1,-1)):
            a=(slice(max(0,-dy),min(self.h,self.h-dy)),slice(max(0,-dx),min(self.w,self.w-dx)))
            b=(slice(max(0,dy),min(self.h,self.h+dy)),slice(max(0,dx),min(self.w,self.w+dx)))
            ay,ax=self.nearest[0][a],self.nearest[1][a]
            by,bx=self.nearest[0][b],self.nearest[1][b]
            qy,qx=yy[a]+dy/2,xx[a]+dx/2
            u0,u1=ay-qy,ax-qx
            v0,v1=by-qy,bx-qx
            denom=np.hypot(u0,u1)*np.hypot(v0,v1)+1e-12
            separated=np.hypot(ay-by,ax-bx)*self.res > max(3*self.res,0.18)
            opposed=(u0*v0+u1*v1)/denom < 0.5
            good=separated & opposed & ~self.occupied[a] & ~self.occupied[b]
            edge[a] |= good
            edge[b] |= good
        return edge

    def cell(self,p):
        x,y=np.floor((np.asarray(p[:2])-self.origin)/self.res).astype(int)
        return int(y),int(x)

    def field_at(self,p):
        y,x=self.cell(p)
        return float(self.field[y,x]) if 0<=y<self.h and 0<=x<self.w else 1.0

    def collision(self,p):
        if not np.all(np.isfinite(p)):
            return True
        x,y,a=p
        hl=self.cfg.length/2+self.cfg.margin
        hw=self.cfg.width/2+self.cfg.margin
        c,s=math.cos(a),math.sin(a)
        ex,ey=hl*abs(c)+hw*abs(s),hl*abs(s)+hw*abs(c)
        # Entire oriented body must be inside map bounds.
        if x-ex < self.origin[0] or y-ey < self.origin[1] or x+ex >= self.origin[0]+self.w*self.res or y+ey >= self.origin[1]+self.h*self.res:
            return True
        cy,cx=self.cell(p)
        radius=math.hypot(hl,hw)
        if self.distance[cy,cx] > radius+math.sqrt(2)*self.res:
            return False
        xmin=max(0,int(math.floor((x-ex-self.origin[0])/self.res)))
        xmax=min(self.w-1,int(math.floor((x+ex-self.origin[0])/self.res)))
        ymin=max(0,int(math.floor((y-ey-self.origin[1])/self.res)))
        ymax=min(self.h-1,int(math.floor((y+ey-self.origin[1])/self.res)))
        oy,ox=np.nonzero(self.occupied[ymin:ymax+1,xmin:xmax+1])
        if not len(oy):
            return False
        dx=self.origin[0]+(ox+xmin+0.5)*self.res-x
        dy=self.origin[1]+(oy+ymin+0.5)*self.res-y
        # SAT on all four axes of oriented body rectangle vs occupied cell squares.
        half=self.res/2
        overlap=(np.abs(dx)<=ex+half)&(np.abs(dy)<=ey+half)
        overlap &= np.abs(dx*c+dy*s)<=hl+half*(abs(c)+abs(s))
        overlap &= np.abs(-dx*s+dy*c)<=hw+half*(abs(c)+abs(s))
        return bool(np.any(overlap))

    def edge(self,p,v,w,duration):
        corner=math.hypot(self.cfg.length/2+self.cfg.margin,self.cfg.width/2+self.cfg.margin)
        n=max(1,int(math.ceil((abs(v)+corner*abs(w))*duration/min(self.cfg.collision_step,self.res/2))))
        dt=duration/n
        samples=[]
        for i in range(1,n+1):
            q=integrate(p,v,w,i*dt)
            if self.collision(q):
                return None
            samples.append((q,v,w,dt))
        return samples


@dataclass
class Node:
    pose: np.ndarray
    g: float
    parent: object
    samples: object
    action: int


def edge_cost(samples, old_action, action, cfg):
    _,v,w,_=samples[0][:4]
    duration=sum(row[3] for row in samples)
    distance=abs(v)*duration
    turn=abs(w)*duration
    geometric=distance*(cfg.reverse_multiplier if v<0 else 1)+cfg.rotation_cost*turn
    field_mean=sum(row[4] for row in samples)/len(samples) if len(samples[0])>4 else 0.0
    return geometric+cfg.field_weight*(distance+cfg.rotation_cost*turn)*field_mean + (cfg.control_change_cost if old_action not in (-1,action) else 0)


def _turn_straight_connector(grid,p,goal,sign):
    cfg=grid.cfg
    diff=goal[:2]-p[:2]
    distance=float(np.linalg.norm(diff))
    a=math.atan2(diff[1],diff[0]) if distance>1e-9 else p[2]
    if sign<0:a=wrap(a+math.pi)
    segments=[]
    current=p.copy()
    for v,w,t in ((0,math.copysign(cfg.nominal_w,wrap(a-current[2])),abs(wrap(a-current[2]))/cfg.nominal_w),
                   (sign*cfg.nominal_v,0,distance/cfg.nominal_v)):
        if t<=1e-9:
            continue
        samples=grid.edge(current,v,w,t)
        if samples is None:
            return None
        segments.extend(samples);current=samples[-1][0]
    angle=wrap(goal[2]-current[2])
    if abs(angle)>1e-9:
        samples=grid.edge(current,0,math.copysign(cfg.nominal_w,angle),abs(angle)/cfg.nominal_w)
        if samples is None:
            return None
        segments.extend(samples)
    return segments


def connector(grid,p,goal):
    """Try both gears and tangent-matched curves, not a forward-only tail."""
    cfg=grid.cfg;distance=float(np.linalg.norm(goal[:2]-p[:2]));candidates=[]
    for sign in (1,-1):
        tail=_turn_straight_connector(grid,p,goal,sign)
        if tail is not None:candidates.append(tail)
        if distance<.03:continue
        a=sign*distance*np.array([math.cos(p[2]),math.sin(p[2])])
        b=sign*distance*np.array([math.cos(goal[2]),math.sin(goal[2])])
        t=np.linspace(0.,1.,max(20,int(math.ceil(distance/.005))+1))[:,None]
        xy=(2*t**3-3*t*t+1)*p[:2]+(t**3-2*t*t+t)*a+(-2*t**3+3*t*t)*goal[:2]+(t**3-t*t)*b
        d=(6*t*t-6*t)*p[:2]+(3*t*t-4*t+1)*a+(-6*t*t+6*t)*goal[:2]+(3*t*t-2*t)*b
        dd=(12*t-6)*p[:2]+(6*t-4)*a+(-12*t+6)*goal[:2]+(6*t-2)*b
        n=np.linalg.norm(d,axis=1)
        if np.any(n<distance*.15):continue
        curvature=(d[:,0]*dd[:,1]-d[:,1]*dd[:,0])/n**3
        if max(abs(curvature))>2.5:continue
        yaw=wrap(np.arctan2(d[:,1],d[:,0])+(math.pi if sign<0 else 0))
        poses=np.c_[xy,yaw];poses[0]=p;poses[-1]=goal
        ds=np.linalg.norm(np.diff(xy,axis=0),axis=1)
        if max(ds)>.015 or max(abs(wrap(np.diff(yaw))))>.04:continue
        if any(grid.collision(q) for q in poses):continue
        tail=[]
        for k in range(1,len(poses)):
            angle=float(wrap(poses[k,2]-poses[k-1,2]))
            dt=max(ds[k-1]/cfg.nominal_v,abs(angle)/cfg.nominal_w,1e-6)
            tail.append((poses[k].copy(),sign*ds[k-1]/dt,angle/dt,dt))
        candidates.append(tail)
    if not candidates:return None
    def cost(rows):
        spin_groups=0;was_spin=False
        for _,v,w,_ in rows:
            spin=abs(v)<1e-8 and abs(w)>1e-8
            if spin and not was_spin:spin_groups+=1
            was_spin=spin
        return cfg.switch_cost*spin_groups+sum((abs(v)*dt*(cfg.reverse_multiplier if v<0 else 1)+cfg.rotation_cost*abs(w)*dt)*(1+cfg.field_weight*grid.field_at(q)) for q,v,w,dt in rows)
    return min(candidates,key=cost)


def plan(grid,start,goal):
    cfg=grid.cfg
    start,goal=np.array(start,dtype=float),np.array(goal,dtype=float)
    if start.shape!=(3,) or goal.shape!=(3,) or not np.all(np.isfinite(start)) or not np.all(np.isfinite(goal)):
        raise ValueError('Start/goal must be finite x,y,yaw')
    start[2],goal[2]=wrap(start[2]),wrap(goal[2])
    if grid.collision(start):
        raise ValueError('Start footprint collides; check scale, origin, margin and placement')
    if grid.collision(goal):
        raise ValueError('Goal footprint collides; changing robot size is not a valid fix')
    def key(p,action):
        return (int(math.floor((p[0]-grid.origin[0])/cfg.xy_bin)),int(math.floor((p[1]-grid.origin[1])/cfg.xy_bin)),
                int(round((wrap(p[2])+math.pi)/(2*math.pi)*cfg.yaw_bins))%cfg.yaw_bins,action)
    # Obstacle-aware 2D guide on an optimistic inscribed-disc centre map.
    # Still weighted/raster search: no claim of global optimality.
    guide=np.full(grid.occupied.shape,np.inf)
    gy,gx=grid.cell(goal);guide[gy,gx]=0.;frontier=[(0.,gy,gx)]
    blocked=grid.occupied | (grid.distance < min(cfg.length,cfg.width)/2+cfg.margin)
    blocked[gy,gx]=False
    while frontier:
        cost,y,x=heapq.heappop(frontier)
        if cost>guide[y,x]+1e-9:continue
        for dy,dx in ((0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)):
            ny,nx=y+dy,x+dx
            if not (0<=ny<grid.h and 0<=nx<grid.w) or blocked[ny,nx]:continue
            if dy and dx and (blocked[y,nx] or blocked[ny,x]):continue
            nc=cost+math.hypot(dx,dy)*grid.res
            if nc+1e-9<guide[ny,nx]:guide[ny,nx]=nc;heapq.heappush(frontier,(nc,ny,nx))
    def heuristic(p):
        y,x=grid.cell(p);g=guide[y,x]
        if not np.isfinite(g):g=np.linalg.norm(p[:2]-goal[:2])+3.
        return max(float(g),np.linalg.norm(p[:2]-goal[:2]))+cfg.rotation_cost*abs(wrap(p[2]-goal[2]))
    actions=[(sign*cfg.nominal_v,w) for sign in (1,-1) for w in (-cfg.nominal_w,-cfg.nominal_w/2,0,cfg.nominal_w/2,cfg.nominal_w)]
    actions += [(0,-cfg.nominal_w),(0,cfg.nominal_w)]
    root=Node(start,0,None,[], -1)
    ids=itertools.count();queue=[(cfg.heuristic_weight*heuristic(start),next(ids),root)]
    best={key(start,-1):0.0}
    begin=time.perf_counter();expansions=0;explored=[]
    while queue and expansions<cfg.max_expansions and time.perf_counter()-begin<cfg.timeout_seconds:
        _,_,node=heapq.heappop(queue)
        if node.g>best.get(key(node.pose,node.action),float('inf'))+1e-9:
            continue
        expansions+=1
        if expansions%5==0: explored.append(node.pose.copy())
        if np.linalg.norm(node.pose[:2]-goal[:2])<=cfg.connector_distance:
            tail=connector(grid,node.pose,goal)
            if tail is not None:
                chain=[];cur=node
                while cur.parent is not None:
                    chain.append(cur.samples);cur=cur.parent
                samples=[(start,0.0,0.0,0.0)]
                for part in reversed(chain):samples.extend(part)
                samples.extend(tail)
                return samples,dict(expansions=expansions,seconds=time.perf_counter()-begin,explored=np.array(explored))
        for action,(v,w) in enumerate(actions):
            samples=grid.edge(node.pose,v,w,cfg.primitive_seconds)
            if samples is None:continue
            valued=[(*row,grid.field_at(row[0])) for row in samples]
            g=node.g+edge_cost(valued,node.action,action,cfg)
            if node.parent is not None:
                old_v=actions[node.action][0]
                if old_v*v<0:g+=cfg.switch_cost
            q=samples[-1][0];k=key(q,action)
            if g>=best.get(k,float('inf'))-1e-9:continue
            best[k]=g
            new=Node(q,g,node,samples,action)
            heapq.heappush(queue,(g+cfg.heuristic_weight*heuristic(q),next(ids),new))
    raise RuntimeError(f'No solution within budget: {expansions} expansions, {time.perf_counter()-begin:.1f}s. This is not proof of infeasibility.')


def demo_map(cfg):
    res=0.05;occ=np.zeros((140,200),bool)
    def rect(x0,y0,x1,y1):
        occ[int(y0/res):int(y1/res),int(x0/res):int(x1/res)]=True
    # Synthetic ward, not a reconstruction/measurement of the actual sickroom.
    rect(3.8,2.5,6.0,4.5)  # bed
    rect(7.8,4.8,9.4,6.4)  # cabinet
    rect(0.4,4.8,1.6,6.4)
    return Grid(occ,res,(0,0),cfg),np.array([1.2,1.4,0]),np.array([8.5,3.7,math.pi/2])


def save_outputs(folder,grid,samples,stats,start,goal):
    folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
    samples = list(samples)+[(samples[-1][0].copy(),0.0,0.0,0.0)]  # explicit final stop event
    poses=np.array([s[0] for s in samples]);v=np.array([s[1] for s in samples]);omega=np.array([s[2] for s in samples]);dt=np.array([s[3] for s in samples])
    times=np.cumsum(dt);left,right=wheels(v,omega,grid.cfg)
    np.savez(folder/'planned_path.npz',path=poses[:,:2],poses=poses,nominal_v=v,nominal_omega=omega,interval_dt=dt,
             nominal_time=times,base_link_v=-v,base_link_omega=omega,logical_left_rad_s=left,logical_right_rad_s=right,origin=grid.origin,resolution=grid.res,
             occupied=grid.occupied,goal=goal,start=start,config_json=json.dumps(asdict(grid.cfg)),command_semantics='row i applies on interval (time[i-1], time[i]]; row 0 is initial pose; NOT hardware commands')
    with (folder/'planned_path.csv').open('w',newline='',encoding='utf-8') as f:
        out=csv.writer(f);out.writerow(['nominal_time_s','x_m','y_m','front_yaw_rad','nominal_v_m_s','nominal_omega_rad_s','logical_left_rad_s','logical_right_rad_s'])
        out.writerows(np.c_[times,poses,v,omega,left,right])
    summary={k:val for k,val in stats.items() if k!='explored'}
    summary.update(length_m=float(np.linalg.norm(np.diff(poses[:,:2],axis=0),axis=1).sum()),
                   final_position_error_m=float(np.linalg.norm(poses[-1,:2]-goal[:2])),
                   final_yaw_error_rad=abs(wrap(poses[-1,2]-goal[2])),
                   collision_free_samples=all(not grid.collision(p) for p in poses),
                   reverse_samples=int(np.sum(v<0)),spin_samples=int(np.sum((v==0)&(omega!=0))),
                   mode='OFFLINE geometric plan, nominal controls; no acceleration/dynamics/controller validation',config=asdict(grid.cfg))
    (folder/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    ext=[grid.origin[0],grid.origin[0]+grid.w*grid.res,grid.origin[1],grid.origin[1]+grid.h*grid.res]
    fig,axes=plt.subplots(1,2,figsize=(15,6),layout='constrained')
    for ax in axes:
        ax.imshow(grid.occupied,origin='lower',extent=ext,cmap='Greys',vmin=0,vmax=1)
        ax.set(xlabel='world X [m]',ylabel='world Y [m]',aspect='equal')
    axes[0].imshow(np.ma.masked_where(grid.occupied,grid.field),origin='lower',extent=ext,cmap='magma',alpha=.80,vmin=0,vmax=1)
    yy,xx=np.nonzero(grid.voronoi);axes[0].scatter(grid.origin[0]+(xx+.5)*grid.res,grid.origin[1]+(yy+.5)*grid.res,s=1,c='cyan',label='Raster Voronoi edges')
    explored=stats['explored']
    if len(explored):axes[1].scatter(explored[:,0],explored[:,1],s=1,c='#cccccc')
    for ax in axes:
        ax.plot(poses[:,0],poses[:,1],color='limegreen',lw=2,label='Hybrid A* path')
        ax.scatter([start[0],goal[0]],[start[1],goal[1]],c=['blue','red'],s=35)
        for p in (start,goal):ax.arrow(p[0],p[1],.4*math.cos(p[2]),.4*math.sin(p[2]),width=.025,color='royalblue')
    for p in poses[::max(1,len(poses)//18)]:
        hl,hw=grid.cfg.length/2,grid.cfg.width/2
        corners=np.array([[-hl,-hw],[hl,-hw],[hl,hw],[-hl,hw]])
        c,s=math.cos(p[2]),math.sin(p[2]);corners=corners@np.array([[c,s],[-s,c]])+p[:2]
        axes[1].add_patch(Polygon(corners,fill=False,edgecolor='royalblue',lw=.6,alpha=.6))
    axes[0].set_title('Voronoi field (dark=low cost), cyan=raster GVD')
    axes[1].set_title('OFFLINE plan and oriented robot footprints')
    axes[0].legend(loc='lower right');fig.savefig(folder/'plan.png',dpi=150);plt.close(fig)
    return summary


def main():
    a=argparse.ArgumentParser(description=__doc__)
    a.add_argument('--map',type=Path,help='NPZ: occupied boolean [rowY,colX], origin XY, resolution scalar')
    a.add_argument('--config',type=Path)
    a.add_argument('--start',nargs=3,type=float,metavar=('X','Y','YAW_DEG'))
    a.add_argument('--goal',nargs=3,type=float,metavar=('X','Y','YAW_DEG'))
    a.add_argument('--out',type=Path,default=Path(__file__).parent/'outputs'/'demo')
    args=a.parse_args();cfg=Config(**json.loads(args.config.read_text(encoding='utf-8-sig'))) if args.config else Config();cfg.validate()
    if args.map:
        with np.load(args.map,allow_pickle=False) as d:
            if not {'occupied','origin','resolution'}<=set(d.files):
                raise ValueError('Map must contain occupied, origin, resolution. Previously inflated free masks are not raw obstacle maps.')
            occ=d['occupied']
            if occ.dtype!=np.bool_:raise ValueError('occupied must be boolean; unknown cells must already be marked occupied')
            grid=Grid(occ,float(d['resolution']),d['origin'],cfg)
        if args.start is None or args.goal is None:raise ValueError('--map requires --start and --goal')
        start,goal=np.array(args.start),np.array(args.goal)
        start[2],goal[2]=np.deg2rad(start[2]),np.deg2rad(goal[2])
    else:
        grid,start,goal=demo_map(cfg)
        for name in ('start','goal'):
            value=getattr(args,name)
            if value is not None:
                p=np.array(value);p[2]=np.deg2rad(p[2])
                if name=='start':start=p
                else:goal=p
    samples,stats=plan(grid,start,goal)
    summary=save_outputs(args.out,grid,samples,stats,start,goal)
    print(json.dumps({k:v for k,v in summary.items() if k!='config'},indent=2))
    print('Outputs:',args.out.resolve())


if __name__=='__main__':main()

