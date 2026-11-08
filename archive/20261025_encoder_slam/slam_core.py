"""SE(2), wheel odometry, scan matching and experimental pose-graph 2D SLAM.
No simulator, USD, controller commands or ground-truth pose inputs.
"""
from dataclasses import dataclass
import math
import numpy as np
from scipy.spatial import cKDTree
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from occupancy import Mapper


def wrap(a):return (a+np.pi)%(2*np.pi)-np.pi

def compose(a,b):
    c,s=np.cos(a[2]),np.sin(a[2])
    return np.array([a[0]+c*b[0]-s*b[1],a[1]+s*b[0]+c*b[1],wrap(a[2]+b[2])])

def inverse(a):
    c,s=np.cos(a[2]),np.sin(a[2])
    return np.array([-c*a[0]-s*a[1],s*a[0]-c*a[1],wrap(-a[2])])

def between(a,b):return compose(inverse(a),b)

def transform(points,p):
    c,s=np.cos(p[2]),np.sin(p[2]);return np.asarray(points)@np.array([[c,s],[-s,c]])+p[:2]


def voxel(points,size=.04):
    if not len(points):return np.empty((0,2))
    _,idx=np.unique(np.floor(points/size).astype(np.int64),axis=0,return_index=True)
    return points[np.sort(idx)]


class WheelOdom:
    def __init__(self,radius=.09,track=.514):
        self.radius=radius;self.track=track;self.pose=np.zeros(3);self.previous=None;self.time=None;self.v=self.w=0.
    def update(self,left_angle,right_angle,t):
        q=np.array([left_angle,right_angle],dtype=float)
        if not np.all(np.isfinite(q)):raise ValueError('Nonfinite actual encoder angles')
        if self.previous is None:self.previous=q;self.time=t;return self.pose.copy()
        dt=t-self.time
        if dt<=0:
            if dt<0:raise RuntimeError('Simulation clock reset; start a new SLAM session.')
            return self.pose.copy()
        delta=wrap(q-self.previous)
        if np.any(abs(delta)>1.8):raise RuntimeError('Encoder jump exceeds safe per-update interpretation.')
        dl,dr=delta*self.radius;distance=(dl+dr)/2;angle=(dr-dl)/self.track
        body=np.array([distance,0.,angle]) if abs(angle)<1e-10 else np.array([distance/angle*np.sin(angle),distance/angle*(1-np.cos(angle)),angle])
        self.pose=compose(self.pose,body);self.v=distance/dt;self.w=angle/dt;self.previous=q;self.time=t
        return self.pose.copy()


@dataclass
class Match:
    pose: np.ndarray
    rmse: float
    overlap: float
    accepted: bool


def match_scan(scan,reference,prior,max_distance=.30,max_translation=.30,max_yaw=.25):
    src=voxel(np.asarray(scan),.04);ref=voxel(np.asarray(reference),.04)
    if len(src)<25 or len(ref)<25:return Match(np.array(prior),float('inf'),0.,False)
    tree=cKDTree(ref);p=np.array(prior,dtype=float)
    for _ in range(18):
        current=transform(src,p);dist,index=tree.query(current)
        good=dist<max_distance
        if good.sum()<25:break
        cutoff=np.quantile(dist[good],.85);good&=dist<=max(.015,cutoff)
        x=current[good];y=ref[index[good]]
        weight=1/np.maximum(.025,dist[good]);weight/=weight.sum()
        cx=(x*weight[:,None]).sum(0);cy=(y*weight[:,None]).sum(0)
        h=(x-cx).T@((y-cy)*weight[:,None]);u,_,vt=np.linalg.svd(h)
        rot=vt.T@u.T
        if np.linalg.det(rot)<0:vt[-1]*=-1;rot=vt.T@u.T
        shift=cy-rot@cx;angle=math.atan2(rot[1,0],rot[0,0])
        p=compose(np.r_[shift,angle],p)
        if np.linalg.norm(shift)<1e-4 and abs(angle)<1e-4:break
    dist,_=tree.query(transform(src,p));good=dist<max_distance
    overlap=float(good.mean());rmse=float(np.sqrt(np.mean(dist[good]**2))) if good.any() else float('inf')
    correction=between(prior,p)
    accepted=overlap>.45 and rmse<.10 and np.linalg.norm(correction[:2])<max_translation and abs(correction[2])<max_yaw
    return Match(p,rmse,overlap,accepted)


def optimize_graph(initial,edges):
    initial=np.array(initial,dtype=float)
    if len(initial)<2:return initial
    fixed=initial[0].copy()
    def unpack(x):return np.vstack((fixed,x.reshape(-1,3)))
    def residual(x):
        poses=unpack(x);out=[]
        for i,j,z,sigma,kind in edges:
            e=between(z,between(poses[i],poses[j]));out.extend(e/np.asarray(sigma))
        return np.asarray(out)
    sparse=lil_matrix((3*len(edges),3*(len(initial)-1)),dtype=int)
    for k,(i,j,*_) in enumerate(edges):
        for n in (i,j):
            if n>0:sparse[3*k:3*k+3,3*(n-1):3*n]=1
    result=least_squares(residual,initial[1:].ravel(),jac_sparsity=sparse.tocsr(),loss='soft_l1',f_scale=1.,max_nfev=60)
    if not np.all(np.isfinite(result.x)):raise RuntimeError('Nonfinite pose graph optimization')
    before=np.sum(residual(initial[1:].ravel())**2);after=np.sum(residual(result.x)**2)
    if after>before+1e-5:raise RuntimeError('Pose graph optimizer increased residual')
    return unpack(result.x)


class Slam2D:
    """Worker-owned state. Runtime consumes completed result snapshots only."""
    def __init__(self,cfg):
        self.cfg=cfg;self.nodes=[];self.scans=[];self.odoms=[];self.edges=[];self.map_odom=np.zeros(3)
        self.failures=0;self.loops=0;self.revision=0;self.pending=None;self.last_loop=-100;self.last_rmse=0.;self.loaded=False;self.relocalized=False;self.seed_hint=None
        n=int(math.ceil(cfg['map_size_m']/cfg['map_resolution']))
        self.mapper=Mapper((-n*cfg['map_resolution']/2,)*2,(n,n),cfg['map_resolution'])
        self.sensor=np.array(cfg['sensor_origin_front_m'],float)
    def reference(self,indices):
        chunks=[transform(self.scans[i],self.nodes[i]) for i in indices]
        return voxel(np.vstack(chunks),.04) if chunks else np.empty((0,2))
    def integrate_keyframe(self,i):
        pose=self.nodes[i];cloud=transform(self.scans[i],pose);sensor=transform(self.sensor[None,:2],pose)[0]
        points=np.c_[cloud,np.full(len(cloud),self.sensor[2])]
        self.mapper.update(np.r_[sensor,self.sensor[2]],points,pose,0.,self.cfg['body'],self.cfg['scan_max_range'])
    def process(self,scan,odom):
        scan=voxel(np.asarray(scan),.035);odom=np.array(odom)
        if len(scan)<25:return self.result(odom,False,'too few valid scan returns')
        if self.loaded:
            prior=compose(self.map_odom,odom) if self.relocalized else np.array(self.seed_hint)
            fit=match_scan(scan,self.reference(range(len(self.nodes))),prior,.4,.6,.4)
            self.last_rmse=fit.rmse
            if fit.accepted:
                self.map_odom=compose(fit.pose,inverse(odom));self.failures=0;self.relocalized=True
            else:self.failures+=1
            return self.result(odom,fit.accepted,'loaded-map localization' if fit.accepted else 'initial pose / scan matching rejected')
        if not self.nodes:
            self.nodes=[np.zeros(3)];self.scans=[scan.copy()];self.odoms=[odom.copy()]
            self.map_odom=compose(self.nodes[0],inverse(odom));self.integrate_keyframe(0)
            return self.result(odom,True,'initialized at map origin')
        prior=compose(self.map_odom,odom)
        ref=self.reference(range(max(0,len(self.nodes)-6),len(self.nodes)))
        fit=match_scan(scan,ref,prior)
        self.last_rmse=fit.rmse
        if not fit.accepted:
            self.failures+=1;return self.result(odom,False,f'scan match rejected: overlap={fit.overlap:.2f}, rmse={fit.rmse:.3f}')
        self.failures=0;self.map_odom=compose(fit.pose,inverse(odom))
        change=between(self.nodes[-1],fit.pose)
        if np.linalg.norm(change[:2])<.18 and abs(change[2])<.15:return self.result(odom,True,'tracking')
        i=len(self.nodes);self.nodes.append(fit.pose.copy());self.scans.append(scan.copy());self.odoms.append(odom.copy())
        # Separate measured odom and scan-derived constraints; this prototype does not model their full correlation.
        self.edges.append((i-1,i,between(self.odoms[i-1],odom),[.04,.04,.06],'odom'))
        self.edges.append((i-1,i,between(self.nodes[i-1],fit.pose),[.035,.035,.04],'scan'))
        loop_added=False
        if i>=14 and i-self.last_loop>=8:
            candidates=[j for j in range(i-12) if np.linalg.norm(self.nodes[j][:2]-fit.pose[:2])<.9]
            if candidates:
                j=min(candidates,key=lambda k:np.linalg.norm(self.nodes[k][:2]-fit.pose[:2]))
                loop_ref=self.reference(range(max(0,j-1),min(i-10,j+3)))
                check=match_scan(scan,loop_ref,fit.pose,.45,.55,.35)
                if check.accepted and check.rmse<.065 and check.overlap>.65:
                    if self.pending is not None and abs(self.pending-j)<=3:
                        self.edges.append((j,i,between(self.nodes[j],check.pose),[.025,.025,.03],'loop'))
                        optimized=optimize_graph(self.nodes,self.edges)
                        self.nodes=[p.copy() for p in optimized];self.map_odom=compose(self.nodes[-1],inverse(odom))
                        self.loops+=1;self.revision+=1;self.last_loop=i;self.pending=None;loop_added=True
                    else:self.pending=j
                else:self.pending=None
            else:self.pending=None
        if loop_added:
            self.mapper.log.fill(0);self.mapper.seen.fill(False);self.mapper.scans=0
            for n in range(len(self.nodes)):self.integrate_keyframe(n)
        else:self.integrate_keyframe(i)
        return self.result(odom,True,'loop optimized / map rebuilt' if loop_added else 'keyframe added')
    def result(self,odom,ok,message):
        return dict(map_odom=self.map_odom.copy(),pose=compose(self.map_odom,odom),ok=ok,failures=self.failures,
                    rmse=self.last_rmse,message=message,keyframes=len(self.nodes),loops=self.loops,revision=self.revision)
    def snapshot(self):
        return self.mapper.origin.copy(),self.mapper.res,(~(self.mapper.seen&(self.mapper.log<-.2))).copy()
    def save(self,folder,metadata):
        from pathlib import Path
        import json
        folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
        self.mapper.save(folder,dict(metadata,method='wheel odometry + scan matching + experimental loop-closure pose graph',loops=self.loops))
        # Non-object ragged arrays: no pickle required.
        offsets=np.r_[0,np.cumsum([len(s) for s in self.scans])]
        pts=np.vstack(self.scans) if self.scans else np.empty((0,2))
        np.savez(folder/'slam_graph.npz',poses=np.array(self.nodes),odoms=np.array(self.odoms),scan_offsets=offsets,scan_points=pts,
                 map_odom=self.map_odom,edges_json=json.dumps([(i,j,z.tolist(),sigma,kind) for i,j,z,sigma,kind in self.edges]),
                 config_json=json.dumps(self.cfg),revision=self.revision,loops=self.loops)

    @classmethod
    def load_localization(cls,folder,cfg,hint):
        import json
        from pathlib import Path
        folder=Path(folder);obj=cls(cfg)
        with np.load(folder/'slam_graph.npz',allow_pickle=False) as d:
            obj.nodes=[x.copy() for x in d['poses']];obj.odoms=[x.copy() for x in d['odoms']]
            offsets=d['scan_offsets'];points=d['scan_points']
            obj.scans=[points[offsets[i]:offsets[i+1]].copy() for i in range(len(offsets)-1)]
            obj.edges=[(i,j,np.array(z),sigma,kind) for i,j,z,sigma,kind in json.loads(str(d['edges_json']))]
            obj.loops=int(d['loops']);obj.revision=int(d['revision'])
        with np.load(folder/'room_map_latest.npz',allow_pickle=False) as d:
            obj.mapper=Mapper(d['origin'],d['log_odds'].shape,float(d['resolution']))
            obj.mapper.log=d['log_odds'].copy();obj.mapper.seen=d['seen'].copy();obj.mapper.scans=int(d['scans'])
        if not obj.nodes:raise RuntimeError('Saved graph is empty')
        obj.loaded=True;obj.seed_hint=np.array(hint);obj.relocalized=False
        return obj


def interpolate_odom(history,t):
    if not history:raise RuntimeError('No encoder history for scan timestamp')
    if t<history[0][0]-.05 or t>history[-1][0]+.15:raise RuntimeError('Scan timestamp outside encoder history')
    if t<=history[0][0]:return history[0][1].copy()
    for (ta,a),(tb,b) in zip(history,history[1:]):
        if ta<=t<=tb:
            f=(t-ta)/max(tb-ta,1e-9)
            return np.r_[a[:2]+f*(b[:2]-a[:2]),wrap(a[2]+f*wrap(b[2]-a[2]))]
    return history[-1][1].copy()
