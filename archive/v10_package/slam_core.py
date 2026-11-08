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



# graph-consistency-v8: uncertainty candidates for this simulated, slipping drive.
def graph_odom_sigma(odom_delta,scan_delta):
    error=between(odom_delta,scan_delta)
    translation=float(np.linalg.norm(error[:2]))
    return [max(.15,3.*translation)]*2+[max(.20,3.*abs(float(error[2])))]

def loop_has_departure(nodes,j,i):
    trace=np.asarray(nodes[j:i+1])[:,:2]
    excursion=float(np.max(np.linalg.norm(trace-trace[0],axis=1)))
    travelled=float(np.sum(np.linalg.norm(np.diff(trace,axis=0),axis=1)))
    return excursion>=1.0 and travelled>=2.0

def guarded_graph_update(nodes,edges):
    try:optimized=optimize_graph(nodes,edges)
    except Exception as exc:return None,dict(accepted=False,reason='graph optimizer refused candidate: '+str(exc))
    original=np.asarray(nodes)
    displacement=np.linalg.norm(optimized[:,:2]-original[:,:2],axis=1)
    angles=np.abs(wrap(optimized[:,2]-original[:,2]))
    # Trustworthy local scans cannot be silently displaced by a contradictory graph.
    if max(displacement)>.10 or max(angles)>math.radians(5):
        return None,dict(accepted=False,reason='graph correction exceeds local-map trust bounds',
                         max_position_shift_m=float(max(displacement)),max_yaw_shift_deg=float(np.rad2deg(max(angles))))
    for a,b,z,sigma,kind in edges:
        if kind!='scan':continue
        old=between(z,between(original[a],original[b]))
        new=between(z,between(optimized[a],optimized[b]))
        if np.linalg.norm(new[:2])>max(.05,np.linalg.norm(old[:2])+.02) or abs(new[2])>max(.04,abs(old[2])+.02):
            return None,dict(accepted=False,reason='graph degrades accepted local scan constraints')
    return optimized,dict(accepted=True,max_position_shift_m=float(max(displacement)),max_yaw_shift_deg=float(np.rad2deg(max(angles))))

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
        if self.loaded:return _process_loaded_auto_v9(self,scan,odom)
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
        # keyframe-reference-refresh-v3: recorded-data replay, preserve scan acceptance gates.
        # measured-odom-keyframe-v4: refresh on measured motion even when fit motion is underestimated.
        if np.linalg.norm(change[:2])<float(self.cfg.get('keyframe_translation_m',.08)) and np.linalg.norm(between(self.odoms[-1],odom)[:2])<float(self.cfg.get('keyframe_translation_m',.08)) and abs(change[2])<.15:return self.result(odom,True,'tracking')
        i=len(self.nodes);self.nodes.append(fit.pose.copy());self.scans.append(scan.copy());self.odoms.append(odom.copy())
        # Separate measured odom and scan-derived constraints; this prototype does not model their full correlation.
        odom_delta=between(self.odoms[i-1],odom);scan_delta=between(self.nodes[i-1],fit.pose)
        self.edges.append((i-1,i,odom_delta,graph_odom_sigma(odom_delta,scan_delta),'odom'))
        self.edges.append((i-1,i,between(self.nodes[i-1],fit.pose),[.035,.035,.04],'scan'))
        loop_added=False
        if i>=14 and i-self.last_loop>=8:
            candidates=[j for j in range(i-12) if np.linalg.norm(self.nodes[j][:2]-fit.pose[:2])<.9 and loop_has_departure(self.nodes,j,i)]
            if candidates:
                j=min(candidates,key=lambda k:np.linalg.norm(self.nodes[k][:2]-fit.pose[:2]))
                loop_ref=self.reference(range(max(0,j-1),min(i-10,j+3)))
                check=match_scan(scan,loop_ref,fit.pose,.45,.55,.35)
                if check.accepted and check.rmse<.065 and check.overlap>.65:
                    if self.pending is not None and abs(self.pending-j)<=3:
                        proposed=(j,i,between(self.nodes[j],check.pose),[.025,.025,.03],'loop')
                        optimized,audit=guarded_graph_update(self.nodes,self.edges+[proposed])
                        if not hasattr(self,'loop_audit'):self.loop_audit=[]
                        self.loop_audit.append(dict(audit,node=i,candidate=j))
                        self.pending=None;self.last_loop=i
                        if optimized is not None:
                            self.edges.append(proposed)
                            self.nodes=[p.copy() for p in optimized];self.map_odom=compose(self.nodes[-1],inverse(odom))
                            self.loops+=1;self.revision+=1;loop_added=True
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
        (folder/'loop_audit.json').write_text(json.dumps(getattr(self,'loop_audit',[]),indent=2),encoding='utf-8')
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


# automatic-global-localization-v9
class GlobalScanLocalizer:
    """Bounded map-wide correlative search + ICP; consumes only local scans and a saved map.

    This is not AMCL. Distinct high-scoring hypotheses cause a stationary hold.
    """
    def __init__(self,mapper,reference,body):
        from scipy.ndimage import distance_transform_edt
        self.origin=mapper.origin.copy();self.res=mapper.res
        self.free=mapper.seen&(mapper.log<-.2)
        hits=mapper.seen&(mapper.log>.2)
        if int(hits.sum())<30 or int(self.free.sum())<100:
            raise RuntimeError('Saved map has too little known geometry for global localization')
        self.distance=distance_transform_edt(~hits)*self.res
        self.reference=voxel(reference,.05)
        self.tree=cKDTree(self.reference)
        self.body=body
        self.last_report=None
        y,x=np.where(self.free)
        positions=self.origin+(np.c_[x,y]+.5)*self.res
        _,indices=np.unique(np.floor(positions/.25).astype(int),axis=0,return_index=True)
        self.positions=positions[np.sort(indices)]
        if len(self.positions)>12000:raise RuntimeError('Global search map exceeds the bounded ward-size budget')

    def score(self,scan,poses):
        poses=np.atleast_2d(poses);cloud=np.asarray(scan)
        # Same bounded angularly/spatially distributed subset for all candidates.
        if len(cloud)>160:cloud=cloud[np.linspace(0,len(cloud)-1,160).astype(int)]
        c=np.cos(poses[:,2]);s=np.sin(poses[:,2])
        x=poses[:,0,None]+c[:,None]*cloud[None,:,0]-s[:,None]*cloud[None,:,1]
        y=poses[:,1,None]+s[:,None]*cloud[None,:,0]+c[:,None]*cloud[None,:,1]
        ix=np.floor((x-self.origin[0])/self.res).astype(int)
        iy=np.floor((y-self.origin[1])/self.res).astype(int)
        valid=(ix>=0)&(iy>=0)&(ix<self.free.shape[1])&(iy<self.free.shape[0])
        distance=np.full(x.shape,1.5)
        distance[valid]=self.distance[iy[valid],ix[valid]]
        # Broad likelihood is deliberate at the coarse 25 cm / 10 degree stage.
        return np.exp(-.5*(distance/.20)**2).mean(axis=1)

    def detailed_score(self,scan,pose):
        distances,_=self.tree.query(transform(voxel(scan,.05),pose))
        inlier=distances<.15
        overlap=float(inlier.mean())
        rmse=float(np.sqrt(np.mean(distances[inlier]**2))) if inlier.any() else float('inf')
        likelihood=float(np.exp(-.5*(np.minimum(distances,1.)/.10)**2).mean())
        return likelihood,overlap,rmse

    def search(self,scan):
        import time
        start=time.perf_counter();scan=voxel(np.asarray(scan),.05)
        if len(scan)<60:return dict(accepted=False,reason='too few scan returns',candidates=[])
        best=[]
        for yaw in np.arange(-np.pi,np.pi,math.radians(10)):
            for offset in range(0,len(self.positions),512):
                xy=self.positions[offset:offset+512]
                poses=np.c_[xy,np.full(len(xy),yaw)]
                scores=self.score(scan,poses)
                n=min(12,len(scores));ids=np.argpartition(scores,-n)[-n:]
                best.extend((float(scores[k]),poses[k].copy()) for k in ids)
        best.sort(key=lambda a:a[0],reverse=True)
        seeds=[]
        for score,pose in best:
            if any(np.linalg.norm(pose[:2]-p[:2])<.45 and abs(wrap(pose[2]-p[2]))<.30 for _,p in seeds):continue
            seeds.append((score,pose))
            if len(seeds)>=20:break
        fits=[]
        for coarse,seed in seeds:
            fit=match_scan(scan,self.reference,seed,.50,.75,.40)
            score,overlap,rmse=self.detailed_score(scan,fit.pose)
            # Test final geometric quality rather than pretending the coarse score is confidence.
            if not fit.accepted or overlap<.75 or rmse>.075:continue
            cell=np.floor((fit.pose[:2]-self.origin)/self.res).astype(int)
            if not (0<=cell[0]<self.free.shape[1] and 0<=cell[1]<self.free.shape[0] and self.free[cell[1],cell[0]]):continue
            if any(np.linalg.norm(fit.pose[:2]-v['pose'][:2])<.25 and abs(wrap(fit.pose[2]-v['pose'][2]))<.15 for v in fits):continue
            fits.append(dict(pose=fit.pose.copy(),score=score,overlap=overlap,rmse=rmse))
        fits.sort(key=lambda x:x['score'],reverse=True)
        if not fits:
            result=dict(accepted=False,reason='no geometrically valid map-wide hypothesis',candidates=[])
        else:
            gap=float(fits[0]['score']-fits[1]['score']) if len(fits)>1 else float(fits[0]['score'])
            accepted=bool(fits[0]['score']>.72 and gap>.06)
            result=dict(accepted=accepted,reason='unique scan/map hypothesis' if accepted else 'ambiguous map/scan hypotheses; stationary hold',
                pose=fits[0]['pose'].copy(),score=fits[0]['score'],gap=gap,candidates=fits[:6])
        result['wall_seconds']=time.perf_counter()-start
        self.last_report=result
        return result

def _process_loaded_auto_v9(self,scan,odom):
    if getattr(self,'global_localizer',None) is None:
        self.global_localizer=GlobalScanLocalizer(self.mapper,self.reference(range(len(self.nodes))),self.cfg['body'])
        self.global_confirmations=0;self.global_candidate_map_odom=None
    if not self.relocalized:
        candidate=getattr(self,'global_candidate_map_odom',None)
        if candidate is None:
            report=self.global_localizer.search(scan)
            self.auto_localization_report=report
            if not report['accepted']:
                self.last_rmse=float('inf');self.global_confirmations=0
                return self.result(odom,False,'AUTO LOCALIZE: '+report['reason'])
            self.global_candidate_map_odom=compose(report['pose'],inverse(odom))
            self.global_confirmations=1;self.last_rmse=report['candidates'][0]['rmse']
            return self.result(odom,False,'AUTO LOCALIZE: candidate found; confirming independent fresh scans 1/3')
        prior=compose(candidate,odom)
        fit=match_scan(scan,self.global_localizer.reference,prior,.30,.20,.15)
        score,overlap,rmse=self.global_localizer.detailed_score(scan,fit.pose)
        self.last_rmse=rmse
        if not fit.accepted or overlap<.75 or rmse>.075 or score<.72:
            self.global_candidate_map_odom=None;self.global_confirmations=0
            return self.result(odom,False,'AUTO LOCALIZE: candidate confirmation failed; searching again')
        proposed=compose(fit.pose,inverse(odom));delta=between(candidate,proposed)
        if np.linalg.norm(delta[:2])>.10 or abs(delta[2])>.07:
            self.global_candidate_map_odom=None;self.global_confirmations=0
            return self.result(odom,False,'AUTO LOCALIZE: unstable hypothesis; stationary hold')
        self.global_candidate_map_odom=proposed;self.global_confirmations+=1
        if self.global_confirmations<3:
            return self.result(odom,False,'AUTO LOCALIZE: confirming fresh scans 2/3')
        self.map_odom=proposed;self.relocalized=True;self.failures=0
        return self.result(odom,True,'AUTO LOCALIZE READY: map pose from lidar; A is current estimate')
    prior=compose(self.map_odom,odom)
    fit=match_scan(scan,self.global_localizer.reference,prior,.4,.6,.4)
    self.last_rmse=fit.rmse
    if fit.accepted:
        self.map_odom=compose(fit.pose,inverse(odom));self.failures=0
    else:
        self.failures+=1
        if self.failures>=3:
            self.relocalized=False;self.global_candidate_map_odom=None;self.global_confirmations=0
    return self.result(odom,fit.accepted,'loaded-map localization' if fit.accepted else 'AUTO LOCALIZE: tracking lost; held stopped')
