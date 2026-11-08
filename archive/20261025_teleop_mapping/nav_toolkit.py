"""Shared pure Python occupancy mapping; no Isaac import at module level."""
import json, math, struct, zlib
from pathlib import Path
import numpy as np


def wrap(a):return (a+math.pi)%(2*math.pi)-math.pi


def load_config(project):
    return json.loads((Path(project)/'nav_config.json').read_text(encoding='utf-8-sig'))


def discover(stage,cfg):
    hints=cfg['paths'];found={}
    prims=list(stage.Traverse())
    for key,name in (('base','base_link'),('lidar','Lidar_2D'),('controller','DifferentialController'),('floor','floor')):
        hint=hints.get(key)
        if hint and stage.GetPrimAtPath(hint).IsValid():found[key]=hint;continue
        choices=[str(p.GetPath()) for p in prims if p.GetName()==name]
        if key=='floor':choices=[p for p in choices if stage.GetPrimAtPath(p).GetTypeName()=='Mesh']
        if len(choices)!=1:raise RuntimeError(f'Cannot uniquely resolve {key}: {choices}. Set paths.{key} in nav_config.json.')
        found[key]=choices[0]
    found['collision_root']=str(stage.GetPrimAtPath(found['floor']).GetParent().GetPath())
    return found


_reader=None
_reader_key=None

def get_reader(stage,path):
    global _reader,_reader_key
    from isaacsim.sensors.experimental.rtx import LidarSensor
    key=(stage,path)
    if _reader is None or _reader_key!=key:
        _reader=LidarSensor(path,annotators=['generic-model-output']);_reader_key=key
    return _reader


def matrix(stage,path):
    from pxr import UsdGeom
    return np.array(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path)),dtype=float)


def pose(stage,base,units,front_sign=-1):
    m=matrix(stage,base)
    yaw=math.atan2(m[0,1],m[0,0])+(math.pi if front_sign<0 else 0)
    return np.array([m[3,0]*units,m[3,1]*units,wrap(yaw)]),m[3,2]*units


def read_cloud(reader,stage,lidar,units,valid_flag):
    from isaacsim.sensors.experimental.rtx import parse_generic_model_output_data
    raw=reader.get_data('generic-model-output');data=raw[0] if isinstance(raw,tuple) else raw
    if data is None or not getattr(data,'size',0):return None
    s=parse_generic_model_output_data(data)
    if 'SPHERICAL' not in str(s.elementsCoordsType).upper() or 'SENSOR' not in str(s.frameOfReference).upper():
        raise RuntimeError('Expected spherical SENSOR-frame lidar data')
    a=np.asarray(s.x,dtype=float).copy();e=np.asarray(s.y,dtype=float).copy();r=np.asarray(s.z,dtype=float).copy();f=np.asarray(s.flags).copy()
    valid=((f&valid_flag)!=0)&np.isfinite(a)&np.isfinite(e)&np.isfinite(r)&(r>.05)&(r<30)
    a,e,r=a[valid],e[valid],r[valid]
    if not len(r):return None
    # One nearest valid ray per 0.5-degree azimuth bin; absent angles stay unknown.
    bins=np.floor(np.mod(a,360)/.5).astype(int)
    order=np.lexsort((r,bins));_,first=np.unique(bins[order],return_index=True);idx=order[first]
    a,e,r=np.deg2rad(a[idx]),np.deg2rad(e[idx]),r[idx]
    local=np.c_[r*np.cos(e)*np.cos(a),r*np.cos(e)*np.sin(a),r*np.sin(e)]
    m=matrix(stage,lidar);points=local@m[:3,:3]+m[3,:3]*units
    return int(s.timestampNs),points,m[3,:3]*units


def body_filter(cloud,p,z,cfg):
    rel=cloud[:,:2]-p[:2];c,s=np.cos(p[2]),np.sin(p[2])
    x=rel[:,0]*c+rel[:,1]*s;y=-rel[:,0]*s+rel[:,1]*c
    own=(abs(x)<cfg['length']/2)&(abs(y)<cfg['width']/2)&(cloud[:,2]>=z-.02)&(cloud[:,2]<z+cfg['height'])
    return cloud[~own]


def bresenham(x0,y0,x1,y1):
    points=[];dx=abs(x1-x0);sx=1 if x0<x1 else -1;dy=-abs(y1-y0);sy=1 if y0<y1 else -1;err=dx+dy
    while True:
        points.append((y0,x0))
        if x0==x1 and y0==y1:return points
        e=2*err
        if e>=dy:err+=dy;x0+=sx
        if e<=dx:err+=dx;y0+=sy


def write_png(path,rgb):
    h,w,_=rgb.shape
    def chunk(t,d):return struct.pack('!I',len(d))+t+d+struct.pack('!I',zlib.crc32(t+d)&0xffffffff)
    raw=b''.join(b'\0'+row.tobytes() for row in rgb)
    Path(path).write_bytes(b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!2I5B',w,h,8,2,0,0,0))+chunk(b'IDAT',zlib.compress(raw))+chunk(b'IEND',b''))


class Mapper:
    def __init__(self,origin,shape,resolution):
        self.origin=np.asarray(origin,dtype=float);self.res=float(resolution)
        self.log=np.zeros(shape,np.float32);self.seen=np.zeros(shape,bool);self.scans=0
    def cell(self,p):
        x,y=np.floor((np.asarray(p[:2])-self.origin)/self.res).astype(int);return int(y),int(x)
    def in_map(self,c):return 0<=c[0]<self.log.shape[0] and 0<=c[1]<self.log.shape[1]
    def update(self,sensor,cloud,robot,z,body,max_range=12):
        # Never free through discarded self hits, invalid rays, or missing directions.
        cloud=body_filter(cloud,robot,z,body)
        # 2D mapping uses returns near scan height, not ceiling/floor projections.
        cloud=cloud[abs(cloud[:,2]-sensor[2])<.10]
        free=set();hits=set();start=self.cell(sensor)
        if not self.in_map(start):raise RuntimeError('Sensor moved outside mapping bounds; enlarge map bounds.')
        for point in cloud:
            dist=np.linalg.norm(point[:2]-sensor[:2])
            if dist<.10 or dist>max_range:continue
            end=self.cell(point)
            if not self.in_map(end):continue  # do not infer free space from clipped rays
            ray=bresenham(start[1],start[0],end[1],end[0])
            # Near sensor blind area is not marked free merely by ray existence.
            blind=int(math.ceil(.10/self.res));free.update(ray[blind:-1]);hits.add(end)
        free.difference_update(hits)
        if free:
            y,x=zip(*free);self.log[y,x]-=.4;self.seen[y,x]=True
        if hits:
            y,x=zip(*hits);self.log[y,x]+=.9;self.seen[y,x]=True
        # The already occupied robot body is known traversed space, not an arbitrary disk.
        cy,cx=self.cell(robot);n=int(math.ceil(math.hypot(body['length'],body['width'])/(2*self.res)))
        yy,xx=np.mgrid[max(0,cy-n):min(self.log.shape[0],cy+n+1),max(0,cx-n):min(self.log.shape[1],cx+n+1)]
        dx=self.origin[0]+(xx+.5)*self.res-robot[0];dy=self.origin[1]+(yy+.5)*self.res-robot[1]
        c,s=math.cos(robot[2]),math.sin(robot[2]);x=dx*c+dy*s;y=-dx*s+dy*c
        half=self.res*math.sqrt(2)/2
        own=(abs(x)<body['length']/2-half)&(abs(y)<body['width']/2-half)&(self.log[yy,xx]<=0)
        self.log[yy[own],xx[own]]=-2.;self.seen[yy[own],xx[own]]=True
        np.clip(self.log,-5,5,out=self.log);self.scans+=1
    def save(self,folder,metadata):
        folder=Path(folder);folder.mkdir(parents=True,exist_ok=True)
        known_free=self.seen&(self.log<-.2)
        occupied=~known_free  # all unknown/ambiguous cells are blocked for planning
        temporary=folder/'room_map_latest.tmp.npz'
        np.savez(temporary,occupied=occupied,seen=self.seen,log_odds=self.log,origin=self.origin,resolution=self.res,
                 metadata_json=json.dumps(metadata,ensure_ascii=False),scans=self.scans)
        temporary.replace(folder/'room_map_latest.npz')
        grey=np.full(self.log.shape,205,np.uint8);grey[known_free]=254;grey[self.seen&(self.log>.2)]=0
        display=np.flipud(grey)
        (folder/'room_map_latest.pgm').write_bytes(f'P5\n{grey.shape[1]} {grey.shape[0]}\n255\n'.encode()+display.tobytes())
        write_png(folder/'room_map_latest.png',np.repeat(display[:,:,None],3,axis=2))
        (folder/'room_map_metadata.json').write_text(json.dumps(dict(metadata,origin=self.origin.tolist(),resolution=self.res,
                  shape=list(self.log.shape),scans=self.scans,known_free_cells=int(known_free.sum()),unknown_cells=int((~self.seen).sum())),ensure_ascii=False,indent=2),encoding='utf-8')
        return folder/'room_map_latest.npz'
    @classmethod
    def load(cls,path,stage_id):
        with np.load(path,allow_pickle=False) as d:
            metadata=json.loads(str(d['metadata_json']))
            if metadata.get('stage_identifier')!=stage_id:raise RuntimeError('Saved map belongs to a different stage file. Rebuild or inspect registration.')
            m=cls(d['origin'],d['log_odds'].shape,float(d['resolution']));m.log=d['log_odds'].copy();m.seen=d['seen'].copy();m.scans=int(d['scans'])
            return m,metadata
