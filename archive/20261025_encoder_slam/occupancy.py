import json,math,struct,zlib
from pathlib import Path
import numpy as np


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