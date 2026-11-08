"""Isaac IO boundary. Live localization reads actual joint angles and lidar ONLY.
Only capture_mount_and_display reads world transforms, ONCE for static mounting and UI alignment.
"""
import math
import numpy as np
from slam_core import compose,inverse,transform

RUNTIME_REVISION = '2026-10-25-shared-view-v3'

def find_robot(stage,cfg):
    out={}
    for key,name in (('base','base_link'),('lidar','Lidar_2D'),('controller','DifferentialController')):
        hint=cfg['paths'].get(key)
        if hint:
            if stage.GetPrimAtPath(hint).IsValid():
                out[key]=hint
                continue
            candidates=[str(p.GetPath()) for p in stage.Traverse() if p.GetName()==name]
            raise RuntimeError(
                f'Configured {key} prim does not exist: {hint}. '
                f'Stage candidates named {name}: {candidates}. '
                f'Update paths.{key} in slam_config.json to the intended AGV prim.'
            )
        candidates=[str(p.GetPath()) for p in stage.Traverse() if p.GetName()==name]
        if len(candidates)!=1:raise RuntimeError(f'{key} ambiguous: {candidates}; set slam_config.json paths.{key}.')
        out[key]=candidates[0]
    return out


class EncoderReader:
    def __init__(self,stage,paths,cfg,simulation_view=None):
        from pxr import UsdPhysics
        # Reuse the entry point attached by Isaac Sim to this running stage.
        # Creating a second view with the default stage_id=-1 can fail even
        # after the manager has completed physics setup.
        if simulation_view is None:
            from isaacsim.core.simulation_manager import SimulationManager
            simulation_view=SimulationManager.get_physics_simulation_view()
        if simulation_view is None:
            raise RuntimeError('Physics simulation view is unavailable. Press Play and wait for physics setup before reading encoders.')
        self.sim=simulation_view
        valid=getattr(self.sim,'is_valid',True)
        if callable(valid):valid=valid()
        if not valid:raise RuntimeError('Physics simulation view is invalid; Stop and restart the session.')
        if not callable(getattr(self.sim,'create_articulation_view',None)):
            raise RuntimeError(f'Physics view {type(self.sim).__module__}.{type(self.sim).__name__} cannot create an articulation view.')
        root=cfg['encoders'].get('articulation_path')
        if not root:
            base=paths['base'];parent=str(stage.GetPrimAtPath(base).GetParent().GetPath())
            candidates=[str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI) and
                        (base.startswith(str(p.GetPath())+'/') or str(p.GetPath())==base or str(p.GetParent().GetPath())==parent)]
            if len(candidates)!=1:raise RuntimeError(f'ArticulationRoot ambiguous: {candidates}. Set encoders.articulation_path.')
            root=candidates[0]
        self.view=self.sim.create_articulation_view(root)
        if self.view.count!=1:raise RuntimeError(f'Expected one actual articulation at {root}, got {self.view.count}')
        names=list(self.view.shared_metatype.dof_names)
        def choose(side):
            explicit=cfg['encoders'].get('original_'+side+'_joint')
            if explicit:
                if explicit not in names:raise RuntimeError(f'{explicit} not in actual DOFs {names}')
                return names.index(explicit)
            ids=[i for i,n in enumerate(names) if side in n.lower() and ('wheel' in n.lower() or 'drive' in n.lower()) and 'caster' not in n.lower() and 'swivel' not in n.lower()]
            if len(ids)!=1:raise RuntimeError(f'Cannot identify {side} drive DOF in {names}. Set explicit joint names.')
            return ids[0]
        original_left,original_right=choose('left'),choose('right')
        self.left,self.right=(original_right,original_left) if cfg['front_axis_sign']<0 else (original_left,original_right)
        self.lsign=cfg['encoders']['logical_left_sign'];self.rsign=cfg['encoders']['logical_right_sign']
        print('Actual encoder DOFs:',names)
        print('Logical front-left:',names[self.left],self.lsign,'front-right:',names[self.right],self.rsign)
    def read(self):
        q=self.view.get_dof_positions()
        # The manager owns the frontend: accept NumPy, CPU/GPU Torch or Warp.
        if callable(getattr(q,'detach',None)):q=q.detach()
        if callable(getattr(q,'cpu',None)):q=q.cpu()
        if callable(getattr(q,'numpy',None)):q=q.numpy()
        q=np.asarray(q,dtype=float)
        if q.ndim!=2 or q.shape[0]!=1:raise RuntimeError('Actual joint tensor invalid')
        if not np.isfinite(q[0,[self.left,self.right]]).all():raise RuntimeError('Wheel encoder angles are not finite')
        return float(q[0,self.left]*self.lsign),float(q[0,self.right]*self.rsign)


def capture_mount_and_display(stage,paths,cfg):
    from pxr import UsdGeom
    units=UsdGeom.GetStageMetersPerUnit(stage)
    cache=UsdGeom.XformCache()
    base=np.array(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(paths['base'])),dtype=float)
    lidar=np.array(cache.GetLocalToWorldTransform(stage.GetPrimAtPath(paths['lidar'])),dtype=float)
    front=np.eye(4)
    if cfg['front_axis_sign']<0:front[0,0]=front[1,1]=-1
    mount=lidar@np.linalg.inv(base)@front
    mount[3,:3]*=units
    if np.linalg.norm(mount[0:2,2])>.05:raise RuntimeError('2D scan plane significantly tilted; correct static mount first.')
    physical=front@base
    # Used ONLY for drawing/map-goal conversions, never passed into SLAM/odom.
    display_pose=np.array([physical[3,0]*units,physical[3,1]*units,math.atan2(physical[0,1],physical[0,0])])
    return mount,Display(stage,display_pose,float(physical[3,2]*units),units,cfg['goal_marker'])


class ScanReader:
    def __init__(self,stage,path,cfg):
        from isaacsim.sensors.experimental.rtx import LidarSensor
        from isaacsim.sensors.experimental.rtx.generic_model_output import ElementFlags
        self.reader=LidarSensor(path,annotators=['generic-model-output']);self.flag=int(ElementFlags.VALID.value)
        self.last=None;self.mount=None;self.cfg=cfg
        prim=stage.GetPrimAtPath(path)
        for name,value in (('omni:sensor:Core:instantLidar',True),('omni:sensor:Core:accumulateOutputs',False)):
            attr=prim.GetAttribute(name)
            if not attr.IsValid() or not attr.Set(value):raise RuntimeError(f'Cannot configure {name}; start from Stop.')
    def read(self):
        from isaacsim.sensors.experimental.rtx import parse_generic_model_output_data
        raw=self.reader.get_data('generic-model-output');data=raw[0] if isinstance(raw,tuple) else raw
        if data is None or not getattr(data,'size',0) or self.mount is None:return None
        s=parse_generic_model_output_data(data);stamp=int(s.timestampNs)
        if stamp==self.last:return None
        self.last=stamp
        if 'SPHERICAL' not in str(s.elementsCoordsType).upper() or 'SENSOR' not in str(s.frameOfReference).upper():
            raise RuntimeError('Lidar must output spherical SENSOR coordinates')
        a=np.asarray(s.x,dtype=float).copy();e=np.asarray(s.y,dtype=float).copy();r=np.asarray(s.z,dtype=float).copy();f=np.asarray(s.flags).copy()
        valid=((f&self.flag)!=0)&np.isfinite(a)&np.isfinite(e)&np.isfinite(r)&(r>.1)&(r<self.cfg['scan_max_range'])
        a,e,r=a[valid],e[valid],r[valid]
        if len(r)<25:return None
        bins=np.floor(np.mod(a,360)/.5).astype(int);order=np.lexsort((r,bins));_,idx=np.unique(bins[order],return_index=True);idx=order[idx]
        a,e,r=np.deg2rad(a[idx]),np.deg2rad(e[idx]),r[idx]
        xyz=np.c_[r*np.cos(e)*np.cos(a),r*np.cos(e)*np.sin(a),r*np.sin(e)]
        xyz=xyz@self.mount[:3,:3]+self.mount[3,:3]
        body=self.cfg['body'];own=(abs(xyz[:,0])<body['length']/2)&(abs(xyz[:,1])<body['width']/2)&(xyz[:,2]<body['height'])
        horizontal=abs(xyz[:,2]-self.mount[3,2])<.10
        return stamp,xyz[~own&horizontal,:2].copy()


class Display:
    """One-time visualization anchor. No method reads current robot world pose."""
    def __init__(self,stage,anchor,z,units,marker):self.stage=stage;self.anchor=anchor;self.z=z;self.units=units;self.marker=marker
    def world(self,p):
        xy=transform(np.asarray(p)[:2][None,:],self.anchor)[0];return (float(xy[0]/self.units),float(xy[1]/self.units),float((self.z+.025)/self.units))
    def select_goal(self,pose):
        from pxr import UsdGeom,Gf
        import omni.usd
        prim=self.stage.GetPrimAtPath(self.marker)
        if not prim.IsValid():
            parent=self.marker.rsplit('/',1)[0];UsdGeom.Xform.Define(self.stage,parent)
            prim=UsdGeom.Xform.Define(self.stage,self.marker).GetPrim()
            target=compose(pose,np.array([1.,0.,0.]));w=compose(self.anchor,target)
            matrix=Gf.Matrix4d(1);matrix.SetRotate(Gf.Rotation(Gf.Vec3d(0,0,1),math.degrees(w[2])))
            matrix.SetTranslateOnly(Gf.Vec3d(w[0]/self.units,w[1]/self.units,(self.z+.025)/self.units))
            parent_w=UsdGeom.XformCache().GetLocalToWorldTransform(prim.GetParent())
            UsdGeom.Xformable(prim).MakeMatrixXform().Set(matrix*parent_w.GetInverse())
        omni.usd.get_context().get_selection().set_selected_prim_paths([self.marker],True)
    def goal(self,pose,use_yaw=False):
        from pxr import UsdGeom
        prim=self.stage.GetPrimAtPath(self.marker)
        if not prim.IsValid():raise RuntimeError('Create and drag B marker first.')
        m=np.array(UsdGeom.XformCache().GetLocalToWorldTransform(prim),dtype=float)
        world=np.array([m[3,0]*self.units,m[3,1]*self.units,math.atan2(m[0,1],m[0,0])])
        p=compose(inverse(self.anchor),world)
        if not use_yaw:p[2]=math.atan2(p[1]-pose[1],p[0]-pose[0])
        return p
