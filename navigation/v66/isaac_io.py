"""Isaac IO boundary. Live localization reads actual joint angles and lidar ONLY.
World pose is used only for static display alignment and inside the simulated range-sensor renderer. It is never supplied as a localization observation.
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
                f'Update paths.{key} in v66_nav_config.json to the intended AGV prim.'
            )
        candidates=[str(p.GetPath()) for p in stage.Traverse() if p.GetName()==name]
        if len(candidates)!=1:raise RuntimeError(f'{key} ambiguous: {candidates}; set v66_nav_config.json paths.{key}.')
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
    sensor=stage.GetPrimAtPath(paths['lidar'])
    xf=UsdGeom.Xformable(sensor)
    lidar=np.array(xf.GetLocalTransformation(),dtype=float)
    if not xf.GetResetXformStack():
        lidar=lidar@np.array(cache.GetLocalToWorldTransform(sensor.GetParent()),dtype=float)
    front=np.eye(4)
    if cfg['front_axis_sign']<0:front[0,0]=front[1,1]=-1
    mount=lidar@np.linalg.inv(base)@front
    mount[3,:3]*=units
    if np.linalg.norm(mount[0:2,2])>.05:raise RuntimeError('2D scan plane significantly tilted; correct static mount first.')
    physical=front@base
    # Used ONLY for drawing/map-goal conversions, never passed into SLAM/odom.
    display_pose=np.array([physical[3,0]*units,physical[3,1]*units,math.atan2(physical[0,1],physical[0,0])])
    return mount,Display(stage,display_pose,float(physical[3,2]*units),units,cfg['goal_marker'])



LIDAR_IDENTITY_REVISION = '2026-10-26-identified-returns-v5'
_NAV_LIDAR_SESSION_LAYERS = []


class ScanReader:
    """Resolve actual rendered return IDs; never infer self hits from distance alone."""
    def __init__(self,stage,path,cfg):
        import carb
        from collections import deque
        from pxr import Usd,UsdGeom,Sdf
        from isaacsim.sensors.experimental.rtx import Lidar,LidarSensor
        from isaacsim.sensors.experimental.rtx.generic_model_output import ElementFlags
        self.stage=stage;self.path=path;self.cfg=cfg
        base=stage.GetPrimAtPath(cfg['paths']['base'])
        if not base.IsValid():raise RuntimeError('Configured base missing before lidar identity setup')
        self.robot_root=str(base.GetParent().GetPath())
        self.last=None;self.mount=None;self.flag=int(ElementFlags.VALID.value)
        self.id_map={};self.identity_history=deque(maxlen=64);self.identity_stats={}
        self.identity_error=None;self.identity_warnings=set();self.return_names=[]
        self.return_ids=[];self.identity_setup={}
        settings=carb.settings.get_settings()
        settings.set_bool('/rtx-transient/stableIds/enabled',True)
        # Author channels before the render product is attached, as required by Replicator.
        layer=Sdf.Layer.CreateAnonymous('amr_lidar_identity_v5.usda')
        session=stage.GetSessionLayer();prim=stage.GetPrimAtPath(path)
        original_transform=np.array(UsdGeom.Xformable(prim).GetLocalTransformation(),dtype=float)
        session.subLayerPaths=[layer.identifier]+list(session.subLayerPaths)
        try:
            with Usd.EditContext(stage,layer):
                channels=prim.GetAttribute('_replicator:rendervar:GenericModelOutput:channels')
                if not channels.IsValid():channels=prim.CreateAttribute('_replicator:rendervar:GenericModelOutput:channels',Sdf.ValueTypeNames.StringArray,custom=True)
                if not channels.Set(['EXTRA']):raise RuntimeError('Cannot request lidar object-ID auxiliary channels')
                for name,value in (('omni:sensor:Core:instantLidar',True),('omni:sensor:Core:accumulateOutputs',False)):
                    attr=prim.GetAttribute(name)
                    if not attr.IsValid() or not attr.Set(value):raise RuntimeError(f'Cannot configure {name}; start from Stop.')
                # Passing the configured wrapper avoids the path-only wrapper's default
                # NONE channels / accumulated outputs overriding the authored settings.
                self.lidar=Lidar(path,accumulate_outputs=False,aux_output_level='EXTRA',reset_xform_op_properties=False)
                self.reader=LidarSensor(self.lidar,annotators=['generic-model-output','stable-id-map'])
            if not np.allclose(original_transform,np.array(UsdGeom.Xformable(prim).GetLocalTransformation(),dtype=float),rtol=0,atol=1e-9):
                raise RuntimeError('Lidar wrapper unexpectedly changed its mounting transform')
            if list(channels.Get())!=['EXTRA'] or bool(prim.GetAttribute('omni:sensor:Core:accumulateOutputs').Get()):
                raise RuntimeError('Lidar wrapper changed EXTRA channels or per-frame output settings')
        except BaseException:
            session.subLayerPaths=[p for p in session.subLayerPaths if p!=layer.identifier]
            raise
        _NAV_LIDAR_SESSION_LAYERS.append(layer)
        self.identity_setup=dict(revision=LIDAR_IDENTITY_REVISION,robot_root=self.robot_root,
            channels=list(prim.GetAttribute('_replicator:rendervar:GenericModelOutput:channels').Get()),
            stable_ids_enabled=bool(settings.get('/rtx-transient/stableIds/enabled')),
            session_layer_identifier=layer.identifier,
            policy='Only exact mapped, existing prims under the configured AGV root are self hits. Unknown/external returns remain obstacles. No ID bit masking or enlarged geometric self mask.')

    def warn_identity(self,message):
        self.identity_error=str(message)
        if message not in self.identity_warnings:
            self.identity_warnings.add(message)
            print('[AMR lidar identity]',message,'Unknown returns stay in the obstacle cloud.')

    def resolve_names(self,scan,indices,total):
        # Decode only retained angular samples, not hundreds of thousands of Python IDs.
        names=['<unknown>']*len(indices);ids=['']*len(indices)
        try:
            from isaacsim.sensors.experimental.rtx import parse_stable_id_map_data,parse_object_ids
        except ImportError as exc:
            self.warn_identity('Installed RTX package has no object-ID parser: '+str(exc))
            return names,ids
        try:
            raw=self.reader.get_data('stable-id-map');data=raw[0] if isinstance(raw,tuple) else raw
            if data is not None:
                mapping=parse_stable_id_map_data(data)
                self.id_map.update({int(k):str(v) for k,v in mapping.items()})
        except Exception as exc:self.warn_identity('Stable ID map unavailable: '+type(exc).__name__+': '+str(exc))
        try:
            packed=np.asarray(scan.objId,dtype=np.uint8).reshape(-1)
            if packed.size!=total*16:raise ValueError(f'objId has {packed.size} bytes; expected {total*16}. EXTRA channels/stable IDs may be unavailable.')
            if not np.any(packed):raise ValueError('Object IDs are all zero; no self-hit classification is allowed')
            selected=np.ascontiguousarray(packed.reshape(total,16)[indices]).reshape(-1)
            parsed=parse_object_ids(selected)
            if len(parsed)!=len(indices):raise ValueError('Decoded ID count differs from retained return count')
            for i,value in enumerate(parsed):
                key=int(value);ids[i]=format(key,'032x')
                name=self.id_map.get(key,'<unknown>')
                # Reject malformed/stale mappings. Missing IDs are never classified as self.
                if name.startswith('/') and self.stage.GetPrimAtPath(name).IsValid():names[i]=name
            self.identity_error=None
        except Exception as exc:self.warn_identity('Object IDs unavailable: '+type(exc).__name__+': '+str(exc))
        return names,ids

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
        total=len(r)
        if not (len(a)==len(e)==len(f)==total):raise RuntimeError('Lidar arrays have inconsistent lengths')
        valid=((f&self.flag)!=0)&np.isfinite(a)&np.isfinite(e)&np.isfinite(r)&(r>.1)&(r<self.cfg['scan_max_range'])
        original_indices=np.flatnonzero(valid);a,e,r=a[valid],e[valid],r[valid]
        if len(r)<25:return None
        bins=np.floor(np.mod(a,360)/.5).astype(int);order=np.lexsort((r,bins));_,idx=np.unique(bins[order],return_index=True);idx=order[idx]
        indices=original_indices[idx]
        a,e,r=np.deg2rad(a[idx]),np.deg2rad(e[idx]),r[idx]
        xyz=np.c_[r*np.cos(e)*np.cos(a),r*np.cos(e)*np.sin(a),r*np.sin(e)]
        xyz=xyz@self.mount[:3,:3]+self.mount[3,:3]
        names,ids=self.resolve_names(s,indices,total)
        self_hit=np.array([name==self.robot_root or name.startswith(self.robot_root+'/') for name in names],dtype=bool)
        horizontal=abs(xyz[:,2]-self.mount[3,2])<.10
        keep=~self_hit&horizontal
        returned=xyz[keep,:2].copy()
        self.return_names=[name for name,k in zip(names,keep) if k]
        self.return_ids=[value for value,k in zip(ids,keep) if k]
        self.identity_history.append(dict(timestamp_ns=stamp,points_front=returned.copy(),
            prim_paths=list(self.return_names),object_ids=list(self.return_ids)))
        body=self.cfg['body'];margin=self.cfg['margin']
        near=np.linalg.norm(xyz[:,:2],axis=1)<.9
        guard=(abs(xyz[:,0])<body['length']/2+margin)&(abs(xyz[:,1])<body['width']/2+margin)&horizontal
        counts={}
        for name,k in zip(names,near&horizontal):
            if k:counts[name]=counts.get(name,0)+1
        # Includes removed own hits and retained hits, with exactly the same selection indices.
        records=[dict(point_front_m=xyz[i].tolist(),prim_path=names[i],object_id=ids[i],
                      confirmed_self=bool(self_hit[i]),retained=bool(keep[i]),inside_current_guard=bool(guard[i]))
                 for i in np.flatnonzero(near&horizontal)]
        self.identity_stats=dict(timestamp_ns=stamp,raw_elements=total,angular_samples=len(xyz),
            known_prim_samples=sum(name!='<unknown>' for name in names),
            confirmed_self_removed=int(np.count_nonzero(self_hit&horizontal)),
            retained_samples=len(returned),unknown_retained=sum(name=='<unknown>' for name in self.return_names),
            current_guard_retained=int(np.count_nonzero(guard&keep)),near_prim_counts=counts,
            near_hits=records,identity_error=self.identity_error,map_entries=len(self.id_map))
        # If no useful external scan remains, existing localization/freshness protection holds zero.
        return stamp,returned




PHYSX_LIDAR_REVISION='2026-10-26-collision-ray-lidar-v6'
_NAV_PHYSX_LIDAR_SESSION_LAYERS=[]
_RTXReferenceReaderV5=ScanReader

class ScanReader:
    """Simulated 2D range sensor over live PhysX collision geometry.

    The live body transform is used ONLY inside this sensor renderer to cast rays.
    read() returns a timestamp and sensor observations in the physical front frame;
    it never returns that world transform to odometry, SLAM or the path controller.
    RTX remains a separately labelled diagnostic input and is NOT a fallback.
    """
    def __init__(self,stage,path,cfg):
        import carb
        import omni.physx
        from collections import deque
        from pxr import Usd,UsdGeom,UsdPhysics,Sdf
        if cfg.get('sim_lidar_backend')!='physx_collision_raycast':
            raise RuntimeError('V6 requires explicit sim_lidar_backend=physx_collision_raycast')
        self.stage=stage;self.cfg=cfg;self.path=path;self.backend='PHYSX_COLLISION_RAYCAST'
        self.units=float(UsdGeom.GetStageMetersPerUnit(stage))
        if not np.isfinite(self.units) or self.units<=0:raise RuntimeError('Invalid stage units for sensor raycasts')
        self.robot_root=str(stage.GetPrimAtPath(cfg['paths']['base']).GetParent().GetPath())
        self.mount=None;self.body_view=None;self.last_time=-math.inf;self.last_rtx_time=-math.inf
        self.rate=float(cfg.get('sim_lidar_hz',10.));self.beams=int(cfg.get('sim_lidar_beams',720))
        if not 5<=self.rate<=30 or not 180<=self.beams<=1440:raise RuntimeError('Invalid simulated lidar rate/beam count')
        self.angles=np.linspace(-math.pi,math.pi,self.beams,endpoint=False)
        self.directions_sensor=np.c_[np.cos(self.angles),np.sin(self.angles),np.zeros(self.beams)]
        self.max_range=float(cfg['scan_max_range'])
        self.identity_history=deque(maxlen=64);self.identity_stats={};self.id_map={}
        self.return_names=[];self.return_ids=[];self.identity_error=None;self.last_frame=None
        self.rtx_reference=None;self.rtx_reference_stats={};self.rtx_reference_error=None
        self.scene_query=omni.physx.get_physx_scene_query_interface();self.carb=carb
        layer=Sdf.Layer.CreateAnonymous('amr_physx_lidar_v6.usda');session=stage.GetSessionLayer()
        session.subLayerPaths=[layer.identifier]+list(session.subLayerPaths)
        scenes=[]
        try:
            with Usd.EditContext(stage,layer):
                for prim in stage.Traverse():
                    if not prim.IsA(UsdPhysics.Scene):continue
                    attr=prim.GetAttribute('physxScene:enableSceneQuerySupport')
                    if not attr.IsValid():attr=prim.CreateAttribute('physxScene:enableSceneQuerySupport',Sdf.ValueTypeNames.Bool,custom=False)
                    if not attr.Set(True):raise RuntimeError('Cannot enable live scene queries')
                    scenes.append(str(prim.GetPath()))
            if not scenes:raise RuntimeError('No active physics scene for the lidar renderer')
        except BaseException:
            session.subLayerPaths=[p for p in session.subLayerPaths if p!=layer.identifier]
            raise
        _NAV_PHYSX_LIDAR_SESSION_LAYERS.append(layer)
        self.reader=None;self.flag=1  # Internal marker for our custom raycast frames, not RTX flags.
        if cfg.get('sim_lidar_rtx_reference',False):
            try:
                self.rtx_reference=_RTXReferenceReaderV5(stage,path,cfg)
                self.reader=self.rtx_reference.reader
                self.flag=self.rtx_reference.flag
            except Exception as exc:
                self.rtx_reference_error=type(exc).__name__+': '+str(exc)
                print('[AMR V7] Optional RTX comparison unavailable:',self.rtx_reference_error)
        self.identity_setup=dict(revision=PHYSX_LIDAR_REVISION,primary_backend=self.backend,
            robot_root=self.robot_root,scan_hz=self.rate,beam_count=self.beams,max_range_m=self.max_range,
            scene_query_scenes=scenes,session_layer_identifier=layer.identifier,
            RTX_reference='DIAGNOSTIC_ONLY; never passed to the estimator/controller',
            world_pose_usage='Only inside sensor simulation to cast rays; not an estimator pose observation',
            policy='Nearest physical hit. A self hit occludes its beam; no farther point is substituted. Unknown external hit paths remain obstacles. No-hit beams do not become occupied endpoints.')
        external=[]
        for prim in stage.Traverse():
            if not prim.HasAPI(UsdPhysics.CollisionAPI):continue
            prim_path=str(prim.GetPath())
            if prim_path==self.robot_root or prim_path.startswith(self.robot_root+'/'):continue
            enabled=UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get()
            if enabled is False:continue
            external.append(dict(path=prim_path,type_name=prim.GetTypeName()))
        self.identity_setup['active_external_collision_prims']=external
        self.identity_setup['collision_model_limitation']='Only collision-enabled geometry is visible to this simulated sensor. Missing furniture colliders must be added before navigation acceptance.'
        if not external:raise RuntimeError('No external colliders; add floor/walls before driving or mapping')
        print('[AMR V6] Primary sensor: PHYSX_COLLISION_RAYCAST; RTX is comparison only. SLAM still uses wheel encoders + local range observations.')

    @staticmethod
    def host(value):
        if callable(getattr(value,'detach',None)):value=value.detach()
        if callable(getattr(value,'cpu',None)):value=value.cpu()
        if callable(getattr(value,'numpy',None)):value=value.numpy()
        return np.asarray(value,dtype=float)

    @staticmethod
    def rotation_row(q):
        q=np.asarray(q,dtype=float)
        if not np.isfinite(q).all() or np.linalg.norm(q)<.5:raise RuntimeError('Invalid quaternion in sensor renderer')
        x,y,z,w=q/np.linalg.norm(q)
        return np.array([[1-2*(y*y+z*z),2*(x*y+z*w),2*(x*z-y*w)],
            [2*(x*y-z*w),1-2*(x*x+z*z),2*(y*z+x*w)],
            [2*(x*z+y*w),2*(y*z-x*w),1-2*(x*x+y*y)]])

    @staticmethod
    def hit_fields(result):
        if isinstance(result,dict):
            if not result.get('hit','distance' in result and ('collision' in result or 'rigidBody' in result)):return None
            return (float(result['distance']),str(result.get('collision','')),str(result.get('rigidBody',result.get('rigid_body',''))))
        if isinstance(result,tuple) and len(result)==2:
            found,hit=result
            if not found:return None
            def get(name,alternate=None):
                if isinstance(hit,dict):return hit.get(name,hit.get(alternate,''))
                return getattr(hit,name,getattr(hit,alternate,'') if alternate else '')
            return float(get('distance')),str(get('collision')),str(get('rigid_body','rigidBody'))
        raise RuntimeError('Unsupported live PhysX raycast return type: '+str(type(result)))

    def read(self):
        from isaacsim.core.simulation_manager import SimulationManager
        now=float(SimulationManager.get_simulation_time())
        if now<self.last_time:raise RuntimeError('Physics clock reset; restart sensor session')
        if self.mount is None or now-self.last_time<1./self.rate-1e-7:return None
        self.last_time=now
        if self.body_view is None:
            sim=SimulationManager.get_physics_simulation_view()
            if sim is None:return None
            self.body_view=sim.create_rigid_body_view(self.cfg['paths']['base'])
            if self.body_view.count!=1:raise RuntimeError('Expected one base body for simulated lidar rendering')
        pose=self.host(self.body_view.get_transforms())[0]
        if not np.isfinite(pose).all():raise RuntimeError('Nonfinite pose inside sensor renderer')
        base_rotation=self.rotation_row(pose[3:7]);front=np.eye(3)
        if self.cfg['front_axis_sign']<0:front[0,0]=front[1,1]=-1.
        front_rotation=front@base_rotation
        mount_rotation=self.mount[:3,:3]
        if not np.allclose(mount_rotation@mount_rotation.T,np.eye(3),atol=1e-4):
            raise RuntimeError('Scaled/skewed lidar mounting is unsupported by the range renderer')
        lidar_rotation=mount_rotation@front_rotation
        origin_stage=pose[:3]+self.mount[3,:3]@front_rotation/self.units
        directions_world=self.directions_sensor@lidar_rotation
        origin=self.carb.Float3(*[float(x) for x in origin_stage])
        ranges=np.full(self.beams,np.nan);flags=np.zeros(self.beams,dtype=np.uint8)
        states=['NO_HIT']*self.beams;all_names=['']*self.beams
        returned=[];names=[];own_count=0;unknown_count=0;near_records=[];counts={}
        wall_start=__import__('time').perf_counter()
        for i,direction in enumerate(directions_world):
            direction=direction/np.linalg.norm(direction)
            ray=self.carb.Float3(*[float(x) for x in direction])
            hit=self.hit_fields(self.scene_query.raycast_closest(origin,ray,self.max_range/self.units,True))
            if hit is None:continue
            stage_range,collision,body=hit
            distance=stage_range*self.units
            if not math.isfinite(distance) or distance<0 or distance>self.max_range+.001:
                raise RuntimeError('Invalid distance from physical scene query')
            name=collision if collision.startswith('/') else body if body.startswith('/') else '<unknown>'
            all_names[i]=name;ranges[i]=distance
            own=any(p==self.robot_root or p.startswith(self.robot_root+'/') for p in (collision,body))
            if own:
                states[i]='SELF_OCCLUDED';own_count+=1
                continue
            # Short external hits are retained conservatively, including below nominal near range.
            point=self.directions_sensor[i]*distance@mount_rotation+self.mount[3,:3]
            if not np.isfinite(point).all():raise RuntimeError('Invalid local range observation')
            flags[i]=self.flag;states[i]='EXTERNAL_HIT';returned.append(point[:2]);names.append(name)
            unknown_count+=int(name=='<unknown>')
            guard=bool(abs(point[0])<self.cfg['body']['length']/2+self.cfg['margin'] and abs(point[1])<self.cfg['body']['width']/2+self.cfg['margin'])
            if np.linalg.norm(point[:2])<.9:
                counts[name]=counts.get(name,0)+1
                near_records.append(dict(point_front_m=point.tolist(),prim_path=name,object_id='',
                    confirmed_self=False,retained=True,inside_current_guard=guard))
        points=np.asarray(returned,dtype=float).reshape(-1,2);stamp=int(round(now*1e9))
        self.return_names=names;self.return_ids=['']*len(names)
        self.identity_history.append(dict(timestamp_ns=stamp,points_front=points.copy(),prim_paths=list(names),object_ids=list(self.return_ids)))
        self.identity_stats=dict(timestamp_ns=stamp,backend=self.backend,raw_elements=self.beams,
            angular_samples=self.beams,known_prim_samples=len(points)-unknown_count,
            confirmed_self_removed=own_count,retained_samples=len(points),unknown_retained=unknown_count,
            no_hit_beams=states.count('NO_HIT'),current_guard_retained=sum(h['inside_current_guard'] for h in near_records),
            near_prim_counts=counts,near_hits=near_records,identity_error=None,map_entries=None,
            query_wall_ms=(__import__('time').perf_counter()-wall_start)*1000.)
        self.last_frame=dict(time_ns=stamp,physics_s=now,backend=self.backend,
            x=np.rad2deg(self.angles).copy(),y=np.zeros(self.beams),z=ranges,flags=flags,
            hit_states=list(states),prim_paths=all_names,
            sensor_origin_world_m=(origin_stage*self.units).tolist(),
            sensor_rotation_to_world=lidar_rotation.tolist())
        # Retain a simultaneous RTX comparison without consuming it as a motion input.
        if self.rtx_reference is not None and now-self.last_rtx_time>=.25:
            self.last_rtx_time=now;self.rtx_reference.mount=self.mount
            try:
                self.rtx_reference.read()
                self.rtx_reference_stats=dict(self.rtx_reference.identity_stats)
            except Exception as exc:self.rtx_reference_error=type(exc).__name__+': '+str(exc)
        return stamp,points


class Display:
    """One-time visualization anchor and user goal, never a robot-pose observation.

    GoalB's projected local +X axis is the requested logical vehicle front.
    The robot's base_link -X convention is handled by wheel/sensor I/O, not
    by reversing this arrow. All public goal poses use metres and radians.
    """
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
    def read_goal_pose(self):
        """Return dragged world XY and projected +X heading in the map frame.

        Z, roll and pitch are not goal constraints. A vertical +X axis has no
        planar heading, so reject it explicitly rather than inventing a yaw.
        """
        from pxr import UsdGeom
        prim=self.stage.GetPrimAtPath(self.marker)
        if not prim.IsValid():raise RuntimeError('Create and drag B marker first.')
        m=np.array(UsdGeom.XformCache().GetLocalToWorldTransform(prim),dtype=float)
        if not np.isfinite(m).all():raise RuntimeError('B marker transform contains nonfinite values.')
        if np.linalg.norm(m[0,:2])<1e-8:raise RuntimeError('B arrow has no planar heading; rotate its +X axis into the floor plane.')
        world=np.array([m[3,0]*self.units,m[3,1]*self.units,math.atan2(m[0,1],m[0,0])])
        return compose(inverse(self.anchor),world)
    def goal_details(self):
        """Human-readable XY/yaw values for UI and exported goal metadata."""
        p=self.read_goal_pose();world=compose(self.anchor,p)
        return dict(map_xy_m=p[:2].tolist(),map_yaw_rad=float(p[2]),map_yaw_deg=float(math.degrees(p[2])),
                    world_xy_m=world[:2].tolist(),world_yaw_rad=float(world[2]),world_yaw_deg=float(math.degrees(world[2])),
                    yaw_constraint='REQUIRED; projected marker +X is desired vehicle-front heading',
                    marker_z_policy='IGNORED; ground projection uses stored floor display plane')
    def goal_arrow(self,pose,length=.45):
        """Three debug-draw segments projected onto the stored ground plane.

        Returns (starts, ends) in stage coordinates, matching debug_draw's
        draw_lines API. Unlike child geometry, these segments remain on the
        floor even if the user drags the marker vertically or tilts it.
        """
        p=np.asarray(pose,dtype=float)
        if p.shape!=(3,) or not np.isfinite(p).all():raise ValueError('Goal arrow requires finite map XY/yaw.')
        tip=compose(p,np.array([float(length),0.,0.]))
        left=compose(tip,np.array([-.12,.065,0.]))
        right=compose(tip,np.array([-.12,-.065,0.]))
        return [self.world(p),self.world(tip),self.world(tip)],[self.world(tip),self.world(left),self.world(right)]
    def goal(self,pose,use_yaw=False):
        p=self.read_goal_pose()
        if not use_yaw:p[2]=math.atan2(p[1]-pose[1],p[0]-pose[0])
        return p




MAPPING_GUARD_REVISION='2026-10-26-volume-guard-mapping-v7'

def _guard_quat_multiply(a,b):
    av=np.asarray(a[:3]);bv=np.asarray(b[:3]);aw=float(a[3]);bw=float(b[3])
    q=np.r_[aw*bv+bw*av+np.cross(av,bv),aw*bw-float(av@bv)]
    return q/np.linalg.norm(q)

def _check_motion_volume(self,v,w):
    """Additional simulated near-field volume sensor; does not supply a SLAM pose."""
    from isaacsim.core.simulation_manager import SimulationManager
    from pxr import Usd,UsdGeom
    now=float(SimulationManager.get_simulation_time())
    if self.body_view is None:raise RuntimeError('Volume guard has no attached sensor body; movement held at zero')
    pose=self.host(self.body_view.get_transforms())[0]
    velocity=self.host(self.body_view.get_velocities())[0]
    if not np.isfinite(pose).all() or not np.isfinite(velocity).all():raise RuntimeError('Nonfinite state inside protective sensor')
    q=pose[3:7]/np.linalg.norm(pose[3:7])
    if self.cfg['front_axis_sign']<0:q=_guard_quat_multiply(q,np.array([0.,0.,1.,0.]))
    rotation=self.rotation_row(q)
    body=self.cfg['body'];margin=float(self.cfg['margin'])
    bottom=float(self.cfg.get('volume_guard_bottom_m',.015));top=float(body['height'])+.02
    half=np.array([body['length']/2+margin,body['width']/2+margin,(top-bottom)/2])
    zcentre=(top+bottom)/2
    # The measured speed is used only inside this simulated safety sensor.
    measured_forward=float(velocity[:3]@rotation[0]*self.units)
    measured_w=float(velocity[5])
    # speed-envelope-v8-090: use the slower configured deceleration for both drive modes.
    decel=min(float(self.cfg.get('teleop_deceleration_m_s2',.5)),float(self.cfg.get('rpp',{}).get('deceleration',.4)))
    max_v=max(abs(float(v)),abs(measured_forward));max_w=max(abs(float(w)),abs(measured_w))
    horizon=max(.65,max_v/(2*max(.1,decel))+.20,float(getattr(self,'recovery_preview_seconds',0.)))
    samples=max(2,int(math.ceil(max_v*horizon/.025))+1,int(math.ceil(max_w*horizon/.05))+1)
    if samples>100:raise RuntimeError('Protective trajectory sampling exceeds configured speed range')
    floor_path=self.cfg.get('volume_guard_floor_path','/World/Colider/floor')
    floor=self.stage.GetPrimAtPath(floor_path);ignore_floor=False
    if floor.IsValid():
        bounds=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render','proxy']).ComputeWorldBound(floor).ComputeAlignedRange()
        if not bounds.IsEmpty():
            lo=np.array(bounds.GetMin())*self.units;hi=np.array(bounds.GetMax())*self.units
            # Only the explicitly named, thin support floor at/below the current base may be ignored.
            ignore_floor=bool(hi[2]-lo[2]<=.20 and hi[2]<=pose[2]*self.units+.025 and min(hi[:2]-lo[:2])>1.)
    ext=self.carb.Float3(*[float(x/self.units) for x in half])
    report=dict(revision=MAPPING_GUARD_REVISION,physics_s=now,blocked=False,
        requested_v_m_s=float(v),requested_w_rad_s=float(w),horizon_s=horizon,
        vertical_band_m=[bottom,top],half_size_m=half.tolist(),floor_ignored=ignore_floor,
        floor_path=floor_path,hits=[],sample_count=samples,
        current_pose_self_hits=0,
        note='3D simulated protective envelope over live collision geometry. Not a localization observation or navigation acceptance result.')
    # Also test measured residual motion when it differs materially from the command.
    trajectories=[(float(v),float(w))]
    if abs(measured_forward-v)>.04 or abs(measured_w-w)>.08:trajectories.append((measured_forward,measured_w))
    for test_v,test_w in trajectories:
        for t in np.linspace(0.,horizon,samples):
            angle=test_w*t
            delta=np.array([test_v*t,0.,0.]) if abs(test_w)<1e-6 else np.array([test_v/test_w*math.sin(angle),test_v/test_w*(1-math.cos(angle)),0.])
            future_q=_guard_quat_multiply(np.array([0.,0.,math.sin(angle/2),math.cos(angle/2)]),q)
            future_rotation=self.rotation_row(future_q)
            centre=pose[:3]+delta@rotation/self.units+np.array([0.,0.,zcentre])@future_rotation/self.units
            pos=self.carb.Float3(*[float(x) for x in centre]);quat=self.carb.Float4(*[float(x) for x in future_q])
            hit_paths=[];errors=[];self_hits=[]
            def callback(hit):
                try:
                    def field(name,alternate):
                        if isinstance(hit,dict):return str(hit.get(name,hit.get(alternate,'')))
                        return str(getattr(hit,name,getattr(hit,alternate,'')))
                    collision=field('collision','collision');actor=field('rigid_body','rigidBody')
                    if any(path==self.robot_root or path.startswith(self.robot_root+'/') for path in (collision,actor)):
                        self_hits.append(collision or actor);return True
                    if ignore_floor and any(path==floor_path for path in (collision,actor)):return True
                    hit_paths.append(dict(collision=collision or '<unknown>',rigid_body=actor or '<unknown>'))
                    return len(hit_paths)<8
                except Exception as exc:errors.append(str(exc));return False
            self.scene_query.overlap_box(ext,pos,quat,callback,False)
            if errors:raise RuntimeError('Volume sensor callback failed: '+errors[0])
            if t==0 and not hit_paths:
                report['current_pose_self_hits']=max(report['current_pose_self_hits'],len(self_hits))
                if not self_hits:raise RuntimeError('Volume query positive control did not see the current AGV body; movement refused')
            if hit_paths:
                report.update(blocked=True,prediction_seconds=float(t),
                    tested_v_m_s=test_v,tested_w_rad_s=test_w,box_centre_world_m=(centre*self.units).tolist(),
                    box_quaternion_xyzw=future_q.tolist(),hits=hit_paths)
                self.last_volume_guard_report=report
                return report
    self.last_volume_guard_report=report
    return report

ScanReader.check_motion_volume=_check_motion_volume



# recovery-sensor-boundary-v10
def _recovery_motion_v10(self):
    from isaacsim.core.simulation_manager import SimulationManager
    if self.body_view is None:raise RuntimeError('Recovery body sensor not attached')
    pose=self.host(self.body_view.get_transforms())[0]
    velocity=self.host(self.body_view.get_velocities())[0]
    if not np.isfinite(pose).all() or not np.isfinite(velocity).all():raise RuntimeError('Invalid recovery body sensor sample')
    q=pose[3:7]/np.linalg.norm(pose[3:7])
    if self.cfg['front_axis_sign']<0:q=_guard_quat_multiply(q,np.array([0.,0.,1.,0.]))
    rotation=self.rotation_row(q)
    return dict(physics_s=float(SimulationManager.get_simulation_time()),
        linear_front_m_s=(velocity[:3]@rotation.T*self.units).tolist(),
        angular_front_rad_s=(velocity[3:]@rotation.T).tolist(),upright_cosine=float(rotation[2,2]),
        usage='Simulated safety/progress sensor only; not a map position, heading prior or SLAM observation')

def _check_recovery_volume_v10(self,v,w,seconds):
    if not math.isfinite(seconds) or not 0.<seconds<=6.:raise RuntimeError('Invalid recovery preview interval')
    self.recovery_preview_seconds=float(seconds)
    try:return self.check_motion_volume(float(v),float(w))
    finally:self.recovery_preview_seconds=0.

ScanReader.read_recovery_motion=_recovery_motion_v10
ScanReader.check_recovery_volume=_check_recovery_volume_v10
