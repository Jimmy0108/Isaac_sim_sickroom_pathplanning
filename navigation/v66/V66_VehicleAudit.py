"""Independent read-only Isaac Sim 6.0.1 vehicle check. RUN, then Play.

No navigation-package imports, motor writes, API authoring, USD save or physics
setup. If already playing, attaches to the existing manager view. Static checks
also run while stopped. Live recording lasts 10 SIMULATION seconds, or press
Export now. A missing runtime API is recorded as unavailable, not as zero.
"""
import asyncio
import csv
import hashlib
import json
import math
import time
import zipfile
from pathlib import Path

import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, Gf

PROJECT = Path('/isaac-sim/.local/share/ov/data/AMR_Project/1019_Scene/Collected_sickroom_with_hand_colaTest/new_car/python_nav/V66_Python_Navigation')
BASE = '/World/AGV_configured_v4/TZAGV_B02_vendor_v4/TZAGV_B02_root/base_link'
CONTROLLER = '/World/AGV_control/AGV_DifferentialGraph/DifferentialController'
LIDAR = BASE + '/Lidar_2D'
RECORD_SIM_SECONDS = 10.0
EXPECTED = dict(wheel_radius=.08, track=.4769668332689978, caster_radius=.0315,
                caster_width=.019262928306058, drive_width=.055,
                length=.898, width=.5698, height=.444, shell_size=[.810,.560,.250],
                lidar_xyz_m=[-.23,0,.345])
WHEELS = ('left_drive_wheel_link','right_drive_wheel_link',
          'front_left_caster_wheel_link','front_right_caster_wheel_link',
          'rear_left_caster_wheel_link','rear_right_caster_wheel_link')


def plain(value):
    if value is None or isinstance(value, (str, bool, int)): return value
    if isinstance(value, (float, np.floating)): return float(value) if math.isfinite(value) else str(value)
    if isinstance(value, np.integer): return int(value)
    if isinstance(value, np.bool_): return bool(value)
    if isinstance(value, dict): return {str(k): plain(v) for k,v in value.items()}
    try: return [plain(v) for v in value]
    except TypeError: return str(value)


def host(value):
    for method in ('detach','cpu','numpy'):
        fn = getattr(value, method, None)
        if callable(fn): value = fn()
    return np.asarray(value)


def write_json(path, value):
    Path(path).write_text(json.dumps(plain(value), ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')


def classify_caster_drive(attributes):
    """A zero-target, zero-stiffness damper dissipates motion; it is not steering.

    Keep the raw damping/force limits in the report. Asymmetry is a separate
    warning, not proof of a drift cause and not a request to retune the vehicle.
    """
    def value(key):return float(attributes.get('drive:angular:physics:'+key,0.) or 0.)
    k,d,target=value('stiffness'),value('damping'),value('targetVelocity')
    valid=all(math.isfinite(x) for x in (k,d,target)) and k>=0 and d>=0
    passive=valid and k==0 and (d==0 or target==0)
    kind='UNDAMPED' if passive and d==0 else 'PASSIVE_DAMPING' if passive else 'SERVO_OR_INVALID_DRIVE'
    return dict(passive=passive,classification=kind,stiffness=k,damping=d,target_velocity=target)


def collect_vehicle(stage, base=BASE, controller=CONTROLLER, lidar=LIDAR, evaluated=False):
    """Inspect the composed stage. Geometry dimensions are NOT contact proof."""
    result = dict(revision='2026-11-06-V66-readonly-audit-1.0.1', stage=stage.GetRootLayer().identifier,
        expected=EXPECTED, checks=[], wheels=[], joints=[], bodies=[], colliders=[], graph={},
        bindings=[], physics_scenes=[], errors=[], base=base, controller=controller, lidar=lidar,
        session_layers=list(stage.GetSessionLayer().subLayerPaths),
        limitations=['Static wheel-bottom equality is not measured ground contact.',
                    'Mass/inertia/friction in V66 assets are simulation baselines, not measured real hardware.',
                    'Lidar mount is approximate; primary nav is custom PhysX collision raycast, not the RTX point cloud.',
                    'No active D555 depth sensor is implied by camera geometry.'])
    def check(name, ok, actual, expected=None, required=True):
        result['checks'].append(dict(name=name, status='PASS' if ok else 'FAIL' if required else 'WARN',
                                    actual=plain(actual), expected=plain(expected)))
    unit = float(UsdGeom.GetStageMetersPerUnit(stage))
    check('metres_per_unit', math.isclose(unit,1.,abs_tol=1e-9),unit,1.)
    check('Z_up', str(UsdGeom.GetStageUpAxis(stage))=='Z', str(UsdGeom.GetStageUpAxis(stage)),'Z')
    b = stage.GetPrimAtPath(base)
    check('base_exists', bool(b), bool(b),True)
    if not b:
        result['blocking_checks']=[x['name'] for x in result['checks'] if x['status']=='FAIL']
        return plain(result)
    root = b.GetParent()
    cache = UsdGeom.XformCache()
    base_world = np.asarray(cache.GetLocalToWorldTransform(b),float)
    base_inv = np.linalg.inv(base_world)
    bbox = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default','render','proxy','guide'])
    check('base_transform_rigid', np.allclose(base_world[:3,:3]@base_world[:3,:3].T,np.eye(3),atol=1e-5), base_world[:3,:3], 'orthonormal, no scaling')
    def matrix(p):
        # Standalone USD (e.g. Blender) may lack the OmniLidar type schema.
        # XformCache then ignores that prim's authored ops even though the
        # ops are readable. Evaluate its local ops explicitly, never invent a
        # zero sensor extrinsic because an optional schema is unavailable.
        if p.GetAttribute('xformOpOrder') and not UsdGeom.Xformable(p):
            xf=UsdGeom.Xformable(p);local=np.asarray(xf.GetLocalTransformation(),float)
            return local if xf.GetResetXformStack() else local@matrix(p.GetParent())
        return np.asarray(cache.GetLocalToWorldTransform(p),float)
    def attrs(p,prefix): return {a.GetName():plain(a.Get()) for a in p.GetAttributes() if a.GetName().startswith(prefix)}
    def local_bounds(p):
        # Compute bound in the base frame, not a rotated world AABB.
        bb = bbox.ComputeWorldBound(p)
        bb.SetMatrix(bb.GetMatrix()*Gf.Matrix4d(base_inv.tolist()))
        r = bb.ComputeAlignedRange()
        return np.asarray(r.GetMin())*unit, np.asarray(r.GetMax())*unit
    roots = []
    for p in Usd.PrimRange(root):
        if p.HasAPI(UsdPhysics.ArticulationRootAPI): roots.append(str(p.GetPath()))
        if p.HasAPI(UsdPhysics.RigidBodyAPI):
            result['bodies'].append(dict(path=str(p.GetPath()), attributes=attrs(p,('physics:','physxRigidBody:','physxArticulation:'))))
        if p.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get() is not False:
            mat, _ = UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial('physics')
            result['colliders'].append(dict(path=str(p.GetPath()),type=p.GetTypeName(),
                owner=next((str(q.GetPath()) for q in ancestors(p) if q.HasAPI(UsdPhysics.RigidBodyAPI)),None),
                material=str(mat.GetPath()) if mat else None,
                material_properties=attrs(mat.GetPrim(),('physics:',)) if mat else {},
                attributes=attrs(p,('physics:','physxCollision:','physxMeshCollision:'))))
        if p.IsA(UsdPhysics.Joint):
            j = UsdPhysics.Joint(p)
            targets=[j.GetBody0Rel().GetTargets(),j.GetBody1Rel().GetTargets()]
            record=dict(path=str(p.GetPath()),body0=[str(x) for x in targets[0]],body1=[str(x) for x in targets[1]],
                        attributes=attrs(p,('physics:','drive:','physxJoint:')))
            axis=str(p.GetAttribute('physics:axis').Get())
            expected_axis='Z' if 'swivel' in p.GetName() else 'Y'
            check('joint_axis:'+p.GetName(),axis==expected_axis,axis,expected_axis)
            if p.GetName() in ('left_wheel_joint','right_wheel_joint'):
                drv=UsdPhysics.DriveAPI(p,'angular')
                values=[drv.GetStiffnessAttr().Get(),drv.GetDampingAttr().Get(),drv.GetMaxForceAttr().Get()] if drv else []
                valid=len(values)==3 and values[0]==0 and values[1] is not None and values[1]>0 and values[2] is not None and values[2]>0
                check('velocity_drive:'+p.GetName(),valid,values,'stiffness=0, damping>0, maxForce>0; values are retained')
            ends=[]; rots=[]
            for i, ts in enumerate(targets):
                body = stage.GetPrimAtPath(ts[0]) if len(ts)==1 else None
                if not body: continue
                m = matrix(body)
                pos = np.asarray((j.GetLocalPos0Attr() if i==0 else j.GetLocalPos1Attr()).Get(),float)
                q = (j.GetLocalRot0Attr() if i==0 else j.GetLocalRot1Attr()).Get()
                ends.append((pos@m[:3,:3]+m[3,:3])*unit)
                rots.append(np.asarray(Gf.Matrix3d(q),float)@m[:3,:3])
            if len(ends)==2:
                record['world_anchor_gap_m']=float(np.linalg.norm(ends[1]-ends[0]))
                check('joint_anchor:'+p.GetName(),record['world_anchor_gap_m']<.001,record['world_anchor_gap_m'],'< 0.001 m')
                axis=str(p.GetAttribute('physics:axis').Get() or 'X')
                idx={'X':0,'Y':1,'Z':2}.get(axis,0)
                dot=abs(float(np.dot(rots[0][idx],rots[1][idx])))
                check('joint_axis_alignment:'+p.GetName(),abs(dot-1)<1e-4,dot,'parallel axes')
            else: check('joint_bodies:'+p.GetName(),False,record,'two existing rigid bodies')
            if p.GetName() not in ('left_wheel_joint','right_wheel_joint'):
                record['drive_classification']=classify_caster_drive(record['attributes'])
                check('passive_caster:'+p.GetName(),record['drive_classification']['passive'],record['attributes'],'zero-stiffness damping with zero velocity target is allowed; no steering servo')
            result['joints'].append(record)
    swivel_damping={j['path'].rsplit('/',1)[-1]:j['drive_classification']['damping'] for j in result['joints'] if 'caster_swivel_joint' in j['path']}
    if len(swivel_damping)==4:
        values=list(swivel_damping.values())
        check('caster_swivel_damping_consistency',max(values)-min(values)<1e-6,swivel_damping,'same damping across four casters; difference is a warning, not a proven drift cause',False)
    check('one_articulation_root', roots==[base],roots,[base])
    check('rigid_body_count',len(result['bodies'])==11,len(result['bodies']),11)
    check('joint_count',len(result['joints'])==10,len(result['joints']),10)
    for name in WHEELS:
        owner=stage.GetPrimAtPath(str(root.GetPath())+'/'+name)
        shapes=[p for p in Usd.PrimRange(owner) if p.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get() is not False] if owner else []
        check('one_wheel_collider:'+name,len(shapes)==1,[str(p.GetPath()) for p in shapes],1)
        if len(shapes)!=1: continue
        p=shapes[0]
        rec=dict(body=str(owner.GetPath()), collider=str(p.GetPath()), type=p.GetTypeName())
        if p.IsA(UsdGeom.Cylinder):
            c=UsdGeom.Cylinder(p); m=matrix(p)@base_inv; axis=str(c.GetAxisAttr().Get())
            scale=np.linalg.norm(m[:3,:3],axis=1)
            idx={'X':0,'Y':1,'Z':2}[axis]; radial=[i for i in range(3) if i!=idx]
            rec.update(radius_m=float(c.GetRadiusAttr().Get())*unit*float(scale[radial].max()),
                       axial_width_m=float(c.GetHeightAttr().Get())*unit*float(scale[idx]),
                       centre_base_m=(m[3,:3]*unit).tolist(), axis=axis,scale=scale.tolist())
            # Analytic support of a finite cylinder, not its rotated box.
            # A wheel rolling about Y must not appear to grow by sqrt(2) at
            # 45 degrees just because USD rotated a rectangular bounding box.
            z_half=(float(c.GetRadiusAttr().Get())*math.hypot(m[radial[0],2],m[radial[1],2])+
                    float(c.GetHeightAttr().Get())*.5*abs(m[idx,2]))*unit
            rec['bottom_base_z_m']=float(m[3,2]*unit-z_half)
            rec['top_base_z_m']=float(m[3,2]*unit+z_half)
            check('rigid_wheel_collider_transform:'+name,np.allclose(m[:3,:3]@m[:3,:3].T,np.eye(3),atol=1e-5),scale,'no scaling/skew')
            drive='drive' in name
            check('wheel_radius:'+name,abs(rec['radius_m']-EXPECTED['wheel_radius' if drive else 'caster_radius'])<1e-5,rec['radius_m'],EXPECTED['wheel_radius' if drive else 'caster_radius'])
            check('wheel_width:'+name,abs(rec['axial_width_m']-EXPECTED['drive_width' if drive else 'caster_width'])<1e-5,rec['axial_width_m'],EXPECTED['drive_width' if drive else 'caster_width'])
        else: check('native_cylinder:'+name,False,p.GetTypeName(),'Cylinder')
        result['wheels'].append(rec)
    wheels={r['body'].rsplit('/',1)[-1]:r for r in result['wheels']}
    if all(n in wheels and 'centre_base_m' in wheels[n] for n in WHEELS[:2]):
        a,bp=[np.asarray(wheels[n]['centre_base_m']) for n in WHEELS[:2]]
        track=abs(float(a[1]-bp[1]));result['measured_drive_track_m']=track
        check('drive_track',abs(track-EXPECTED['track'])<1e-5,track,EXPECTED['track'])
        check('drive_axles_aligned_XZ',np.linalg.norm((a-bp)[[0,2]])<1e-5,(a-bp)[[0,2]],[0,0])
    bottoms=[r['bottom_base_z_m'] for r in result['wheels'] if 'bottom_base_z_m' in r]
    if len(bottoms)==6: check('six_bottoms_coplanar_static',max(bottoms)-min(bottoms)<.001,bottoms,'spread < 0.001 m')
    shells=[p for p in Usd.PrimRange(root) if p.GetName()=='chassis_visual']
    if len(shells)==1:
        lo,hi=local_bounds(shells[0]); result['shell_base_bounds_m']=[lo.tolist(),hi.tolist()]
        check('shell_dimensions',np.allclose(hi-lo,EXPECTED['shell_size'],atol=1e-4),hi-lo,EXPECTED['shell_size'])
    else: check('shell_measurement',False,[str(p.GetPath()) for p in shells],'one chassis_visual',False)
    lo,hi=local_bounds(root)
    result['whole_vehicle_base_bounds_m']=[lo.tolist(),hi.tolist()]
    # This is a conservative footprint check, including active collision guides,
    # bumpers and visible wheels. It is different from the shell dimensions.
    check('navigation_footprint_contains_geometry',bool(np.all(lo[:2]>=-np.array([EXPECTED['length'],EXPECTED['width']])/2-.001) and np.all(hi[:2]<=np.array([EXPECTED['length'],EXPECTED['width']])/2+.001)),[lo[:2],hi[:2]],[EXPECTED['length'],EXPECTED['width']])
    lp=stage.GetPrimAtPath(lidar)
    check('lidar_mount_exists',bool(lp),bool(lp),True)
    if lp:
        lm=matrix(lp)@base_inv; result['lidar_to_base_matrix']=lm.tolist()
        check('lidar_mount_translation',np.allclose(lm[3,:3]*unit,EXPECTED['lidar_xyz_m'],atol=1e-5),lm[3,:3]*unit,EXPECTED['lidar_xyz_m'])
        check('lidar_horizontal',np.linalg.norm(lm[:2,2])<.01,lm[:2,2],'horizontal')
        result['lidar_profile']=attrs(lp,('omni:sensor:Core:',))
        result['lidar_transform_opinion_sources']={a.GetName():[
            dict(layer=s.layer.identifier,path=str(s.path),default=plain(s.default)) for s in a.GetPropertyStack()]
            for a in lp.GetAttributes() if a.GetName().startswith('xformOp')}
        inside=[]
        for item in result['colliders']:
            prim=stage.GetPrimAtPath(item['path'])
            if not prim.IsA(UsdGeom.Cube):continue
            world_point=matrix(lp)[3,:3]
            inv=np.linalg.inv(matrix(prim));local=world_point@inv[:3,:3]+inv[3,:3]
            size=float(UsdGeom.Cube(prim).GetSizeAttr().Get())
            if np.all(abs(local)<size*.5-1e-7):inside.append(item['path'])
        result['sensor_inside_own_cube_colliders']=inside
        check('sensor_origin_outside_chassis_colliders',not inside,inside,'sensor origin outside own solid chassis collision')
    cp=stage.GetPrimAtPath(controller)
    check('controller_exists',bool(cp),bool(cp),True)
    if cp:
        for a in cp.GetAttributes():
            if a.GetName().startswith(('inputs:','outputs:')):
                value=a.Get()
                if evaluated:
                    try:
                        import omni.graph.core as og
                        value=og.Controller.get(og.Controller.attribute(str(a.GetPath())))
                    except Exception as exc:result['errors'].append(str(a.GetPath())+': '+str(exc))
                result['graph'][a.GetName()]=plain(value)
        for key,expected in [('wheelRadius',EXPECTED['wheel_radius']),('wheelDistance',EXPECTED['track'])]:
            value=result['graph'].get('inputs:'+key)
            check('controller_'+key,isinstance(value,(float,int)) and abs(value-expected)<1e-5,value,expected)
        for p in Usd.PrimRange(cp.GetParent()):
            if p.GetAttribute('inputs:robotPath'):
                result['bindings'].append(dict(path=str(p.GetPath()),
                    robot_path=p.GetAttribute('inputs:robotPath').Get(),
                    joint_names=plain(p.GetAttribute('inputs:jointNames').Get()),
                    joint_name_connections=[str(x) for x in p.GetAttribute('inputs:jointNames').GetConnections()],
                    targets=[str(x) for x in p.GetRelationship('inputs:targetPrim').GetTargets()]))
        check('single_articulation_controller',len(result['bindings'])==1,result['bindings'],'one node')
        if len(result['bindings'])==1:
            bind=result['bindings'][0]
            # The control USD is referenced separately from the robot USD.
            # Out-of-reference targetPrim relations can be pruned by USD;
            # the explicit robotPath string is the supported alternate input.
            check('articulation_target',bind['robot_path']==base and bind['targets'] in ([],[base]),bind,base)
            result['articulation_target_policy']='Explicit robotPath; targetPrim may be empty after USD reference composition. Evaluated wheel joint names are verified after Play.'
    for p in stage.Traverse():
        if p.IsA(UsdPhysics.Scene): result['physics_scenes'].append(dict(path=str(p.GetPath()),attributes=attrs(p,('physics:','physxScene:'))))
    writers=[]
    for p in stage.Traverse():
        a=p.GetAttribute('inputs:robotPath')
        targets=p.GetRelationship('inputs:targetPrim').GetTargets()
        if p.GetAttribute('inputs:velocityCommand') and ((a and a.Get()==base) or any(str(t)==base for t in targets)):
            writers.append(str(p.GetPath()))
    result['matching_articulation_writers']=writers
    check('one_graph_motor_writer',len(writers)==1,writers,'one articulation controller targeting this base')
    check('one_active_physics_scene',len(result['physics_scenes'])==1,[r['path'] for r in result['physics_scenes']],1)
    result['blocking_checks']=[c['name'] for c in result['checks'] if c['status']=='FAIL']
    result['static_contract_ok']=not result['blocking_checks']
    result['runtime_acceptance']='NOT_TESTED; no movement is commanded by this audit'
    return plain(result)


def ancestors(prim):
    while prim and not prim.IsPseudoRoot():
        yield prim
        prim=prim.GetParent()


class AuditSession:
    def __init__(self):
        self.closed=False;self.rows=[];self.callback=None;self.attached=False;self.error=None
        self.start_sim=self.start_wall=None;self.last_sample=-math.inf
        self.last_physics=None;self.callback_count=0;self.clock_mismatch_count=0;self.clock_mismatch_examples=[]
        self.phase='CREATED';self.view_wait_updates=0
        self.output=PROJECT/'validation_outputs'/('vehicle_audit_'+str(time.time_ns()))
        self.output.mkdir(parents=True)

    def sample(self,*args):
        if self.closed:return
        try:
            now=float(self.manager.get_simulation_time())
            if self.start_sim is None:self.start_sim=now;self.start_wall=time.monotonic()
            self.callback_count+=1
            dt=float(args[0]) if args and np.isscalar(args[0]) else None
            if self.last_physics is not None and dt is not None:
                delta=now-self.last_physics
                if not math.isclose(delta,dt,rel_tol=1e-4,abs_tol=1e-8):
                    self.clock_mismatch_count+=1
                    if len(self.clock_mismatch_examples)<20:self.clock_mismatch_examples.append(dict(manager_s=now,delta_manager_s=delta,callback_dt_s=dt))
            self.last_physics=now
            if now<self.last_sample:raise RuntimeError('Simulation clock reset; audit stopped')
            if now-self.last_sample<.05-1e-8:return
            self.last_sample=now
            row=dict(physics_s=now,wall_elapsed_s=time.monotonic()-self.start_wall,
                     callback_dt_s=dt)
            for method,key in [('get_dof_positions','joint_positions_rad'),('get_dof_velocities','joint_velocities_rad_s'),('get_dof_velocity_targets','joint_targets_rad_s')]:
                try:row[key]=host(getattr(self.art,method)())[0].astype(float).tolist()
                except Exception as exc:row[key]=None;row[key+'_error']=str(exc)
            row['body_pose_xyzw']=host(self.body.get_transforms())[0].astype(float).tolist()
            row['body_velocity_world']=host(self.body.get_velocities())[0].astype(float).tolist()
            if getattr(self,'wheel_bodies',None) is not None:
                row['six_wheel_pose_world_xyzw']=host(self.wheel_bodies.get_transforms()).astype(float).tolist()
                row['six_wheel_velocity_world']=host(self.wheel_bodies.get_velocities()).astype(float).tolist()
            import omni.graph.core as og
            for key in ['inputs:linearVelocity','inputs:angularVelocity','outputs:velocityCommand']:
                try:row[key]=plain(og.Controller.get(og.Controller.attribute(CONTROLLER+'.'+key)))
                except Exception as exc:row[key]=None;row[key+'_error']=str(exc)
            self.rows.append(row)
            self.label.text=f'Recording (read-only): {now-self.start_sim:.2f}/{RECORD_SIM_SECONDS:.1f} simulation seconds; {len(self.rows)} samples.\nStatic failures: {self.report.get("blocking_checks")}\nUse your existing manual panel if you want to record movement. Audit sends no commands.'
            if now-self.start_sim>=RECORD_SIM_SECONDS:self.closed=True
        except Exception as exc:self.error=type(exc).__name__+': '+str(exc);self.closed=True

    async def run(self):
        import omni.usd,omni.timeline,omni.kit.app,omni.ui as ui
        from isaacsim.core.simulation_manager import SimulationManager,SimulationEvent
        self.manager=SimulationManager;self.stage=omni.usd.get_context().get_stage()
        if self.stage is None:raise RuntimeError('Open your V66 scene first')
        self.report=collect_vehicle(self.stage,evaluated=True)
        self.phase='WAITING_FOR_PLAY'
        write_json(self.output/'vehicle_static.json',self.report)
        self.window=ui.Window('V66 | Read-only vehicle audit',width=750,height=320)
        with self.window.frame:
            with ui.VStack():
                self.label=ui.Label('Static report saved. Press Play to record actual joint/body data.\nBlocking checks: '+str(self.report.get('blocking_checks'))+'\nNo movement or USD changes are made.',word_wrap=True)
                ui.Button('Export now / end read-only recording',clicked_fn=lambda:setattr(self,'closed',True),height=40)
        timeline=omni.timeline.get_timeline_interface();app=omni.kit.app.get_app()
        try:
            while not self.closed:
                await app.next_update_async()
                if not self.window.visible:self.closed=True;break
                if omni.usd.get_context().get_stage()!=self.stage:raise RuntimeError('Stage changed')
                if timeline.is_stopped() and self.attached:break
                if not timeline.is_playing():
                    self.phase='PAUSED' if self.attached else 'WAITING_FOR_PLAY'
                    self.label.text='WAITING FOR PLAY: static report saved; runtime samples='+str(len(self.rows))+'.\nPress Play, then wait for RECORDING. Export now would contain static data only.\nStatic failures: '+str(self.report.get('blocking_checks'))
                    continue
                if not self.attached:
                    self.phase='WAITING_FOR_PHYSICS_VIEW'
                    view=SimulationManager.get_physics_simulation_view()
                    if view is None:
                        self.view_wait_updates+=1
                        self.label.text='PLAY detected, waiting for attached Physics view. No runtime samples yet.\nThe audit will not create or configure another simulation.\nYou can export this state if it remains stuck.'
                        continue
                    self.art=view.create_articulation_view(BASE);self.body=view.create_rigid_body_view(BASE)
                    if self.art.count!=1 or self.body.count!=1:raise RuntimeError('Expected exactly one V66 articulation/body')
                    self.report['joint_order']=list(self.art.shared_metatype.dof_names)
                    self.wheel_bodies=None
                    try:
                        wheels=view.create_rigid_body_view(BASE.rsplit('/',1)[0]+'/*wheel_link')
                        if wheels.count!=6:raise RuntimeError('Expected six wheel bodies')
                        self.report['six_wheel_body_order']=[str(x) for x in wheels.prim_paths]
                        self.wheel_bodies=wheels
                    except Exception as exc:self.report['six_wheel_body_read_error']=str(exc)
                    for name in ('get_masses','get_inertias','get_coms'):
                        try:self.report['runtime_'+name]=plain(host(getattr(self.art,name)()))
                        except Exception as exc:self.report['runtime_'+name+'_error']=str(exc)
                    self.callback=SimulationManager.register_callback(self.sample,event=SimulationEvent.PHYSICS_POST_STEP)
                    self.attached=True;self.phase='RECORDING'
        except Exception as exc:self.error=type(exc).__name__+': '+str(exc)
        finally:
            if self.callback is not None:SimulationManager.deregister_callback(self.callback)
            self.report['recording_error']=self.error;self.report['live_samples']=len(self.rows)
            self.report['runtime_attachment']=dict(last_phase=self.phase,attached=self.attached,view_wait_updates=self.view_wait_updates)
            self.report['clock_audit']=dict(physics_callbacks=self.callback_count,mismatches=self.clock_mismatch_count,
                examples=self.clock_mismatch_examples,policy='Compare per-step callback dt with manager-clock increment, never rescale timestamps')
            self.report['runtime_acceptance']='RECORDED_ONLY; motion/contact/SLAM success not asserted' if self.rows else 'NO_RUNTIME_DATA; static data only, no clock or motion acceptance'
            write_json(self.output/'vehicle_static.json',self.report);write_json(self.output/'runtime_samples.json',self.rows)
            lines=['V66 READ-ONLY VEHICLE REPORT','Static contract: '+str(self.report.get('static_contract_ok')),
                   'Runtime samples: '+str(len(self.rows)),'Runtime error: '+str(self.error),
                   'Simulation did not command motion or modify USD.',
                   'Static dimensions are not proof of ground contact or stable motion.','']
            for item in self.report.get('checks',[]):
                lines.append(item['status']+' '+item['name']+' | actual='+str(item['actual'])+' | expected='+str(item['expected']))
            (self.output/'REPORT.txt').write_text('\n'.join(lines),encoding='utf-8')
            with (self.output/'runtime_samples.csv').open('w',encoding='utf-8',newline='') as f:
                fields=sorted({k for r in self.rows for k in r})
                writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader()
                writer.writerows({k:json.dumps(v) if isinstance(v,(list,dict)) else v for k,v in r.items()} for r in self.rows)
            if self.rows:
                sim=self.rows[-1]['physics_s']-self.rows[0]['physics_s'];wall=self.rows[-1]['wall_elapsed_s']-self.rows[0]['wall_elapsed_s']
                write_json(self.output/'timing_summary.json',dict(recorded_sim_seconds=sim,recorded_wall_seconds=wall,RTF=sim/wall if wall>0 else None))
            archive=self.output.with_suffix('.zip')
            with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
                for p in self.output.iterdir():z.write(p,p.name)
            self.label.text='AUDIT FINISHED. No motor commands or USD writes were made.\nZIP: '+str(archive)+'\nError: '+str(self.error)
            print('[V66 audit] FINISHED:',archive,'; error:',self.error)


def launch_audit():
    old=globals().get('_v66_audit_session')
    if old is not None:old.closed=True
    async def restart():
        previous=globals().get('_v66_audit_task')
        # The public launcher assigns the new task after this coroutine is scheduled.
        if old is not None:
            oldtask=getattr(old,'task',None)
            if oldtask is not None and not oldtask.done():await oldtask
        session=AuditSession();globals()['_v66_audit_session']=session
        session.task=asyncio.current_task()
        await session.run()
    globals()['_v66_audit_task']=asyncio.ensure_future(restart())


if __name__=='__main__':launch_audit()

