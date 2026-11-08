"""Proven V10.4 startup adapters. No tuning matrix; session overrides are reversible."""
import math
import numpy as np
NAV_PATCH_REVISION = 'v10.5.6-single-verified-physics-clock'
DRIVE_PATCH_REVISION = 'v10.5-preserved-effective-drive'
_NAV_CLOCK_SESSION_LAYERS=[]
_NAV_DRIVE_SESSION_LAYERS=[]

CASTER_WHEELS = ('front_left_caster_wheel_link', 'front_right_caster_wheel_link',
                 'rear_left_caster_wheel_link', 'rear_right_caster_wheel_link')


def _caster_repair_target(counts):
    """Only repair the recorded single missing caster, not unknown geometries."""
    if set(counts) != set(CASTER_WHEELS) or any(n > 1 or n < 0 for n in counts.values()):
        raise RuntimeError('Unexpected caster collider inventory: '+str(counts))
    missing = [name for name in CASTER_WHEELS if counts[name] == 0]
    if not missing:
        return None
    if missing != ['front_left_caster_wheel_link']:
        raise RuntimeError('Different caster is missing; refusing to guess a repair: '+str(missing))
    return missing[0]


def _validate_caster_points(points):
    """Owner-local measurements must match this run's 30mm/25mm caster."""
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 16 or not np.isfinite(points).all():
        raise RuntimeError('Invalid reference caster vertices')
    lo, hi = points.min(0), points.max(0)
    if not np.allclose((hi+lo)/2, 0, atol=1e-5):
        raise RuntimeError('Reference caster is offset from wheel joint origin')
    if not np.allclose(hi-lo, [.06,.025,.06], atol=1e-5):
        raise RuntimeError('Reference caster does not match recorded 60mm x 25mm size')
    return points


def _repair_missing_caster(stage, paths):
    """Clone the proven opposite caster's collision mesh in the owning frame.

    No extra drive or rigid body is created. Restores with the session layer.
    """
    from pxr import Usd, UsdGeom, UsdPhysics, UsdShade, PhysxSchema, Sdf, Gf
    parent = stage.GetPrimAtPath(paths['base']).GetParent()
    wheels, shapes = {}, {}
    for name in CASTER_WHEELS:
        found = [p for p in Usd.PrimRange(parent) if p.GetName() == name and p.HasAPI(UsdPhysics.RigidBodyAPI)]
        if len(found) != 1:
            raise RuntimeError('Cannot resolve caster rigid body: '+name)
        wheels[name] = found[0]
        shapes[name] = [p for p in Usd.PrimRange(found[0], Usd.TraverseInstanceProxies())
                        if p.IsA(UsdGeom.Gprim) and p.HasAPI(UsdPhysics.CollisionAPI)
                        and UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()]
    before = {name:len(p) for name,p in shapes.items()}
    target = _caster_repair_target(before)
    if target is None:
        return dict(revision='v10.5.4-caster-collision', action='already_complete', before=before)
    reference = shapes['front_right_caster_wheel_link'][0]
    if not reference.IsA(UsdGeom.Mesh) or wheels[target].IsInstance() or wheels[target].IsInstanceProxy():
        raise RuntimeError('Cannot safely clone reference caster mesh into this body')
    wheel_joints = [p for p in Usd.PrimRange(parent) if p.IsA(UsdPhysics.RevoluteJoint)
                    and wheels[target].GetPath() in UsdPhysics.Joint(p).GetBody1Rel().GetTargets()]
    if len(wheel_joints) != 1:
        raise RuntimeError('Cannot identify missing caster wheel joint uniquely')
    joint = UsdPhysics.RevoluteJoint(wheel_joints[0])
    if str(joint.GetAxisAttr().Get()) != 'Y' or np.linalg.norm(joint.GetLocalPos1Attr().Get()) > 1e-6:
        raise RuntimeError('Missing caster joint axis/origin differs from recorded model')
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    units = float(UsdGeom.GetStageMetersPerUnit(stage))
    source_mesh = UsdGeom.Mesh(reference)
    points = np.asarray(source_mesh.GetPointsAttr().Get(), dtype=float)
    matrix = (np.array(cache.GetLocalToWorldTransform(reference)) @
              np.linalg.inv(np.array(cache.GetLocalToWorldTransform(wheels['front_right_caster_wheel_link']))))
    local = (points @ matrix[:3,:3]+matrix[3,:3])*units
    _validate_caster_points(local)
    # Confirm the other two caster geometries before using a single reference.
    for name in ('rear_left_caster_wheel_link', 'rear_right_caster_wheel_link'):
        prim = shapes[name][0]
        if not prim.IsA(UsdGeom.Mesh):
            raise RuntimeError('Other caster is not the recorded mesh: '+name)
        m = np.array(cache.GetLocalToWorldTransform(prim)) @ np.linalg.inv(np.array(cache.GetLocalToWorldTransform(wheels[name])))
        p = np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get(), dtype=float)
        _validate_caster_points((p @ m[:3,:3]+m[3,:3])*units)
    destination = wheels[target].GetPath().AppendChild('v1054_caster_collision')
    if stage.GetPrimAtPath(destination).IsValid():
        raise RuntimeError('Caster repair path already exists; stop and inspect before retry')
    layer = Sdf.Layer.CreateAnonymous('amr_v1054_missing_caster.usda')
    session = stage.GetSessionLayer()
    try:
        session.subLayerPaths = [layer.identifier]+list(session.subLayerPaths)
        with Usd.EditContext(stage, layer):
            mesh = UsdGeom.Mesh.Define(stage, destination)
            mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in local/units])
            mesh.CreateFaceVertexCountsAttr(source_mesh.GetFaceVertexCountsAttr().Get())
            mesh.CreateFaceVertexIndicesAttr(source_mesh.GetFaceVertexIndicesAttr().Get())
            mesh.CreateSubdivisionSchemeAttr('none')
            mesh.CreateOrientationAttr(source_mesh.GetOrientationAttr().Get())
            mesh.CreateExtentAttr([Gf.Vec3f(*(local.min(0)/units)),Gf.Vec3f(*(local.max(0)/units))])
            mesh.CreateVisibilityAttr('invisible')
            mesh.CreatePurposeAttr('proxy')
            prim = mesh.GetPrim()
            UsdPhysics.CollisionAPI.Apply(prim).CreateCollisionEnabledAttr(True)
            UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr('convexHull')
            PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim)
            # Preserve the reference collision settings, including hull cook
            # limits. Do not copy its transforms or rigid-body properties.
            for attr in reference.GetAuthoredAttributes():
                name = attr.GetName()
                if name.startswith(('physxCollision:', 'physxConvexHullCollision:')):
                    value = attr.Get()
                    if value is not None:
                        prim.CreateAttribute(name, attr.GetTypeName()).Set(value)
            material, _ = UsdShade.MaterialBindingAPI(reference).ComputeBoundMaterial('physics')
            if material:
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(material, UsdShade.Tokens.strongerThanDescendants, 'physics')
        if not UsdPhysics.CollisionAPI(stage.GetPrimAtPath(destination)).GetCollisionEnabledAttr().Get():
            raise RuntimeError('New caster collision did not resolve enabled')
    except BaseException:
        session.subLayerPaths = [p for p in session.subLayerPaths if p != layer.identifier]
        raise
    _NAV_DRIVE_SESSION_LAYERS.append(layer)
    return dict(revision='v10.5.4-caster-collision', action='added_missing_front_left_collision',
                before=before, reference=str(reference.GetPath()), destination=str(destination),
                radius_m=.03, width_m=.025, vertex_count=len(local),
                reference_to_owner_transform=matrix.tolist(), session_layer_identifier=layer.identifier,
                note='Only missing front-left caster collider added from existing reference. No mass, inertia, joint, drive, friction or visual geometry changed; no USD save.')


def _prepare_straight_drive(stage, paths):
    """Stopped-only, reversible binding normalization and solver candidate.

    Explicit names are in DifferentialController's original +X frame. Encoder
    logical-front swapping remains independent. No gains/torque/friction edit.
    """
    from pxr import Usd, UsdPhysics, PhysxSchema, Sdf, Vt
    base = stage.GetPrimAtPath(paths['base'])
    parent = base.GetParent()
    roots = [p for p in Usd.PrimRange(parent) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    if len(roots) != 1:
        raise RuntimeError('Cannot apply drive fix: expected one AGV ArticulationRoot, got '+str([str(p.GetPath()) for p in roots]))
    art_root = roots[0]
    joints = {}
    for name in ('left_wheel_joint', 'right_wheel_joint'):
        found = [p for p in Usd.PrimRange(parent) if p.IsA(UsdPhysics.RevoluteJoint) and p.GetName() == name]
        if len(found) != 1:
            raise RuntimeError('Cannot bind drive wheel uniquely: '+name)
        joint = UsdPhysics.Joint(found[0])
        ends = list(joint.GetBody0Rel().GetTargets())+list(joint.GetBody1Rel().GetTargets())
        if base.GetPath() not in ends:
            raise RuntimeError('Drive joint does not connect expected base: '+str(found[0].GetPath()))
        joints[name] = str(found[0].GetPath())
    diff = stage.GetPrimAtPath(paths['controller'])
    graph = diff.GetParent()
    # Locate by the actual incoming velocity connection, not a guessed node name.
    candidates = []
    for p in Usd.PrimRange(graph):
        attr = p.GetAttribute('inputs:velocityCommand')
        if attr.IsValid() and Sdf.Path(paths['controller']+'.outputs:velocityCommand') in attr.GetConnections():
            if p.GetAttribute('inputs:jointNames').IsValid() and p.GetAttribute('inputs:robotPath').IsValid():
                candidates.append(p)
    if len(candidates) != 1:
        raise RuntimeError('Expected one articulation controller connected to this DifferentialController; got '+str([str(p.GetPath()) for p in candidates]))
    node = candidates[0]
    expected = ['left_wheel_joint', 'right_wheel_joint']
    specs = [('inputs:jointNames', Vt.TokenArray(expected)),
             ('inputs:jointIndices', Vt.IntArray([])),
             ('inputs:positionCommand', Vt.DoubleArray([])),
             ('inputs:effortCommand', Vt.DoubleArray([])),
             ('inputs:robotPath', str(art_root.GetPath()))]
    before = {}
    for name, _ in specs:
        attr = node.GetAttribute(name)
        if not attr.IsValid():
            raise RuntimeError('Missing articulation input: '+str(node.GetPath())+'.'+name)
        before[name] = dict(value=str(attr.Get()), connections=[str(p) for p in attr.GetConnections()])
    velocity = node.GetAttribute('inputs:velocityCommand')
    if list(velocity.GetConnections()) != [Sdf.Path(paths['controller']+'.outputs:velocityCommand')]:
        raise RuntimeError('Multiple velocity writers feed articulation controller; inspect graph before applying fix')
    api = PhysxSchema.PhysxArticulationAPI(art_root)
    old_velocity_iterations = api.GetSolverVelocityIterationCountAttr().Get() if api else None
    solver_types = [PhysxSchema.PhysxSceneAPI(p).GetSolverTypeAttr().Get()
                    for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
    if not solver_types or any(str(s) != 'TGS' for s in solver_types):
        raise RuntimeError('V10.5.4 solver rollback is for the recorded TGS scene; inspect changed solver type first')
    # TGS documentation recommends few (~1) velocity iterations. V10.5.3's
    # trial of 8 did not cure the drift; withdraw that trial, keep wheel binding.
    iterations = 1
    layer = Sdf.Layer.CreateAnonymous('amr_v1053_drive_binding_solver.usda')
    session = stage.GetSessionLayer()
    try:
        session.subLayerPaths = [layer.identifier]+list(session.subLayerPaths)
        with Usd.EditContext(stage, layer):
            for name, value in specs:
                attr = node.GetAttribute(name)
                attr.SetConnections([])  # block connected constants in weaker layers
                if not attr.Set(value):
                    raise RuntimeError('Failed authoring articulation input '+name)
            if not PhysxSchema.PhysxArticulationAPI.Apply(art_root).CreateSolverVelocityIterationCountAttr(iterations):
                raise RuntimeError('Failed authoring articulation solver iterations')
        if list(node.GetAttribute('inputs:jointNames').Get()) != expected:
            raise RuntimeError('Explicit wheel binding did not resolve')
        if int(PhysxSchema.PhysxArticulationAPI(art_root).GetSolverVelocityIterationCountAttr().Get()) != iterations:
            raise RuntimeError('Articulation velocity iteration candidate did not resolve')
    except BaseException:
        session.subLayerPaths = [p for p in session.subLayerPaths if p != layer.identifier]
        raise
    _NAV_DRIVE_SESSION_LAYERS.append(layer)
    return dict(revision='v10.5.4-explicit-binding-TGS-baseline',
                articulation_root=str(art_root.GetPath()), articulation_controller=str(node.GetPath()),
                joint_names_controller_order=expected, joint_paths=joints,
                previous_inputs=before, previous_velocity_iterations=old_velocity_iterations,
                velocity_iterations=iterations, session_layer_identifier=layer.identifier,
                note='Candidate only. No position iteration, mass, collision shape, friction, drive gain, force, dimensions or odometry sign edited. No USD save.')

def capture_runtime_clock(stage, timeline):
    """Capture before any setup call, including calls that might fail part-way."""
    import carb
    from pxr import UsdPhysics
    from isaacsim.core.rendering_manager import RenderingManager
    settings=carb.settings.get_settings()
    names=('/app/player/useFastMode','/app/player/useFixedTimeStepping',
           '/app/runLoops/main/rateLimitFrequency','/app/runLoops/main/rateLimitEnabled')
    attributes=[]
    for prim in stage.Traverse():
        if prim.IsA(UsdPhysics.Scene):
            attr=prim.GetAttribute('physxScene:timeStepsPerSecond')
            if attr.IsValid():
                attributes.append((str(attr.GetPath()),attr.Get(),attr.HasAuthoredValueOpinion()))
    return dict(loop_dt=float(RenderingManager.get_dt()),
                timeline_fps=float(timeline.get_target_framerate()),
                timeline_tcps=float(timeline.get_time_codes_per_seconds()),
                stage_metadata={key:(stage.GetMetadata(key),stage.HasAuthoredMetadata(key))
                                for key in ('timeCodesPerSecond','framesPerSecond')},
                settings={name:settings.get(name) for name in names},physics_attributes=attributes)


def restore_runtime_clock(stage,timeline,before):
    import carb
    from isaacsim.core.rendering_manager import RenderingManager
    errors=[]
    def attempt(name,fn):
        try:fn()
        except Exception as exc:errors.append(name+': '+str(exc))
    dt=before['loop_dt']
    if math.isfinite(dt) and dt>0:
        attempt('RenderingManager dt',lambda:RenderingManager.set_dt(dt))
    attempt('timeline fps',lambda:timeline.set_target_framerate(before['timeline_fps']))
    attempt('timeline time codes',lambda:timeline.set_time_codes_per_second(before['timeline_tcps']))
    for key,(value,authored) in before['stage_metadata'].items():
        attempt('stage '+key,lambda key=key,value=value,authored=authored:
                stage.SetMetadata(key,value) if authored else stage.ClearMetadata(key))
    for path,value,authored in before['physics_attributes']:
        attempt(path,lambda path=path,value=value,authored=authored:
                stage.GetAttributeAtPath(path).Set(value) if authored else stage.GetAttributeAtPath(path).Clear())
    settings=carb.settings.get_settings()
    for name,value in before['settings'].items():
        attempt(name,lambda name=name,value=value:settings.set(name,value) if value is not None else settings.destroy_item(name))
    return dict(errors=errors,note='Restores captured effective clock values; no USD file saved.')
def _single_identical_scene_trial(stage):
    from pxr import Usd,UsdPhysics,Sdf
    import re
    scenes=[p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
    before=sorted(str(p.GetPath()) for p in scenes)
    result=dict(active_scenes_before=before,session_layer_identifier=None,
                candidate_note='Guarded candidate configuration, not proof of which setting caused the clock mismatch.')
    if before!=['/PhysicsScene','/World/PhysicsScene']:
        result['single_scene_trial']='not_applicable'
        return result
    by_path={str(p.GetPath()):p for p in scenes}
    duplicate=by_path['/PhysicsScene'];keep=by_path['/World/PhysicsScene']
    def settings(prim):
        return {a.GetName():str(a.Get()) for a in prim.GetAttributes() if a.GetName().startswith(('physics:','physxScene:'))}
    reasons=[]
    if settings(duplicate)!=settings(keep):reasons.append('scene settings differ')
    if sorted(duplicate.GetAppliedSchemas())!=sorted(keep.GetAppliedSchemas()):reasons.append('applied schemas differ')
    if list(duplicate.GetAllChildren()):reasons.append('duplicate scene has descendants')
    target=Sdf.Path('/PhysicsScene')
    path_pattern=re.compile(r'(?<![A-Za-z0-9_/])/PhysicsScene(?=$|[^A-Za-z0-9_])')
    for prim in stage.Traverse():
        if prim==duplicate:continue
        for rel in prim.GetRelationships():
            if any(p.HasPrefix(target) for p in rel.GetTargets()):reasons.append('relationship '+str(rel.GetPath()))
        for attr in prim.GetAttributes():
            if any(p.HasPrefix(target) for p in attr.GetConnections()):reasons.append('connection '+str(attr.GetPath()))
            if path_pattern.search(str(attr.Get())):reasons.append('attribute '+str(attr.GetPath()))
    if reasons:
        result.update(single_scene_trial='skipped_guard_failed',guard_reasons=reasons[:30])
        return result
    layer=Sdf.Layer.CreateAnonymous('amr_single_physics_scene.usda')
    session=stage.GetSessionLayer()
    try:
        session.subLayerPaths=[layer.identifier]+list(session.subLayerPaths)
        with Usd.EditContext(stage,layer):stage.OverridePrim('/PhysicsScene').SetActive(False)
        active=sorted(str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.Scene))
        if active!=['/World/PhysicsScene']:raise RuntimeError('Unexpected active scene set after candidate override.')
    except BaseException:
        session.subLayerPaths=[p for p in session.subLayerPaths if p!=layer.identifier]
        raise
    _NAV_CLOCK_SESSION_LAYERS.append(layer)
    result.update(single_scene_trial='temporary_session_override',active_scenes_after=active,
                  session_layer_identifier=layer.identifier,session_layer_text=layer.ExportToString())
    return result

def _configure_navigation_clock(stage,cfg):
    import carb
    from isaacsim.core.simulation_manager import SimulationManager
    from isaacsim.core.rendering_manager import RenderingManager
    hz=float(cfg.get('physics_hz',60.0))
    if not math.isfinite(hz) or not 30<=hz<=240:raise RuntimeError('Invalid configured physics_hz')
    settings=carb.settings.get_settings()
    before={name:settings.get(name) for name in ('/app/player/useFastMode','/app/player/useFixedTimeStepping','/app/runLoops/main/rateLimitFrequency')}
    settings.set_bool('/app/player/useFastMode',False)
    tracked_getter=getattr(SimulationManager,'get_physics_scenes',None)
    tracked_before=[str(x) for x in tracked_getter()] if callable(tracked_getter) else []
    from pxr import Usd,UsdPhysics,Sdf
    paths_before=sorted(str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.Scene))
    if not tracked_before and '/PhysicsScene' not in paths_before:
        raise RuntimeError('Physics manager has no tracked scene and default /PhysicsScene is absent; refusing to create another scene.')
    if not paths_before or not math.isclose(hz,round(hz),abs_tol=1e-8):
        raise RuntimeError('Expected existing PhysicsScenes and an integer steps-per-second setting.')
    # Synchronize before auditing the duplicate, then isolate only an identical
    # unreferenced /PhysicsScene. Two active scenes showed a 2x clock discrepancy
    # even at equal rates. Live callback/delta validation remains mandatory.
    rates_before={}
    for path in paths_before:
        attr=stage.GetPrimAtPath(path).GetAttribute('physxScene:timeStepsPerSecond')
        if not attr.IsValid():
            raise RuntimeError('PhysicsScene lacks the recorded PhysX step-rate attribute: '+path)
        rates_before[path]=attr.Get()
    layer=Sdf.Layer.CreateAnonymous('amr_v10552_all_scene_clocks.usda')
    session=stage.GetSessionLayer()
    owned_start=len(_NAV_CLOCK_SESSION_LAYERS)
    scene_trial=None
    try:
        session.subLayerPaths=[layer.identifier]+list(session.subLayerPaths)
        with Usd.EditContext(stage,layer):
            # Set all scene rates before the manager reads them, then verify
            # again after manager/render setup, which may rewrite settings.
            for path in paths_before:
                if not stage.GetPrimAtPath(path).GetAttribute('physxScene:timeStepsPerSecond').Set(int(round(hz))):
                    raise RuntimeError('Could not author scene step rate: '+path)
            SimulationManager.setup_simulation(dt=1.0/hz)
            RenderingManager.set_dt(1.0/hz)
            for path in paths_before:
                if not stage.GetPrimAtPath(path).GetAttribute('physxScene:timeStepsPerSecond').Set(int(round(hz))):
                    raise RuntimeError('Could not retain scene step rate: '+path)
        paths_after=sorted(str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.Scene))
        if paths_before!=paths_after:
            raise RuntimeError('Clock setup changed the PhysicsScene path set. Stop and inspect.')
        rates_after={path:stage.GetPrimAtPath(path).GetAttribute('physxScene:timeStepsPerSecond').Get()
                     for path in paths_after}
        if any(value!=int(round(hz)) for value in rates_after.values()):
            raise RuntimeError('PhysicsScene rate readback differs: '+str(rates_after))
        scene_trial=_single_identical_scene_trial(stage)
        active_paths=sorted(str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.Scene))
        if len(active_paths)!=1:
            raise RuntimeError('Cannot establish one physics clock without guessing ownership: '+str(scene_trial))
    except BaseException:
        failed={layer.identifier}|{x.identifier for x in _NAV_CLOCK_SESSION_LAYERS[owned_start:]}
        session.subLayerPaths=[p for p in session.subLayerPaths if p not in failed]
        del _NAV_CLOCK_SESSION_LAYERS[owned_start:]
        raise
    _NAV_CLOCK_SESSION_LAYERS.append(layer)
    scene_trial.update(scene_rates_before=rates_before,scene_rates_after=rates_after,
                       active_scenes_verified=active_paths)
    print('[AMR V10.5.6] One active physics scene verified: '+str(active_paths)+
          '; identical unreferenced duplicate is session-disabled only. Live clock audit follows Play.')
    return dict(revision=NAV_PATCH_REVISION,hz=hz,before_settings=before,scene_trial=scene_trial,
                manager_scene_api_available=callable(tracked_getter),manager_tracked_before=tracked_before,
                manager_tracked_after=[str(x) for x in tracked_getter()] if callable(tracked_getter) else [],
                physics_scenes=[str(p.GetPath()) for p in stage.Traverse() if p.GetTypeName()=='PhysicsScene'],
                note='Guarded session isolation of an identical unreferenced duplicate, no scene deletion or USD save. Live callback dt and manager-clock delta must agree before motion; no timestamp scaling.')

def _prepare_drive_validation(stage,cfg):
    from pxr import Usd,UsdPhysics,UsdShade,PhysxSchema,Sdf
    base=stage.GetPrimAtPath(cfg['paths']['base'])
    root=base.GetParent()
    names={'left_wheel_joint','right_wheel_joint'}
    joints=[p for p in Usd.PrimRange(root) if p.IsA(UsdPhysics.RevoluteJoint) and p.GetName() in names]
    if len(joints)!=2:raise RuntimeError('Expected the two recorded drive joints for V4 validation.')
    force=float(cfg.get('drive_validation_max_force_nm',10.))
    if not math.isfinite(force) or not 5<=force<=20:raise RuntimeError('V4 candidate drive force outside allowed range.')
    material_path=cfg.get('drive_grip_material','/World/PhysicsMaterials/grip')
    material=UsdShade.Material.Get(stage,material_path)
    if not material or not material.GetPrim().HasAPI(UsdPhysics.MaterialAPI):raise RuntimeError('Recorded grip physics material missing.')
    mat=UsdPhysics.MaterialAPI(material.GetPrim())
    static=float(mat.GetStaticFrictionAttr().Get());dynamic=float(mat.GetDynamicFrictionAttr().Get())
    if not 0<=dynamic<=static<=2:raise RuntimeError('Inspect grip coefficients before candidate validation.')
    colliders=[];before=[]
    lidar=stage.GetPrimAtPath(cfg['paths']['lidar'])
    near_attr=lidar.GetAttribute('omni:sensor:Core:nearRangeM')
    near=float(cfg.get('lidar_near_range_m',.10))
    if not near_attr.IsValid() or not .05<=near<=.20:raise RuntimeError('Invalid short-range lidar setting for this AMR.')
    previous_near=float(near_attr.Get())
    for prim in joints:
        joint=UsdPhysics.Joint(prim)
        targets=list(joint.GetBody0Rel().GetTargets())+list(joint.GetBody1Rel().GetTargets())
        wheels=[stage.GetPrimAtPath(p) for p in targets if p!=base.GetPath()]
        if len(wheels)!=1 or base.GetPath() not in targets:raise RuntimeError('Unexpected drive joint topology.')
        drive=UsdPhysics.DriveAPI(prim,'angular')
        if not drive or float(drive.GetStiffnessAttr().Get())!=0:raise RuntimeError('Expected velocity-mode angular drive.')
        before.append(dict(path=str(prim.GetPath()),max_force=float(drive.GetMaxForceAttr().Get())))
        colliders.extend(p for p in Usd.PrimRange(wheels[0]) if p.HasAPI(UsdPhysics.CollisionAPI))
    if len(colliders)!=2:raise RuntimeError('Expected exactly two drive tire collision shapes.')
    layer=Sdf.Layer.CreateAnonymous('amr_drive_validation_v4.usda')
    session=stage.GetSessionLayer()
    reports=[];report_errors={}
    try:
        session.subLayerPaths=[layer.identifier]+list(session.subLayerPaths)
        with Usd.EditContext(stage,layer):
            for prim in joints:UsdPhysics.DriveAPI(prim,'angular').GetMaxForceAttr().Set(force)
            for prim in colliders:
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(material,UsdShade.Tokens.strongerThanDescendants,'physics')
            near_attr.Set(near)
            for prim in Usd.PrimRange(root):
                if prim.HasAPI(UsdPhysics.RigidBodyAPI) and ('wheel' in prim.GetName().lower() or 'caster' in prim.GetName().lower()):
                    try:
                        report=PhysxSchema.PhysxContactReportAPI.Apply(prim)
                        create=getattr(report,'CreateThresholdAttr',None) or getattr(report,'CreatePhysxContactReportThresholdAttr',None)
                        if not callable(create):raise RuntimeError('Contact threshold authoring API unavailable')
                        create(0.)
                        reports.append(str(prim.GetPath()))
                    except Exception as exc:report_errors[str(prim.GetPath())]=str(exc)
        for prim in colliders:
            resolved,_=UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial('physics')
            if not resolved or str(resolved.GetPath())!=material_path:raise RuntimeError('Grip material did not resolve as requested.')
        if any(abs(float(UsdPhysics.DriveAPI(p,'angular').GetMaxForceAttr().Get())-force)>1e-5 for p in joints):raise RuntimeError('Drive candidate limit did not resolve.')
        if abs(float(near_attr.Get())-near)>1e-5:raise RuntimeError('Lidar short-range setting did not resolve.')
    except BaseException:
        session.subLayerPaths=[p for p in session.subLayerPaths if p!=layer.identifier]
        raise
    _NAV_DRIVE_SESSION_LAYERS.append(layer)
    return dict(revision=DRIVE_PATCH_REVISION,candidate=True,original_drive_limits=before,
        candidate_max_force_nm=force,material=material_path,static_friction=static,dynamic_friction=dynamic,
        tire_colliders=[str(p.GetPath()) for p in colliders],contact_report_bodies=reports,
        contact_report_setup_errors=report_errors,lidar_near_range_before_m=previous_near,lidar_near_range_after_m=near,
        session_layer_identifier=layer.identifier,
        note='Candidate model settings, not real hardware calibration. No mass, dimensions, caster drive or floor material changed; no USD saved.')
