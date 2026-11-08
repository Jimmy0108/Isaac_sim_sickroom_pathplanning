"""V66 startup validation. No inherited V4 torque/friction/geometry/clock patch."""
import math

_NAV_CLOCK_SESSION_LAYERS=[]
_NAV_DRIVE_SESSION_LAYERS=[]


def capture_runtime_clock(stage,timeline):
    from pxr import UsdPhysics
    return dict(policy='READ_ONLY; no runtime clock setting is changed by V66',
                timeline_fps=float(timeline.get_target_framerate()),
                scenes=[str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)])


def restore_runtime_clock(stage,timeline,before):
    return dict(errors=[],action='NONE: V66 did not change simulation/render dt or timeline metadata')


def _configure_navigation_clock(stage,cfg):
    from pxr import PhysxSchema,UsdPhysics
    scenes=[p for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]
    if len(scenes)!=1:
        raise RuntimeError('Expected one active PhysicsScene; audit and resolve duplicates before RUN: '+str([str(p.GetPath()) for p in scenes]))
    hz=float(PhysxSchema.PhysxSceneAPI(scenes[0]).GetTimeStepsPerSecondAttr().Get())
    if not math.isfinite(hz) or not 30<=hz<=1000:raise RuntimeError('Invalid scene physics frequency: '+str(hz))
    expected=cfg.get('physics_hz')
    if expected is not None and not math.isclose(float(expected),hz,abs_tol=1e-5):
        raise RuntimeError('Configured physics_hz differs from scene; no automatic rewrite: '+str((expected,hz)))
    cfg['physics_hz']=hz
    return dict(revision='V66-existing-scene-clock',hz=hz,physics_scenes=[str(scenes[0].GetPath())],
                policy='READ_ONLY scene step rate; no SimulationManager.setup_simulation or RenderingManager.set_dt. Live callback-vs-manager dt validation follows Play.')


def _prepare_drive_validation(stage,cfg,raise_on_error=True):
    from V66_VehicleAudit import collect_vehicle
    report=collect_vehicle(stage,**{k:cfg['paths'][k] for k in ('base','controller','lidar')})
    for key in ('wheel_radius','track','length','width','height'):
        if not math.isclose(float(cfg['body'][key]),float(report['expected'][key]),abs_tol=1e-6):
            report['blocking_checks'].append('navigation_config:'+key)
    if report['blocking_checks'] and raise_on_error:
        raise RuntimeError('V66 vehicle contract failed: '+', '.join(report['blocking_checks'])+'. Run V66_VehicleAudit.py for full report.')
    return report


def _repair_missing_caster(stage,paths):
    # Kept as an adapter name to minimize changes in the proven state machine.
    # Unlike the old function this performs no repair or collider creation.
    from pxr import Usd,UsdPhysics
    root=stage.GetPrimAtPath(paths['base']).GetParent()
    names=('front_left','front_right','rear_left','rear_right')
    counts={}
    for name in names:
        p=stage.GetPrimAtPath(str(root.GetPath())+'/'+name+'_caster_wheel_link')
        shapes=[q for q in Usd.PrimRange(p) if q.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(q).GetCollisionEnabledAttr().Get() is not False] if p else []
        counts[name]=len(shapes)
    if any(n!=1 for n in counts.values()):raise RuntimeError('V66 caster collider inventory mismatch: '+str(counts))
    return dict(action='validated_existing_V66_four_casters',before=counts,stage_edits=False)


def _prepare_straight_drive(stage,paths):
    from pxr import Usd
    import omni.graph.core as og
    root=stage.GetPrimAtPath(paths['controller']).GetParent()
    candidates=[p for p in Usd.PrimRange(root) if p.GetAttribute('inputs:robotPath')]
    if len(candidates)!=1:raise RuntimeError('Expected one articulation controller')
    p=candidates[0]
    expected_source=paths['controller']+'.outputs:velocityCommand'
    connections=[str(x) for x in p.GetAttribute('inputs:velocityCommand').GetConnections()]
    if connections!=[expected_source]:raise RuntimeError('Unexpected wheel velocity writer: '+str(connections))
    # Runtime evaluated joint-name values are checked after physics has started.
    base=stage.GetPrimAtPath(paths['base'])
    return dict(articulation_controller=str(p.GetPath()),articulation_root=paths['base'],
                joint_names_controller_order=['left_wheel_joint','right_wheel_joint'],
                velocity_iterations=base.GetAttribute('physxArticulation:solverVelocityIterationCount').Get(),
                policy='READ_ONLY. No wheel binding, solver, gains, friction or force rewrite.')
