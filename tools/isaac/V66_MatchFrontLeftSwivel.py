"""Isaac Script Editor: STOP -> RUN once. Match one swivel to the front-right.
No motion, wheel geometry, friction, drive-wheel gain or clock changes.
"""

def _match_front_left_swivel():
    import json,time,math
    from pathlib import Path
    import omni.usd,omni.timeline
    from pxr import Usd,UsdPhysics

    PROJECT=Path('/isaac-sim/.local/share/ov/data/AMR_Project/1019_Scene/Collected_sickroom_with_hand_colaTest/new_car/python_nav/V66_Python_Navigation')
    ROOT='/World/AGV_configured_v4/TZAGV_B02_vendor_v4/TZAGV_B02_root'
    if not omni.timeline.get_timeline_interface().is_stopped():
        raise RuntimeError('Press STOP, close running navigation/contact sessions, then RUN this script.')
    stage=omni.usd.get_context().get_stage()
    if stage is None:raise RuntimeError('Open the V66 scene first.')
    target=stage.GetPrimAtPath(ROOT+'/front_left_caster_swivel_link/front_left_caster_swivel_joint')
    reference=stage.GetPrimAtPath(ROOT+'/front_right_caster_swivel_link/front_right_caster_swivel_joint')
    if not target or not reference:raise RuntimeError('Expected front-left and front-right swivel joints were not found.')
    if target.IsInstanceProxy():raise RuntimeError('Target is an instance proxy; inspect the asset before editing.')
    for p in (target,reference):
        if not p.IsA(UsdPhysics.RevoluteJoint) or p.GetAttribute('physics:axis').Get()!='Z':
            raise RuntimeError('Expected a Z-axis revolute swivel joint: '+str(p.GetPath()))
    ref=UsdPhysics.DriveAPI(reference,'angular')
    if not ref:raise RuntimeError('Front-right reference has no angular drive; no values were guessed.')
    values=dict(damping=ref.GetDampingAttr().Get(),stiffness=ref.GetStiffnessAttr().Get(),
        targetVelocity=ref.GetTargetVelocityAttr().Get(),targetPosition=ref.GetTargetPositionAttr().Get(),
        maxForce=ref.GetMaxForceAttr().Get(),type=ref.GetTypeAttr().Get())
    if values['stiffness']!=0 or values['targetVelocity']!=0 or values['type']!='force':
        raise RuntimeError('Reference is not the recorded passive zero-target damper; stopped without editing.')
    if not math.isclose(float(values['damping']),.00034906598739326,rel_tol=.01) or not math.isclose(float(values['maxForce']),.1,rel_tol=.01):
        raise RuntimeError('Reference damping/force differs from the measured baseline; no automatic retuning: '+str(values))
    before={a.GetName():str(a.Get()) for a in target.GetAttributes() if a.GetName().startswith('drive:')}
    folder=PROJECT/'validation_outputs';folder.mkdir(parents=True,exist_ok=True)
    backup=folder/('front_left_swivel_change_'+str(time.time_ns())+'.json')
    report=dict(stage=stage.GetRootLayer().identifier,target=str(target.GetPath()),reference=str(reference.GetPath()),
                before_attributes=before,before_applied_schemas=list(target.GetAppliedSchemas()),
                copied_reference=values,usd_file_saved=False,motion_tested=False,
                scope='Only front-left swivel angular damper. If absent, apply DriveAPI with the reference zero-target settings.')
    backup.write_text(json.dumps(report,indent=2),encoding='utf-8')
    with Usd.EditContext(stage,stage.GetRootLayer()):
        api=UsdPhysics.DriveAPI.Apply(target,'angular')
        api.CreateDampingAttr().Set(values['damping'])
        api.CreateStiffnessAttr().Set(values['stiffness'])
        api.CreateTargetVelocityAttr().Set(values['targetVelocity'])
        api.CreateTargetPositionAttr().Set(values['targetPosition'])
        api.CreateMaxForceAttr().Set(values['maxForce'])
        api.CreateTypeAttr().Set(values['type'])
    actual=UsdPhysics.DriveAPI(target,'angular')
    if not math.isclose(float(actual.GetDampingAttr().Get()),float(values['damping']),rel_tol=1e-6):
        raise RuntimeError('Readback differs, possibly a stronger layer override. Backup: '+str(backup))
    print('[V66 swivel] Front-left matched to front-right:',values)
    print('[V66 swivel] Changed the open scene only. Ctrl+S to retain the scene override; source vehicle USD not saved by this script.')
    print('[V66 swivel] Backup:',backup)
    print('[V66 swivel] No motion test performed. Re-run navigation/contact entry after this change.')


_match_front_left_swivel()
