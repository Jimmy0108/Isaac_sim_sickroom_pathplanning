"""Isaac Script Editor: STOP -> RUN this file -> RUN navigation -> PLAY.

Candidate A/B contact-material change, NOT a validated repair or hardware value.
Only four caster collider material bindings are changed in an anonymous layer.
Set APPLY=False and RUN while stopped to remove this candidate layer.
No motion is commanded. No geometry, wheel dimensions, drive or clock is edited.
"""

def _install_v66_caster_candidate(apply=True):
    import builtins
    import json
    from datetime import datetime
    from pathlib import Path
    import omni.usd
    import omni.timeline
    from pxr import Sdf, Usd, UsdPhysics, UsdShade

    if not omni.timeline.get_timeline_interface().is_stopped():
        raise RuntimeError('Press STOP before applying or removing the contact candidate.')
    stage=omni.usd.get_context().get_stage()
    if stage is None:raise RuntimeError('No open stage.')
    root='/World/AGV_configured_v4/TZAGV_B02_vendor_v4/TZAGV_B02_root'
    names=('front_left','front_right','rear_left','rear_right')
    paths=[root+'/'+n+'_caster_wheel_link/wheel_contact_collider' for n in names]
    def material_of(path):
        prim=stage.GetPrimAtPath(path)
        mat,_=UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial(materialPurpose='physics')
        if not mat:return dict(material=None)
        return dict(material=str(mat.GetPath()),attributes={a.GetName():str(a.Get()) for a in mat.GetPrim().GetAttributes()
            if a.GetName().startswith(('physics:','physxMaterial:'))})
    # Check before removing an existing candidate or authoring anything.
    for path in paths:
        prim=stage.GetPrimAtPath(path)
        if not prim or not prim.HasAPI(UsdPhysics.CollisionAPI) or prim.GetTypeName()!='Cylinder':
            raise RuntimeError('Expected V66 caster Cylinder collider: '+path)
    session=stage.GetSessionLayer()
    tag='V66_caster_contact_candidate_v1'
    prior=[s for s in session.subLayerPaths if (Sdf.Layer.Find(s) and Sdf.Layer.Find(s).customLayerData.get('v66_candidate')==tag)]
    for sid in prior:session.subLayerPaths.remove(sid)
    if not apply:
        setattr(builtins,'_v66_caster_contact_candidate_layer',None)
        print('[V66 contact candidate] Removed temporary layer; original bindings restored. No stage save required.')
        return
    before={p:material_of(p) for p in paths}
    drive_before={root+'/'+n+'_drive_wheel_link/wheel_contact_collider':material_of(root+'/'+n+'_drive_wheel_link/wheel_contact_collider') for n in ('left','right')}
    layer=Sdf.Layer.CreateAnonymous('V66_CasterContactCandidate.usda')
    layer.customLayerData={'v66_candidate':tag}
    session.subLayerPaths.insert(0,layer.identifier)
    try:
        with Usd.EditContext(stage,layer):
            mat=UsdShade.Material.Define(stage,root+'/PhysicsMaterials/CasterContactCandidate')
            api=UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
            api.CreateStaticFrictionAttr(.25);api.CreateDynamicFrictionAttr(.20);api.CreateRestitutionAttr(0.)
            # Intentionally retain the scene's existing combine policy. Verify
            # resulting motion; these authored coefficients are not the pair's
            # guaranteed effective friction when combined with the floor.
            for path in paths:
                UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(path)).Bind(
                    mat,UsdShade.Tokens.strongerThanDescendants,'physics')
        after={p:material_of(p) for p in paths}
        expected=str(mat.GetPath())
        if any(a.get('material')!=expected for a in after.values()):
            raise RuntimeError('Candidate material did not resolve on all four colliders.')
        if any(material_of(p)!=v for p,v in drive_before.items()):
            raise RuntimeError('Drive wheel material unexpectedly changed.')
        folder=Path('/isaac-sim/.local/share/ov/data/AMR_Project/1019_Scene/Collected_sickroom_with_hand_colaTest/new_car/python_nav/V66_Python_Navigation/validation_outputs')
        folder.mkdir(parents=True,exist_ok=True)
        out=folder/('caster_contact_candidate_'+datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'.json')
        out.write_text(json.dumps(dict(candidate_only=True,isaac_motion_verified=False,
            change='Four caster collider material bindings only',layer=layer.identifier,
            before=before,after=after,drive_materials_unchanged=drive_before,
            authored_static_friction=.25,authored_dynamic_friction=.20,
            floor_combination='Unchanged; authored values are not guaranteed pair-effective values',
            rollback='STOP; set APPLY=False in this script and RUN. Do not flatten/save this candidate as a validated asset.'),indent=2),encoding='utf-8')
        setattr(builtins,'_v66_caster_contact_candidate_layer',layer)
        print('[V66 contact candidate] APPLIED: caster static=.25 dynamic=.20; drive wheels untouched.')
        print('[V66 contact candidate] Candidate only, not a confirmed repair. Report: '+str(out))
    except Exception:
        if layer.identifier in session.subLayerPaths:session.subLayerPaths.remove(layer.identifier)
        raise

APPLY=True
_install_v66_caster_candidate(APPLY)
