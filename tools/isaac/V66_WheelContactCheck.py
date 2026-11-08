"""Diagnostic observer: STOP -> RUN this -> RUN navigation entry -> PLAY.
No independent motor writer. Temporary contact-report APIs only; no tuning.
"""

def _start_wheel_contact_check(editor_scope):
    import asyncio, json, math, time, zipfile, types, sys
    from pathlib import Path
    from collections import deque, Counter
    import numpy as np
    import omni.usd, omni.timeline, omni.kit.app, omni.ui as ui, omni.physx
    from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, PhysicsSchemaTools, Sdf
    from isaacsim.core.simulation_manager import SimulationManager, SimulationEvent

    PROJECT=Path('/isaac-sim/.local/share/ov/data/AMR_Project/1019_Scene/Collected_sickroom_with_hand_colaTest/new_car/python_nav/V66_Python_Navigation')
    BASE='/World/AGV_configured_v4/TZAGV_B02_vendor_v4/TZAGV_B02_root/base_link'
    ROOT=BASE.rsplit('/',1)[0]
    state=globals()
    old=state.get('_v66_contact_check')
    if old is not None and not old.done:
        raise RuntimeError('Previous contact check is running; stop/export it first.')

    def plain(x):
        if isinstance(x,dict):return {str(k):plain(v) for k,v in x.items()}
        if x is None or isinstance(x,(str,bool,int)):return x
        if isinstance(x,(float,np.floating)):return float(x) if math.isfinite(float(x)) else str(x)
        try:return [plain(v) for v in x]
        except TypeError:return str(x)

    def host(x):
        for name in ('detach','cpu','numpy'):
            f=getattr(x,name,None)
            if callable(f):x=f()
        return np.asarray(x,dtype=float)

    def path_of(x):
        try:return str(PhysicsSchemaTools.intToSdfPath(x))
        except Exception:
            try:return str(PhysicsSchemaTools.decodeSdfPath(x))
            except Exception:return '<UNDECODED:'+str(x)+'>'

    def own(path):return path==ROOT or path.startswith(ROOT+'/')

    class Monitor:
        def __init__(self):
            self.done=False;self.finish=False;self.error=None;self.attached=False
            self.contacts=deque(maxlen=40000);self.rows=[];self.counts=Counter();self.dropped=0
            self.contact_errors=[];self.contact_callback_count=0;self.step_callback=None;self.subscription=None
            self.layer=None;self.body_view=None;self.art=None;self.last_sample=-math.inf
            self.stage=omni.usd.get_context().get_stage();self.timeline=omni.timeline.get_timeline_interface()
            self.output=PROJECT/'validation_outputs'/('wheel_contact_'+str(time.time_ns()))
            self.output.mkdir(parents=True)
            self.meta=dict(revision='V66-contact-direction-1.0',motor_writer='Existing v66_app only',
                policy='ContactReportAPI threshold=0 in temporary layer; no gains/friction/mass/geometry/dt tuning',
                developer_isaac_runtime_verified=False,automatic_abort_tilt_deg=5.,record_interval_sim_s=.02,
                limitations=['No contact events is NOT proof of no contact.',
                    'Velocity at nominal wheel bottom is a flat-ground kinematic indicator, not a measured contact-patch slip ratio.',
                    'Caster alignment is measured from wheel centre relative to swivel origin; small speed makes it undefined.',
                    'Per-pair impulses are reported values, not isolated motor torque.'])
            self.window=ui.Window('V66 | Wheel direction and contact check',width=780,height=400)
            with self.window.frame:
                with ui.VStack(spacing=5):
                    ui.Label('Run this while STOPPED, then run AMR_V66_Navigation.py and press Play.',height=40,word_wrap=True)
                    ui.Label('Observer records wheel direction / caster trail / collision pairs. Stops simulation at tilt >5 deg.',height=40,word_wrap=True)
                    with ui.HStack(height=32):
                        for speed in (.2,.4,.6,.9):ui.Button(str(speed)+' m/s',clicked_fn=lambda s=speed:self.limit(s))
                    with ui.HStack(height=32):
                        for label,v,w in [('Forward',1,0),('Reverse',-1,0),('Left',0,1),('Right',0,-1)]:
                            ui.Button(label,clicked_fn=lambda v=v,w=w:self.drive(v,w))
                    ui.Button('STOP vehicle (keep recording)',clicked_fn=self.stop,height=32)
                    self.label=ui.Label('Preparing diagnostics; motors are not commanded by this observer.',word_wrap=True,height=140)
                    ui.Button('STOP simulation / export both sessions',clicked_fn=lambda:setattr(self,'finish',True),height=36)
            self.speed=.2;self.limited_app=None

        def live_app(self):return state.get('_v66_application',editor_scope.get('_v66_application'))

        def stop(self):
            a=self.live_app()
            if a is not None and getattr(a,'ready',False):a.stop()

        def limit(self,speed):
            self.stop();self.speed=speed
            a=self.live_app()
            if a is not None and getattr(a,'ready',False):
                a.teleop_v=speed;a.teleop_w=.3
                self.limited_app=a
            self.label.text='Selected manual v='+str(speed)+' m/s, spin=0.3 rad/s. STOP before changing direction.'

        def drive(self,v,w):
            a=self.live_app()
            if a is None or not getattr(a,'ready',False):self.label.text='Wait for navigation Manual ready.';return
            if self.meta.get('contact_reporting_ok') is not True:
                self.label.text='No verified contact reports yet. Do not drive; export diagnostics.';return
            a.teleop_v=self.speed;a.teleop_w=.3
            a.set_manual(v,w)

        def on_contacts(self,headers,data):
            self.contact_callback_count+=1
            try:
                now=float(SimulationManager.get_simulation_time())
                for h in headers:
                    actors=[path_of(h.actor0),path_of(h.actor1)]
                    if not any(own(x) for x in actors):continue
                    colliders=[path_of(h.collider0),path_of(h.collider1)]
                    item=dict(physics_s=now,event_type=str(h.type),actors=actors,colliders=colliders,
                              contact_count=int(h.num_contact_data),points=[],sum_impulse_magnitudes=0.)
                    offset=int(h.contact_data_offset);count=int(h.num_contact_data)
                    for k in range(offset,offset+count):
                        p=data[k];imp=np.asarray(p.impulse,dtype=float)
                        item['sum_impulse_magnitudes']+=float(np.linalg.norm(imp))
                        if len(item['points'])<8:
                            item['points'].append(dict(position=plain(p.position),normal=plain(p.normal),
                                impulse=plain(p.impulse),separation=plain(p.separation),
                                material0=path_of(p.material0),material1=path_of(p.material1),
                                face0=int(p.face_index0),face1=int(p.face_index1)))
                    key=' <-> '.join(sorted(colliders));self.counts[key]+=1
                    if len(self.contacts)==self.contacts.maxlen:self.dropped+=1
                    self.contacts.append(item)
                    if count>0:self.meta['contact_reporting_ok']=True
            except Exception as exc:
                if len(self.contact_errors)<20:self.contact_errors.append(type(exc).__name__+': '+str(exc))

        def on_step(self,*args):
            if self.finish or self.done:return
            try:
                now=float(SimulationManager.get_simulation_time())
                if now-self.last_sample<.02-1e-8:return
                self.last_sample=now
                pose=host(self.body_view.get_transforms());vel=host(self.body_view.get_velocities())
                q=host(self.art.get_dof_positions())[0];qd=host(self.art.get_dof_velocities())[0]
                targets=host(self.art.get_dof_velocity_targets())[0]
                names=self.meta['body_order'];byname={p.rsplit('/',1)[-1]:i for i,p in enumerate(names)}
                bi=byname['base_link'];b=pose[bi];x,y,z,w=b[3:]/np.linalg.norm(b[3:])
                tilt=math.degrees(math.acos(np.clip(1-2*(x*x+y*y),-1,1)))
                app=self.live_app()
                record=dict(physics_s=now,wall_monotonic_s=time.monotonic(),callback_dt_s=plain(args[0] if args else None),
                    body_pose_world=pose,body_velocity_world=vel,joint_positions_rad=q,
                    joint_velocities_rad_s=qd,joint_targets_rad_s=targets,tilt_deg=tilt,
                    command_v_w=plain(getattr(app,'last_control',None)),wheels=[])
                for name,i in byname.items():
                    if not name.endswith('wheel_link'):continue
                    drive='drive' in name;r=.08 if drive else .0315
                    x,y,z,w=pose[i,3:]/np.linalg.norm(pose[i,3:])
                    axis=np.array([2*(x*y-z*w),1-2*(x*x+z*z),2*(y*z+x*w)])
                    rolling=np.cross(axis,[0.,0.,1.]);norm=np.linalg.norm(rolling)
                    if norm<1e-8:continue
                    rolling/=norm;lateral=np.cross([0.,0.,1.],rolling)
                    velocity=vel[i,:3];omega=vel[i,3:]
                    nominal=velocity+np.cross(omega,[0.,0.,-r])
                    wheel=dict(name=name,axis_world=axis,rolling_direction_world=rolling,
                        centre_velocity_roll_m_s=float(velocity@rolling),centre_velocity_lateral_m_s=float(velocity@lateral),
                        nominal_bottom_velocity_world=nominal,
                        nominal_bottom_roll_residual_m_s=float(nominal@rolling),
                        nominal_bottom_lateral_m_s=float(nominal@lateral))
                    if not drive:
                        swivel=name.replace('_wheel_link','_swivel_link')
                        if swivel in byname:
                            offset=pose[i,:2]-pose[byname[swivel],:2];speed=np.linalg.norm(velocity[:2])
                            wheel['swivel_to_wheel_xy_m']=offset
                            wheel['trail_dot_motion_m']=float(offset@velocity[:2]/speed) if speed>.03 else None
                            wheel['trail_interpretation']='negative=wheel trails pivot; positive=wheel leads pivot; only meaningful during planar motion above 0.03 m/s'
                    record['wheels'].append(wheel)
                self.rows.append(plain(record))
                if tilt>5.:
                    self.meta['abort_reason']='Measured tilt exceeded 5 degrees; diagnostic abort, not successful physical stop'
                    self.finish=True
                    self.timeline.stop()
            except Exception as exc:
                self.error=type(exc).__name__+': '+str(exc);self.finish=True

        async def run(self):
            kit=omni.kit.app.get_app()
            try:
                if self.stage is None or not self.timeline.is_stopped():raise RuntimeError('Press STOP before RUN this contact observer.')
                base=self.stage.GetPrimAtPath(BASE)
                if not base:raise RuntimeError('Configured V66 base not found')
                # Reuse the independent audit without executing its launcher.
                p=PROJECT/'V66_VehicleAudit.py'
                m=types.ModuleType('V66_VehicleAudit');m.__file__=str(p)
                exec(compile(p.read_text(encoding='utf-8-sig'),str(p),'exec'),m.__dict__)
                snapshot=m.collect_vehicle(self.stage,evaluated=True)
                (self.output/'vehicle_geometry.json').write_text(json.dumps(snapshot,indent=2),encoding='utf-8')
                bbox=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render','proxy','guide'])
                inventory=[]
                for prim in self.stage.Traverse():
                    if not prim.HasAPI(UsdPhysics.CollisionAPI):continue
                    if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:continue
                    bounds=bbox.ComputeWorldBound(prim).ComputeAlignedRange()
                    attrs={a.GetName():plain(a.Get()) for a in prim.GetAttributes() if a.GetName().startswith(('physics:','physxCollision:','physxMeshCollision:','xformOp'))}
                    inventory.append(dict(path=str(prim.GetPath()),type=prim.GetTypeName(),world_aabb=[plain(bounds.GetMin()),plain(bounds.GetMax())],attributes=attrs,
                        note='AABB is a broad-phase bound, not proof that a mesh is solid or overlapping.'))
                (self.output/'collider_inventory.json').write_text(json.dumps(inventory,indent=2),encoding='utf-8')
                self.layer=Sdf.Layer.CreateAnonymous('V66_contact_reports_only.usda')
                self.stage.GetSessionLayer().subLayerPaths=[self.layer.identifier]+list(self.stage.GetSessionLayer().subLayerPaths)
                with Usd.EditContext(self.stage,self.layer):
                    for prim in Usd.PrimRange(base.GetParent()):
                        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                            api=PhysxSchema.PhysxContactReportAPI.Apply(prim)
                            create=getattr(api,'CreateThresholdAttr',None) or getattr(api,'CreatePhysxContactReportThresholdAttr',None)
                            if create is None:raise RuntimeError('Installed ContactReport threshold API unavailable')
                            create().Set(0.)
                interface=omni.physx.get_physx_simulation_interface()
                self.subscription=interface.subscribe_contact_report_events(self.on_contacts)
                self.label.text='READY: now RUN AMR_V66_Navigation.py, then Play. Initial diagnostic manual limit will be 0.2 m/s; spin 0.3 rad/s.'
                started=False
                while not self.finish:
                    await kit.next_update_async()
                    if not self.window.visible:self.finish=True;break
                    if omni.usd.get_context().get_stage()!=self.stage:raise RuntimeError('Stage changed')
                    if self.timeline.is_playing():
                        started=True
                        if not self.attached:
                            view=SimulationManager.get_physics_simulation_view()
                            if view is None:continue
                            self.art=view.create_articulation_view(BASE)
                            self.body_view=view.create_rigid_body_view(ROOT+'/*_link')
                            self.meta['body_order']=[str(x) for x in self.body_view.prim_paths]
                            self.meta['joint_order']=list(self.art.shared_metatype.dof_names)
                            self.step_callback=SimulationManager.register_callback(self.on_step,event=SimulationEvent.PHYSICS_POST_STEP)
                            self.attached=True
                        a=self.live_app()
                        if a is not None and getattr(a,'ready',False) and a is not self.limited_app:self.limit(self.speed)
                        self.label.text=f'Recording: rows={len(self.rows)}, pair events={len(self.contacts)}, contact errors={len(self.contact_errors)}.\nManual limit={self.speed} m/s; spin 0.3 rad/s.\nUse this panel buttons. Stop between directions/speed stages.\nTilt >5 deg automatically ends the simulation and exports.'
                    elif started and self.timeline.is_stopped():break
            except Exception as exc:self.error=type(exc).__name__+': '+str(exc)
            finally:
                # End the test before removing contact reporting from the stage.
                if omni.usd.get_context().get_stage()==self.stage:
                    self.timeline.stop();await kit.next_update_async()
                if self.step_callback is not None:SimulationManager.deregister_callback(self.step_callback)
                self.subscription=None
                if self.layer is not None:
                    self.stage.GetSessionLayer().subLayerPaths=[x for x in self.stage.GetSessionLayer().subLayerPaths if x!=self.layer.identifier]
                self.meta.update(error=self.error,contact_errors=self.contact_errors,contact_callback_count=self.contact_callback_count,
                    contact_reporting_ok=self.meta.get('contact_reporting_ok',False),pair_counts=dict(self.counts),
                    retained_pair_events=len(self.contacts),dropped_pair_events=self.dropped,kinematic_samples=len(self.rows),
                    acceptance='DIAGNOSTIC_ONLY; no guarantee of stable motion or navigation')
                for name,value in [('metadata.json',self.meta),('contact_pairs.json',list(self.contacts)),('wheel_kinematics.json',self.rows)]:
                    (self.output/name).write_text(json.dumps(plain(value),indent=2),encoding='utf-8')
                target=self.output.with_suffix('.zip')
                with zipfile.ZipFile(target,'w',zipfile.ZIP_DEFLATED) as z:
                    for p in self.output.iterdir():z.write(p,p.name)
                self.done=True;self.label.text='FINISHED: '+str(target)+'\nError: '+str(self.error)+'\nContact evidence available: '+str(self.meta['contact_reporting_ok'])
                print('[V66 wheel contact] FINISHED',target,'error=',self.error)

    monitor=Monitor();state['_v66_contact_check']=monitor;editor_scope['_v66_contact_check']=monitor
    monitor.task=asyncio.ensure_future(monitor.run())


_start_wheel_contact_check(locals())
