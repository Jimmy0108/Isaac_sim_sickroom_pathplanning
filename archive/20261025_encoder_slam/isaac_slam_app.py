"""Isaac panel: actual wheel odom + experimental SLAM + Hybrid A* / Voronoi + RPP.
All navigation poses are estimated. World transforms are limited to Display calibration/markers.
"""
import asyncio,json,math,time
from pathlib import Path
from collections import deque
import numpy as np
from slam_core import WheelOdom,Slam2D,compose,inverse,transform,wrap,interpolate_odom
from rpp_core import RPP
import amr_planner_core as planner
from isaac_io import find_robot,EncoderReader,ScanReader,capture_mount_and_display,Display

RUNTIME_REVISION = '2026-10-25-continuous-drive-v6'

class Application:
    def __init__(self,project,path_overrides=None):
        self.project=Path(project);self.closed=False;self.path_overrides=dict(path_overrides or {})
    async def run(self):
        self.status('Importing Isaac Sim runtime modules...')
        import omni.usd,omni.timeline,omni.kit.app,omni.graph.core as og
        from isaacsim.core.simulation_manager import SimulationEvent,SimulationManager
        from pxr import UsdGeom
        self.simulation_time=SimulationManager.get_simulation_time
        self.cfg=json.loads((self.project/'slam_config.json').read_text(encoding='utf-8-sig'))
        self.cfg['paths'].update(self.path_overrides)
        self.teleop_v=float(np.clip(self.cfg.get('teleop_cruise_v',1.0),.05,1.4))
        self.teleop_w=float(np.clip(self.cfg.get('teleop_cruise_w',.8),.1,1.2))
        if not math.isfinite(self.teleop_v) or not math.isfinite(self.teleop_w):
            raise RuntimeError('Teleop cruise speeds must be finite')
        self.app=omni.kit.app.get_app();self.timeline=omni.timeline.get_timeline_interface();self.stage=omni.usd.get_context().get_stage()
        if self.stage is None:raise RuntimeError('Open the USD scene first')
        if self.timeline.is_playing():raise RuntimeError('Start from Stop, then Run, then Play')
        if UsdGeom.GetStageUpAxis(self.stage)!='Z':raise RuntimeError('Z-up stage required')
        self.status('Resolving configured robot paths...')
        self.paths=find_robot(self.stage,self.cfg);self.og=og
        self.linear=og.Controller.attribute(self.paths['controller']+'.inputs:linearVelocity');self.angular=og.Controller.attribute(self.paths['controller']+'.inputs:angularVelocity')
        if not self.linear.is_valid() or not self.angular.is_valid():raise RuntimeError('DifferentialController inputs not found')
        self.command()
        self.worker=None;self.plan_job=None;self.mode='TELEOP';self.armed=False;self.keys={};self.button_command=None
        self.ready=False;self.nav_active=False;self.plan_requested=None;self.save_requested=False;self.load_requested=False
        self.was_healthy=False;self.control_status='Waiting for scans';self.result=None;self.map_odom=np.zeros(3);self.last_good=-1e9;self.last_scan=-1e9;self.last_scan_submit=-1e9;self.last_draw=-1
        self.last_timing_report=-math.inf;self.waiting_scan_history=False
        self.odom_history=deque(maxlen=600);self.clouds=deque(maxlen=8);self.trail=[];self.global_path=np.empty((0,3));self.grid=None;self.replan_count=0;self.replan_after=0.
        self.session_saved=False;self.rpp=RPP(self.cfg['rpp']);self.last_control=(0,0);self.logs=[];self.slam=None;self.initial_hint_ready=False;self.sub=None
        self.create_ui()
        self.physics_ready=False
        self.physics_callback_id=None
        try:
            self.status('Enabling RTX lidar and debug draw extensions...')
            self.app.get_extension_manager().set_extension_enabled_immediate('isaacsim.sensors.experimental.rtx',True)
            self.app.get_extension_manager().set_extension_enabled_immediate('isaacsim.util.debug_draw',True)
            from isaacsim.util.debug_draw import _debug_draw
            self.draw=_debug_draw.acquire_debug_draw_interface()
            self.status('Creating RTX lidar reader...')
            self.scanner=ScanReader(self.stage,self.paths['lidar'],self.cfg)
            self.status('Subscribing to physics startup; press Play...')
            self.physics_callback_id=SimulationManager.register_callback(
                lambda *args:setattr(self,'physics_ready',True),event=SimulationEvent.SIMULATION_STARTED)
            while not self.timeline.is_playing():
                if self.closed:return
                await self.app.next_update_async()
            # Wait for Isaac Sim's warm-up and tensor view attachment. One UI update
            # after Play is not sufficient on every scene/remote workstation.
            self.status('Play detected; waiting for PhysX setup and tensor attachment...')
            ready_wait=time.monotonic()
            while not self.physics_ready:
                if self.closed:return
                if self.timeline.is_stopped():raise RuntimeError('Timeline stopped before PhysX setup completed')
                await self.app.next_update_async()
                if time.monotonic()-ready_wait>30:
                    raise RuntimeError('Timed out waiting for Isaac Sim SIMULATION_STARTED. Check the active PhysicsScene and PhysX errors.')
            simulation_view=SimulationManager.get_physics_simulation_view()
            if simulation_view is None:
                raise RuntimeError('SIMULATION_STARTED fired but PhysX tensor simulation view is still unavailable')
            self.status('Physics ready; reusing attached simulation view for wheel encoders...')
            self.encoders=EncoderReader(self.stage,self.paths,self.cfg,simulation_view=simulation_view)
            began=last=self.simulation_time();stable=0.;previous=np.array(self.encoders.read())
            self.status('Waiting >=3 simulation seconds and 1 second of quiet wheel encoders...')
            while stable<1 or self.simulation_time()-began<3:
                self.command();await self.app.next_update_async()
                if self.timeline.is_stopped():raise RuntimeError('Stopped during startup')
                now=self.simulation_time();q=np.array(self.encoders.read());dt=max(0,now-last)
                stable=stable+dt if np.linalg.norm(wrap(q-previous))<.002 else 0.;previous=q;last=now
                if now-began>20:raise RuntimeError('Wheel encoders do not settle')
                if self.closed:return
            self.odom=WheelOdom(self.cfg['body']['wheel_radius'],self.cfg['body']['track'])
            initial_time=self.simulation_time()
            self.odom.update(*self.encoders.read(),initial_time)
            self.odom_history.append((initial_time,self.odom.pose.copy()))
            mount,self.display=capture_mount_and_display(self.stage,self.paths,self.cfg)
            self.scanner.mount=mount;self.cfg['sensor_origin_front_m']=mount[3,:3].tolist();self.slam=Slam2D(self.cfg)
            self.ready=True;last=self.simulation_time();self.session_start=last
            self.status('SLAM starts at map (0,0,0). Enable teleop to scan; no USD pose feeds localization.')
            while not self.closed:
                await self.app.next_update_async()
                if not self.window.visible:self.closed=True;break
                if omni.usd.get_context().get_stage()!=self.stage:raise RuntimeError('Scene changed; session ended')
                if self.timeline.is_stopped():raise RuntimeError('Timeline reset. Rerun for new odom origin, or load map + manual initial pose.')
                if not self.timeline.is_playing():
                    self.command();self.keys.clear();self.button_command=None;self.armed=False
                    last=self.simulation_time();continue
                now=self.simulation_time();dt=now-last
                if dt<0:raise RuntimeError('Physics clock reset; restart the session for a new odom origin.')
                last=now
                if dt==0:continue
                if dt>.25:raise RuntimeError('Encoder sampling gap >0.25 simulation seconds; stop to avoid angle aliasing.')
                self.odom.update(*self.encoders.read(),now)
                self.odom_history.append((now,self.odom.pose.copy()))
                await self.update(now,dt)
        except asyncio.CancelledError:
            self.closed=True
            raise
        except Exception as exc:
            self.armed=False
            try:self.command()
            except Exception:pass
            message=f'Session stopped: {type(exc).__name__}: {exc}'
            print('SLAM/RPP stopped:',message)
            if hasattr(self,'label'):
                self.label.text=message+'\nMotor commands are zero. Fix the reported issue, then press Stop, Run, Play.'
            raise
        finally:
            callback_id=getattr(self,'physics_callback_id',None)
            if callback_id is not None:
                try:SimulationManager.deregister_callback(callback_id)
                except Exception:pass
            try:self.command()
            except Exception:pass
            self.armed=False
            if self.plan_job is not None:self.plan_job.cancel()
            if self.worker is not None:
                try:await asyncio.shield(self.worker)
                except (asyncio.CancelledError,Exception):pass
            if self.slam is not None and self.slam.nodes:
                try:self.save_bundle()
                except Exception as e:print('Final SLAM save failed:',e)
            if self.sub is not None:self.input.unsubscribe_to_keyboard_events(self.keyboard,self.sub)
            if getattr(self,'window',None) is not None and self.closed:self.window.destroy()
            if hasattr(self,'label') and not self.closed:
                self.label.text+='\nControl window stays open for this diagnostic. Re-run the entry after fixing it.'
            try:self.command()
            except Exception:pass
            print('SLAM/RPP session stopped; commands zero.')
    def command(self,v=0.,w=0.):
        self.og.Controller.set(self.linear,float(v*self.cfg['linear_command_sign']))
        self.og.Controller.set(self.angular,float(w*self.cfg['angular_command_sign']))
        self.last_control=(v,w)
    def status(self,text):
        print(text)
        if hasattr(self,'label'):self.label.text=text
    def record_scan_timing(self,now,scan_time,force=False):
        # Timeline is read ONLY for diagnostics; estimation uses physics time.
        timeline_time=self.timeline.get_current_time()
        oldest=self.odom_history[0][0] if self.odom_history else float('nan')
        newest=self.odom_history[-1][0] if self.odom_history else float('nan')
        details=(f'physics={now:.6f}s; lidar={scan_time:.6f}s; '
                 f'age={now-scan_time:.6f}s; timeline={timeline_time:.6f}s; '
                 f'encoder_history=[{oldest:.6f}, {newest:.6f}]s')
        wall=time.monotonic()
        if force or wall-self.last_timing_report>=5:
            self.last_timing_report=wall
            print('Sensor clock:',details)
            try:
                file=self.project/'AMR_SensorTiming.csv'
                header=not file.exists() or file.stat().st_size==0
                with file.open('a',encoding='utf-8') as stream:
                    if header:stream.write('wall_unix,physics_s,lidar_s,age_s,timeline_s,encoder_oldest_s,encoder_newest_s\n')
                    stream.write(f'{time.time():.6f},{now:.9f},{scan_time:.9f},{now-scan_time:.9f},{timeline_time:.9f},{oldest:.9f},{newest:.9f}\n')
            except OSError as exc:print('Timing report could not be written:',exc)
        return details
    def create_ui(self):
        import omni.ui as ui
        import carb.input,omni.appwindow
        try:
            self.input=carb.input.acquire_input_interface();self.keyboard=omni.appwindow.get_default_app_window().get_keyboard()
            if self.keyboard is not None:self.sub=self.input.subscribe_to_keyboard_events(self.keyboard,self.on_key)
        except Exception as e:print('Keyboard unavailable; use direction and STOP buttons:',e)
        self.window=ui.Window('AMR | Encoder SLAM + RPP',width=480,height=510)
        with self.window.frame:
            with ui.VStack(spacing=5):
                ui.Label('W/S forward/reverse | A/D turn | SPACE stop',height=25)
                ui.Label(f'Click direction once: continuous drive at {self.teleop_v:.2f} m/s / {self.teleop_w:.2f} rad/s. SPACE or STOP ends it.',height=35,word_wrap=True)
                with ui.HStack(height=30):
                    ui.Button('Enable teleop / scan',clicked_fn=self.enable)
                    ui.Button('STOP / disarm',clicked_fn=self.stop)
                with ui.HStack(height=30):
                    for text,v,w in [('Forward (continuous)',1,0),('Reverse (continuous)',-1,0),('Left (continuous)',0,1),('Right (continuous)',0,-1)]:
                        ui.Button(text,clicked_fn=lambda v=v,w=w:self.nudge(v,w))
                with ui.HStack(height=30):
                    ui.Button('Save SLAM map',clicked_fn=self.request_save)
                    ui.Button('Load saved map',clicked_fn=self.request_load)
                ui.Button('Use dragged initial pose (loaded map only)',clicked_fn=self.use_initial,height=30)
                with ui.HStack(height=30):
                    ui.Button('Create / select B',clicked_fn=self.select_b)
                    ui.Button('Print estimated pose',clicked_fn=self.print_pose)
                with ui.HStack(height=30):
                    ui.Button('Preview Hybrid path',clicked_fn=lambda:self.request_plan(True))
                    ui.Button('Navigate with RPP',clicked_fn=lambda:self.request_plan(False))
                self.label=ui.Label('Run, then Play. Wheel DOF names will be checked.',height=145,word_wrap=True)
                ui.Button('Close / stop',clicked_fn=self.close,height=30)
    def on_key(self,event,*args):
        import carb.input as ci
        key=str(event.input).split('.')[-1]
        if key=='SPACE' and event.type in (ci.KeyboardEventType.KEY_PRESS,ci.KeyboardEventType.KEY_REPEAT):self.stop();return True
        if not self.armed or self.mode!='TELEOP':return True
        if key in ('W','A','S','D','UP','DOWN','LEFT','RIGHT'):
            if event.type in (ci.KeyboardEventType.KEY_PRESS,ci.KeyboardEventType.KEY_REPEAT):self.keys[key]=time.monotonic()
            elif event.type==ci.KeyboardEventType.KEY_RELEASE:self.keys.pop(key,None)
        return True
    def enable(self):
        if not self.ready:return
        if self.mode in ('PLAN','NAV','WAIT_INITIAL'):self.status('Stop navigation or finish initial-pose localization first.');return
        self.armed=True;self.keys.clear();self.button_command=None
        self.status('Teleop enabled. Hold W/S or A/D. Release/SPACE stops. SLAM uses actual encoders + lidar.')
    def nudge(self,v,w):
        if self.ready and self.mode=='TELEOP':
            if not self.was_healthy:
                self.status('Waiting for healthy lidar + SLAM localization before enabling continuous drive.')
                return
            self.armed=False;self.keys.clear()
            self.command()
            self.button_command=(v,w)
            self.status(f'Continuous drive: {v*self.teleop_v:.2f} m/s, {w*self.teleop_w:.2f} rad/s. SPACE or STOP ends it; obstacles/localization loss also cancel it.')
    def stop(self):
        self.armed=False;self.keys.clear();self.button_command=None;self.command();self.nav_active=False;self.plan_requested=None
        if self.plan_job is not None:self.plan_job.cancel();self.plan_job=None
        if self.mode!='WAIT_INITIAL':self.mode='TELEOP'
    def close(self):self.stop();self.closed=True
    def estimated(self):return compose(self.map_odom,self.odom.pose)
    def save_bundle(self):
        folder=self.project/'slam_maps'
        if not self.session_saved and (folder/'slam_graph.npz').exists():
            import shutil
            archive=folder/'archive'/str(time.time_ns());archive.mkdir(parents=True,exist_ok=False)
            for file in folder.iterdir():
                if file.is_file():shutil.copy2(file,archive/file.name)
        self.session_saved=True
        self.slam.save(folder,self.metadata())
        if self.logs:
            np.savetxt(folder/'estimated_poses.csv',np.asarray(self.logs.copy()),delimiter=',',
                       header='sim_time,odom_x,odom_y,odom_yaw,map_x,map_y,map_yaw,odom_v,odom_w',comments='')
        (folder/'frame_contract.json').write_text(json.dumps(dict(map_to_odom=self.map_odom.tolist(),
             initial_display_anchor=self.display.anchor.tolist(),note='display anchor is visual only; estimator inputs are actual wheel angles and local lidar'),indent=2),encoding='utf-8')
    def metadata(self):
        return dict(stage_identifier=self.stage.GetRootLayer().identifier,display_anchor=self.display.anchor.tolist(),display_z=self.display.z,
                    body=self.cfg['body'],sensor_origin_front_m=self.cfg['sensor_origin_front_m'],localization='actual encoders + lidar only',paths=self.paths)
    def request_save(self):self.stop();self.save_requested=True
    def request_load(self):self.stop();self.load_requested=True
    def select_b(self):
        if self.ready:self.stop();self.display.select_goal(self.estimated());self.status('B selected. Drag Move gizmo on floor. A is the current SLAM estimate.')
    def print_pose(self):
        if self.ready:self.status(f'odom={np.round(self.odom.pose,3)}, SLAM map pose={np.round(self.estimated(),3)}; v={self.odom.v:.3f}, w={self.odom.w:.3f}')
    def use_initial(self):
        if self.mode!='WAIT_INITIAL' or self.worker is not None:return
        hint=self.init_display.goal(self.estimated(),True);self.slam.seed_hint=hint;self.slam.relocalized=False
        self.map_odom=compose(hint,inverse(self.odom.pose));self.initial_hint_ready=True;self.mode='TELEOP';self.last_good=-1e9
        self.status('Manual initial hint accepted; waiting for scan match. Hint is not ground-truth localization.')
    def request_plan(self,preview):
        if not self.ready:return
        if self.mode=='WAIT_INITIAL':self.status('Place and confirm the initial pose before planning.');return
        if self.mode=='WAIT_INITIAL':self.status('Set initial pose and obtain a valid scan match first.');return
        self.stop();self.plan_requested=bool(preview);self.mode='PLAN';self.replan_count=0
    def obstacles(self,now):
        chunks=[transform(points,compose(self.map_odom,odom)) for t,points,odom in self.clouds if now-t<.75]
        return np.vstack(chunks) if chunks else np.empty((0,2))
    def make_grid(self,now):
        origin,res,occ=self.slam.snapshot()
        cloud=self.obstacles(now)
        if len(cloud):
            cells=np.floor((cloud-origin)/res).astype(int)
            good=(cells[:,0]>=0)&(cells[:,1]>=0)&(cells[:,0]<occ.shape[1])&(cells[:,1]<occ.shape[0])
            occ[cells[good,1],cells[good,0]]=True
        b=self.cfg['body'];pc=planner.Config(length=b['length'],width=b['width'],track=b['track'],wheel_radius=b['wheel_radius'],
                                            margin=self.cfg['margin'],nominal_v=self.cfg['rpp']['max_v'],nominal_w=self.cfg['rpp']['max_w'])
        return planner.Grid(occ,res,origin,pc)
    async def update(self,now,dt):
        if self.worker is not None and self.worker.done():
            value=self.worker.result();self.worker=None
            if self.worker_kind=='scan':
                self.result=value;self.map_odom=value['map_odom'].copy()
                if value['ok']:self.last_good=self.worker_scan_time
                if value['revision']!=getattr(self,'path_revision',value['revision']) and self.nav_active:
                    self.command();self.nav_active=False;self.plan_requested=False;self.mode='PLAN'
                if value['message'].startswith('loop'):self.status(value['message'])
            elif self.worker_kind=='save':self.status('SLAM map + pose graph saved in slam_maps/.')
        if self.worker is None and self.save_requested:
            self.save_requested=False
            if self.slam.nodes:
                self.worker_kind='save';self.worker=asyncio.ensure_future(asyncio.to_thread(self.save_bundle))
        if self.worker is None and self.load_requested:
            self.load_requested=False;self.command()
            folder=self.project/'slam_maps'
            try:
                with np.load(folder/'room_map_latest.npz',allow_pickle=False) as data:meta=json.loads(str(data['metadata_json']))
                if meta['stage_identifier']!=self.stage.GetRootLayer().identifier:raise RuntimeError('Map stage differs; inspect registration first.')
                if np.linalg.norm(np.array(meta['sensor_origin_front_m'])-self.cfg['sensor_origin_front_m'])>.02:raise RuntimeError('Lidar mounting changed; map registration invalid.')
                self.slam=Slam2D.load_localization(folder,self.cfg,[0,0,0])
                self.display.anchor=np.array(meta['display_anchor']);self.display.z=meta['display_z']
                self.init_display=Display(self.stage,self.display.anchor,self.display.z,self.display.units,self.cfg['initial_marker'])
                guess=self.slam.nodes[-1];self.init_display.select_goal(compose(guess,[-1,0,0]))
                self.initial_hint_ready=False;self.mode='WAIT_INITIAL';self.clouds.clear();self.last_good=-1e9
                self.status('Loaded map: drag INITIAL marker onto robot and rotate its +X toward lidar front; then Use dragged initial pose.')
            except Exception as e:self.status(f'Load failed: {e}')
        result=self.scanner.read()
        if result is not None:
            stamp,points=result
            if len(points)>=25:
                scan_time=stamp*1e-9
                details=self.record_scan_timing(now,scan_time)
                if scan_time>now+.15 or now-scan_time>self.cfg['scan_timeout']:
                    self.record_scan_timing(now,scan_time,force=True)
                    raise RuntimeError('Lidar/physics timestamp mismatch: '+details)
                # A first buffered scan can precede the newly established odom
                # origin. Discard it rather than attach it to the current pose.
                if scan_time<self.odom_history[0][0]-.05:
                    self.command()
                    if not self.waiting_scan_history:
                        self.status('Waiting for a fresh scan within encoder history. '+details)
                        self.waiting_scan_history=True
                else:
                    self.waiting_scan_history=False
                    scan_odom=interpolate_odom(list(self.odom_history),scan_time)
                    self.last_scan=scan_time;self.clouds.append((scan_time,points,scan_odom.copy()))
                    if self.worker is None and now-self.last_scan_submit>=self.cfg['scan_period'] and self.mode!='WAIT_INITIAL' and self.plan_requested is None:
                        self.worker_kind='scan';self.worker_scan_time=scan_time;self.last_scan_submit=now
                        self.worker=asyncio.ensure_future(asyncio.to_thread(self.slam.process,points.copy(),scan_odom.copy()))
        if self.plan_requested is not None and self.plan_job is None and self.worker is None:
            if now-self.last_good>self.cfg['localization_timeout']:
                self.command();self.status('Need fresh accepted SLAM localization before planning.');self.plan_requested=None;self.mode='TELEOP'
            else:
                try:
                    self.goal=self.display.goal(self.estimated(),self.cfg['goal_yaw_from_marker']);self.grid=self.make_grid(now)
                    self.plan_start=self.estimated();self.plan_revision=self.slam.revision;self.plan_preview=self.plan_requested;self.plan_requested=None
                    self.plan_job=asyncio.ensure_future(asyncio.to_thread(planner.plan,self.grid,self.plan_start.copy(),self.goal.copy()))
                    self.status('Planning in SLAM map frame; velocity held at zero.')
                except Exception as e:self.command();self.mode='TELEOP';self.plan_requested=None;self.status(f'Plan refused: {e}')
        if self.plan_job is not None and self.plan_job.done():
            job=self.plan_job;self.plan_job=None
            try:
                samples,stats=job.result()
                if self.plan_revision!=self.slam.revision or np.linalg.norm(self.estimated()[:2]-self.plan_start[:2])>.08:
                    self.plan_requested=self.plan_preview;self.mode='PLAN'
                else:
                    self.rpp.set_path(samples);self.global_path=np.array([r[0] for r in samples]);self.path_revision=self.slam.revision
                    output=self.project/'slam_outputs';output.mkdir(exist_ok=True)
                    np.savez(output/'hybrid_path.npz',poses=self.global_path,path=self.global_path[:,:2],goal=self.goal,frame='map')
                    self.nav_active=not self.plan_preview;self.mode='NAV' if self.nav_active else 'TELEOP'
                    self.status(f'Plan ready: {stats["expansions"]} expansions. Green global; yellow RPP rollout; blue estimated trail.')
                    self.last_progress=now;self.progress_pose=self.estimated()
            except (asyncio.CancelledError,Exception) as e:self.mode='TELEOP';self.nav_active=False;self.status(f'Planning stopped: {e}')
        healthy=(now-self.last_good<self.cfg['localization_timeout'] and now-self.last_scan<self.cfg['scan_timeout'] and
                 self.result is not None and self.result['failures']<3 and self.mode!='WAIT_INITIAL')
        self.control_status='Localization healthy' if healthy else 'STOP: stale/untrusted scan localization or waiting for initial pose'
        if not healthy:
            self.command()
            self.button_command=None;self.armed=False;self.keys.clear()
            if self.was_healthy:self.save_stop(self.control_status,now)
        self.was_healthy=healthy
        if healthy and self.mode=='NAV':
            p=self.estimated();obs=self.obstacles(now)
            v,w,state=self.rpp.command(p,self.odom.v,self.grid,obs,dt)
            if state=='done':
                self.command();self.nav_active=False;self.mode='VERIFY';self.verify_start=now;self.verify_stable=0.
                self.status('Checking final estimated pose after wheel speeds settle...')
            elif state.startswith('replan'):
                self.command()
                if now>=self.replan_after:
                    self.save_stop(state,now)
                    self.replan_count+=1;self.replan_after=now+2
                    if self.replan_count>3:self.stop();self.status('Three blocked replans; stopped. Inspect scan/map or choose another B.')
                    else:self.nav_active=False;self.mode='PLAN';self.plan_requested=False;self.status(state+'; stopped before global replan.')
            else:
                self.command(v,w)
                if np.linalg.norm(p[:2]-self.progress_pose[:2])>.15 or abs(wrap(p[2]-self.progress_pose[2]))>.15:
                    self.last_progress=now;self.progress_pose=p.copy();self.replan_count=0
                if now-self.last_progress>12:self.stop();self.status('No estimated progress for 12 seconds; stopped.')
        elif healthy and self.mode=='VERIFY':
            self.command()
            self.verify_stable=self.verify_stable+dt if abs(self.odom.v)<.02 and abs(self.odom.w)<.04 else 0.
            if self.verify_stable>.3:
                p=self.estimated()
                valid_goal=np.linalg.norm(p[:2]-self.goal[:2])<=self.cfg['rpp']['position_tolerance'] and abs(wrap(p[2]-self.goal[2]))<=self.cfg['rpp']['yaw_tolerance']
                self.stop();self.status('Goal reached by SLAM estimate (not USD ground truth).' if valid_goal else 'Settled pose outside tolerance; stopped without reporting arrival.')
            elif now-self.verify_start>3:
                self.stop();self.status('Wheel velocities did not settle at goal; stopped without confirming arrival.')
        elif healthy and self.mode=='TELEOP':
            wall=time.monotonic();active={k for k,t in self.keys.items() if wall-t<self.cfg['keyboard_lease']}
            f=int(bool(active&{'W','UP'}))-int(bool(active&{'S','DOWN'})) if self.armed else 0
            turn=int(bool(active&{'A','LEFT'}))-int(bool(active&{'D','RIGHT'})) if self.armed else 0
            if self.button_command is not None:f,turn=self.button_command
            v=f*self.teleop_v;w=turn*self.teleop_w
            # Manual exploration may enter unobserved space, but observed close hits still stop it.
            obs=self.obstacles(now);p=self.estimated();blocked=False
            for t in np.linspace(0,.6,8):
                q=planner.integrate(p,v,w,t);local=transform(obs,inverse(q))
                if len(local) and np.any((abs(local[:,0])<self.cfg['body']['length']/2+self.cfg['margin'])&(abs(local[:,1])<self.cfg['body']['width']/2+self.cfg['margin'])):blocked=True;break
            if blocked:
                self.command();self.keys.clear();self.button_command=None;self.armed=False
                if f or turn:self.status('Teleop stopped by obstacle clearance. Choose a clear direction to resume.')
            else:self.command(v,w)
        else:self.command()
        if now-self.last_draw>.25:
            self.render(now);self.last_draw=now
            self.logs.append([now,*self.odom.pose,*self.estimated(),self.odom.v,self.odom.w])
    def save_stop(self,reason,now):
        folder=self.project/'slam_outputs';folder.mkdir(exist_ok=True)
        points=self.obstacles(now)
        np.savez(folder/'stop_diagnostic.npz',reason=reason,sim_time=now,odom=self.odom.pose,map_pose=self.estimated(),
                 map_odom=self.map_odom,obstacle_points_map=points,path=self.global_path,local_path=self.rpp.local_path,
                 logical_encoder_angles=np.array(self.encoders.read()))
    def render(self,now):
        self.draw.clear_lines();self.draw.clear_points()
        if len(self.global_path)>1:
            pts=[self.display.world(p) for p in self.global_path]
            self.draw.draw_lines(pts[:-1],pts[1:],[(0,1,0,1)]*(len(pts)-1),[4.]*(len(pts)-1))
        if self.ready:
            p=self.estimated();self.trail.append(p.copy());self.trail=self.trail[-2000:]
            if len(self.trail)>1:
                pts=[self.display.world(p) for p in self.trail];self.draw.draw_lines(pts[:-1],pts[1:],[(0,.4,1,1)]*(len(pts)-1),[2.]*(len(pts)-1))
            local=self.rpp.local_path
            if self.nav_active and len(local)>1:
                pts=[self.display.world(p) for p in local];self.draw.draw_lines(pts[:-1],pts[1:],[(1,.8,0,1)]*(len(pts)-1),[5.]*(len(pts)-1))
            try:
                g=self.display.goal(p,self.cfg['goal_yaw_from_marker']);self.draw.draw_points([self.display.world(g)],[(1,0,0,1)],[18.])
            except RuntimeError:pass
            if self.result is not None:
                v,w=self.last_control
                self.label.text=f'{self.mode} | odom XY {np.round(self.odom.pose[:2],3)} | SLAM XY {np.round(p[:2],3)}\nCommand {v:.2f} m/s, {w:.2f} rad/s | buttons drive continuously until STOP\nKeyframes {self.result["keyframes"]}; loops {self.result["loops"]}; scan RMSE {self.result["rmse"]:.3f}\n{self.control_status}; {self.result["message"]}\nGreen global / yellow local / blue estimated trail. Not ground truth.'
