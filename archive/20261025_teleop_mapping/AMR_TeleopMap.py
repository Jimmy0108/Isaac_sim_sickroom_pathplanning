# Run in Isaac Sim Script Editor, then Play. Upload entire python_nav folder.
import asyncio
import math
import sys
import time
from pathlib import Path
import numpy as np

PROJECT=Path('/isaac-sim/.local/share/ov/data/AMR_Project/1019_Scene/Collected_sickroom_with_hand_colaTest/python_nav')
if str(PROJECT) not in sys.path:sys.path.insert(0,str(PROJECT))


class TeleopMapPanel:
    async def start(self):
        import omni.usd,omni.timeline,omni.kit.app,omni.graph.core as og
        import omni.ui as ui
        import carb.input
        import omni.appwindow
        from pxr import UsdGeom
        import nav_toolkit as kit
        self.kit=kit;self.cfg=kit.load_config(PROJECT);self.stage=omni.usd.get_context().get_stage()
        self.app=omni.kit.app.get_app();self.timeline=omni.timeline.get_timeline_interface()
        self.paths=kit.discover(self.stage,self.cfg);self.units=UsdGeom.GetStageMetersPerUnit(self.stage)
        if UsdGeom.GetStageUpAxis(self.stage)!='Z':raise RuntimeError('Z-up stage required.')
        self.og=og;self.linear=og.Controller.attribute(self.paths['controller']+'.inputs:linearVelocity');self.angular=og.Controller.attribute(self.paths['controller']+'.inputs:angularVelocity')
        if not self.linear.is_valid() or not self.angular.is_valid():raise RuntimeError('Controller velocity inputs unavailable.')
        self.mode='TELEOP';self.armed=False;self.mapping=False;self.mapper=None;self.keys={};self.pulse=(0.,0.,0.);self.closed=False
        self.saved_new_session=False;self.nav_task=None;self.nav_namespace=None;self.fresh=-float('inf');self.stamp=None;self.cloud=None;self.last_pose=None
        self.last_map=-1.;self.last_draw=-1.;self.last_save=-1.;self.drew=False;self.was_playing=False;self.message='Press Play, enable keyboard, then Start Mapping.'
        self.command()
        try:
            if self.timeline.is_playing():raise RuntimeError('Press Stop, then Run this panel, then Play.')
            self.app.get_extension_manager().set_extension_enabled_immediate('isaacsim.sensors.experimental.rtx',True)
            self.app.get_extension_manager().set_extension_enabled_immediate('isaacsim.util.debug_draw',True)
            from isaacsim.sensors.experimental.rtx.generic_model_output import ElementFlags
            from isaacsim.util.debug_draw import _debug_draw
            self.draw=_debug_draw.acquire_debug_draw_interface();self.valid_flag=int(ElementFlags.VALID.value)
            self.reader=kit.get_reader(self.stage,self.paths['lidar'])
            for name,value in (('omni:sensor:Core:instantLidar',True),('omni:sensor:Core:accumulateOutputs',False)):
                attr=self.stage.GetPrimAtPath(self.paths['lidar']).GetAttribute(name)
                if not attr.IsValid() or not attr.Set(value):raise RuntimeError(f'Cannot configure {name}')
            self.sub=None
            try:
                self.input=carb.input.acquire_input_interface();self.keyboard=omni.appwindow.get_default_app_window().get_keyboard()
                if self.keyboard is not None:self.sub=self.input.subscribe_to_keyboard_events(self.keyboard,self.on_key)
            except Exception as keyboard_error:print('Keyboard unavailable; use pulse buttons:',keyboard_error)
            self.window=ui.Window('AMR | Teleop, Mapping, Goal B',width=460,height=500)
            with self.window.frame:
                with ui.VStack(spacing=5):
                    ui.Label('W/S: forward/reverse | A/D: turn | SPACE: stop',height=25)
                    ui.Label('Keyboard watchdog: hold/repeat refreshes; no input = stop.',height=25)
                    with ui.HStack(height=28):
                        ui.Button('Enable keyboard',clicked_fn=self.enable)
                        ui.Button('STOP / disarm',clicked_fn=self.stop)
                    ui.Label('Mouse buttons below are short movement pulses.',height=22)
                    with ui.HStack(height=28):
                        ui.Button('Forward',clicked_fn=lambda:self.nudge(1,0))
                        ui.Button('Reverse',clicked_fn=lambda:self.nudge(-1,0))
                        ui.Button('Left',clicked_fn=lambda:self.nudge(0,1))
                        ui.Button('Right',clicked_fn=lambda:self.nudge(0,-1))
                    with ui.HStack(height=28):
                        ui.Button('Start / resume mapping',clicked_fn=self.begin_map)
                        ui.Button('Pause mapping',clicked_fn=self.pause_map)
                    with ui.HStack(height=28):
                        ui.Button('Save map',clicked_fn=self.save)
                        ui.Button('Load saved map',clicked_fn=self.load)
                    ui.Separator(height=8)
                    ui.Label('A = current robot. B = drag GoalB using Move tool.',height=25)
                    with ui.HStack(height=28):
                        ui.Button('Create / select B',clicked_fn=self.select_goal)
                        ui.Button('Print current A/B',clicked_fn=self.print_poses)
                    with ui.HStack(height=28):
                        ui.Button('Preview A -> B',clicked_fn=lambda:self.launch(True))
                        ui.Button('Navigate A -> B',clicked_fn=lambda:self.launch(False))
                    self.label=ui.Label(self.message,height=110,word_wrap=True)
                    ui.Button('Close panel / stop',clicked_fn=self.close,height=28)
            print('Paths:',self.paths)
            print('Map directory:',PROJECT/'maps')
            while not self.closed:
                await self.app.next_update_async()
                if not self.window.visible:self.close();break
                await self.update()
        finally:
            self.command()
            if getattr(self,'nav_task',None) is not None and not self.nav_task.done():
                self.nav_task.cancel()
                try:await self.nav_task
                except (asyncio.CancelledError,Exception):pass
            if getattr(self,'mapper',None) is not None:
                try:self.save()
                except Exception as exc:print('Final map save failed:',exc)
            if getattr(self,'sub',None) is not None:self.input.unsubscribe_to_keyboard_events(self.keyboard,self.sub)
            if getattr(self,'window',None) is not None:self.window.destroy()
            self.command();print('Teleop panel stopped; velocity zero.')

    def command(self,v=0.,w=0.):
        self.og.Controller.set(self.linear,float(v*self.cfg['linear_command_sign']))
        self.og.Controller.set(self.angular,float(w*self.cfg['angular_command_sign']))
    def status(self,text):
        self.message=text
        if hasattr(self,'label'):self.label.text=text
    def enable(self):
        if self.mode=='NAV':self.status('Stop navigation first.');return
        self.keys.clear();self.armed=True;self.status('Keyboard armed. Click streamed viewport to send W/A/S/D; SPACE stops.')
    def stop(self):
        self.armed=False;self.keys.clear();self.pulse=(0,0,0)
        if self.nav_task is not None and not self.nav_task.done():self.nav_task.cancel()
        self.mode='TELEOP';self.command();self.status('Stopped and keyboard disarmed.')
    def close(self):self.stop();self.closed=True
    def on_key(self,event,*args):
        import carb.input as ci
        name=str(event.input).split('.')[-1]
        if name=='SPACE' and event.type in (ci.KeyboardEventType.KEY_PRESS,ci.KeyboardEventType.KEY_REPEAT):self.stop();return True
        if not self.armed or self.mode!='TELEOP':return True
        if name not in ('W','A','S','D','UP','DOWN','LEFT','RIGHT'):return True
        if event.type in (ci.KeyboardEventType.KEY_PRESS,ci.KeyboardEventType.KEY_REPEAT):self.keys[name]=time.monotonic()
        elif event.type==ci.KeyboardEventType.KEY_RELEASE:self.keys.pop(name,None)
        return True
    def nudge(self,v,w):
        if self.mode=='NAV':self.status('Stop navigation first.');return
        self.armed=False;self.keys.clear();self.pulse=(v,w,time.monotonic()+.25)
    def current(self):return self.kit.pose(self.stage,self.paths['base'],self.units,self.cfg['front_axis_sign'])
    def metadata(self):
        return dict(stage_identifier=self.stage.GetRootLayer().identifier,paths=self.paths,front_axis_sign=self.cfg['front_axis_sign'],
                    body=self.cfg['body'],method='known USD world pose + lidar ray tracing, NOT SLAM',sensor_world_z=self.last_sensor_z)
    def begin_map(self):
        if self.mode=='NAV':self.status('Stop navigation before mapping.');return
        if not self.timeline.is_playing():self.status('Press Play and wait for landing before mapping.');return
        p,z=self.current()
        if self.mapper is None:
            size=self.cfg['map_size_m'];res=self.cfg['map_resolution'];n=int(math.ceil(size/res))
            self.mapper=self.kit.Mapper(p[:2]-size/2,(n,n),res)
            self.last_sensor_z=float(self.kit.matrix(self.stage,self.paths['lidar'])[3,2]*self.units)
        self.mapping=True;self.status('Mapping active. Enable keyboard or use pulse buttons. Save before navigating.')
    def pause_map(self):self.mapping=False;self.command();self.armed=False;self.keys.clear();self.pulse=(0,0,0);self.status('Mapping paused; vehicle stopped.')
    def save(self):
        if self.mapper is None:self.status('No map collected yet.');return
        if self.mapper.scans==0:self.status('No valid scans collected; map not saved.');return
        folder=PROJECT/'maps'
        if not self.saved_new_session and (folder/'room_map_latest.npz').exists():
            import shutil
            archive=folder/'archive'/str(time.time_ns());archive.mkdir(parents=True,exist_ok=False)
            for name in ('room_map_latest.npz','room_map_latest.png','room_map_latest.pgm','room_map_metadata.json'):
                source=folder/name
                if source.is_file():shutil.copy2(source,archive/name)
        self.saved_new_session=True
        path=self.mapper.save(folder,self.metadata());self.status(f'Saved {path.name}; scans={self.mapper.scans}. Unknown remains blocked.')
        print('Map saved:',path)
    def load(self):
        if self.mode=='NAV' or self.mapping:self.status('Pause mapping/navigation before loading.');return
        try:
            self.mapper,meta=self.kit.Mapper.load(PROJECT/'maps'/'room_map_latest.npz',self.stage.GetRootLayer().identifier)
            self.saved_new_session=True;self.last_sensor_z=meta['sensor_world_z'];self.status('Loaded previous map. Use Start / resume mapping.')
        except Exception as exc:self.status(f'Load failed: {exc}')
    def select_goal(self):
        from pxr import UsdGeom,Gf
        import omni.usd
        if self.mode=='NAV':self.status('Stop navigation before editing B.');return
        self.stop();path=self.cfg['goal_marker'];prim=self.stage.GetPrimAtPath(path)
        if not prim.IsValid():
            parent=str(Path(path).parent).replace('\\','/')
            UsdGeom.Xform.Define(self.stage,parent)
            prim=UsdGeom.Xform.Define(self.stage,path).GetPrim()
            p,z=self.current();target=p[:2]+np.array([math.cos(p[2]),math.sin(p[2])])
            world=Gf.Matrix4d(1);world.SetRotate(Gf.Rotation(Gf.Vec3d(0,0,1),math.degrees(p[2])))
            world.SetTranslateOnly(Gf.Vec3d(float(target[0]/self.units),float(target[1]/self.units),float((z+.03)/self.units)))
            parent_world=UsdGeom.XformCache().GetLocalToWorldTransform(prim.GetParent())
            UsdGeom.Xformable(prim).MakeMatrixXform().Set(world*parent_world.GetInverse())
        omni.usd.get_context().get_selection().set_selected_prim_paths([path],True)
        self.status('B selected. Keyboard disarmed. Use Move gizmo to drag B on the floor; preview when ready.')
    def print_poses(self):
        p,z=self.current();prim=self.stage.GetPrimAtPath(self.cfg['goal_marker'])
        b=self.kit.matrix(self.stage,str(prim.GetPath()))[3,:3]*self.units if prim.IsValid() else None
        print('Current A [x,y,front_yaw]:',p,'B world:',b);self.status(f'A XY={np.round(p[:2],3)}; B={np.round(b[:2],3) if b is not None else "create B first"}')
    def launch(self,preview):
        if self.mode=='NAV':self.status('Navigation already running.');return
        self.pause_map();self.save()
        path=PROJECT/'AMR_Hybrid_Navigate_Isaac.py'
        if not self.stage.GetPrimAtPath(self.cfg['goal_marker']).IsValid():self.select_goal();self.status('Drag the new B marker, then press Preview/Navigate again.');return
        if self.cfg['map_source']=='SCANNED' and not (PROJECT/'maps'/'room_map_latest.npz').is_file():self.status('Scan and save a map first, or explicitly select SCENE map mode in nav_config.json.');return
        self.mode='NAV';self.armed=False
        namespace={'__name__':'__main__','__file__':str(path),'PANEL_PREVIEW_OVERRIDE':preview}
        try:
            exec(compile(path.read_text(encoding='utf-8-sig'),str(path),'exec'),namespace)
            self.nav_namespace=namespace;self.nav_task=namespace['_amr_hybrid_task'];self.status('Planning/navigation active. Green planned route, blue driven trail.')
        except Exception as exc:self.mode='TELEOP';self.command();self.status(f'Launch failed: {exc}')
    async def update(self):
        import omni.usd
        if omni.usd.get_context().get_stage()!=self.stage:
            self.close();return
        now=time.monotonic()
        if self.mode=='NAV':
            if self.nav_task is not None and self.nav_task.done():
                try:self.nav_task.result()
                except (asyncio.CancelledError,Exception) as exc:print('Navigation task ended:',exc)
                self.mode='TELEOP';self.command();self.status('Navigation task ended; see Script Editor output. Keyboard remains disarmed.')
            return  # Never fight the navigation controller with teleop commands.
        if not self.timeline.is_playing():
            self.command();self.keys.clear();self.pulse=(0,0,0)
            if self.timeline.is_stopped() and self.was_playing:
                self.mapping=False;self.armed=False;self.last_map=-1.;self.status('Timeline stopped. Play, wait for landing, then resume mapping explicitly.')
            self.was_playing=False
            return
        self.was_playing=True
        p,z=self.current();sim=self.timeline.get_current_time()
        result=self.kit.read_cloud(self.reader,self.stage,self.paths['lidar'],self.units,self.valid_flag)
        if result is not None and result[0]!=self.stamp:
            self.stamp,cloud,sensor=result
            self.cloud=self.kit.body_filter(cloud,p,z,self.cfg['body']);self.fresh=now
            if self.mapping and sim-self.last_map>=.10:
                if abs(sensor[2]-self.last_sensor_z)>.10:
                    self.mapping=False;self.armed=False;self.status('Sensor height changed >10 cm; stop and inspect before merging scans.')
                else:
                    self.mapper.update(sensor,cloud,p,z,self.cfg['body'],self.cfg['scan_max_range']);self.last_map=sim
        lease=self.cfg['keyboard_lease_wall_seconds']
        active={k for k,t in self.keys.items() if now-t<lease}
        f=int(bool(active&{'W','UP'}))-int(bool(active&{'S','DOWN'})) if self.armed else 0
        turn=int(bool(active&{'A','LEFT'}))-int(bool(active&{'D','RIGHT'})) if self.armed else 0
        if self.pulse[2]>now:f,turn=self.pulse[:2]
        v=f*self.cfg['teleop_speed'];w=turn*self.cfg['teleop_turn']
        if now-self.fresh>.75:v=w=0.
        if self.cfg['proximity_stop'] and self.cloud is not None and (v or w):
            body=self.cfg['body'];blocked=False
            for t in np.linspace(0,.60,8):
                a=p[2]+w*t
                q=p[:2]+(v*t*np.array([math.cos(p[2]),math.sin(p[2])]) if abs(w)<1e-9 else v/w*np.array([math.sin(a)-math.sin(p[2]),-math.cos(a)+math.cos(p[2])]))
                rel=self.cloud[:,:2]-q;c,s=math.cos(a),math.sin(a)
                inside=(abs(rel[:,0]*c+rel[:,1]*s)<body['length']/2+.04)&(abs(-rel[:,0]*s+rel[:,1]*c)<body['width']/2+.04)
                if np.any(inside):blocked=True;break
            if blocked:v=w=0.;self.keys.clear();self.pulse=(0,0,0);self.status('Proximity stop. Inspect lidar/self returns; do not force through obstacles.')
        self.command(v,w)
        if self.mapping and now-self.last_save>15 and self.mapper.scans>0:self.save();self.last_save=now
        if now-self.last_draw>.5:
            prim=self.stage.GetPrimAtPath(self.cfg['goal_marker'])
            if prim.IsValid():
                b=self.kit.matrix(self.stage,self.cfg['goal_marker'])[3,:3];b[2]=(z+.025)/self.units
                self.draw.clear_lines();self.draw.clear_points()
                d=.10/self.units
                self.draw.draw_lines([(b[0]-d,b[1],b[2]),(b[0],b[1]-d,b[2])],[(b[0]+d,b[1],b[2]),(b[0],b[1]+d,b[2])],[(1,0,0,1)]*2,[5.]*2)
            self.last_draw=now


async def boot(old_tasks):
    for task in old_tasks:
        if task is not None and not task.done():
            task.cancel()
            try:await task
            except (asyncio.CancelledError,Exception):pass
    panel=TeleopMapPanel()
    globals()['_python_nav_panel']=panel
    try:await panel.start()
    except asyncio.CancelledError:raise
    except Exception as exc:print(f'AMR panel stopped: {type(exc).__name__}: {exc}')

_old=tuple(globals().get(n) for n in ('_python_nav_panel_task','_amr_hybrid_task','_agv_navigation_task','_agv_follow_task','_agv_goal_task','_agv_map_task','_agv_test_task'))
_python_nav_panel_task=asyncio.ensure_future(boot(_old))
