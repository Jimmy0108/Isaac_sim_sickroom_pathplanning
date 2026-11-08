"""Isaac Sim Script Editor; use only with the V66 configured/control pair.
No SLAM, autonomous motion, obstacle avoidance or steering compensation.
"""
import asyncio
import omni.ui as ui
import omni.usd,omni.timeline,omni.kit.app
import omni.graph.core as og

GRAPH='/World/AGV_control/AGV_DifferentialGraph/DifferentialController'
BASE='/World/AGV_configured_v4/TZAGV_B02_vendor_v4/TZAGV_B02_root/base_link'
class V66Panel:
 def __init__(self):
  self.timeline=omni.timeline.get_timeline_interface();self.closed=False;self.ready=False;self.play_start=None
  self.window=ui.Window('V6.6 manual acceptance - no obstacle protection',width=490,height=300)
  with self.window.frame:
   with ui.VStack(spacing=8):
    ui.Label('Open V66_TestScene.usda. Run, then Play. Wait for READY.')
    ui.Label('Start at 0.20 m/s / 0.30 rad/s; buttons continue until STOP.')
    self.status=ui.Label('WAITING FOR PLAY',word_wrap=True)
    self.fast=ui.SimpleBoolModel(False);ui.CheckBox(self.fast);ui.Label('Enable 0.90 m/s / 0.90 rad/s after low-speed checks')
    with ui.HStack():
     ui.Button('Forward',clicked_fn=lambda:self.drive(1,0));ui.Button('Reverse',clicked_fn=lambda:self.drive(-1,0))
    with ui.HStack():
     ui.Button('Left spin',clicked_fn=lambda:self.drive(0,1));ui.Button('Right spin',clicked_fn=lambda:self.drive(0,-1))
    ui.Button('STOP',clicked_fn=self.stop)
    ui.Button('Close / STOP',clicked_fn=self.close)
  self.window.set_visibility_changed_fn(lambda visible:self.stop() if not visible else None)
  self.task=asyncio.ensure_future(self.watch())
 def command(self,v,w):
  for name,value in [('linearVelocity',-v),('angularVelocity',w)]:
   a=og.Controller.attribute(GRAPH+'.inputs:'+name)
   if a.is_valid():og.Controller.set(a,float(value))
 def stop(self):
  self.command(0,0)
  if hasattr(self,'status'):self.status.text='STOP commanded. Wait for physical standstill before switching direction.'
 def drive(self,v,w):
  if not self.ready or not self.timeline.is_playing():self.status.text='Not ready: press Play and wait 3 simulation seconds.';return
  speed=.9 if self.fast.get_value_as_bool() else .2;turn=.9 if self.fast.get_value_as_bool() else .3
  self.command(v*speed,w*turn);self.status.text=f'Command v={v*speed:.2f} m/s, w={w*turn:.2f} rad/s. STOP before changing direction.'
 def close(self):self.stop();self.closed=True;self.window.visible=False
 async def watch(self):
  app=omni.kit.app.get_app();self.stop()
  while not self.closed:
   await app.next_update_async()
   if not self.timeline.is_playing():self.ready=False;self.play_start=None;self.command(0,0);continue
   stage=omni.usd.get_context().get_stage()
   if not stage or not stage.GetPrimAtPath(BASE):self.stop();self.status.text='Wrong scene/robot path. Open V66_TestScene.usda.';continue
   radius=og.Controller.attribute(GRAPH+'.inputs:wheelRadius')
   if not radius.is_valid() or abs(float(og.Controller.get(radius))-.08)>1e-5:
    self.stop();self.status.text='Wrong controller or old override: wheel radius must be 0.08.';continue
   now=self.timeline.get_current_time()
   if self.play_start is None:self.play_start=now;self.command(0,0)
   if now<self.play_start:self.ready=False;self.play_start=now;self.command(0,0)
   if not self.ready and now-self.play_start>=3:
    self.ready=True;self.status.text='READY. Confirm visually that the robot has landed and is still, then try Forward.'
  self.command(0,0)
old=globals().get('_v66_manual_panel')
if old is not None:old.close()
_v66_manual_panel=V66Panel()
