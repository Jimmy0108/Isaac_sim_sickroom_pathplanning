"""V6.6 single owner: direct teleop, encoder/range SLAM, oriented Hybrid/RPP.

No manual heading compensation. World body observations are diagnostics/safety,
never position or heading feedback to odometry, localization or path tracking.
"""
import asyncio
import csv
import hashlib
import json
import math
import shutil
import time
import traceback
import zipfile
from collections import deque
from pathlib import Path

import numpy as np
import amr_planner_core as planner
from isaac_io import EncoderReader, ScanReader, capture_mount_and_display, find_robot, Display
from slam_core import Slam2D, WheelOdom, compose, inverse, transform, wrap, interpolate_odom
from rpp_core import RPP
from path_smoothing import smooth_samples
from v66_control import ManualDrive, wheel_limit, local_swept_clear, StopMonitor, PhysicsClockAudit
from v66_io import sample_diagnostics, snapshot_effective_physics, _plain, WheelContactRecorder
from v66_bootstrap import (_configure_navigation_clock, _prepare_drive_validation,
                            _NAV_CLOCK_SESSION_LAYERS, _NAV_DRIVE_SESSION_LAYERS,
                            capture_runtime_clock, restore_runtime_clock, _prepare_straight_drive,
                            _repair_missing_caster)

REVISION = '2026-11-06-v66-nav-1.1.1-replan-stop-clock'
# Same tolerance as scan/odom history bracketing. Raycast time is rounded to
# integer nanoseconds and converted back to float seconds (up to 0.5 ns ahead).
SCAN_TIME_EPS_S = 1e-8


def write_json(path, data):
    Path(path).write_text(json.dumps(_plain(data), ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')


def build_grid(snapshot, body, margin, rpp, live_points, planner_settings=None):
    """Worker input consists entirely of owned numpy copies, never USD views."""
    origin, resolution, occupied = snapshot
    if len(live_points):
        cells = np.floor((live_points-origin)/resolution).astype(int)
        valid = ((cells[:, 0] >= 0) & (cells[:, 0] < occupied.shape[1]) &
                 (cells[:, 1] >= 0) & (cells[:, 1] < occupied.shape[0]))
        occupied[cells[valid, 1], cells[valid, 0]] = True
    config = planner.Config(length=body['length'], width=body['width'], track=body['track'],
                            wheel_radius=body['wheel_radius'], margin=margin,
                            **(planner_settings or dict(nominal_v=min(.4, rpp['max_v']), nominal_w=rpp['max_w'])))
    return planner.Grid(occupied, resolution, origin, config)


def plan_owned(snapshot, body, margin, rpp, obstacles, start, goal, planner_settings=None):
    began = time.monotonic()
    grid = build_grid(snapshot, body, margin, rpp, obstacles, planner_settings)
    samples, stats = planner.plan(grid, start, goal)
    samples, stats['smoothing'] = smooth_samples(grid,samples)
    stats['wall_seconds_including_grid'] = time.monotonic()-began
    return grid, samples, stats


class Application:
    def __init__(self, project):
        self.project = Path(project)
        self.closed = self.closing = self.ready = False
        self.mode = 'STARTUP'
        self.cfg = json.loads((self.project/'v66_nav_config.json').read_text(encoding='utf-8-sig'))
        self.teleop_v = float(self.cfg.get('teleop_v', .9))
        self.teleop_w = float(self.cfg.get('teleop_w', .9))
        if not (0 < self.teleop_v <= .9 and 0 < self.teleop_w <= .9):
            raise ValueError('Manual limits must be finite, positive and <=0.9 (m/s, rad/s).')
        self.paths = dict(self.cfg['paths'])
        self.og = self.linear = self.angular = self.window = self.label = None
        self.sub = None
        self.callbacks = []
        self.last_control = (0., 0.)
        self.manual_target = (0., 0.)
        self.nav_target = (0., 0.)
        self.manual = ManualDrive(acceleration=self.cfg['teleop_acceleration_m_s2'],
                                  deceleration=self.cfg['teleop_deceleration_m_s2'],
                                  angular_acceleration=self.cfg['teleop_angular_acceleration_rad_s2'],
                                  max_v=self.teleop_v, max_w=self.teleop_w)
        self.keys = {}
        self.keyboard_enabled = True
        self.scan = None
        self.scan_queue = deque(maxlen=20)
        self.pending_sensor_frames = deque(maxlen=20)
        self.sensor_queue_drops = 0
        self.clouds = deque(maxlen=10)
        self.odom_history = deque(maxlen=1800)
        self.last_scan = self.last_good = self.last_submit = -math.inf
        self.map_odom = np.zeros(3)
        self.slam = self.result = None
        self.worker = self.plan_job = None
        self.io_job = None
        self.action = None
        self.mapping = self.map_ready = False
        self.epoch = 0
        self.trail = []
        self.global_path = np.empty((0, 3))
        self.grid = None
        self.goal = None
        self.goal_info = None
        self.rpp = RPP(self.cfg['rpp'])
        self.telemetry = []
        self.frames = []
        self.consumed = []
        self.events = []
        self.last_message = 'Run, then Play; direct manual mode first.'
        self.last_fault = None
        self.control_status = 'Waiting for Play'
        self.manual_stop_reason = None
        self.manual_stop_events = []
        self.pre_steps = self.post_steps = 0
        self.last_draw = self.last_diag = -math.inf
        self.replan_count = 0
        self.navigation_result = 'NOT_STARTED'
        self.session_id = str(time.time_ns())
        self.output = self.project/'slam_outputs'/('mapping_session_'+self.session_id)
        self.output.mkdir(parents=True, exist_ok=False)
        self.candidate = None
        self.physics_error = None
        self.export_requested = False
        self.control_epoch = 0
        self.contact_recorder = None
        self.stop_monitor = StopMonitor(self.cfg['body']['wheel_radius'])
        self.clock_audit = None
        self.stop_records = []
        self.last_stop_sample = -math.inf
        self.export_stop_evidence = None
        self.normal_export_verified = False

    def status(self, text):
        self.last_message = text
        entry = dict(wall_unix_s=time.time(), wall_monotonic_s=time.monotonic(),
                     physics_s=float(self.simulation_time()) if hasattr(self, 'simulation_time') else None,
                     mode=self.mode, message=text)
        self.events.append(entry)
        print('[AMR V66]', text)

    def zero(self):
        """Emergency stop remains valid during startup, pause and exceptions."""
        self.last_control = (0., 0.)
        self.stop_monitor.note_command(0.,0.)
        if self.og is None:
            return
        errors = []
        for attr in (self.linear, self.angular):
            if attr is not None:
                try:
                    self.og.Controller.set(attr, 0.)
                except Exception as exc:
                    errors.append(str(exc))
        if errors:
            raise RuntimeError('Failed writing motor zero: '+'; '.join(errors))

    def stop(self):
        was_moving = any(self.last_control) or any(self.manual_target) or any(self.nav_target)
        self.action = None
        self.manual_stop_reason = None
        self.control_status = 'Stopped by operator / operation request'
        self.manual_target = self.nav_target = (0., 0.)
        self.keys.clear()
        self.manual.stop()
        if was_moving:self.stop_monitor.invalidate_dwell()
        self.rpp.reset_output()
        self.control_epoch += 1
        self.zero()
        if self.mode in ('NAV', 'VERIFY', 'PLAN'):
            self.mode = 'READY' if self.map_ready else 'MANUAL'
        self.status('Stop requested: targets zero; physical standstill must be verified. Manual compensation is OFF.')

    def close(self):
        self.stop()
        self.closed = True

    def request(self, action):
        if not self.ready:
            self.status('Wait for Play and the settled wheel/sensor interfaces.')
            return
        self.stop()
        self.action = action
        self.action_started = float(self.simulation_time())
        self.status('Requested: '+action+'; waiting for wheel/body standstill and pending worker.')

    def set_manual(self, v, w):
        if (not self.ready or self.action or self.mode in ('STARTUP', 'BUSY')
                or self.clock_audit is None or not self.clock_audit.ready):
            self.status('Manual IO not ready or a map operation is running.')
            return
        self.control_epoch += 1
        self.nav_target = (0., 0.)
        self.rpp.reset_output()
        self.mode = 'MAPPING' if self.mapping else 'READY' if self.map_ready else 'MANUAL'
        self.keys.clear()
        self.manual_stop_reason = None
        self.manual_target = (float(v)*self.teleop_v, float(w)*self.teleop_w)
        self.status(f'Direct target {self.manual_target[0]:+.2f} m/s, {self.manual_target[1]:+.2f} rad/s; no heading compensation.')

    def on_key(self, event, *args):
        import carb.input as ci
        key = str(event.input).split('.')[-1]
        if key == 'SPACE' and event.type in (ci.KeyboardEventType.KEY_PRESS, ci.KeyboardEventType.KEY_REPEAT):
            self.stop()
            return True
        choices = {'W': (1, 0), 'UP': (1, 0), 'S': (-1, 0), 'DOWN': (-1, 0),
                   'A': (0, 1), 'LEFT': (0, 1), 'D': (0, -1), 'RIGHT': (0, -1)}
        if key not in choices:
            return False
        if event.type == ci.KeyboardEventType.KEY_PRESS:
            self.set_manual(*choices[key])
            self.keys[key] = time.monotonic()
        elif event.type == ci.KeyboardEventType.KEY_REPEAT and key in self.keys:
            self.keys[key] = time.monotonic()
        elif event.type == ci.KeyboardEventType.KEY_RELEASE and key in self.keys:
            self.stop()
        return True

    def create_ui(self):
        import omni.ui as ui
        import carb.input
        import omni.appwindow
        self.window = ui.Window('AMR V66 | Direct drive / SLAM / Hybrid + RPP', width=710, height=710)
        with self.window.frame:
            with ui.VStack(spacing=6):
                ui.Label('W/S forward/reverse | A/D spin | SPACE stop. Buttons continue until STOP.', height=32, word_wrap=True)
                ui.Label(f'Direct manual: max {self.teleop_v:.2f} m/s, {self.teleop_w:.2f} rad/s. No straight-line compensation. No IMU.', height=30, word_wrap=True)
                with ui.HStack(height=32):
                    for name, v, w in [('Forward', 1, 0), ('Reverse', -1, 0), ('Left spin', 0, 1), ('Right spin', 0, -1)]:
                        ui.Button(name, clicked_fn=lambda v=v, w=w: self.set_manual(v, w))
                ui.Button('STOP / cancel navigation', clicked_fn=self.stop, height=34)
                with ui.HStack(height=32):
                    ui.Button('Start NEW map', clicked_fn=lambda: self.request('new_map'))
                    ui.Button('Finish mapping / use candidate', clicked_fn=lambda: self.request('finish_map'))
                with ui.HStack(height=32):
                    ui.Button('Load saved map (stationary localization)', clicked_fn=lambda: self.request('load_map'))
                    ui.Button('Confirm candidate / archive old / save', clicked_fn=lambda: self.request('promote_map'))
                ui.Button('Create / select B: drag XY, rotate arrow to final heading', clicked_fn=self.select_goal, height=32)
                with ui.HStack(height=32):
                    ui.Button('Preview Hybrid A* + Voronoi', clicked_fn=lambda: self.request('preview'))
                    ui.Button('Navigate A -> B with RPP', clicked_fn=lambda: self.request('navigate'))
                ui.Label('Green = global | yellow = local rollout | blue = estimated trail | red arrow = B heading', height=32, word_wrap=True)
                self.label = ui.Label(self.last_message, height=270, word_wrap=True)
                ui.Button('Stop / export this session ZIP', clicked_fn=lambda: self.request('export'), height=34)
        try:
            self.input = carb.input.acquire_input_interface()
            self.keyboard = omni.appwindow.get_default_app_window().get_keyboard()
            if self.keyboard is not None:
                self.sub = self.input.subscribe_to_keyboard_events(self.keyboard, self.on_key)
        except Exception as exc:
            self.status('Keyboard unavailable; direction buttons remain usable: '+str(exc))

    def estimated(self):
        return compose(self.map_odom, self.odom.pose)

    def scan_ready(self, now):
        if self.scan is None or len(self.scan[1]) < 25:
            return False
        age = float(now)-float(self.scan[0])
        return (math.isfinite(age) and
                -SCAN_TIME_EPS_S <= age <= self.cfg['scan_timeout'])

    def localized(self, now):
        return (self.result is not None and bool(self.result['ok']) and self.scan_ready(now) and
                now-self.last_good <= self.cfg['localization_timeout'] and
                (not self.slam.loaded or self.slam.relocalized))

    def current_local_points(self):
        if self.scan is None:
            return np.empty((0, 2))
        _, points, old_odom = self.scan
        return transform(points, compose(inverse(self.odom.pose), old_odom))

    def obstacles(self, now):
        clouds = [transform(points, compose(self.map_odom, odom))
                  for stamp, points, odom in self.clouds if now-stamp <= .75]
        return np.vstack(clouds) if clouds else np.empty((0, 2))

    def _write_motion(self, v, w, now):
        if not self.scan_ready(now):
            self.latch_manual_stop('SCAN_NOT_READY', now, v, w)
            return
        body = self.cfg['body']
        v, w, scale = wheel_limit(v, w, body['wheel_radius'], body['track'], self.max_wheel_speed)
        if scale < 1:
            self.control_status = f'Wheel speed limit: uniform scale {scale:.3f}'
        if self.mode != 'NAV':
            # This guard has no map, SLAM heading, gain or hidden speed search.
            clear, hit = local_swept_clear(self.current_local_points(), v, w,
                                          body['length'], body['width'], self.cfg['margin'])
            if not clear:
                self.latch_manual_stop('LOCAL_FOOTPRINT_BLOCKED', now, v, w, hit)
                return
        self.og.Controller.set(self.linear, float(v*self.cfg['linear_command_sign']))
        self.og.Controller.set(self.angular, float(w*self.cfg['angular_command_sign']))
        self.last_control = (float(v), float(w))
        self.stop_monitor.note_command(v,w)

    def latch_manual_stop(self, reason, now, v, w, hit=None):
        """Keep the first zero-command cause visible; never silently remove a hit."""
        source = '<not available>'
        point = None if hit is None else np.asarray(hit, dtype=float).tolist()
        if hit is not None and self.scan is not None:
            points = self.current_local_points()
            index = int(np.argmin(np.linalg.norm(points-np.asarray(hit), axis=1)))
            scanner = getattr(self, 'scanner', None)
            for frame in reversed(getattr(scanner, 'identity_history', [])):
                if abs(frame['timestamp_ns']*1e-9-self.scan[0]) < 1e-6 and len(frame['points_front']) == len(points):
                    names = frame.get('prim_paths', [])
                    if index < len(names):
                        source = str(names[index])
                    break
        record = dict(reason=reason, physics_s=float(now), requested_manual_v_w=list(self.manual_target),
                      attempted_v_w=[float(v), float(w)], scan_age_s=float(now-self.scan[0]) if self.scan is not None else float('inf'),
                      retained_returns=len(self.scan[1]) if self.scan is not None else 0,
                      hit_point_front_m=point, hit_prim=source,
                      pre_steps=self.pre_steps, post_steps=self.post_steps,
                      footprint_length_m=self.cfg['body']['length'], footprint_width_m=self.cfg['body']['width'],
                      footprint_margin_m=self.cfg['margin'])
        self.manual_stop_events.append(record)
        if reason == 'SCAN_NOT_READY':
            text = f'STOP: no fresh local scan; age={record["scan_age_s"]:.9f}s, returns={record["retained_returns"]}'
        elif reason == 'KEYBOARD_TIMEOUT':
            text = 'STOP: keyboard input lease expired; use continuous direction buttons or press the key again'
        else:
            text = f'STOP: local 2D scan blocks swept footprint; hit front XY={np.round(hit,3).tolist()} m; prim={source}'
        self.manual_stop_reason = self.control_status = text
        self.manual_target = self.nav_target = (0., 0.)
        self.manual.stop()
        self.zero()
        self.status(text)

    def physics_post(self, *args):
        """Encoder acquisition and odometry happen on every completed physics step."""
        if not self.ready or self.closed or self.closing:
            return
        try:
            self.post_steps += 1
            now = float(self.simulation_time())
            if not args or not np.isscalar(args[0]):
                raise RuntimeError('Physics callback did not supply a scalar dt; refusing a configured-time fallback')
            if not self.clock_audit.observe(now,float(args[0]),float(self.timeline.get_current_time())):
                return
            if now < self.odom.time-1e-8:
                raise RuntimeError('Physics time reset: rerun for a new odometry origin')
            if now <= self.odom.time:
                return
            if now-self.odom.time > .25:
                raise RuntimeError('Actual encoder acquisition gap exceeds 0.25 simulation seconds')
            wheels = np.array(self.encoders.read(),dtype=float)
            self.odom.update(*wheels, now)
            self.odom_history.append((now, self.odom.pose.copy()))
            if now-self.last_stop_sample >= .02-1e-8:
                # World body state is used ONLY as a simulator stop interlock.
                # It never sets odom/map pose or supplies a steering correction.
                view=self.scanner.body_view
                pose=self.scanner.host(view.get_transforms())[0].astype(float)
                velocity=self.scanner.host(view.get_velocities())[0].astype(float)
                if pose.shape!=(7,) or velocity.shape!=(6,):
                    raise RuntimeError('Invalid stop-interlock body view')
                self.stop_monitor.observe(now,wheels,pose[:3]*self.scanner.units,pose[3:7],
                                          velocity[:3]*self.scanner.units,velocity[3:])
                self.stop_records.append(dict(physics_s=now,**self.stop_monitor.snapshot(now)))
                self.last_stop_sample=now
            # Sensor and encoders share the completed physical step. UI/render
            # cadence does not decide when the 10-Hz custom sensor takes a scan.
            raw = self.scanner.read()
            if raw is not None:
                if len(self.pending_sensor_frames) == self.pending_sensor_frames.maxlen:
                    self.sensor_queue_drops += 1
                self.pending_sensor_frames.append((raw, dict(self.scanner.last_frame)))
            if self.contact_recorder is not None:
                self.contact_recorder.sample(now, self.last_control)
            if now-self.last_diag >= .05-1e-8:
                data = sample_diagnostics(self, now)
                data.update(mode=self.mode, odom=self.odom.pose.tolist(),
                            estimated_map_pose=self.estimated().tolist(), command_v_w=list(self.last_control),
                            requested_manual_v_w=list(self.manual_target), control_status=self.control_status)
                data['stop_interlock']=self.stop_monitor.snapshot(now)
                data['rpp_tracking']=_plain(dict(self.rpp.telemetry)) if self.mode=='NAV' else None
                self.telemetry.append(data)
                self.last_diag = now
                # Safety stop only. These diagnostics do not steer or localize.
                body = data.get('body')
                if body and body['tilt_deg'] > 20:
                    self.physics_error = 'Vehicle tilt exceeds 20 degrees; stopped. Inspect contacts before resuming.'
        except Exception as exc:
            self.physics_error = type(exc).__name__+': '+str(exc)
            self.zero()

    def physics_pre(self, *args):
        """Single motor writer; worker/UI only change desired commands."""
        if not self.ready or self.closed or self.closing:
            return
        try:
            self.pre_steps += 1
            now = float(self.simulation_time())
            if self.physics_error or not self.timeline.is_playing():
                self.zero()
                return
            if self.clock_audit is None or not self.clock_audit.ready:
                self.zero()
                self.control_status='Validating common physics clock before motion'
                return
            if self.keys and time.monotonic()-max(self.keys.values()) > self.cfg['keyboard_lease']:
                self.keys.clear()
                self.latch_manual_stop('KEYBOARD_TIMEOUT', now, *self.last_control)
            if self.mode in ('MANUAL', 'MAPPING', 'READY') and self.action is None:
                if not args or not np.isscalar(args[0]):
                    raise RuntimeError('Missing physics pre-step dt')
                step_dt = float(args[0])
                if not math.isclose(step_dt,self.clock_audit.dt,rel_tol=1e-4,abs_tol=1e-8):
                    raise RuntimeError('Physics pre-step dt differs from verified clock')
                self.control_dt = step_dt
                v, w, step_status = self.manual.step(self.manual_target, step_dt,
                                                    self.odom.v, self.odom.w, now,
                                                    stop_confirmed=self.stop_monitor.confirmed(now))
                if not any(self.manual_target):
                    step_status='STOP_CONFIRMED' if self.stop_monitor.confirmed(now) else 'BRAKING: waiting for physical standstill'
                self.control_status = self.manual_stop_reason or step_status
                if v or w:
                    self._write_motion(v, w, now)
                else:
                    self.zero()
            elif self.mode == 'NAV' and self.localized(now) and now-self.nav_target_time <= .25:
                v, w = self.nav_target
                if v or w:
                    self._write_motion(v, w, now)
                else:
                    self.zero()
            else:
                self.zero()
        except Exception as exc:
            self.physics_error = type(exc).__name__+': '+str(exc)
            self.zero()

    def select_goal(self):
        if not self.ready or not self.map_ready:
            self.status('Finish mapping / use candidate, or load and localize a saved map first.')
            return
        self.stop()
        self.display.select_goal(self.estimated())
        self.status('B selected. Move XY; rotate local +X arrow to desired final vehicle front. Z is ignored.')

    def process_scan(self, points, odom, stamp, epoch):
        began = time.monotonic()
        result = self.slam.process(points, odom)
        record = dict(time=stamp, points=points, odom=odom, fitted_pose=result['pose'].copy(),
                      accepted=bool(result['ok']), rmse=result['rmse'], revision=result['revision'],
                      wall_seconds=time.monotonic()-began, epoch=epoch)
        return result, record

    def metadata(self):
        return dict(scan_backend=self.scanner.backend, stage_identifier=self.stage.GetRootLayer().identifier,
                    display_anchor=self.display.anchor.tolist(), display_z=self.display.z,
                    vehicle_contract_id=self.cfg['vehicle_contract_id'],
                    body=self.cfg['body'], sensor_origin_front_m=self.cfg['sensor_origin_front_m'],
                    paths=self.paths, revision=REVISION, localization='actual encoders + local lidar ONLY')

    async def owned_io(self, function, *args):
        # Keep a reference even if the outer UI task is cancelled during save/load.
        self.io_job = asyncio.create_task(asyncio.to_thread(function, *args))
        value = await asyncio.shield(self.io_job)
        self.io_job = None
        return value

    async def perform_action(self, action, now):
        if action == 'new_map':
            self.epoch += 1
            self.slam = Slam2D(self.cfg)
            mount, self.display = capture_mount_and_display(self.stage, self.paths, self.cfg)
            self.scanner.mount = mount
            self.map_odom = np.zeros(3)
            self.result = None
            self.last_good = self.last_submit = -math.inf
            self.mapping, self.map_ready = True, False
            self.clouds.clear()
            self.scan_queue.clear()
            self.global_path = np.empty((0, 3))
            self.trail = []
            self.candidate = None
            self.mode = 'MAPPING'
            self.status('New map started at local origin. Use direct buttons/WASD to scan; saved map unchanged.')
        elif action == 'finish_map':
            if not self.mapping or not self.slam.nodes or not self.localized(now):
                self.status('Cannot finish: need a new map and a fresh accepted scan. Manual control remains available.')
                return
            self.mapping = False
            self.map_ready = True
            self.slam.loaded = self.slam.relocalized = True
            self.candidate = self.output/'candidate_map'
            await self.owned_io(self.slam.save, self.candidate, self.metadata())
            self.mode = 'READY'
            self.status('Candidate saved and frozen for navigation. A=current SLAM estimate. Create B, preview, navigate.')
        elif action == 'promote_map':
            if self.candidate is None or not (self.candidate/'slam_graph.npz').is_file():
                self.status('Finish mapping first; there is no candidate to confirm.')
                return
            if not self.map_ready or not self.localized(now):
                self.status('Confirm refused: candidate must be active with healthy current localization.')
                return
            target = self.project/'slam_maps'
            if target.is_dir():
                archive = target/'archive'/str(time.time_ns())
                archive.mkdir(parents=True)
                for path in target.iterdir():
                    if path.is_file():
                        shutil.copy2(path, archive/path.name)
            target.mkdir(exist_ok=True)
            # Motors stopped, no worker; confirmation is explicit. Copy via temp files.
            for path in self.candidate.iterdir():
                if path.is_file():
                    temporary = target/(path.name+'.v105_tmp')
                    shutil.copy2(path, temporary)
                    temporary.replace(target/path.name)
            self.status('Candidate confirmed; old saved files archived, slam_maps updated.')
        elif action == 'load_map':
            folder = self.project/'slam_maps'
            with np.load(folder/'room_map_latest.npz', allow_pickle=False) as saved:
                meta = json.loads(str(saved['metadata_json']))
            if meta.get('vehicle_contract_id') != self.cfg['vehicle_contract_id']:
                raise RuntimeError('Map vehicle contract differs or is missing: build a new V66 map; old maps are not silently converted.')
            for key in ('wheel_radius', 'track', 'length', 'width'):
                if not math.isclose(float(meta.get('body', {}).get(key, -1)), self.cfg['body'][key], abs_tol=1e-6):
                    raise RuntimeError('Saved map body/odometry contract differs: '+key)
            if meta.get('scan_backend') != self.scanner.backend:
                raise RuntimeError('Saved/live scan backends differ')
            if meta.get('stage_identifier') != self.stage.GetRootLayer().identifier:
                raise RuntimeError('Saved map belongs to a different stage')
            if np.linalg.norm(np.asarray(meta['sensor_origin_front_m'])-self.cfg['sensor_origin_front_m']) > .02:
                raise RuntimeError('Saved/live lidar mounting differs')
            self.slam = await self.owned_io(Slam2D.load_localization, folder, self.cfg, None)
            self.epoch += 1
            self.result = None
            self.last_good = self.last_submit = -math.inf
            self.mapping, self.map_ready = False, True
            self.clouds.clear()
            self.global_path = np.empty((0, 3))
            self.mode = 'LOCALIZE'
            self.candidate = None
            self.display = Display(self.stage, np.asarray(meta['display_anchor']), float(meta['display_z']),
                                   self.scanner.units, self.cfg['goal_marker'])
            self.status('Saved map loaded: stationary lidar localization only. No recovery motion.')
        elif action in ('preview', 'navigate', 'replan'):
            if not self.map_ready or not self.localized(now):
                self.status('Navigation needs a frozen map and fresh accepted localization; manual mode is independent.')
                return
            if action != 'replan':
                self.goal = self.display.read_goal_pose()
                self.goal_info = self.display.goal_details()
                self.replan_count = 0
            write_json(self.output/'goal.json', self.goal_info)
            start = self.estimated().copy()
            self.plan_epoch = (self.epoch, self.control_epoch)
            self.plan_preview = action == 'preview'
            self.plan_start = start
            self.plan_goal = self.goal.copy()
            self.mode = 'PLAN'
            self.plan_job = asyncio.create_task(asyncio.to_thread(plan_owned, self.slam.snapshot(),
                dict(self.cfg['body']), self.cfg['margin'], dict(self.cfg['rpp']), self.obstacles(now).copy(),
                start, self.goal.copy(), dict(self.cfg.get('planner', {}))))
            self.status('Planning XY AND final heading with differential Hybrid A* + Voronoi; wheels stopped.')
        elif action == 'export':
            if not self.stop_monitor.confirmed(now):
                raise RuntimeError('Export cannot complete normally while physical stop is unverified')
            self.export_stop_evidence=self.stop_monitor.snapshot(now)
            self.normal_export_verified=True
            self.export_requested = True
            self.closed = True

    async def update(self, now, dt):
        if self.physics_error:
            raise RuntimeError(self.physics_error)
        if self.worker is not None and self.worker.done():
            job, self.worker = self.worker, None
            result, record = job.result()
            self.consumed.append(record)
            if record['epoch'] == self.epoch:
                self.result = result
                if result['ok']:
                    self.map_odom = result['map_odom'].copy()
                    self.last_good = record['time']
                    if self.mode == 'LOCALIZE' and self.slam.relocalized:
                        self.mode = 'READY'
                        self.status('Localized from lidar + odom. A=current estimate; set B then navigate.')
                if not result['ok']:
                    self.control_status = 'SCAN MATCH REJECTED: '+result['message']+'; mapping integration held'
        if self.plan_job is not None and self.plan_job.done():
            job, self.plan_job = self.plan_job, None
            try:
                grid, samples, stats = job.result()
                if self.plan_epoch != (self.epoch, self.control_epoch) or self.mode != 'PLAN':
                    self.status('Discarded cancelled/stale plan.')
                elif (np.linalg.norm(self.estimated()[:2]-self.plan_start[:2]) > .08 or
                      abs(wrap(self.estimated()[2]-self.plan_start[2])) > math.radians(5)):
                    self.mode = 'READY'
                    self.status('Robot estimate moved during planning; preview/navigate again from current A.')
                elif not self.localized(now):
                    self.mode = 'READY'
                    self.status('Plan finished but localization is no longer healthy; remain stopped.')
                else:
                    self.grid = grid
                    self.rpp.set_path(samples)  # Keep terminal rotation group; B heading is required.
                    self.global_path = np.asarray([sample[0] for sample in samples])
                    np.savez_compressed(self.output/'hybrid_path.npz', poses=self.global_path,
                                        goal=self.plan_goal, frame='map')
                    write_json(self.output/'planner_stats.json', stats)
                    self.mode = 'READY' if self.plan_preview else 'NAV'
                    self.navigation_result = 'PREVIEW_ONLY' if self.plan_preview else 'IN_PROGRESS'
                    self.nav_target = (0., 0.)
                    self.nav_target_time = now
                    self.progress_pose = self.estimated()
                    self.progress_time = now
                    self.status(f'Plan ready: {stats["expansions"]} expansions; final XY and yaw required.')
            except Exception as exc:
                self.mode = 'READY'
                self.status('Planning refused: '+str(exc))
        # State changes and saving never race a scan or planning thread.
        if self.action and not self.stop_monitor.confirmed(now) and now-self.action_started>8.:
            raise RuntimeError('STOP_NOT_VERIFIED: requested '+self.action+' aborted after 8 physics seconds; diagnostic export is not a successful stop')
        if self.action and self.worker is None and self.plan_job is None:
            if self.stop_monitor.confirmed(now):
                action, self.action = self.action, None
                try:
                    self.mode = 'BUSY'
                    await self.perform_action(action, now)
                    if self.mode == 'BUSY':
                        self.mode = 'READY' if self.map_ready else 'MAPPING' if self.mapping else 'MANUAL'
                except Exception as exc:
                    self.mode = 'READY' if self.map_ready else 'MAPPING' if self.mapping else 'MANUAL'
                    self.status('Operation refused: '+str(exc))
        while self.pending_sensor_frames:
            raw, captured_frame = self.pending_sensor_frames.popleft()
            stamp, points = raw
            stamp *= 1e-9
            self.scan_queue.append((stamp, points.copy()))
            frame = dict(captured_frame)
            frame.pop('x', None)
            frame.pop('y', None)
            frame.pop('z', None)
            frame.pop('flags', None)
            frame.update(odom_pose=interpolate_odom(list(self.odom_history),stamp).tolist(), SLAM_pose_at_consumption=self.estimated().tolist(),
                         command_v_w=list(self.last_control), mode=self.mode,
                         consumption_physics_s=now,
                         recorder_wall_monotonic_s=time.monotonic(), scan_match_result=self.result)
            self.frames.append(frame)
            if not hasattr(self, 'raw_scans'):
                self.raw_scans = []
            self.raw_scans.append((stamp, np.array(captured_frame['x']),
                                   np.array(captured_frame['z']),
                                   np.array(captured_frame['hit_states']),
                                   np.array(captured_frame['prim_paths'])))
        # Queue future samples until odom actually brackets the timestamp; no clamping.
        while self.scan_queue and self.scan_queue[0][0] <= self.odom_history[-1][0]+SCAN_TIME_EPS_S:
            stamp, points = self.scan_queue.popleft()
            if stamp < self.odom_history[0][0]-SCAN_TIME_EPS_S:
                continue
            if len(points) < 25:
                self.control_status = f'Not enough external lidar returns: {len(points)}'
                continue
            scan_odom = interpolate_odom(list(self.odom_history), stamp)
            self.scan = (stamp, points, scan_odom)
            self.last_scan = stamp
            self.clouds.append(self.scan)
            if (self.slam is not None and self.worker is None and self.action is None and
                    stamp-self.last_submit >= self.cfg['scan_period']-1e-8):
                self.last_submit = stamp
                self.worker = asyncio.create_task(asyncio.to_thread(self.process_scan, points.copy(),
                                                   scan_odom.copy(), stamp, self.epoch))
        if self.mode == 'NAV':
            if not self.localized(now):
                self.nav_target = (0., 0.)
                self.navigation_result = 'STOPPED_FOR_LOCALIZATION'
                self.stop()
                self.status('Navigation stopped: scan/localization unhealthy. No automatic recovery movement.')
            else:
                v, w, state = self.rpp.command(self.estimated(), self.odom.v, self.grid,
                                               self.obstacles(now), dt, measured_w=self.odom.w,
                                               stop_confirmed=self.stop_monitor.confirmed(now))
                self.nav_target, self.nav_target_time = (v, w), now
                self.control_status = state
                if state == 'done':
                    self.zero()
                    self.stop_monitor.invalidate_dwell()
                    self.mode = 'VERIFY'
                    self.verify_start, self.verify_quiet = now, None
                elif state.startswith('replan'):
                    self.zero()
                    self.nav_target = (0., 0.)
                    self.replan_count += 1
                    self.control_epoch += 1
                    if self.replan_count <= 3:
                        self.mode = 'READY'
                        self.action = 'replan'
                        self.action_started = float(now)
                        self.status('Zero targets requested; waiting for verified stop before global replan: '+state)
                    else:
                        self.stop()
                        self.navigation_result = 'BLOCKED'
                        self.status('Path remains blocked after 3 replans; choose another B or remove obstacle.')
                elif np.linalg.norm(self.estimated()[:2]-self.progress_pose[:2]) > .1 or abs(wrap(self.estimated()[2]-self.progress_pose[2])) > .1:
                    self.progress_pose, self.progress_time = self.estimated(), now
                elif now-self.progress_time > 12:
                    self.navigation_result = 'NO_PROGRESS'
                    self.stop()
                    self.status('Navigation stopped: no estimated progress for 12 simulation seconds.')
        elif self.mode == 'VERIFY':
            self.zero()
            if not self.localized(now):
                self.navigation_result = 'LOCALIZATION_LOST_AT_GOAL'
                self.stop()
            else:
                if self.stop_monitor.confirmed(now):
                    p = self.estimated()
                    position_error = float(np.linalg.norm(p[:2]-self.goal[:2]))
                    yaw_error = float(abs(wrap(p[2]-self.goal[2])))
                    passed = position_error <= .07 and yaw_error <= math.radians(5)
                    self.navigation_result = 'GOAL_REACHED_BY_SLAM_ESTIMATE' if passed else 'FINAL_POSE_OUTSIDE_TOLERANCE'
                    write_json(self.output/'arrival.json', dict(result=self.navigation_result,
                        position_error_m=position_error, yaw_error_deg=math.degrees(yaw_error), pose=p,
                        goal=self.goal, stop_interlock=self.stop_monitor.snapshot(now),
                        basis='SLAM goal pose; separate simulator-only wheel/body standstill interlock'))
                    self.stop()
                    self.status(f'{self.navigation_result}: XY error {position_error:.3f} m, yaw error {math.degrees(yaw_error):.2f} deg.')
                elif now-self.verify_start > 3:
                    self.navigation_result = 'GOAL_STOP_NOT_VERIFIED'
                    self.stop()
        if now-self.last_draw >= .2:
            self.render(now)
            self.last_draw = now

    def render(self, now):
        self.draw.clear_lines()
        self.draw.clear_points()
        def lines(path, color, width):
            if len(path) > 1:
                pts = [self.display.world(p) for p in path]
                self.draw.draw_lines(pts[:-1], pts[1:], [color]*(len(pts)-1), [width]*(len(pts)-1))
        lines(self.global_path, (0, 1, 0, 1), 4.)
        if self.result is not None:
            self.trail.append(self.estimated().copy())
            self.trail = self.trail[-2500:]
            lines(self.trail, (0, .4, 1, 1), 2.)
        if self.mode == 'NAV':
            lines(self.rpp.local_path, (1, .8, 0, 1), 4.)
        goal_text = 'Create B, then drag XY and rotate its arrow.'
        try:
            goal = self.display.read_goal_pose()
            starts, ends = self.display.goal_arrow(goal)
            self.draw.draw_lines(starts, ends, [(1, .1, .1, 1)]*len(starts), [5.]*len(starts))
            data = self.display.goal_details()
            goal_text = f'B world XY {np.round(data["world_xy_m"],3)} / yaw {data["world_yaw_deg"]:.1f} deg\nB map XY {np.round(data["map_xy_m"],3)} / yaw {data["map_yaw_deg"]:.1f} deg'
        except RuntimeError:
            pass
        scan_age = now-self.last_scan
        match_text = self.result['message'] if self.result else 'SLAM not started (manual IO is independent)'
        visible_status = self.manual_stop_reason or self.control_status
        contacts = self.contact_recorder
        contact_text = ('Contacts: not attached' if contacts is None else
                        f'Contacts: samples={len(contacts.samples)}, errors={contacts.error_count}, floor signal={contacts.ever_positive}')
        evidence=self.stop_monitor.snapshot(now)
        stop_text=(f'Stop verified={evidence["confirmed"]}; quiet {evidence["quiet_dwell_s"]:.2f}/{evidence["required_dwell_s"]:.2f}s; '
                   f'body v={evidence.get("body_median_speed_m_s",float("nan")):.3f} m/s')
        self.label.text = (f'{REVISION}\n{self.mode} | requested v,w {np.round(self.manual_target,3)} | command {np.round(self.last_control,3)}\n'
                           f'Physics callbacks PRE/POST {self.pre_steps}/{self.post_steps}; dt={getattr(self,"control_dt",0.):.4f}s\n'
                           f'{stop_text}\n'
                           f'{contact_text}\n'
                           f'Scan age {scan_age:.3f}s | {self.scanner.backend} | 720 beams, 10 Hz\n'
                           f'Odom {np.round(self.odom.pose,3)} | estimated A {np.round(self.estimated(),3)}\n'
                           f'{goal_text}\n{match_text}\n{visible_status}\n{self.last_message}')

    async def run(self):
        import omni.usd
        import omni.timeline
        import omni.kit.app
        import omni.graph.core as og
        from pxr import UsdGeom
        from isaacsim.core.simulation_manager import SimulationManager, SimulationEvent
        self.manager, self.event = SimulationManager, SimulationEvent
        self.simulation_time = SimulationManager.get_simulation_time
        self.app = omni.kit.app.get_app()
        self.timeline = omni.timeline.get_timeline_interface()
        self.stage = omni.usd.get_context().get_stage()
        if self.stage is None or self.timeline.is_playing():
            raise RuntimeError('Open scene, press Stop, then RUN this entry, then Play.')
        if UsdGeom.GetStageUpAxis(self.stage) != 'Z':
            raise RuntimeError('This navigation frame requires Z-up.')
        self.paths = find_robot(self.stage, self.cfg)
        self.cfg['paths'] = dict(self.paths)
        self.og = og
        self.linear = og.Controller.attribute(self.paths['controller']+'.inputs:linearVelocity')
        self.angular = og.Controller.attribute(self.paths['controller']+'.inputs:angularVelocity')
        if not self.linear.is_valid() or not self.angular.is_valid():
            raise RuntimeError('DifferentialController velocity inputs not found.')
        self.zero()
        self.create_ui()
        self.initial_session_layers = set(self.stage.GetSessionLayer().subLayerPaths)
        self.clock_setup = None
        self.drive_setup = None
        self.runtime_clock_before = capture_runtime_clock(self.stage, self.timeline)
        self.cleanup_errors = []
        try:
            self.clock_setup = _configure_navigation_clock(self.stage, self.cfg)
            write_json(self.output/'physics_clock_setup.json', self.clock_setup)
            self.drive_setup = _prepare_drive_validation(self.stage, self.cfg, raise_on_error=False)
            write_json(self.output/'vehicle_preflight.json', self.drive_setup)
            if self.drive_setup['blocking_checks']:
                raise RuntimeError('V66 vehicle contract failed: '+', '.join(self.drive_setup['blocking_checks'])+'; full details saved in vehicle_preflight.json')
            self.status('V66 dimensions, six wheel colliders and graph bindings checked. No old torque/friction/caster repair applied.')
            self.caster_setup = _repair_missing_caster(self.stage, self.paths)
            write_json(self.output/'caster_collision_repair.json', self.caster_setup)
            self.status('Caster inventory: '+self.caster_setup['action']+'; no geometry edits.')
            self.straight_setup = _prepare_straight_drive(self.stage, self.paths)
            write_json(self.output/'straight_drive_setup.json', self.straight_setup)
            self.status('Existing wheel binding verified; solver, drive gains, force and materials preserved.')
            if self.cfg.get('sim_lidar_rtx_reference', False):
                self.app.get_extension_manager().set_extension_enabled_immediate('isaacsim.sensors.experimental.rtx', True)
            self.app.get_extension_manager().set_extension_enabled_immediate('isaacsim.util.debug_draw', True)
            from isaacsim.util.debug_draw import _debug_draw
            self.draw = _debug_draw.acquire_debug_draw_interface()
            self.scanner = ScanReader(self.stage, self.paths['lidar'], self.cfg)
            self.physics_started = False
            self.callbacks.append(SimulationManager.register_callback(
                lambda *args: setattr(self, 'physics_started', True), event=SimulationEvent.SIMULATION_STARTED))
            self.status('Waiting for Play; no saved-map load or active recovery movement on startup.')
            while not self.timeline.is_playing():
                if self.closed:
                    return
                await self.app.next_update_async()
            wait_wall = time.monotonic()
            while not self.physics_started:
                if self.closed:
                    return
                if self.timeline.is_stopped() or time.monotonic()-wait_wall > 30:
                    raise RuntimeError('PhysX startup did not attach to the stage.')
                await self.app.next_update_async()
            view = SimulationManager.get_physics_simulation_view()
            from pxr import UsdPhysics
            active_scenes=[str(p.GetPath()) for p in self.stage.Traverse() if p.IsA(UsdPhysics.Scene)]
            if active_scenes != self.clock_setup['physics_scenes']:
                raise RuntimeError('PhysicsScene set changed during Play; no automatic isolation: '+str(active_scenes))
            self.encoders = EncoderReader(self.stage, self.paths, self.cfg, simulation_view=view)
            # Contact-report APIs were prepared while stopped by the existing
            # drive adapter. Creating tensor views here does not edit the stage.
            if self.cfg.get('record_wheel_contacts', False):
                try:
                    self.contact_recorder = WheelContactRecorder(view, self.paths['base'].rsplit('/',1)[0],
                        self.cfg['contact_floor_path'], self.cfg['physics_hz'])
                except Exception as exc:
                    self.status('Optional contact recorder unavailable: '+str(exc)+'; joint/body telemetry remains enabled.')
            self.status('Scene physics rate retained: '+str(self.cfg['physics_hz'])+' Hz. Render timing is unchanged.')
            # Read the evaluated graph inputs after startup; USD None is not a
            # sufficient way to inspect connected/cached graph inputs.
            node = self.straight_setup['articulation_controller']
            binding_names = list(og.Controller.get(og.Controller.attribute(node+'.inputs:jointNames')))
            robot_path = str(og.Controller.get(og.Controller.attribute(node+'.inputs:robotPath')))
            if binding_names != self.straight_setup['joint_names_controller_order'] or robot_path != self.straight_setup['articulation_root']:
                raise RuntimeError('Evaluated articulation wheel binding differs from V66 asset contract; no movement allowed.')
            began = last = float(self.simulation_time())
            previous = np.array(self.encoders.read())
            stable = 0.
            self.status('Waiting >=3 simulation seconds and 1 second of quiet wheel encoders...')
            while stable < 1. or self.simulation_time()-began < 3.:
                self.zero()
                await self.app.next_update_async()
                if self.closed:
                    return
                if self.timeline.is_stopped():
                    raise RuntimeError('Stopped during settling.')
                now = float(self.simulation_time())
                q = np.array(self.encoders.read())
                stable = stable+max(0., now-last) if np.linalg.norm(wrap(q-previous)) < .002 else 0.
                previous, last = q, now
                if now-began > 20:
                    raise RuntimeError('Wheel encoders did not settle in 20 simulation seconds.')
            self.odom = WheelOdom(self.cfg['body']['wheel_radius'], self.cfg['body']['track'])
            now = float(self.simulation_time())
            self.odom.update(*self.encoders.read(), now)
            self.odom_history.append((now, self.odom.pose.copy()))
            mount, self.display = capture_mount_and_display(self.stage, self.paths, self.cfg)
            self.scanner.mount = mount
            self.cfg['sensor_origin_front_m'] = mount[3, :3].tolist()
            # Attach the existing body view before the first stop observation;
            # scan generation is not required to initialize this interlock.
            self.scanner.body_view=view.create_rigid_body_view(self.paths['base'])
            if self.scanner.body_view.count!=1:
                raise RuntimeError('Expected exactly one base body for stop verification')
            self.physics_snapshot = snapshot_effective_physics(self)
            graph = self.physics_snapshot['graph']
            wheel_attr = self.og.Controller.attribute(self.paths['controller']+'.inputs:maxWheelSpeed')
            if not wheel_attr.is_valid():
                raise RuntimeError('Cannot inspect maxWheelSpeed on the active DifferentialController.')
            self.max_wheel_speed = float(self.og.Controller.get(wheel_attr))
            if math.isnan(self.max_wheel_speed) or self.max_wheel_speed < 0:
                raise RuntimeError('Invalid effective maxWheelSpeed; refusing an unbounded fallback.')
            for key, expected in [('wheelRadius', self.cfg['body']['wheel_radius']), ('wheelDistance', self.cfg['body']['track'])]:
                actual = graph.get(key)
                if actual is None or not math.isfinite(float(actual)) or abs(float(actual)-expected) > 1e-4:
                    raise RuntimeError(f'Effective graph {key}={actual} differs from software {expected}; correct binding/config before movement.')
            write_json(self.output/'effective_physics.json', self.physics_snapshot)
            write_json(self.output/'sensor_setup.json', self.scanner.identity_setup)
            self.clock_audit=PhysicsClockAudit(1./float(self.cfg['physics_hz']),now)
            self.ready = True
            self.mode = 'STARTUP'
            self.callbacks.append(SimulationManager.register_callback(self.physics_post, event=SimulationEvent.PHYSICS_POST_STEP))
            self.callbacks.append(SimulationManager.register_callback(self.physics_pre, event=SimulationEvent.PHYSICS_PRE_STEP))
            self.status('Validating one common physics clock and wheel/body standstill before manual motion...')
            calibration_start=now
            while not (self.clock_audit.ready and self.stop_monitor.confirmed(float(self.simulation_time()))):
                await self.app.next_update_async()
                if self.physics_error:raise RuntimeError(self.physics_error)
                if self.closed or self.timeline.is_stopped():
                    raise RuntimeError('Session ended before clock/standstill validation')
                if self.simulation_time()-calibration_start>5.:
                    raise RuntimeError('Initial body/wheel standstill not verified; inspect stop_tracking.json')
            self.mode='MANUAL'
            await self.update(float(self.simulation_time()),0.)
            self.status('Manual ready. Test four directions; then Start NEW map. A is never read from world pose.')
            last = float(self.simulation_time())
            while not self.closed:
                await self.app.next_update_async()
                if not self.window.visible:
                    self.close()
                    break
                if omni.usd.get_context().get_stage() != self.stage:
                    raise RuntimeError('Stage changed; session ended and pending results discarded.')
                if self.timeline.is_stopped():
                    self.status('Timeline stopped by operator; exporting interruption diagnostics, not claiming a verified physical stop.')
                    break
                if not self.timeline.is_playing():
                    self.action = None
                    self.manual_target = self.nav_target = (0., 0.)
                    self.keys.clear()
                    self.manual.stop()
                    self.zero()
                    if self.mode in ('NAV', 'VERIFY', 'PLAN'):
                        self.control_epoch += 1
                        self.mode = 'READY'
                    last = float(self.simulation_time())
                    continue
                now = float(self.simulation_time())
                if now < last:
                    raise RuntimeError('Physics clock moved backwards.')
                if now > last:
                    await self.update(now, now-last)
                    last = now
        except asyncio.CancelledError:
            self.last_fault = 'Session cancelled by restart'
            raise
        except Exception as exc:
            self.last_fault = type(exc).__name__+': '+str(exc)
            self.status('Session stopped: '+self.last_fault)
            traceback.print_exc()
        finally:
            if self.export_stop_evidence is None:
                try:self.export_stop_evidence=self.stop_monitor.snapshot(float(self.simulation_time()))
                except Exception:self.export_stop_evidence=dict(confirmed=False,reason='No valid final stop evidence')
            self.closing = True
            self.manual_target = self.nav_target = (0., 0.)
            try:
                self.zero()
            except Exception as exc:
                self.cleanup_errors.append('motor zero: '+str(exc))
            self.ready = False
            for callback in self.callbacks:
                try:
                    SimulationManager.deregister_callback(callback)
                except Exception:
                    pass
            self.callbacks.clear()
            # Do not cancel to_thread futures: wait until their underlying work ends.
            for job in (self.worker, self.plan_job, self.io_job):
                if job is not None:
                    try:
                        value = await asyncio.shield(job)
                        if job is self.worker:
                            self.consumed.append(value[1])
                    except (asyncio.CancelledError, Exception):
                        pass
            if self.sub is not None:
                try:
                    self.input.unsubscribe_to_keyboard_events(self.keyboard, self.sub)
                except Exception as exc:
                    self.cleanup_errors.append('keyboard unsubscribe: '+str(exc))
                self.sub = None
            try:
                # Stop first; removing collision/scene layers during live simulation is unsafe.
                self.timeline.stop()
                await self.app.next_update_async()
            except Exception as exc:
                self.cleanup_errors.append('timeline stop: '+str(exc))
            try:
                self.restore_session()
            except Exception as exc:
                self.cleanup_errors.append('session restore: '+str(exc))
                write_json(self.output/'session_restore.json', dict(errors=self.cleanup_errors))
            try:
                await asyncio.to_thread(self.export_session)
            except Exception as exc:
                self.status('Export error: '+str(exc))
                traceback.print_exc()
            if self.label is not None:
                self.label.text = f'Session ended; motors zero.\n{self.last_fault or self.navigation_result}\nZIP: {self.output.with_suffix(".zip")}\nRerun the entry after Stop for a new session.'

    def restore_session(self):
        # Remove exactly the layers created by this session, never unrelated layers.
        from isaac_io import _NAV_PHYSX_LIDAR_SESSION_LAYERS
        known = _NAV_CLOCK_SESSION_LAYERS+_NAV_DRIVE_SESSION_LAYERS+_NAV_PHYSX_LIDAR_SESSION_LAYERS
        ours = {layer.identifier for layer in known if layer.identifier not in self.initial_session_layers}
        session = self.stage.GetSessionLayer()
        session.subLayerPaths = [path for path in session.subLayerPaths if path not in ours]
        clock_restore = restore_runtime_clock(self.stage, self.timeline, self.runtime_clock_before)
        write_json(self.output/'session_restore.json', dict(removed_layers=sorted(ours),
                    timeline_stopped=self.timeline.is_stopped(), usd_saved=False,
                    clock_restore=clock_restore, cleanup_errors=self.cleanup_errors))

    def export_session(self):
        """Never auto-promote a map. Incomplete data remains explicitly candidate."""
        if self.slam is not None and self.slam.nodes:
            if self.mapping:
                folder = self.output/'incomplete_map'
                self.slam.save(folder, dict(self.metadata(), validity='INCOMPLETE; not confirmed for navigation'))
            elif self.candidate:
                # Candidate already frozen; its files are retained without writing slam_maps.
                pass
        write_json(self.output/'summary.json', dict(revision=REVISION, error=self.last_fault,
            navigation_acceptance=self.navigation_result, sensor_frames=len(self.frames),
            dropped_sensor_queue_frames=self.sensor_queue_drops,
            map_accuracy='REQUIRES_REVIEW', candidate_map=str(self.candidate) if self.candidate else None,
            manual_compensation=False, active_recovery=False, fresh_world_pose_is_localization=False,
            final_command_v_w=self.last_control, normal_export_stop_verified=self.normal_export_verified,
            final_stop_evidence=self.export_stop_evidence))
        write_json(self.output/'stop_tracking.json', self.stop_records)
        write_json(self.output/'physics_clock_audit.json', dict(
            source='Official SimulationManager clock + callback dt; no timestamp scaling',
            setup=self.clock_setup,validated=self.clock_audit.ready if self.clock_audit is not None else False,
            rows=self.clock_audit.rows if self.clock_audit is not None else []))
        write_json(self.output/'events.json', self.events)
        write_json(self.output/'manual_stops.json', self.manual_stop_events)
        write_json(self.output/'mapping_sensor_frames.json', self.frames)
        write_json(self.output/'control_diagnostics.json', self.telemetry)
        write_json(self.output/'effective_config.json', self.cfg)
        if self.contact_recorder is not None:
            recorder = self.contact_recorder
            write_json(self.output/'wheel_contact_metadata.json', recorder.metadata())
            if recorder.samples:
                samples = recorder.samples
                np.savez_compressed(self.output/'wheel_contacts.npz',
                    physics_s=[x[0] for x in samples],
                    sensor_paths=np.asarray(recorder.paths,dtype=str),
                    net_force_world=[x[1] for x in samples],
                    floor_force_world=[x[2] for x in samples],
                    command_v_w=[x[3] for x in samples],
                    physics_dt_s=recorder.dt)
        with (self.output/'control_tracking_v66.csv').open('w', newline='', encoding='utf-8') as stream:
            fields = ['physics_s', 'mode', 'odom_x', 'odom_y', 'odom_yaw', 'command_v', 'command_w',
                      'actual_left_rad_s', 'actual_right_rad_s', 'graph_left_rad_s', 'graph_right_rad_s',
                      'body_vx_world', 'body_vy_world', 'body_wz_world', 'tilt_deg', 'control_status']
            writer = csv.writer(stream)
            writer.writerow(fields)
            for item in self.telemetry:
                body = item.get('body') or {}
                velocity = body.get('world_linear_velocity_m_s') or [None]*3
                angular = body.get('world_angular_velocity_rad_s') or [None]*3
                writer.writerow([item['physics_s'], item['mode'], *item['odom'], *item['command_v_w'],
                    *(item.get('actual_logical_wheel_velocities_rad_s') or [None]*2),
                    *(item.get('graph_wheel_targets_rad_s') or [None]*2), *velocity[:2], angular[2],
                    body.get('tilt_deg'), item['control_status']])
        if self.consumed:
            c = self.consumed
            np.savez_compressed(self.output/'consumed_slam_inputs.npz',
                offsets=np.r_[0, np.cumsum([len(x['points']) for x in c])], points=np.vstack([x['points'] for x in c]),
                scan_time=[x['time'] for x in c], odom=[x['odom'] for x in c],
                fitted_pose=[x['fitted_pose'] for x in c], accepted=[x['accepted'] for x in c],
                rmse=[x['rmse'] for x in c], revision=[x['revision'] for x in c],
                match_wall_seconds=[x['wall_seconds'] for x in c])
        raw = getattr(self, 'raw_scans', [])
        if raw:
            np.savez_compressed(self.output/'mapping_sensor_samples.npz', timestamp_ns=[round(x[0]*1e9) for x in raw],
                azimuth_deg=[x[1] for x in raw], ranges_m=[x[2] for x in raw],
                hit_states=[x[3] for x in raw], prim_paths=[x[4] for x in raw])
        sources = self.output/'runtime_sources'
        sources.mkdir(exist_ok=True)
        for name in ['v66_app.py', 'v66_control.py', 'v66_io.py', 'v66_bootstrap.py', 'isaac_io.py',
                     'slam_core.py', 'occupancy.py', 'rpp_core.py', 'amr_planner_core.py', 'path_smoothing.py', 'v66_nav_config.json', 'V66_VehicleAudit.py']:
            shutil.copy2(self.project/name, sources/name)
        with zipfile.ZipFile(self.output.with_suffix('.zip'), 'w', zipfile.ZIP_DEFLATED) as archive:
            for path in self.output.rglob('*'):
                if path.is_file():
                    archive.write(path, path.relative_to(self.output))
        print('[AMR V66] SESSION EXPORTED:', self.output.with_suffix('.zip'))


