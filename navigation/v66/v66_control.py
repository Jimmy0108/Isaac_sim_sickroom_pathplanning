"""Pure manual-drive helpers: no localization, heading correction or simulator IO.

All velocities use the logical physical-front frame: +v forward, +w left.
Zero targets are immediate; verified physical standstill is a separate state.
An external stop interlock can hold a direction/motion-kind change while the
body is still coasting. It never supplies a steering correction.
"""
import math
from collections import deque
import numpy as np


def _finite(*values):
    if not all(math.isfinite(float(v)) for v in values):
        raise ValueError('Nonfinite manual drive value')


def _kind(v, w):
    if v == 0. and w == 0.: return ('idle', 0, 0)
    return ('arc' if v and w else 'linear' if v else 'spin',
            int(math.copysign(1, v)) if v else 0,
            int(math.copysign(1, w)) if w else 0)


def _toward(value, target, dt, acceleration, deceleration):
    rate = acceleration if abs(target) > abs(value) else deceleration
    return value + float(np.clip(target-value, -rate*dt, rate*dt))


class ManualDrive:
    """Ramped direct commands with a measured-stop transition between directions.

    ``step(target, dt, measured_v, measured_w, now)`` returns logical ``v,w,status``.
    No pose, heading error or SLAM status is accepted by this interface.
    """
    def __init__(self, acceleration=.3, deceleration=.5, angular_acceleration=1.2,
                 max_v=.9, max_w=.9, quiet_seconds=.3):
        _finite(acceleration, deceleration, angular_acceleration, max_v, max_w, quiet_seconds)
        if min(acceleration, deceleration, angular_acceleration, max_v, max_w) <= 0 or quiet_seconds < 0:
            raise ValueError('Invalid manual drive limits')
        self.acceleration = float(acceleration)
        self.deceleration = float(deceleration)
        self.angular_acceleration = float(angular_acceleration)
        self.max_v, self.max_w = float(max_v), float(max_w)
        self.quiet_seconds = float(quiet_seconds)
        self.stop()

    def stop(self):
        self.v = self.w = 0.
        self.active_kind = ('idle', 0, 0)
        self.pending_kind = None
        self.quiet_since = None
        self.last_time = None
        self.pending_since = None

    def step(self, target, dt, measured_v, measured_w, now, stop_confirmed=None):
        if len(target) != 2:
            raise ValueError('Expected target (v,w)')
        v, w = map(float, target)
        _finite(v, w, dt, measured_v, measured_w, now)
        if dt < 0 or (self.last_time is not None and now < self.last_time-1e-9):
            self.stop()
            raise ValueError('Manual drive physics time moved backwards')
        self.last_time = float(now)
        v = float(np.clip(v, -self.max_v, self.max_v))
        w = float(np.clip(w, -self.max_w, self.max_w))
        requested = _kind(v, w)
        if requested[0] == 'idle':
            self.stop()
            return 0., 0., 'stopped'
        quiet = abs(measured_v) < .02 and abs(measured_w) < .04
        if stop_confirmed is not None:
            quiet = quiet and bool(stop_confirmed)
        changing = requested != self.active_kind and self.active_kind[0] != 'idle'
        if changing or self.pending_kind is not None or (self.active_kind[0] == 'idle' and (not quiet or stop_confirmed is not None)):
            self.v = self.w = 0.
            if self.pending_kind != requested:
                self.pending_kind = requested
                self.quiet_since = None
                self.pending_since = float(now)
            if now-self.pending_since > 8.:
                self.stop()
                raise RuntimeError('DIRECTION_STOP_NOT_VERIFIED: previous motion did not settle; new direction was not applied')
            if not quiet:
                self.quiet_since = None
            elif self.quiet_since is None:
                self.quiet_since = float(now)
            # The external interlock already requires its own continuous dwell.
            dwell = 0. if stop_confirmed is not None else self.quiet_seconds
            if self.quiet_since is None or now-self.quiet_since < dwell-1e-9:
                return 0., 0., 'waiting for measured stop before direction change'
            self.active_kind = requested
            self.pending_kind = None
            self.quiet_since = None
            self.pending_since = None
        elif self.active_kind[0] == 'idle':
            self.active_kind = requested
        self.v = _toward(self.v, v, float(dt), self.acceleration, self.deceleration) if v else 0.
        self.w = _toward(self.w, w, float(dt), self.angular_acceleration, self.angular_acceleration) if w else 0.
        return self.v, self.w, 'direct manual command'


class PhysicsClockAudit:
    """Validate a single common clock; never multiply/divide sensor timestamps."""
    def __init__(self, dt, now, required=12):
        _finite(dt, now)
        if dt <= 0:raise ValueError('Invalid physics dt')
        self.dt, self.last = float(dt), float(now)
        self.required, self.valid_steps, self.duplicates = int(required), 0, 0
        self.rows = []

    @property
    def ready(self):return self.valid_steps >= self.required

    def observe(self, now, callback_dt, timeline_s=None):
        _finite(now, callback_dt)
        delta = float(now)-self.last
        row = dict(manager_s=float(now), callback_dt_s=float(callback_dt), delta_manager_s=delta,
                   expected_dt_s=self.dt, timeline_s=timeline_s)
        self.rows.append(row)
        if not math.isclose(callback_dt,self.dt,rel_tol=1e-4,abs_tol=1e-8):
            raise RuntimeError('CLOCK_CALLBACK_DT_MISMATCH: '+str(row))
        if abs(delta) <= 1e-9:
            self.duplicates += 1
            if self.duplicates > 2:raise RuntimeError('CLOCK_NOT_ADVANCING: '+str(row))
            return False
        self.duplicates = 0
        if not math.isclose(delta,callback_dt,rel_tol=1e-4,abs_tol=1e-8):
            raise RuntimeError('CLOCK_STEP_MISMATCH: '+str(row)+'; no automatic timestamp scaling')
        self.last = float(now)
        self.valid_steps += 1
        return True


class StopMonitor:
    """Simulator-only stop interlock, NOT an odom/localization/steering input.

    Wheel-angle increments and body displacement/rotation over a short window
    reject isolated near-zero velocity reports while the chassis still slides.
    Absolute world pose is never returned to the steering/SLAM pipeline.
    """
    def __init__(self, radius, window=.12, dwell=.35, freshness=.08):
        _finite(radius,window,dwell,freshness)
        if min(radius,window,dwell,freshness)<=0:raise ValueError('Invalid stop monitor limits')
        self.radius, self.window, self.dwell, self.freshness = radius,window,dwell,freshness
        self.samples = deque()
        self.last_q = self.unwrapped = None
        self.last_time = -math.inf
        self.quiet_since = None
        self.command = (0.,0.)
        self.values = dict(reason='No fresh stop measurements')

    def note_command(self,v,w):
        self.command=(float(v),float(w))
        if v or w:self.quiet_since=None

    def invalidate_dwell(self):self.quiet_since=None

    def observe(self,now,wheel_angles,position,quaternion,linear_velocity,angular_velocity):
        q=np.asarray(wheel_angles,dtype=float);p=np.asarray(position,dtype=float)
        r=np.asarray(quaternion,dtype=float);v=np.asarray(linear_velocity,dtype=float);w=np.asarray(angular_velocity,dtype=float)
        if (q.shape!=(2,) or p.shape!=(3,) or r.shape!=(4,) or v.shape!=(3,) or w.shape!=(3,)
                or not np.isfinite(np.r_[q,p,r,v,w,now]).all() or np.linalg.norm(r)<.5):
            self.quiet_since=None
            raise RuntimeError('Invalid stop observation; cannot certify standstill')
        if now <= self.last_time:raise RuntimeError('Nonincreasing stop-sensor time')
        r=r/np.linalg.norm(r)
        self.unwrapped=q.copy() if self.last_q is None else self.unwrapped+np.arctan2(np.sin(q-self.last_q),np.cos(q-self.last_q))
        self.last_q=q.copy();self.last_time=float(now)
        self.samples.append((float(now),self.unwrapped.copy(),p.copy(),r.copy(),float(np.linalg.norm(v[:2])),float(abs(w[2]))))
        while len(self.samples)>2 and now-self.samples[1][0]>=self.window:
            self.samples.popleft()
        first,last=self.samples[0],self.samples[-1];span=last[0]-first[0]
        if span<self.window-1e-7 or len(self.samples)<3:
            self.quiet_since=None;self.values=dict(reason='Collecting stop measurement window');return
        wheel_speed=float(np.max(np.abs(last[1]-first[1]))*self.radius/span)
        body_speed=float(np.linalg.norm(last[2][:2]-first[2][:2])/span)
        rotation_speed=float(2*math.acos(np.clip(abs(np.dot(first[3],last[3])),0.,1.))/span)
        median_speed=float(np.median([x[4] for x in self.samples]))
        median_yaw_rate=float(np.median([x[5] for x in self.samples]))
        quiet=(not any(self.command) and max(wheel_speed,body_speed,median_speed)<.02
               and max(rotation_speed,median_yaw_rate)<.04)
        if not quiet:self.quiet_since=None
        elif self.quiet_since is None:self.quiet_since=float(now)
        self.values=dict(reason='Standstill dwell' if quiet else 'Motion or nonzero motor target remains',
            wheel_surface_speed_m_s=wheel_speed,body_window_speed_m_s=body_speed,
            body_window_rotation_rad_s=rotation_speed,body_median_speed_m_s=median_speed,
            body_median_yaw_rate_rad_s=median_yaw_rate,window_s=span)

    def confirmed(self,now):
        age=float(now)-self.last_time
        return (not any(self.command) and -1e-8<=age<=self.freshness and self.quiet_since is not None
                and now-self.quiet_since>=self.dwell-1e-9)

    def snapshot(self,now):
        return dict(self.values,confirmed=self.confirmed(now),sample_time_s=self.last_time,
                    age_s=float(now)-self.last_time,quiet_dwell_s=0. if self.quiet_since is None else max(0.,now-self.quiet_since),
                    required_dwell_s=self.dwell,command_v_w=self.command,
                    usage='SIMULATOR STOP INTERLOCK ONLY; never steering/SLAM pose feedback')


def wheel_limit(v, w, radius, track, limit):
    """Scale both command components equally for a positive finite wheel limit.

    Return ``(v,w,scale)``. None, zero or infinite limits mean unrestricted,
    matching the existing graph convention; NaN/negative limits are invalid.
    """
    _finite(v, w, radius, track)
    if radius <= 0 or track <= 0:
        raise ValueError('Wheel radius and track must be positive')
    if limit is None or limit == 0 or limit == math.inf:
        return float(v), float(w), 1.
    _finite(limit)
    if limit < 0:
        raise ValueError('Wheel limit must be nonnegative')
    peak = max(abs((v-w*track/2)/radius), abs((v+w*track/2)/radius))
    scale = min(1., float(limit)/peak) if peak else 1.
    return float(v)*scale, float(w)*scale, scale


def local_swept_clear(points_front, v, w, length, width, margin, horizon=.6):
    """Check a rectangular swept footprint against a current body-local scan.

    Return ``(clear, blocked_point_front_or_None)``. This never changes speed.
    Caller must separately check scan freshness and compensate old scans into
    the current front frame using wheel odometry, without map localization.
    """
    _finite(v, w, length, width, margin, horizon)
    if length <= 0 or width <= 0 or margin < 0 or horizon < 0:
        raise ValueError('Invalid footprint or prediction interval')
    points = np.asarray(points_front, dtype=float)
    if points.size == 0:
        return True, None
    if points.ndim != 2 or points.shape[1] != 2:
        raise ValueError('Expected body-local Nx2 obstacle points')
    if not np.isfinite(points).all():
        return False, None
    hl, hw = length/2+margin, width/2+margin
    perimeter_speed = abs(v)+math.hypot(hl, hw)*abs(w)
    count = max(8, int(math.ceil(perimeter_speed*horizon/.025))+1)
    for t in np.linspace(0., horizon, count):
        angle = w*t
        if abs(w) < 1e-9:
            x, y = v*t, 0.
        else:
            x, y = v/w*math.sin(angle), v/w*(1-math.cos(angle))
        delta = points-np.array([x, y])
        c, s = math.cos(angle), math.sin(angle)
        local_x = c*delta[:, 0]+s*delta[:, 1]
        local_y = -s*delta[:, 0]+c*delta[:, 1]
        hit = (abs(local_x) <= hl) & (abs(local_y) <= hw)
        if hit.any():
            candidates = points[hit]
            return False, candidates[np.argmin(np.linalg.norm(candidates, axis=1))].copy()
    return True, None

