"""Pure Python differential RPP-style tracker; V10.4 terminal-aware steering.

Inputs are estimated map pose and encoder velocity, never USD ground truth.
Footprint/map/lidar projection evaluates the final smoothed command.
"""
import math
import numpy as np
from slam_core import wrap, transform, inverse


def segments_from_samples(samples):
    groups = []
    for i, (p, v, w, dt) in enumerate(samples[1:], 1):
        kind = ('spin', 1 if w >= 0 else -1) if abs(v) < 1e-8 else ('move', 1 if v > 0 else -1)
        if groups and groups[-1]['kind'] == kind:
            groups[-1]['points'].append(np.array(p, float))
        else:
            # Include the segment start, rather than skipping the first sample.
            groups.append(dict(kind=kind, points=[np.array(samples[i-1][0], float), np.array(p, float)]))
    for g in groups:
        pts = np.asarray(g['points'])
        if g['kind'][0] == 'move':
            keep = np.r_[True, np.linalg.norm(np.diff(pts[:, :2], axis=0), axis=1) > 1e-9]
            pts = pts[keep]
        g['points'] = pts
        g['arc'] = np.r_[0., np.cumsum(np.linalg.norm(np.diff(pts[:, :2], axis=0), axis=1))]
    return groups


class RPP:
    def __init__(self, cfg):
        self.cfg = cfg
        self.groups = []
        self.index = self.cursor = 0
        self.progress = 0.
        self.transition = self.aligning = False
        self.align_stop_pending = False
        self.local_path = np.empty((0, 3))
        self.telemetry = {}
        self.reset_output()

    def reset_output(self):
        self.last_v = self.last_w = self.steering_w = 0.
        self.measured_w_filtered = 0.

    def set_path(self, samples):
        self.groups = segments_from_samples(samples)
        self.index = self.cursor = 0
        self.progress = 0.
        self.transition = self.aligning = False
        self.align_stop_pending = False
        self.reset_output()
        self.telemetry = {}
        # Kept for compatibility with fixed-yaw callers. The XY-only app removes it.
        if self.groups and self.groups[-1]['kind'][0] == 'move':
            self.groups.append(dict(kind=('spin', 1), points=np.array([samples[-1][0].copy()]), arc=np.array([0.])))

    def _zero(self, state):
        self.reset_output()
        self.local_path = np.empty((0, 3))
        self.telemetry['state'] = state
        return 0., 0., state

    def _target(self, seg, pose, look):
        pts, arc = seg['points'], seg['arc']
        if len(pts) < 2:
            return pts[-1, :2], float(np.linalg.norm(pts[-1, :2]-pose[:2])), 0., 0.
        # Search a bounded, monotonically advancing arc window, not all crossings.
        window = float(self.cfg.get('projection_search_distance', 1.0))
        stop = min(len(pts)-1, max(self.cursor+1, int(np.searchsorted(arc, self.progress+window))+1))
        best = None
        for i in range(self.cursor, stop):
            a, b = pts[i, :2], pts[i+1, :2]
            vector = b-a
            length = arc[i+1]-arc[i]
            t = float(np.clip(np.dot(pose[:2]-a, vector)/max(length*length, 1e-12), 0., 1.))
            s = max(self.progress, float(arc[i]+t*length))
            if s > arc[i+1]+1e-9:
                continue
            t = (s-arc[i])/max(length, 1e-12)
            q = a+t*vector
            distance = float(np.linalg.norm(q-pose[:2]))
            if best is None or distance < best[0]:
                best = distance, s, i
        if best is None:
            best = float(np.linalg.norm(pts[-1, :2]-pose[:2])), float(arc[-1]), len(pts)-2
        distance, self.progress, self.cursor = best
        target_s = min(float(arc[-1]), self.progress+look)
        j = min(len(pts)-2, max(0, int(np.searchsorted(arc, target_s, side='right'))-1))
        t = (target_s-arc[j])/max(float(arc[j+1]-arc[j]), 1e-12)
        target = pts[j, :2]+t*(pts[j+1, :2]-pts[j, :2])
        # Virtual carrot continues along the final tangent. The actual goal and
        # footprint collision projection remain unchanged; this is not a new goal.
        if target_s >= arc[-1]-1e-8 and self.progress+look > arc[-1]:
            tangent = pts[-1,:2]-pts[-2,:2]
            tangent /= max(float(np.linalg.norm(tangent)),1e-12)
            target = target+tangent*(self.progress+look-float(arc[-1]))

        # Curvature reference of the planned path, independent of tracking error.
        fraction = (self.progress-arc[self.cursor])/max(float(arc[self.cursor+1]-arc[self.cursor]), 1e-12)
        desired_yaw = pts[self.cursor, 2]+fraction*wrap(pts[self.cursor+1, 2]-pts[self.cursor, 2])
        span = max(target_s-self.progress, .10)
        target_yaw = pts[j, 2]+t*wrap(pts[j+1, 2]-pts[j, 2])
        path_curvature = float(wrap(target_yaw-desired_yaw)/span)
        return target, distance, float(arc[-1]-self.progress), path_curvature

    def command(self, pose, measured_v, grid, obstacles, dt, measured_w=0., stop_confirmed=None):
        c = self.cfg
        pose = np.asarray(pose, float)
        if not np.isfinite(np.r_[pose, measured_v, measured_w, dt]).all() or dt <= 0:
            return self._zero('replan: invalid tracking input')
        dt = min(float(dt), .1)
        a = dt/(float(c.get('yaw_rate_filter_s', .12))+dt)
        self.measured_w_filtered += a*(float(measured_w)-self.measured_w_filtered)
        self.telemetry = dict(lookahead=float('nan'), cross_track=float('nan'),
                              heading_error=float('nan'), raw_w=0., target_x=float('nan'), target_y=float('nan'))
        if self.align_stop_pending:
            if abs(measured_v)>.03 or abs(measured_w)>.04 or stop_confirmed is False:
                return self._zero('stopping after path alignment')
            self.align_stop_pending=False
            self.aligning=False
            self.reset_output()
        if self.transition:
            if abs(measured_v) > .03 or abs(measured_w) > .04 or stop_confirmed is False:
                return self._zero('stopping at direction/rotation boundary')
            self.index += 1
            self.cursor = 0
            self.progress = 0.
            self.transition = self.aligning = False
            self.reset_output()
        if self.index >= len(self.groups):
            return self._zero('done')
        seg = self.groups[self.index]
        goal = seg['points'][-1]
        tracking = seg['kind'][0] == 'move'
        curvature = 0.
        if not tracking:
            error = float(wrap(goal[2]-pose[2]))
            if abs(error) < c['yaw_tolerance']:
                self.transition = True
                return self._zero('segment complete')
            if abs(measured_v) > .03:
                return self._zero('stopping before planned rotation')
            v = 0.
            raw_w = c.get('rotate_gain', 1.5)*error-c.get('yaw_rate_damping', .25)*self.measured_w_filtered
            self.telemetry['heading_error'] = error
        else:
            goal_distance = float(np.linalg.norm(goal[:2]-pose[:2]))
            if goal_distance < c['position_tolerance']:
                self.transition = True
                return self._zero('segment complete')
            look = float(c.get('fixed_lookahead_m',.70)) if c.get('use_fixed_lookahead',False) else float(np.clip(c['lookahead_time']*max(abs(measured_v),abs(self.last_v)), c['min_lookahead'], c['max_lookahead']))
            target, deviation, remaining, path_curvature = self._target(seg, pose, look)
            self.telemetry.update(lookahead=look, cross_track=deviation, target_x=float(target[0]), target_y=float(target[1]))
            if deviation > .5:
                return self._zero('replan: path deviation')
            local = transform(target[None, :], inverse(pose))[0]
            sign = seg['kind'][1]
            error = float(wrap(math.atan2(target[1]-pose[1], target[0]-pose[0])-pose[2]-(math.pi if sign < 0 else 0)))
            self.telemetry['heading_error'] = error
            # Separate entry/exit thresholds prevent rotate/drive mode chatter.
            if self.aligning:
                if abs(error) < c.get('align_exit_angle', math.radians(20)):
                    self.align_stop_pending=True
                    return self._zero('stopping after path alignment')
            elif abs(error) > c.get('align_enter_angle', math.radians(55)):
                self.aligning = True
            if self.aligning:
                if abs(measured_v) > .03:
                    return self._zero('stopping before path alignment')
                v = 0.
                raw_w = c.get('rotate_gain', 1.5)*error-c.get('yaw_rate_damping', .25)*self.measured_w_filtered
            else:
                curvature = 2*float(local[1])/max(.01, float(local@local))
                speed = float(c['max_v'])
                speed = min(speed, c['max_v']/max(1., c['min_turn_radius']*abs(curvature)))
                speed = min(speed, c['max_v']*remaining/c['approach_distance'])
                speed = min(speed, math.sqrt(2*c['deceleration']*max(0., goal_distance-c['position_tolerance'])))
                if len(obstacles):
                    obs = transform(obstacles, inverse(pose))
                    clearance = float(np.min(np.linalg.norm(np.maximum(np.abs(obs)-[grid.cfg.length/2, grid.cfg.width/2], 0), axis=1)))
                    speed = min(speed, c['max_v']*max(0., clearance-grid.cfg.margin)/c['cost_scaling_distance'])
                if abs(curvature) > 1e-6:
                    speed = min(speed, c['max_w']/abs(curvature))
                    speed = min(speed, math.sqrt(c.get('max_lateral_acceleration',.35)/abs(curvature)))
                previous = abs(self.last_v) if self.last_v*sign >= 0 else 0.
                # Acceleration is gradual. Obstacle/goal deceleration may be immediate.
                v = sign*min(speed, previous+c['acceleration']*dt)
                feedforward = v*path_curvature
                raw_w = v*curvature-c.get('tracking_yaw_damping', .18)*(self.measured_w_filtered-feedforward)
        raw_w = float(np.clip(raw_w, -c['max_w'], c['max_w']))
        if not tracking or self.aligning:
            raw_w=float(np.clip(raw_w,-c.get('rotate_max_w',.45),c.get('rotate_max_w',.45)))
        alpha = dt/(float(c.get('steering_filter_s', .10))+dt)
        self.steering_w += alpha*(raw_w-self.steering_w)
        change = float(c.get('max_angular_accel', 1.2))*dt
        w = float(np.clip(self.steering_w, self.last_w-change, self.last_w+change))
        w = float(np.clip(w, -c['max_w'], c['max_w']))
        self.steering_w = w
        self.telemetry['raw_w'] = raw_w
        horizon = max(c['collision_time'], .20+max(abs(v), abs(measured_v))/c['deceleration'])
        safety_v = math.copysign(max(abs(v), abs(measured_v)), measured_v) if abs(measured_v) > .03 else v
        rollout = grid.edge(pose, safety_v, w, horizon)
        if rollout is None:
            return self._zero('replan: predicted costmap collision')
        self.local_path = np.array([pose]+[row[0] for row in rollout])
        if len(obstacles):
            for q in self.local_path:
                local = transform(obstacles, inverse(q))
                hit = (abs(local[:, 0]) < grid.cfg.length/2+grid.cfg.margin) & (abs(local[:, 1]) < grid.cfg.width/2+grid.cfg.margin)
                if hit.any():
                    return self._zero('replan: local obstacle')
        self.last_v, self.last_w = float(v), float(w)
        state = 'aligning to path' if self.aligning else 'tracking'
        self.telemetry['state'] = state
        return self.last_v, self.last_w, state
