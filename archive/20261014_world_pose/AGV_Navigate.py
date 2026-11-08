# AGV_Navigate.py -- run in Isaac Sim Script Editor, then press Play.
import asyncio
import math
import heapq
import time
from pathlib import Path
import numpy as np
import omni.usd
import omni.timeline
import omni.kit.app
import omni.graph.core as og
from pxr import Usd, UsdGeom, UsdPhysics

ROOT = '/World/TZAGV_B02_vendor_v4/TZAGV_B02_root'
BASE = ROOT + '/base_link'
LIDAR = BASE + '/Lidar_2D'
CONTROLLER = '/World/AGV_DifferentialGraph/DifferentialController'
FOLDER = Path('/isaac-sim/.local/share/ov/data/AMR_Project')
FRONT_AXIS_SIGN = -1  # physical front is the lidar end, base_link -X
LINEAR_COMMAND_SIGN = -1  # positive controller input is assumed to drive +X
RESOLUTION = 0.05
ROBOT_RADIUS = 0.57
MAX_SPEED = 1.4
MAX_TURN = 0.8
GOAL_XY = (-0.516, -0.515)  # e.g. (1.0, 2.0), WORLD coordinates in metres
PREVIEW_ONLY = False  # True: draw/export plan without driving
WALL_DISTANCE = 0.90  # vehicle centre to wall; not lidar to wall


def inflate(mask, radius):
    result = mask.copy()
    h, w = mask.shape
    n = int(math.ceil(radius / RESOLUTION))
    for dy in range(-n, n + 1):
        for dx in range(-n, n + 1):
            if math.hypot(dx, dy) * RESOLUTION > radius:
                continue
            sy = slice(max(0, -dy), min(h, h - dy))
            sx = slice(max(0, -dx), min(w, w - dx))
            ty = slice(max(0, dy), min(h, h + dy))
            tx = slice(max(0, dx), min(w, w + dx))
            result[ty, tx] |= mask[sy, sx]
    return result


def search(free, start):
    costs = {start: 0.0}
    parents = {}
    queue = [(0.0, start)]
    h, w = free.shape
    while queue:
        cost, cell = heapq.heappop(queue)
        if cost > costs[cell]:
            continue
        y, x = cell
        for dy, dx in ((0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)):
            ny, nx = y + dy, x + dx
            if not (0 <= ny < h and 0 <= nx < w and free[ny,nx]):
                continue
            if dy and dx and not (free[y,nx] and free[ny,x]):
                continue
            nc = cost + math.hypot(dx,dy) * RESOLUTION
            if nc < costs.get((ny,nx), float('inf')):
                costs[(ny,nx)] = nc
                parents[(ny,nx)] = cell
                heapq.heappush(queue, (nc, (ny,nx)))
    return costs, parents


async def navigate():
    stage = omni.usd.get_context().get_stage()
    timeline = omni.timeline.get_timeline_interface()
    app = omni.kit.app.get_app()
    linear = og.Controller.attribute(CONTROLLER + '.inputs:linearVelocity')
    angular = og.Controller.attribute(CONTROLLER + '.inputs:angularVelocity')
    if stage is None or not stage.GetPrimAtPath(BASE).IsValid():
        raise RuntimeError('AGV base_link not found.')
    if not linear.is_valid() or not angular.is_valid():
        raise RuntimeError('DifferentialController inputs not found.')
    units = UsdGeom.GetStageMetersPerUnit(stage)

    def speed(v=0.0, w=0.0):
        og.Controller.set(linear, float(LINEAR_COMMAND_SIGN*v))
        og.Controller.set(angular, float(w))

    def matrix(path):
        return np.array(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path)), dtype=float)

    def pose():
        m = matrix(BASE)
        heading = math.atan2(m[0,1], m[0,0]) + (math.pi if FRONT_AXIS_SIGN < 0 else 0)
        return m[3,:3] * units, (heading+math.pi) % (2*math.pi)-math.pi

    reader = None
    try:
        speed()
        lidar_prim = stage.GetPrimAtPath(LIDAR)
        if not lidar_prim.IsValid():
            raise RuntimeError('Lidar_2D missing.')
        # Apply before Play; per-frame instantaneous scan avoids using a moving
        # sensor's single latest pose for an entire accumulated revolution.
        for name, value in (
            ('omni:sensor:Core:instantLidar', True),
            ('omni:sensor:Core:accumulateOutputs', False),
        ):
            attr = lidar_prim.GetAttribute(name)
            if not attr.IsValid() or not attr.Set(value):
                raise RuntimeError(f'Cannot configure {name}.')
        print('AGV Navigate revision: 2026-10-14 returned-angle check corrected.')
        print('Front=-X; linear command sign=-1; instantaneous per-frame lidar enabled.')
        print('Waiting for Play...')
        while not timeline.is_playing():
            await app.next_update_async()
        print('Waiting for vehicle to land and settle...')
        began = timeline.get_current_time()
        stable = 0.0
        previous, _ = pose()
        last = began
        while stable < 1.0 or timeline.get_current_time() - began < 2.5:
            speed()
            await app.next_update_async()
            if timeline.is_stopped():
                raise RuntimeError('Simulation stopped.')
            now = timeline.get_current_time()
            current, _ = pose()
            dt = max(0.0, now-last)
            stable = stable + dt if np.linalg.norm(current-previous) < max(0.0005, dt*0.01) else 0.0
            previous, last = current, now
            if now-began > 20:
                raise RuntimeError('Vehicle did not settle within 20 simulation seconds.')
        position, yaw = pose()
        floor = stage.GetPrimAtPath('/World/Collision/floor')
        collision = stage.GetPrimAtPath('/World/Collision')
        if not floor.IsValid() or not collision.IsValid():
            raise RuntimeError('Expected /World/Collision/floor and collision meshes.')
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default','render','proxy'])
        bounds = cache.ComputeWorldBound(floor).ComputeAlignedRange()
        lower = np.array(bounds.GetMin()) * units
        upper = np.array(bounds.GetMax()) * units
        origin = lower[:2] - 0.1
        shape = np.ceil((upper[:2]-origin+0.1)/RESOLUTION).astype(int)
        if np.any(shape > 1000) or np.any(shape < 10):
            raise RuntimeError('Unexpected floor size; check scene units.')
        xx, yy = np.meshgrid(origin[0]+(np.arange(shape[0])+0.5)*RESOLUTION,
                             origin[1]+(np.arange(shape[1])+0.5)*RESOLUTION)
        extent = UsdGeom.Boundable(floor).GetExtentAttr().Get()
        if extent is None:
            raise RuntimeError('Floor extent missing.')
        fm = matrix(str(floor.GetPath()))
        inv = np.linalg.inv(fm)
        world = np.stack((xx, yy, np.full_like(xx, (lower[2]+upper[2])/2)), axis=-1)/units
        local = world @ inv[:3,:3] + inv[3,:3]
        inside = ((local[:,:,0] >= extent[0][0]) & (local[:,:,0] <= extent[1][0]) &
                  (local[:,:,1] >= extent[0][1]) & (local[:,:,1] <= extent[1][1]))
        blocked = ~inside
        walls = []
        count = 0
        obstacle_details = []
        # Conservative world AABBs: includes low furniture unseen by a horizontal lidar.
        print('Planning with direct simplified meshes under /World/Collision.')
        print('Nested sickroomOBJ reconstruction meshes are not used as solid AABB obstacles.')
        for prim in collision.GetChildren():
            if not prim.IsA(UsdGeom.Mesh) or prim == floor:
                continue
            if prim.GetName().lower() in ('ceiling', 'floor'):
                continue
            b = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            lo, hi = np.array(b.GetMin())*units, np.array(b.GetMax())*units
            if not np.all(np.isfinite(lo)) or not np.all(np.isfinite(hi)):
                continue
            if hi[2] < position[2]+0.02 or lo[2] > position[2]+0.40:
                continue
            blocked |= (xx >= lo[0]) & (xx <= hi[0]) & (yy >= lo[1]) & (yy <= hi[1])
            count += 1
            gap = float(np.linalg.norm(np.maximum(np.maximum(lo[:2]-position[:2], position[:2]-hi[:2]), 0)))
            obstacle_details.append((gap, str(prim.GetPath()), lo.copy(), hi.copy()))
            if prim.GetName().lower().startswith('wall'):
                walls.append((prim.GetName(), lo, hi))
        if count < 3:
            raise RuntimeError('Too few direct simplified obstacle meshes. Expected bed, cabinet and wall proxies under /World/Collision.')
        if not walls:
            raise RuntimeError('No walls crossing vehicle height found.')
        free = ~inflate(blocked, ROBOT_RADIUS + RESOLUTION*math.sqrt(2))
        def cell(p):
            c = np.floor((np.asarray(p[:2])-origin)/RESOLUTION).astype(int)
            return int(c[1]), int(c[0])
        def xy(c):
            return origin + (np.array([c[1],c[0]])+0.5)*RESOLUTION
        start = cell(position)
        if not (0 <= start[0] < free.shape[0] and 0 <= start[1] < free.shape[1] and free[start]):
            print(f'Settled base XYZ: {position}; stage metres/unit: {units}')
            print(f'Floor world bounds: {lower} to {upper}; start cell: {start}')
            print('Nearest obstacle AABB distances from vehicle centre:')
            for gap, name, lo, hi in sorted(obstacle_details, key=lambda item: item[0])[:8]:
                print(f'  {name}: distance={gap:.3f} m; bounds={lo} to {hi}')
            sy, sx = start
            if 0 <= sy < inside.shape[0] and 0 <= sx < inside.shape[1]:
                print(f'Start inside floor footprint: {bool(inside[sy,sx])}')
            print(f'Required centre clearance: {ROBOT_RADIUS + RESOLUTION*math.sqrt(2):.3f} m')
            raise RuntimeError('Start overlaps inflated collision geometry. See nearest obstacle details above.')
        costs, parents = search(free, start)
        FOLDER.mkdir(parents=True, exist_ok=True)
        def export_map(route=None):
            h, width = free.shape
            reachable = np.zeros_like(free)
            for cy, cx in costs:
                reachable[cy,cx] = True
            # SVG rows are flipped so increasing world Y goes upwards.
            palette = ['#222222','#e9aa52','#b9b9b9','#c7efd5']
            labels = np.where(blocked,0,np.where(~free,1,np.where(reachable,3,2)))
            parts = []
            for cy in range(h):
                row = labels[cy]
                changes = np.r_[0,np.flatnonzero(row[1:] != row[:-1])+1,width]
                for left,right in zip(changes[:-1],changes[1:]):
                    parts.append(f'<rect x="{left}" y="{h-1-cy}" width="{right-left}" height="1" fill="{palette[int(row[left])]}"/>')
            if route is not None:
                coords = (route-origin)/RESOLUTION
                poly = ' '.join(f'{x:.2f},{h-y:.2f}' for x,y in coords)
                parts.append(f'<polyline points="{poly}" fill="none" stroke="blue" stroke-width="1.8"/>')
                gx,gy = coords[-1]
                parts.append(f'<circle cx="{gx}" cy="{h-gy}" r="3" fill="red"/>')
            ax,ay = (position[:2]-origin)/RESOLUTION
            parts.append(f'<circle cx="{ax}" cy="{h-ay}" r="3" fill="blue"/>')
            html = '<!doctype html><meta charset="utf-8"><title>AGV navigation map</title>'
            html += '<h2>Click a green area to select B</h2><p>Green: reachable centre positions. Orange: clearance excluded. Black: obstacle/outside floor. Grey: disconnected.</p>'
            html += '<p>World coordinates, metres. Top is +Y, right is +X. Blue: planned route; red: B.</p><pre id="result">Click map for GOAL_XY</pre>'
            html += f'<svg id="map" viewBox="0 0 {width} {h}" style="width:90vw;max-height:80vh;border:1px solid black">'+''.join(parts)+'</svg>'
            html += f"<script>const ox={origin[0]},oy={origin[1]},r={RESOLUTION},h={h};"
            html += "document.getElementById('map').onclick=function(e){let p=this.createSVGPoint();p.x=e.clientX;p.y=e.clientY;p=p.matrixTransform(this.getScreenCTM().inverse());let x=ox+p.x*r,y=oy+(h-p.y)*r;document.getElementById('result').textContent='GOAL_XY = ('+x.toFixed(3)+', '+y.toFixed(3)+')';};</script>"
            (FOLDER/'AGV_NavigationMap.html').write_text(html,encoding='utf-8')
        export_map()
        options = []
        for name, lo, hi in walls:
            for c, distance in costs.items():
                if not 1.0 <= distance <= 6.0:
                    continue
                p = xy(c)
                delta = np.maximum(np.maximum(lo[:2]-p, p-hi[:2]), 0)
                clearance = np.linalg.norm(delta)
                if WALL_DISTANCE <= clearance <= WALL_DISTANCE+0.15:
                    options.append((abs(clearance-WALL_DISTANCE)+0.03*abs(distance-2.0), c, name, clearance))
        if GOAL_XY is not None:
            requested = np.asarray(GOAL_XY,dtype=float)
            if requested.shape != (2,) or not np.all(np.isfinite(requested)):
                raise RuntimeError('GOAL_XY must be two finite world coordinates.')
            goal = cell(requested)
            if goal not in costs:
                raise RuntimeError('Requested B is not reachable. Open AGV_NavigationMap.html and select a green location.')
            wall_name = 'manual B'
            clearance = min(float(np.linalg.norm(np.maximum(np.maximum(lo[:2]-xy(goal),xy(goal)-hi[:2]),0))) for _,lo,hi in walls)
        else:
            if not options:
                raise RuntimeError('No reachable wall goal. Open AGV_NavigationMap.html to select manual B.')
            _, goal, wall_name, clearance = min(options)
        cells = [goal]
        while cells[-1] != start:
            cells.append(parents[cells[-1]])
        cells.reverse()
        path = np.array([xy(c) for c in cells])
        FOLDER.mkdir(parents=True, exist_ok=True)
        np.savez(FOLDER/'AGV_NavigationPath.npz', path=path, origin=origin,
                 resolution=RESOLUTION, free=free, goal=xy(goal))
        export_map(path)
        print(f'Navigation map: {FOLDER / "AGV_NavigationMap.html"}')
        draw = None
        try:
            app.get_extension_manager().set_extension_enabled_immediate('isaacsim.util.debug_draw', True)
            from isaacsim.util.debug_draw import _debug_draw
            draw = _debug_draw.acquire_debug_draw_interface()
            draw.clear_lines()
            z = (position[2]+0.06)/units
            vertices = [(float(x/units),float(y/units),float(z)) for x,y in path]
            if len(vertices)>1:
                draw.draw_lines(vertices[:-1],vertices[1:],[(0,1,0,1)]*(len(vertices)-1),[5.0]*(len(vertices)-1))
            draw.draw_points([vertices[0],vertices[-1]],[(0,0.4,1,1),(1,0,0,1)],[14.0,18.0])
        except Exception as visual_error:
            print(f'Viewport drawing unavailable: {visual_error}; HTML map is still available.')
        print(f'Geometry obstacles: {count}; goal wall: {wall_name}')
        print(f'Start XY: {position[:2]}; goal XY: {xy(goal)}; centre-wall clearance: {clearance:.2f} m')
        print(f'Path length: {costs[goal]:.2f} m; maximum speed: {MAX_SPEED:.2f} m/s')
        if PREVIEW_ONLY:
            print('Preview only: no navigation commands sent. Set PREVIEW_ONLY=False to drive.')
            return
        trail_position = position.copy()
        app.get_extension_manager().set_extension_enabled_immediate('isaacsim.sensors.experimental.rtx', True)
        from isaacsim.sensors.experimental.rtx import LidarSensor, parse_generic_model_output_data
        from isaacsim.sensors.experimental.rtx.generic_model_output import ElementFlags
        if not stage.GetPrimAtPath(LIDAR).IsValid():
            raise RuntimeError('Lidar_2D missing.')
        global _agv_lidar_reader
        reader = globals().get('_agv_lidar_reader')
        if reader is None:
            reader = LidarSensor(LIDAR, annotators=['generic-model-output'])
            _agv_lidar_reader = reader
        last_stamp = None
        last_fresh = timeline.get_current_time()
        points = None
        scan_summary_printed = False
        last_self_count = 0
        scan_snapshot = {}
        began = timeline.get_current_time()
        index = 0
        previous_position = position.copy()
        measured_speed = 0.0
        v_previous = 0.0
        last = began
        last_report = began
        print('Waiting for fresh lidar data, then navigating...')
        while True:
            await app.next_update_async()
            if timeline.is_stopped():
                raise RuntimeError('Simulation stopped.')
            if not timeline.is_playing():
                speed()
                v_previous = 0.0
                continue
            now = timeline.get_current_time()
            actual_dt = max(0.0, now-last)
            dt = min(0.1, actual_dt)
            last = now
            p, heading = pose()
            if actual_dt > 1e-6:
                measured_speed = float(np.linalg.norm(p[:2]-previous_position[:2])/actual_dt)
            previous_position = p.copy()
            if np.linalg.norm(p[:2]-path[-1]) < 0.08:
                print('Goal reached. Vehicle stopped beside wall.')
                break
            if now-began > 120:
                raise RuntimeError('Navigation timeout.')
            raw = reader.get_data('generic-model-output')
            data = raw[0] if isinstance(raw, tuple) else raw
            if data is not None and getattr(data, 'size', 0):
                scan = parse_generic_model_output_data(data)
                stamp = int(scan.timestampNs)
                if stamp != last_stamp:
                    last_stamp, last_fresh = stamp, now
                    # This program expects Example_Rotary_2D spherical SENSOR output.
                    coord = str(scan.elementsCoordsType).upper()
                    frame = str(scan.frameOfReference).upper()
                    if 'SPHERICAL' not in coord or 'SENSOR' not in frame:
                        raise RuntimeError(f'Expected SPHERICAL/SENSOR lidar output, got {coord}/{frame}.')
                    az = np.asarray(scan.x, dtype=float).copy()
                    el = np.asarray(scan.y, dtype=float).copy()
                    ranges = np.asarray(scan.z, dtype=float).copy()
                    flags = np.asarray(scan.flags).copy()
                    scan_snapshot = dict(raw_azimuth_deg=az.copy(), raw_elevation_deg=el.copy(),
                                         raw_range_m=ranges.copy(), raw_flags=flags.copy(),
                                         lidar_matrix=matrix(LIDAR), scan_timestamp_ns=np.array(stamp))
                    # Returned angles describe output samples, not proof of emitted-ray coverage.
                    # A no-hit sector may be absent; never label it free from this count.
                    bins = np.unique(np.floor(np.mod(az[np.isfinite(az)],360.0)/5.0).astype(int))
                    if not scan_summary_printed:
                        print(f'Lidar returned-angle sectors: {len(bins)}/72 (diagnostic only, not scan coverage).')
                        print('Static proxy map remains authoritative for planned clearance; lidar adds return-based stopping.')
                        scan_summary_printed = True
                    if not (az.ndim == el.ndim == ranges.ndim == flags.ndim == 1 and
                            len(az) == len(el) == len(ranges) == len(flags)):
                        raise RuntimeError('Malformed lidar arrays. No movement.')
                    valid = ((flags & int(ElementFlags.VALID.value)) != 0) & np.isfinite(az) & np.isfinite(el) & np.isfinite(ranges) & (ranges > 0.05) & (ranges < 12)
                    if not np.any(valid):
                        raise RuntimeError('No valid lidar returns available; cannot verify live obstacle sensing.')
                    az, el, ranges = np.deg2rad(az[valid]), np.deg2rad(el[valid]), ranges[valid]
                    xyz = np.column_stack((ranges*np.cos(el)*np.cos(az), ranges*np.cos(el)*np.sin(az), ranges*np.sin(el)))
                    lm = matrix(LIDAR)
                    points = xyz @ lm[:3,:3] + lm[3,:3]*units
                    # Ignore returns on this vehicle's known rectangular body.
                    rel = points[:,:2]-p[:2]
                    bx = rel[:,0]*math.cos(heading)+rel[:,1]*math.sin(heading)
                    by = -rel[:,0]*math.sin(heading)+rel[:,1]*math.cos(heading)
                    self_hit = (np.abs(bx)<0.44) & (np.abs(by)<0.28) & (points[:,2]>=p[2]-0.02) & (points[:,2]<p[2]+0.45)
                    points = points[~self_hit]
                    last_self_count = int(np.count_nonzero(self_hit))
            if points is None or now-last_fresh > 0.5:
                speed()
                v_previous = 0.0
                if now-last_fresh > 4.0:
                    raise RuntimeError('Lidar missing or stale; navigation stopped.')
                continue
            py, px = cell(p)
            if not (0 <= py < free.shape[0] and 0 <= px < free.shape[1] and free[py,px]):
                raise RuntimeError('Vehicle left verified free area.')
            # Advance along nearby path; small lookahead avoids corner cutting.
            while index+1 < len(path) and np.linalg.norm(path[index]-p[:2]) < 0.18:
                index += 1
            target_index = index
            while target_index+1 < len(path) and np.linalg.norm(path[target_index]-p[:2]) < 0.30:
                target_index += 1
            delta = path[target_index]-p[:2]
            error = (math.atan2(delta[1],delta[0])-heading+math.pi) % (2*math.pi)-math.pi
            remaining = np.linalg.norm(path[-1]-p[:2])
            # Distance to next bend limits speed before reaching the bend.
            bend_distance = 0.0
            for j in range(index, len(path)-2):
                a, b = path[j+1]-path[j], path[j+2]-path[j+1]
                bend_distance += float(np.linalg.norm(a))
                cosine = float(np.dot(a,b)/(np.linalg.norm(a)*np.linalg.norm(b)))
                if cosine < math.cos(math.radians(20)):
                    break
            # Assumed deceleration is a tuning parameter, not verified wheel performance.
            deceleration = 0.40
            stop_limit = math.sqrt(2*deceleration*max(0.0, remaining-0.08))
            bend_limit = math.sqrt(0.25**2+2*deceleration*max(0.0,bend_distance-0.30))
            desired = min(MAX_SPEED, stop_limit, bend_limit, remaining*0.8)
            if abs(error) > math.radians(8):
                desired = min(desired, 0.35)
            if abs(error) > math.radians(25):
                desired = 0.0
            v = min(desired, v_previous+0.4*dt)
            w = float(np.clip(2.0*error, -MAX_TURN, MAX_TURN))
            # Include response delay and assumed braking distance in prediction.
            horizon = max(0.7, 0.30+max(measured_speed,v)/0.40)
            failure = {}
            # Validate swept circle against both geometry and fresh lidar.
            def clear_velocity(test_v):
                samples = max(9, int(math.ceil(max(test_v,0.05)*horizon/0.025))+1)
                for t in np.linspace(0, horizon, samples):
                    if abs(w) < 1e-5:
                        q = p[:2]+test_v*t*np.array([math.cos(heading),math.sin(heading)])
                    else:
                        q = p[:2]+test_v/w*np.array([math.sin(heading+w*t)-math.sin(heading), -math.cos(heading+w*t)+math.cos(heading)])
                    cy,cx = cell(q)
                    if not (0 <= cy < free.shape[0] and 0 <= cx < free.shape[1] and free[cy,cx]):
                        failure.update(source='geometry', prediction_seconds=float(t), position=q.copy())
                        return False
                    if len(points):
                        distances = np.linalg.norm(points[:,:2]-q, axis=1)
                        nearest = int(np.argmin(distances))
                        if distances[nearest] < ROBOT_RADIUS+0.06:
                            failure.update(source='lidar', prediction_seconds=float(t),
                                           position=q.copy(), point=points[nearest].copy(),
                                           distance=float(distances[nearest]))
                            return False
                return True
            if not clear_velocity(v):
                if not clear_velocity(0.0):
                    speed()
                    print(f'Protective stop source: {failure.get("source")}')
                    hit = failure.get('point')
                    if hit is not None:
                        delta_hit = hit-p
                        local_hit = [delta_hit[0]*math.cos(heading)+delta_hit[1]*math.sin(heading),
                                     -delta_hit[0]*math.sin(heading)+delta_hit[1]*math.cos(heading),delta_hit[2]]
                        print(f'Nearest return FRONT/LEFT/UP metres: {np.round(local_hit,4)}')
                        print(f'Nearest return WORLD metres: {np.round(hit,4)}; centre distance: {failure.get("distance",0):.4f} m')
                        if draw is not None:
                            draw.draw_points([tuple((hit/units).tolist())],[(1,1,0,1)],[24.0])
                    print(f'Prediction time: {failure.get("prediction_seconds")}; scan elements: {len(scan_snapshot.get("raw_range_m",[]))}')
                    print(f'Filtered body-projection returns: {last_self_count}')
                    print(f'Base XYZ: {p}; measured speed: {measured_speed:.3f} m/s; scan age: {now-last_fresh:.3f} s')
                    np.savez(FOLDER/'AGV_StopDiagnostic.npz', base_position=p, heading=heading,
                             lidar_points=points, measured_speed=measured_speed, path=path,
                             front_axis_sign=FRONT_AXIS_SIGN, linear_command_sign=LINEAR_COMMAND_SIGN,
                             self_filtered_count=last_self_count, **scan_snapshot)
                    raise RuntimeError('Protective clearance blocked. See source and nearest return above.')
                for candidate in (v*0.5, v*0.25, 0.0):
                    if clear_velocity(candidate):
                        v = candidate
                        break
                if v == 0.0 and abs(error) < math.radians(10):
                    print(f'Predicted path blockage: {failure}; measured speed: {measured_speed:.3f} m/s')
                    raise RuntimeError('Path blocked. Vehicle stopped; inspect diagnostic before rerunning.')
            speed(v,w)
            v_previous = v
            if now-last_report >= 1.0:
                print(f'Remaining: {remaining:.2f} m; actual: {measured_speed:.2f} m/s; command: {v:.2f} m/s, {w:.2f} rad/s')
                if draw is not None:
                    a = tuple((trail_position/units+np.array([0,0,0.07/units])).tolist())
                    b = tuple((p/units+np.array([0,0,0.07/units])).tolist())
                    draw.draw_lines([a],[b],[(0,0.5,1,1)],[4.0])
                    trail_position = p.copy()
                last_report = now
    except asyncio.CancelledError:
        print('Navigation cancelled.')
        raise
    except Exception as exc:
        print(f'Navigation stopped: {type(exc).__name__}: {exc}')
    finally:
        speed()
        print('Speed reset to zero.')


async def launch():
    # Await previous tasks so their finally blocks cannot reset a new command.
    for name in ('_agv_test_task','_agv_goal_task','_agv_map_task','_agv_follow_task','_agv_nav_task'):
        old = globals().get(name)
        if old is not None and old is not asyncio.current_task() and not old.done():
            old.cancel()
            try:
                await old
            except (asyncio.CancelledError, Exception):
                pass
    await navigate()

_agv_navigation_task_old = globals().get('_agv_navigation_task')
if _agv_navigation_task_old is not None and not _agv_navigation_task_old.done():
    _agv_navigation_task_old.cancel()

async def restart():
    if _agv_navigation_task_old is not None:
        try:
            await _agv_navigation_task_old
        except (asyncio.CancelledError, Exception):
            pass
    await launch()

_agv_navigation_task = asyncio.ensure_future(restart())



