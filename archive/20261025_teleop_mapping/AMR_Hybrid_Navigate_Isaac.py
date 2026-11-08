# Run this FILE in Isaac Sim Script Editor. Upload amr_planner_core.py beside it.
import asyncio
import importlib.util
import itertools
import math
import sys
import time
from pathlib import Path
import numpy as np

PROJECT = Path('/isaac-sim/.local/share/ov/data/AMR_Project/1019_Scene/Collected_sickroom_with_hand_colaTest/python_nav')
if str(PROJECT) not in sys.path:sys.path.insert(0,str(PROJECT))
import nav_toolkit as navkit
_CONFIG=navkit.load_config(PROJECT)
BASE = '/World/TZAGV_B02_vendor_v4/TZAGV_B02_root/base_link'
CONTROLLER = '/World/AGV_DifferentialGraph/DifferentialController'
COLLISION_ROOT = '/World/Collision'
FLOOR = COLLISION_ROOT + '/floor'
LIDAR = BASE + '/Lidar_2D'
# Goal is read from the draggable GoalB marker below; edit nav_config.json for modes.
PREVIEW_ONLY = False
LIVE_LIDAR_STOP = True  # False explicitly selects STATIC-MAP-ONLY navigation
FRONT_AXIS_SIGN = -1  # lidar end of existing base_link is physical front
LINEAR_COMMAND_SIGN = -1  # assumed positive controller v moves original base_link +X
ANGULAR_COMMAND_SIGN = 1
RESOLUTION = 0.05
BODY_HEIGHT = 0.45
SAFETY_MARGIN = 0.04
MAX_SPEED = 0.40
MAX_TURN = 0.70
ACCELERATION = 0.30
ASSUMED_DECELERATION = 0.40  # tuning assumption; not experimentally verified
POSITION_TOLERANCE = 0.06
YAW_TOLERANCE = math.radians(5)
OUTPUT = PROJECT / 'outputs'
PREVIEW_ONLY = bool(globals().get('PANEL_PREVIEW_OVERRIDE',_CONFIG['preview_only']))
LIVE_LIDAR_STOP = _CONFIG['live_lidar_stop']
FRONT_AXIS_SIGN = _CONFIG['front_axis_sign']
LINEAR_COMMAND_SIGN = _CONFIG['linear_command_sign']
ANGULAR_COMMAND_SIGN = _CONFIG['angular_command_sign']
RESOLUTION = _CONFIG['map_resolution']
BODY_HEIGHT = _CONFIG['body']['height']
SAFETY_MARGIN = _CONFIG['safety_margin']
MAX_SPEED = _CONFIG['nav_speed']
MAX_TURN = _CONFIG['nav_turn']


def wrap(a):
    return (a+math.pi)%(2*math.pi)-math.pi


def convex_hull(points):
    pts=sorted(set(map(tuple,np.asarray(points,dtype=float))))
    def cross(o,a,b):return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    lo=[];hi=[]
    for p in pts:
        while len(lo)>=2 and cross(lo[-2],lo[-1],p)<=0:lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(hi)>=2 and cross(hi[-2],hi[-1],p)<=0:hi.pop()
        hi.append(p)
    return np.array(lo[:-1]+hi[:-1])


def raster_polygon(xx,yy,poly,resolution):
    # SAT: mark every square cell intersecting the convex projected mesh bounds.
    half=resolution/2
    hit=(xx+half>=poly[:,0].min())&(xx-half<=poly[:,0].max())
    hit&=(yy+half>=poly[:,1].min())&(yy-half<=poly[:,1].max())
    for a,b in zip(poly,np.roll(poly,-1,axis=0)):
        normal=np.array([b[1]-a[1],a[0]-b[0]])
        proj=poly@normal
        centre=xx*normal[0]+yy*normal[1]
        radius=half*np.abs(normal).sum()
        hit&=(centre+radius>=proj.min())&(centre-radius<=proj.max())
    return hit


def group_segments(samples):
    result=[]
    for p,v,w,dt in samples[1:]:
        if result and abs(result[-1]['v']-v)<1e-9 and abs(result[-1]['w']-w)<1e-9:
            result[-1]['points'].append(p)
        else:
            result.append(dict(v=v,w=w,points=[p]))
    for seg in result:seg['points']=np.array(seg['points'])
    return result


def follow_command(p,segment,cursor,max_speed,max_turn):
    points=segment['points'];end=points[-1]
    if abs(segment['v'])<1e-8:
        err=wrap(end[2]-p[2])
        return 0.0,float(np.clip(1.5*err,-max_turn,max_turn)),cursor,abs(err)<YAW_TOLERANCE
    remaining=float(np.linalg.norm(end[:2]-p[:2]))
    if remaining<POSITION_TOLERANCE:return 0.,0.,cursor,True
    # Progress only within this segment; never skip a reverse or spin primitive.
    while cursor+1<len(points) and np.linalg.norm(points[cursor,:2]-p[:2])<0.10:cursor+=1
    target=cursor
    while target+1<len(points) and np.linalg.norm(points[target,:2]-p[:2])<0.22:target+=1
    delta=points[target,:2]-p[:2]
    sign=1 if segment['v']>0 else -1
    motion_heading=p[2]+(math.pi if sign<0 else 0)
    error=wrap(math.atan2(delta[1],delta[0])-motion_heading)
    if abs(error)>math.radians(30):
        return 0.,float(np.clip(1.5*error,-max_turn,max_turn)),cursor,False
    # Stop at primitive boundaries: conservative feedback before next motion.
    speed=min(max_speed,abs(segment['v']),math.sqrt(2*ASSUMED_DECELERATION*max(0,remaining-POSITION_TOLERANCE)),remaining)
    speed=max(0.035,speed)
    curvature=2*math.sin(error)/max(.12,np.linalg.norm(delta))
    if abs(curvature)>.01:speed=min(speed,max_turn/abs(curvature))
    return sign*speed,float(np.clip(speed*curvature,-max_turn,max_turn)),cursor,False


def load_core():
    path=PROJECT/'amr_planner_core.py'
    if not path.is_file():raise RuntimeError(f'Missing {path}; upload both Python files.')
    name='amr_hybrid_core_runtime'
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec)
    sys.modules[name]=module
    spec.loader.exec_module(module)
    return module


async def navigate():
    global BASE, CONTROLLER, FLOOR, COLLISION_ROOT, LIDAR
    import omni.usd
    import omni.timeline
    import omni.kit.app
    import omni.graph.core as og
    from pxr import Usd,UsdGeom
    app=omni.kit.app.get_app()
    timeline=omni.timeline.get_timeline_interface()
    stage=omni.usd.get_context().get_stage()
    if stage is None:raise RuntimeError('Open the scene first.')
    paths=navkit.discover(stage,_CONFIG)
    BASE,CONTROLLER,FLOOR,COLLISION_ROOT,LIDAR=(paths[k] for k in ('base','controller','floor','collision_root','lidar'))
    linear=og.Controller.attribute(CONTROLLER+'.inputs:linearVelocity')
    angular=og.Controller.attribute(CONTROLLER+'.inputs:angularVelocity')
    if stage is None or not linear.is_valid() or not angular.is_valid():
        raise RuntimeError('Scene or DifferentialController inputs missing.')
    def command(v=0.,w=0.):
        og.Controller.set(linear,float(v*LINEAR_COMMAND_SIGN))
        og.Controller.set(angular,float(w*ANGULAR_COMMAND_SIGN))
    command()
    try:
        core=load_core()
        cfg=core.Config(length=_CONFIG['body']['length'],width=_CONFIG['body']['width'],track=_CONFIG['body']['track'],wheel_radius=_CONFIG['body']['wheel_radius'],margin=SAFETY_MARGIN,nominal_v=min(MAX_SPEED,.40),nominal_w=MAX_TURN)
        cfg.validate()
        units=UsdGeom.GetStageMetersPerUnit(stage)
        if UsdGeom.GetStageUpAxis(stage)!='Z':raise RuntimeError('This adapter expects a Z-up stage.')
        for path in (BASE,FLOOR,COLLISION_ROOT):
            if not stage.GetPrimAtPath(path).IsValid():raise RuntimeError(f'Missing scene prim: {path}')
        def matrix(path):
            return np.array(UsdGeom.XformCache().GetLocalToWorldTransform(stage.GetPrimAtPath(path)),dtype=float)
        def pose():
            m=matrix(BASE);yaw=math.atan2(m[0,1],m[0,0])+(math.pi if FRONT_AXIS_SIGN<0 else 0)
            return np.array([m[3,0]*units,m[3,1]*units,wrap(yaw)]),float(m[3,2]*units)
        # Require viewport path drawing; do not silently navigate without requested visual.
        app.get_extension_manager().set_extension_enabled_immediate('isaacsim.util.debug_draw',True)
        from isaacsim.util.debug_draw import _debug_draw
        draw=_debug_draw.acquire_debug_draw_interface()
        reader=None;parse=None;valid_flag=0
        if LIVE_LIDAR_STOP and not PREVIEW_ONLY:
            prim=stage.GetPrimAtPath(LIDAR)
            if not prim.IsValid():raise RuntimeError('Lidar_2D missing; live obstacle sensing requested.')
            app.get_extension_manager().set_extension_enabled_immediate('isaacsim.sensors.experimental.rtx',True)
            from isaacsim.sensors.experimental.rtx import LidarSensor,parse_generic_model_output_data
            from isaacsim.sensors.experimental.rtx.generic_model_output import ElementFlags
            global _agv_lidar_reader
            reader=navkit.get_reader(stage,LIDAR)
            _agv_lidar_reader=reader
            for name,value in (('omni:sensor:Core:instantLidar',True),('omni:sensor:Core:accumulateOutputs',False)):
                attr=prim.GetAttribute(name)
                if not attr.IsValid():raise RuntimeError(f'Cannot find {name}')
                if attr.Get()!=value:
                    if timeline.is_playing():raise RuntimeError('Initialize teleop panel from Stop first, or Stop/Run/Play this navigator.')
                    if not attr.Set(value):raise RuntimeError(f'Cannot set {name}')
            parse=parse_generic_model_output_data;valid_flag=int(ElementFlags.VALID.value)
        print('Hybrid A* + raster Voronoi / differential AMR. Waiting for Play...')
        while not timeline.is_playing():await app.next_update_async()
        print('Waiting for vehicle to land and settle...')
        began=last=timeline.get_current_time();stable=0.;previous,z=pose()
        while stable<1.0 or timeline.get_current_time()-began<2.5:
            command();await app.next_update_async()
            if timeline.is_stopped():raise RuntimeError('Simulation stopped during settling.')
            now=timeline.get_current_time();current,current_z=pose();dt=max(0,now-last)
            displacement=math.hypot(np.linalg.norm(current[:2]-previous[:2]),current_z-z)
            stable=stable+dt if displacement<max(.0005,dt*.01) and abs(wrap(current[2]-previous[2]))<max(.001,dt*.01) else 0.
            previous,z,last=current,current_z,now
            if now-began>20:raise RuntimeError('Vehicle failed to settle.')
        start,base_z=pose()
        floor=stage.GetPrimAtPath(FLOOR);fm=matrix(FLOOR)
        if abs(fm[0,2])+abs(fm[1,2])>1e-5:raise RuntimeError('Tilted floor unsupported by this 2D adapter.')
        bound=UsdGeom.BBoxCache(Usd.TimeCode.Default(),['default','render','proxy'],ignoreVisibility=True)
        r=bound.ComputeWorldBound(floor).ComputeAlignedRange()
        lo,hi=np.array(r.GetMin())*units,np.array(r.GetMax())*units
        floor_z=float(hi[2]);origin=lo[:2]-RESOLUTION
        shape=np.ceil((hi[:2]-origin+RESOLUTION)/RESOLUTION).astype(int)
        if np.any(shape<5) or np.any(shape>1000):raise RuntimeError('Floor size/unit abnormal or map too large.')
        xx,yy=np.meshgrid(origin[0]+(np.arange(shape[0])+.5)*RESOLUTION,origin[1]+(np.arange(shape[1])+.5)*RESOLUTION)
        inv=np.linalg.inv(fm)
        points=np.stack((xx,yy,np.full_like(xx,(lo[2]+hi[2])/2)),axis=-1)/units
        local=points@inv[:3,:3]+inv[3,:3]
        extent=UsdGeom.Boundable(floor).GetExtentAttr().Get()
        if extent is None:raise RuntimeError('Floor mesh extent missing.')
        # Keep cells wholly inside floor (floor itself is not an obstacle).
        shrink_x=RESOLUTION/(2*units)*(abs(inv[0,0])+abs(inv[1,0]))
        shrink_y=RESOLUTION/(2*units)*(abs(inv[0,1])+abs(inv[1,1]))
        inside=(local[:,:,0]>=extent[0][0]+shrink_x)&(local[:,:,0]<=extent[1][0]-shrink_x)&(local[:,:,1]>=extent[0][1]+shrink_y)&(local[:,:,1]<=extent[1][1]-shrink_y)
        occupied=~inside;names=[]
        for prim in stage.GetPrimAtPath(COLLISION_ROOT).GetChildren():
            if not prim.IsA(UsdGeom.Mesh) or prim==floor or prim.GetName().lower()=='ceiling':continue
            ext=UsdGeom.Boundable(prim).GetExtentAttr().Get()
            if ext is None:raise RuntimeError(f'Missing mesh bounds: {prim.GetPath()}')
            vertices=np.array(list(itertools.product(*[(ext[0][i],ext[1][i]) for i in range(3)])),dtype=float)
            m=matrix(str(prim.GetPath()));world=(vertices@m[:3,:3]+m[3,:3])*units
            if world[:,2].max()<base_z+.02 or world[:,2].min()>base_z+BODY_HEIGHT:continue
            poly=convex_hull(world[:,:2])
            if len(poly)<3:continue
            occupied|=raster_polygon(xx,yy,poly,RESOLUTION)
            names.append(str(prim.GetPath()))
        if len(names)<3:raise RuntimeError('Too few direct simplified obstacle meshes; check Collision hierarchy.')
        print(f'Static map: {len(names)} direct simplified meshes; nested reconstruction meshes excluded.')
        source=_CONFIG['map_source'].upper()
        if source=='SCANNED':
            file=PROJECT/'maps'/'room_map_latest.npz'
            if not file.is_file():raise RuntimeError('Saved scan map missing. Use AMR_TeleopMap.py to scan and save first.')
            with np.load(file,allow_pickle=False) as data:
                meta=__import__('json').loads(str(data['metadata_json']))
                if meta['stage_identifier']!=stage.GetRootLayer().identifier:raise RuntimeError('Scan map stage registration mismatch; rebuild or inspect map alignment.')
                lidar_height=matrix(LIDAR)[3,2]*units
                if abs(lidar_height-meta['sensor_world_z'])>.10:raise RuntimeError('Lidar height differs from captured map; rebuild after sensor placement is fixed.')
                scan_occ=data['occupied'];scan_origin=data['origin'];scan_res=float(data['resolution'])
                iy=np.floor((yy-scan_origin[1])/scan_res).astype(int)
                ix=np.floor((xx-scan_origin[0])/scan_res).astype(int)
                valid=(iy>=0)&(ix>=0)&(iy<scan_occ.shape[0])&(ix<scan_occ.shape[1])
                scan_blocked=np.ones_like(occupied)
                scan_blocked[valid]=scan_occ[iy[valid],ix[valid]]
                occupied |= scan_blocked
            print('SCANNED mode: unknown/ambiguous cells blocked; simplified static obstacles also retained.')
        elif source!='SCENE':raise RuntimeError('map_source must be SCANNED or SCENE in nav_config.json.')
        grid=core.Grid(occupied,RESOLUTION,origin,cfg)
        marker=stage.GetPrimAtPath(_CONFIG['goal_marker'])
        if not marker.IsValid():raise RuntimeError('GoalB marker missing. Create/select it in AMR_TeleopMap panel.')
        gm=matrix(_CONFIG['goal_marker'])
        goal=np.array([gm[3,0]*units,gm[3,1]*units,0.])
        goal[2]=math.atan2(gm[0,1],gm[0,0]) if _CONFIG['goal_yaw_from_marker'] else math.atan2(goal[1]-start[1],goal[0]-start[0])
        print(f'Goal from viewport marker {_CONFIG["goal_marker"]}; A is measured current robot pose.')
        OUTPUT.mkdir(parents=True,exist_ok=True)
        np.savez(OUTPUT/'scene_map.npz',occupied=grid.occupied,resolution=grid.res,origin=grid.origin)
        print(f'A={start}; B={goal}; planning in worker thread while holding zero speed...')
        job=asyncio.ensure_future(asyncio.to_thread(core.plan,grid,start,goal))
        try:
            while not job.done():
                command();await app.next_update_async()
                if timeline.is_stopped():raise RuntimeError('Simulation stopped while planning.')
            samples,stats=job.result()
        finally:
            if not job.done():job.cancel()  # worker touches no USD/controls; bounded by its timeout
        p,z=pose()
        if np.linalg.norm(p[:2]-start[:2])>.025 or abs(wrap(p[2]-start[2]))>math.radians(3):
            raise RuntimeError('Vehicle moved during planning; rerun from settled position.')
        route=np.array([row[0] for row in samples])
        np.savez(OUTPUT/'hybrid_path.npz',path=route[:,:2],poses=route,start=start,goal=goal,origin=origin,resolution=RESOLUTION,voronoi_field=grid.field,
                 nominal_v=np.array([r[1] for r in samples]),nominal_omega=np.array([r[2] for r in samples]),nominal_dt=np.array([r[3] for r in samples]))
        draw.clear_lines();draw.clear_points()  # shared Debug Draw channel; documented in README
        def ground(p):return (float(p[0]/units),float(p[1]/units),float((floor_z+.025)/units))
        vertices=[ground(p) for p in route]
        if len(vertices)>1:draw.draw_lines(vertices[:-1],vertices[1:],[(0,1,0,1)]*(len(vertices)-1),[5.]*(len(vertices)-1))
        draw.draw_points([ground(start),ground(goal)],[(0,.35,1,1),(1,0,0,1)],[16.,20.])
        # Arrow at B shows physical-front goal yaw; height is floor+2.5cm.
        tip=goal[:2]+.30*np.array([math.cos(goal[2]),math.sin(goal[2])])
        draw.draw_lines([ground(goal)],[ground(tip)],[(1,0,0,1)],[5.])
        print(f'Plan ready: {len(route)} samples; {stats["expansions"]} expansions; green=plan, blue=A/trail, red=B.')
        if PREVIEW_ONLY:
            print('Preview only. Set PREVIEW_ONLY=False to drive.');return
        segments=group_segments(samples)
        seg_index=cursor=0;began=last=timeline.get_current_time();last_report=last
        last_move=last;last_marker=last;p_previous=p.copy();progress_reference=p.copy();trail=p.copy()
        v_previous=0.;previous_command=(0.,0.);cloud=None;stamp=None;fresh=last;raw_snapshot={}
        while seg_index<len(segments):
            await app.next_update_async()
            if timeline.is_stopped():raise RuntimeError('Simulation stopped.')
            if not timeline.is_playing():
                command();v_previous=0.;previous_command=(0.,0.);last=timeline.get_current_time();p_previous=pose()[0];continue
            now=timeline.get_current_time();dt=max(0.,now-last);last=now
            p,z=pose();displacement=p[:2]-p_previous[:2]
            actual_speed=np.linalg.norm(displacement)/max(dt,1e-6)
            projected=float(np.dot(displacement,[math.cos(p[2]),math.sin(p[2])]))/max(dt,1e-6)
            if abs(previous_command[0])>.08 and actual_speed>.05 and projected*previous_command[0]<-.003:
                raise RuntimeError('Actual motion opposes commanded front. Verify LINEAR_COMMAND_SIGN at low speed.')
            p_previous=p.copy()
            if grid.collision(p):raise RuntimeError('Actual robot footprint entered static-map clearance.')
            if now-began>180:raise RuntimeError('Navigation timed out.')
            if np.linalg.norm(p[:2]-progress_reference[:2])>.025 or abs(wrap(p[2]-progress_reference[2]))>.05:
                last_move=now;progress_reference=p.copy()
            if now-last_move>12:raise RuntimeError('No measured progress for 12 simulation seconds.')
            if reader is not None:
                raw=reader.get_data('generic-model-output');data=raw[0] if isinstance(raw,tuple) else raw
                if data is not None and getattr(data,'size',0):
                    scan=parse(data);new_stamp=int(scan.timestampNs)
                    if new_stamp!=stamp:
                        stamp=new_stamp
                        if 'SPHERICAL' not in str(scan.elementsCoordsType).upper() or 'SENSOR' not in str(scan.frameOfReference).upper():
                            raise RuntimeError('Expected spherical SENSOR-frame Example_Rotary_2D data.')
                        az=np.asarray(scan.x,dtype=float).copy();el=np.asarray(scan.y,dtype=float).copy();r=np.asarray(scan.z,dtype=float).copy();flags=np.asarray(scan.flags).copy()
                        valid=((flags&valid_flag)!=0)&np.isfinite(az)&np.isfinite(el)&np.isfinite(r)&(r>.05)&(r<12)
                        if np.any(valid):
                            a,e,d=np.deg2rad(az[valid]),np.deg2rad(el[valid]),r[valid]
                            xyz=np.c_[d*np.cos(e)*np.cos(a),d*np.cos(e)*np.sin(a),d*np.sin(e)]
                            lm=matrix(LIDAR);cloud=xyz@lm[:3,:3]+lm[3,:3]*units
                            rel=cloud[:,:2]-p[:2];c,s=math.cos(p[2]),math.sin(p[2])
                            bx,by=rel[:,0]*c+rel[:,1]*s,-rel[:,0]*s+rel[:,1]*c
                            self_hit=(abs(bx)<cfg.length/2)&(abs(by)<cfg.width/2)&(cloud[:,2]>=z-.02)&(cloud[:,2]<z+BODY_HEIGHT)
                            cloud=cloud[~self_hit];fresh=now
                            raw_snapshot=dict(raw_azimuth_deg=az,raw_elevation_deg=el,raw_range_m=r,raw_flags=flags,lidar_matrix=lm)
                if cloud is None or now-fresh>.50:
                    command();v_previous=0.;previous_command=(0.,0.)
                    if now-fresh>4:raise RuntimeError('Lidar missing/stale. No motion.')
                    continue
            seg=segments[seg_index]
            v,w,cursor,done=follow_command(p,seg,cursor,MAX_SPEED,MAX_TURN)
            if done:
                command();v_previous=0.;previous_command=(0.,0.);seg_index+=1;cursor=0;last_move=now;continue
            if v*v_previous<0:v=0.
            if abs(v)>abs(v_previous):v=math.copysign(min(abs(v),abs(v_previous)+ACCELERATION*min(dt,.10)),v)
            horizon=max(.5,.25+max(abs(v),actual_speed)/ASSUMED_DECELERATION)
            safety_points=grid.edge(p,v,w,horizon)
            cause=None;hit=None
            if safety_points is None:cause='predicted static footprint collision'
            if cause is None and cloud is not None and len(cloud):
                # Grid-sized voxel reduction keeps the stop check responsive.
                voxel=np.floor(cloud[:,:2]/.025).astype(np.int64)
                _,idx=np.unique(voxel,axis=0,return_index=True);near=cloud[idx]
                near=near[np.linalg.norm(near[:,:2]-p[:2],axis=1)<math.hypot(cfg.length/2,cfg.width/2)+max(abs(v),actual_speed)*horizon+.3]
                for q in [p]+[row[0] for row in safety_points]:
                    rel=near[:,:2]-q[:2];c,s=math.cos(q[2]),math.sin(q[2])
                    bx,by=rel[:,0]*c+rel[:,1]*s,-rel[:,0]*s+rel[:,1]*c
                    mask=(abs(bx)<cfg.length/2+cfg.margin)&(abs(by)<cfg.width/2+cfg.margin)
                    if np.any(mask):cause='lidar return inside predicted rectangular footprint';hit=near[np.flatnonzero(mask)[0]];break
            if cause:
                command()
                np.savez(OUTPUT/'stop_diagnostic.npz',pose=p,base_z=z,path=route,lidar_points=cloud if cloud is not None else np.empty((0,3)),
                         cause=cause,measured_speed=actual_speed,front_axis_sign=FRONT_AXIS_SIGN,**raw_snapshot)
                if hit is not None:
                    draw.draw_points([tuple((hit/units).tolist())],[(1,1,0,1)],[24.])
                    print(f'Nearest triggering WORLD return: {hit}; base={p}; actual speed={actual_speed:.3f}')
                raise RuntimeError(cause+'; diagnostic saved. No automatic dynamic detour.')
            command(v,w);previous_command=(v,w);v_previous=v
            if now-last_marker>.2:
                draw.draw_lines([ground(trail)],[ground(p)],[(0,.4,1,1)],[3.]);trail=p.copy();last_marker=now
            if now-last_report>1:
                print(f'Segment {seg_index+1}/{len(segments)}, remaining XY={np.linalg.norm(p[:2]-goal[:2]):.2f} m; command v={v:.2f}, w={w:.2f}; actual={actual_speed:.2f} m/s')
                last_report=now
        p,z=pose()
        if np.linalg.norm(p[:2]-goal[:2])>POSITION_TOLERANCE or abs(wrap(p[2]-goal[2]))>YAW_TOLERANCE:
            raise RuntimeError('Final pose outside tolerance; not reporting arrival.')
        print('Goal reached: position and physical-front heading verified.')
    except asyncio.CancelledError:
        print('Hybrid navigation cancelled.');raise
    except Exception as exc:
        print(f'Hybrid navigation stopped: {type(exc).__name__}: {exc}')
    finally:
        command();print('Both velocity commands reset to zero.')


async def restart(previous_tasks):
    for task in previous_tasks:
        if task is not None and task is not asyncio.current_task() and not task.done():
            task.cancel()
            try:await task
            except (asyncio.CancelledError,Exception):pass
    await navigate()

_previous=tuple(globals().get(name) for name in ('_amr_hybrid_task','_agv_navigation_task','_agv_follow_task','_agv_goal_task','_agv_map_task','_agv_test_task','_agv_nav_task'))
_amr_hybrid_task=asyncio.ensure_future(restart(_previous))
