"""Isaac Sim Script Editor entry: STOP -> RUN -> PLAY. Run this file only."""
import asyncio
import sys
import types
import traceback
from pathlib import Path

PROJECT=Path('/isaac-sim/.local/share/ov/data/AMR_Project/1019_Scene/Collected_sickroom_with_hand_colaTest/python_nav/V66_Python_Navigation')
MODULES=('occupancy','slam_core','amr_planner_core','rpp_core','isaac_io',
         'V66_VehicleAudit','v66_control','v66_io','v66_bootstrap','v66_app')


async def boot(previous_tasks,previous_apps,manual_panel):
    if manual_panel is not None:
        manual_panel.close()
        task=getattr(manual_panel,'task',None)
        if task is not None and not task.done():await task
    for app in previous_apps:
        if app is not None:
            try:app.close()
            except Exception:pass
    for task in previous_tasks:
        if task is None or task is asyncio.current_task() or task.done():continue
        task.cancel()
        try:await task
        except (asyncio.CancelledError,Exception):pass
    try:
        missing=[str(PROJECT/(m+'.py')) for m in MODULES if not (PROJECT/(m+'.py')).is_file()]
        if missing:raise RuntimeError('Missing files. Upload the whole V66 package; check PROJECT: '+str(missing))
        if not (PROJECT/'v66_nav_config.json').is_file():raise RuntimeError('Missing v66_nav_config.json')
        # Load this directory's exact sources, avoiding old modules in sys.modules
        # and stale pyc. Worker tasks from the previous session have ended first.
        sys.path.insert(0,str(PROJECT)) if str(PROJECT) not in sys.path else None
        for name in MODULES:
            path=PROJECT/(name+'.py')
            module=types.ModuleType(name);module.__file__=str(path)
            sys.modules[name]=module
            exec(compile(path.read_text(encoding='utf-8-sig'),str(path),'exec'),module.__dict__)
        app=sys.modules['v66_app'].Application(PROJECT)
        globals()['_v66_application']=app
        await app.run()
    except asyncio.CancelledError:raise
    except Exception:
        traceback.print_exc()
        print('[AMR V66] Startup failed. Read the traceback above; no success is implied.')


_names=('_v66_navigation_task','_v105_task','_amr_mapping_task','_amr_one_shot_task',
        '_slam_panel_task','_python_nav_panel_task','_amr_hybrid_task','_agv_navigation_task',
        '_agv_nav_task','_agv_follow_task','_agv_map_task','_agv_test_task','_agv_goal_task','_actual_motion_task')
_old=tuple(globals().get(n) for n in _names)
_apps=tuple(globals().get(n) for n in ('_v66_application','_v105_application','_slam_application'))
_v66_navigation_task=asyncio.ensure_future(boot(_old,_apps,globals().get('_v66_manual_panel')))
