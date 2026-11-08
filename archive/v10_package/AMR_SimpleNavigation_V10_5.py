# Isaac Sim Script Editor entry: STOP -> RUN this file -> PLAY.
# Upload all files from this delivery together into the remote python_nav folder.
import asyncio
import importlib
import sys
import traceback
from pathlib import Path

PROJECT = Path(
    "/isaac-sim/.local/share/ov/data/AMR_Project/"
    "1019_Scene/Collected_sickroom_with_hand_colaTest/"
    "python_nav/V10_5_SimpleNavigation"
)

async def boot_v105(old_tasks, old_app):
    # Retire previous command owners and await finalizers before replacing modules.
    if old_app is not None:
        try:
            old_app.close()
        except Exception as exc:
            print('[V10.5] Previous app close:', exc)
    for task in old_tasks:
        if task is not None and task is not asyncio.current_task():
            if not task.done() and not getattr(old_app, 'closing', False):
                task.cancel()
            try:
                await task
            except (asyncio.CancelledError, Exception):
                pass
    if old_app is not None and getattr(old_app, 'window', None) is not None:
        try:
            old_app.window.destroy()
        except Exception:
            pass
    if not PROJECT.is_dir():
        print('[V10.5] Missing python_nav folder. Edit PROJECT to the remote folder containing all delivery files:', PROJECT)
        return
    required = ['v105_config.json', 'v105_app.py', 'v105_bootstrap.py', 'v105_control.py', 'v105_io.py',
                'isaac_io.py', 'slam_core.py', 'occupancy.py', 'rpp_core.py', 'amr_planner_core.py']
    missing = [name for name in required if not (PROJECT/name).is_file()]
    if missing:
        print('[V10.5] Upload the complete delivery; missing:', missing)
        return
    path = str(PROJECT)
    if path in sys.path:
        sys.path.remove(path)
    sys.path.insert(0, path)
    importlib.invalidate_caches()
    # Resolve imported source directly, bypassing stale pyc files and old modules.
    import types
    for name in ['occupancy', 'slam_core', 'amr_planner_core', 'rpp_core', 'isaac_io',
                 'v105_control', 'v105_io', 'v105_bootstrap', 'v105_app']:
        source = PROJECT/(name+'.py')
        module = types.ModuleType(name)
        module.__file__ = str(source)
        sys.modules[name] = module
        exec(compile(source.read_text(encoding='utf-8-sig'), str(source), 'exec'), module.__dict__)
    from v105_app import Application
    app = Application(PROJECT)
    globals()['_v105_application'] = app
    globals()['_slam_application'] = app
    try:
        await app.run()
    except asyncio.CancelledError:
        raise
    except Exception:
        print('[V10.5] Startup failed:\n'+traceback.format_exc())


_old_names_v105 = ('_v105_task', '_amr_mapping_task', '_amr_one_shot_task', '_slam_panel_task',
    '_python_nav_panel_task', '_amr_hybrid_task', '_agv_navigation_task', '_agv_nav_task',
    '_agv_follow_task', '_agv_map_task', '_agv_test_task', '_agv_goal_task', '_actual_motion_task')
_old_tasks_v105 = tuple(globals().get(name) for name in _old_names_v105)
_old_app_v105 = globals().get('_v105_application') or globals().get('_slam_application')
_v105_task = asyncio.ensure_future(boot_v105(_old_tasks_v105, _old_app_v105))


