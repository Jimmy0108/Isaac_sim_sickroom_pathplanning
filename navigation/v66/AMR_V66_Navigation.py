"""Isaac Sim Script Editor entry 1.0.2: STOP -> RUN -> PLAY.

Imports, configuration and async runner share one closure. Scheduled functions
do not rely on Script Editor preserving top-level names in their globals.
"""


def _start_v66_navigation(editor_scope):
    import asyncio
    import sys
    import types
    import traceback
    from pathlib import Path

    PROJECT = Path('/isaac-sim/.local/share/ov/data/AMR_Project/1019_Scene/Collected_sickroom_with_hand_colaTest/new_car/python_nav/V66_Python_Navigation')
    modules = ('occupancy','slam_core','path_geometry','amr_planner_core','trajectory_optimization',
               'speed_profile','tracking_safety','rpp_core','isaac_io',
               'V66_VehicleAudit','v66_control','v66_io','v66_bootstrap','path_smoothing','v66_app')
    state = globals()

    def lookup(name):
        return editor_scope.get(name, state.get(name))

    names = ('_v66_navigation_task','_v105_task','_amr_mapping_task','_amr_one_shot_task',
             '_slam_panel_task','_python_nav_panel_task','_amr_hybrid_task','_agv_navigation_task',
             '_agv_nav_task','_agv_follow_task','_agv_map_task','_agv_test_task','_agv_goal_task','_actual_motion_task')
    previous_tasks = tuple(lookup(n) for n in names)
    previous_apps = tuple(lookup(n) for n in ('_v66_application','_v105_application','_slam_application'))
    manual_panel = lookup('_v66_manual_panel')

    async def boot():
        try:
            if manual_panel is not None:
                manual_panel.close()
                task = getattr(manual_panel, 'task', None)
                if task is not None and not task.done():
                    await task
            for old_app in previous_apps:
                if old_app is not None:
                    try:
                        old_app.close()
                    except Exception:
                        pass
            for task in previous_tasks:
                if task is None or task is asyncio.current_task() or task.done():
                    continue
                task.cancel()
                try:
                    await task
                except (asyncio.CancelledError, Exception):
                    pass

            missing = [str(PROJECT/(m+'.py')) for m in modules if not (PROJECT/(m+'.py')).is_file()]
            if missing:
                raise RuntimeError('Missing files. Upload the whole V66 package; check PROJECT: '+str(missing))
            if not (PROJECT/'v66_nav_config.json').is_file():
                raise RuntimeError('Missing v66_nav_config.json')
            if str(PROJECT) not in sys.path:
                sys.path.insert(0, str(PROJECT))
            # Each module receives an explicit global namespace. Old worker
            # tasks have finished before replacing any module objects.
            for name in modules:
                path = PROJECT/(name+'.py')
                module = types.ModuleType(name)
                module.__file__ = str(path)
                sys.modules[name] = module
                exec(compile(path.read_text(encoding='utf-8-sig'), str(path), 'exec'), module.__dict__)
            app = sys.modules['v66_app'].Application(PROJECT)
            state['_v66_application'] = app
            editor_scope['_v66_application'] = app
            print('[AMR V66] Continuous 2.0 loaded:', PROJECT)
            await app.run()
        except asyncio.CancelledError:
            raise
        except Exception:
            traceback.print_exc()
            print('[AMR V66] Startup failed. Read the traceback above; no success is implied.')

    task = asyncio.ensure_future(boot())
    state['_v66_navigation_task'] = task
    editor_scope['_v66_navigation_task'] = task


_start_v66_navigation(locals())
