# Isaac Sim Script Editor entry. Upload files together under this python_nav directory.
import asyncio,sys,importlib,traceback,hashlib,json,time
from pathlib import Path
PROJECT=Path('/isaac-sim/.local/share/ov/data/AMR_Project/1019_Scene/Collected_sickroom_with_hand_colaTest/python_nav')
if str(PROJECT) not in sys.path:sys.path.insert(0,str(PROJECT))

def load_source_module(name,path):
    """Execute the current .py bytes, bypassing cached modules and .pyc files."""
    import importlib.util,linecache
    path=Path(path).resolve()
    source=path.read_bytes()
    code=compile(source,str(path),'exec')
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec)
    prior=sys.modules.get(name)
    sys.modules[name]=module
    linecache.cache.pop(str(path),None)
    try:exec(code,module.__dict__)
    except BaseException:
        if prior is None:sys.modules.pop(name,None)
        else:sys.modules[name]=prior
        raise
    return module

async def boot(old_tasks):
    # Keep a persistent report even when initialization fails before the main UI.
    import carb
    report_path=PROJECT/'AMR_StartupDiagnostic.log'
    diagnostic={'lines':[],'label':None}
    globals()['_slam_startup_diagnostic']=diagnostic
    def report(message,level='info'):
        line=time.strftime('%H:%M:%S')+' '+str(message)
        diagnostic['lines'].append(line)
        print('[AMR startup]',line,flush=True)
        log=getattr(carb,{'info':'log_info','warning':'log_warn','error':'log_error'}[level])
        log('[AMR startup] '+line)
        try:report_path.write_text('\n'.join(diagnostic['lines'])+'\n',encoding='utf-8')
        except OSError as exc:carb.log_warn('Cannot write startup report: '+str(exc))
        if diagnostic['label'] is not None:diagnostic['label'].text='\n'.join(diagnostic['lines'][-8:])
    try:
        report('Revision 2026-10-25 continuous-drive-v6; project='+str(PROJECT))
        for task in old_tasks:
            if task is not None and not task.done():
                task.cancel()
                try:await task
                except (asyncio.CancelledError,Exception):pass
        import omni.ui as ui
        prior=globals().get('_slam_startup_window')
        if prior is not None:prior.destroy()
        window=ui.Window('AMR | Startup diagnostic',width=650,height=320)
        globals()['_slam_startup_window']=window
        with window.frame:
            with ui.VStack():
                ui.Label('Startup report: '+str(report_path),word_wrap=True,height=50)
                diagnostic['label']=ui.Label('Loading...',word_wrap=True)
        import omni.usd
        from pxr import UsdPhysics
        stage=omni.usd.get_context().get_stage()
        if stage is None:raise RuntimeError('No open USD stage')
        report('Stage: '+stage.GetRootLayer().identifier)
        report('PhysicsScene prims: '+str([str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.Scene)]))
        config=json.loads((PROJECT/'slam_config.json').read_text(encoding='utf-8-sig'))
        for role,path in config.get('paths',{}).items():
            report('Configured '+role+': '+str(path)+'; exists='+str(bool(path and stage.GetPrimAtPath(path).IsValid())))
        overrides={}
        configured=config.get('paths',{}).get('controller')
        if not configured or not stage.GetPrimAtPath(configured).IsValid():
            controllers=[]
            for prim in stage.Traverse():
                type_attr=prim.GetAttribute('node:type')
                node_type=type_attr.Get() if type_attr.IsValid() else None
                is_controller=('differentialcontroller' in str(node_type).lower() or
                               prim.GetName().lower()=='differentialcontroller')
                if (is_controller and prim.GetAttribute('inputs:linearVelocity').IsValid() and
                        prim.GetAttribute('inputs:angularVelocity').IsValid()):
                    controllers.append(str(prim.GetPath()))
            report('Actual differential controller candidates: '+str(controllers))
            if len(controllers)!=1:
                raise RuntimeError('Configured controller is missing; expected exactly one existing DifferentialController. Set paths.controller using the reported candidates.')
            overrides['controller']=controllers[0]
            report('Using unique existing controller for this session: '+controllers[0])
        # Reload the IO boundary too: reloading only the app retains old imported classes.
        for name in ('isaac_io','isaac_slam_app'):
            source=PROJECT/(name+'.py')
            data=source.read_bytes()
            revision=('2026-10-25-shared-view-v3' if name=='isaac_io' else '2026-10-25-continuous-drive-v6')
            if ("RUNTIME_REVISION = '"+revision+"'").encode() not in data:
                raise RuntimeError('Outdated file on disk: '+str(source)+'. Expected '+revision+'. Upload and save the matching patch file before Run.')
            load_source_module(name,source)
            report('Source loaded: '+str(source)+'; sha256='+hashlib.sha256(data).hexdigest()[:16])
        isaac_slam_app=sys.modules['isaac_slam_app']
        source=Path(isaac_slam_app.__file__)
        report('Loaded app: '+str(source)+'; sha256='+hashlib.sha256(source.read_bytes()).hexdigest()[:16])
        app=isaac_slam_app.Application(PROJECT,path_overrides=overrides)
        globals()['_slam_application']=app
        original_status=app.status
        def traced_status(message):
            report(message)
            original_status(message)
        app.status=traced_status
        report('Entering Application.run')
        await app.run()
        report('Application.run returned; closed='+str(app.closed))
    except asyncio.CancelledError:
        report('Task cancelled (rerun, external cancellation, or shutdown).')
        raise
    except Exception:
        report('STARTUP FAILED:\n'+traceback.format_exc(),level='error')

_old=tuple(globals().get(n) for n in ('_slam_panel_task','_python_nav_panel_task','_amr_hybrid_task','_agv_navigation_task','_agv_follow_task','_agv_map_task'))
_slam_panel_task=asyncio.ensure_future(boot(_old))
