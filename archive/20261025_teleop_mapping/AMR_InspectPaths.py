# Read-only scene path inspection. Run this if auto-discovery reports ambiguity.
import omni.usd
stage=omni.usd.get_context().get_stage()
if stage is None:
    print('Open a scene first.')
else:
    print('STAGE:',stage.GetRootLayer().identifier)
    for p in stage.Traverse():
        if p.GetName() in ('base_link','Lidar_2D','DifferentialController','floor') or p.GetTypeName()=='OmniLidar':
            print(p.GetName(),p.GetTypeName(),str(p.GetPath()))
    print('No movement commands or scene edits were made.')
