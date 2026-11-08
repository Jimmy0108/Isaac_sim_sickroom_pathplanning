"""Optional Isaac Script Editor read-only installation check, no motor command."""
def _check_continuous_installation():
    import hashlib,json
    from pathlib import Path
    root=Path('/isaac-sim/.local/share/ov/data/AMR_Project/1019_Scene/Collected_sickroom_with_hand_colaTest/new_car/python_nav/V66_Python_Navigation')
    manifest=root/'RELEASE_MANIFEST_2_0.json'
    if not manifest.is_file():raise RuntimeError('Missing release manifest at '+str(root)+'; do not create another nested V66_Python_Navigation folder.')
    data=json.loads(manifest.read_text(encoding='utf-8'));errors=[]
    for name,sha in data['runtime_sha256'].items():
        p=root/name
        if not p.is_file():errors.append('MISSING '+name)
        elif hashlib.sha256(p.read_bytes()).hexdigest()!=sha:errors.append('DIFFERENT '+name)
    if errors:raise RuntimeError('Incomplete/mixed installation:\n'+'\n'.join(errors))
    print('[V66 2.0] INSTALLATION HASH CHECK PASS:',root)
    print('[V66 2.0] No motion or physics test performed. RUN AMR_V66_Navigation.py, then PLAY.')

_check_continuous_installation()
