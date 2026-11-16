"""Read-only integrity and packaging checks for a checkout with Git LFS files."""
import ast
import hashlib
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DIRECTORIES = ('navigation', 'archive', 'assets', 'evidence', 'docs', 'tools')

def main():
    errors = []
    imported = json.loads((ROOT/'docs/source_manifest.json').read_text(encoding='utf-8'))
    supplement = ROOT/'docs/supplement_manifest.json'
    if supplement.is_file():
        imported += json.loads(supplement.read_text(encoding='utf-8'))
    for entry in imported:
        path = ROOT/entry['path']
        if not path.is_file():
            errors.append(f"Missing {entry['path']}")
            continue
        data = path.read_bytes()
        if data.startswith(b'version https://git-lfs.github.com/spec/v1'):
            errors.append(f"LFS pointer only, run git lfs pull: {entry['path']}")
            continue
        elif hashlib.sha256(data).hexdigest() != entry['sha256']:
            errors.append(f"Source hash differs: {entry['path']}")

    runtime = ROOT/'navigation/v66'
    release = json.loads((runtime/'RELEASE_MANIFEST_2_0.json').read_text(encoding='utf-8'))
    for name, digest in release['runtime_sha256'].items():
        path = runtime/name
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            errors.append(f"Release mismatch: {name}")

    files = [p for directory in DIRECTORIES for p in (ROOT/directory).rglob('*')
             if p.is_file() and '__pycache__' not in p.parts]
    parsed_python = 0
    parsed_json = 0
    sensitive = re.compile(r'(?:gh[pousr]_[A-Za-z0-9]{30,}|github_pat_[A-Za-z0-9_]{30,}|AKIA[A-Z0-9]{16}|-----BEGIN (?:RSA |OPENSSH |EC )?PRIVATE KEY-----)')
    for path in files:
        try:
            if path.read_bytes().startswith(b'version https://git-lfs.github.com/spec/v1'):
                continue
            if path.suffix == '.py':
                value = path.read_text(encoding='utf-8-sig')
                ast.parse(value, filename=str(path))
                parsed_python += 1
            elif path.suffix == '.json':
                value = path.read_text(encoding='utf-8-sig')
                json.loads(value)
                parsed_json += 1
            elif path.suffix in ('.md', '.txt', '.mmd', '.usda'):
                value = path.read_text(encoding='utf-8-sig')
            else:
                continue
            if sensitive.search(value):
                errors.append(f"Potential credential, inspect locally: {path.relative_to(ROOT)}")
        except (ValueError, SyntaxError, UnicodeError) as exc:
            errors.append(f"Parse failure: {path.relative_to(ROOT)} ({type(exc).__name__})")

    cfg = json.loads((runtime/'v66_nav_config.json').read_text(encoding='utf-8'))
    geometry = json.loads((ROOT/'assets/v66/vehicle_geometry_config.json').read_text(encoding='utf-8'))
    for key in ('wheel_radius', 'track', 'length', 'width', 'height'):
        if abs(cfg['body'][key] - geometry[key]) > 1e-9:
            errors.append(f"Geometry/config mismatch: {key}")
    if abs(cfg['rpp']['wheel_radius_m'] - cfg['body']['wheel_radius']) > 1e-9:
        errors.append('RPP wheel radius mismatch')
    if abs(cfg['rpp']['wheel_track_m'] - cfg['body']['track']) > 1e-9:
        errors.append('RPP wheel track mismatch')

    print(json.dumps({
        'status': 'FAIL' if errors else 'PASS',
        'imported_sources': len(imported),
        'python_parsed': parsed_python,
        'json_parsed': parsed_json,
        'runtime_release': release['release'],
        'isaac_sim_executed': False,
        'errors': errors
    }, ensure_ascii=False, indent=2))
    return bool(errors)

if __name__ == '__main__':
    raise SystemExit(main())
