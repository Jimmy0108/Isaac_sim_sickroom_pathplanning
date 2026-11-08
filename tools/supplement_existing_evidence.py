"""Copy existing research evidence, preserving bytes and recording provenance."""
import hashlib
import json
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
manifest = ROOT / 'docs/supplement_manifest.json'
entries = json.loads(manifest.read_text(encoding='utf-8')) if manifest.exists() else []

def save(data, destination, source):
    target = ROOT / destination
    if target.exists():
        if target.read_bytes() != data:
            raise RuntimeError('Refusing to replace differing evidence: ' + destination)
        return
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_bytes(data)
    entries.append(dict(path=destination, source=source, bytes=len(data),
                        sha256=hashlib.sha256(data).hexdigest()))

allowed = {'.json', '.csv', '.npz', '.pgm', '.png', '.txt'}
for name in ('mapping_session_1793961409915912543',
             'mapping_session_1793963081030061346',
             'vehicle_audit_1793959316512928162',
             'wheel_contact_1793961390414977160'):
    for p in sorted((ROOT / name).iterdir()):
        if p.is_file() and p.suffix.lower() in allowed:
            save(p.read_bytes(), f'evidence/{name}/{p.name}', f'{name}/{p.name}')

for pattern in ('mapping_session_*.zip', 'wheel_contact_*.zip', 'vehicle_audit_*.zip'):
    for p in sorted(ROOT.glob(pattern)):
        with zipfile.ZipFile(p) as z:
            for item in z.infolist():
                parts = Path(item.filename).parts
                if item.is_dir() or 'runtime_sources' in parts:
                    continue
                if len(parts) > 2 or Path(item.filename).suffix.lower() not in allowed:
                    continue
                save(z.read(item), f'evidence/{p.stem}/{Path(item.filename).name}',
                     f'{p.name}!/{item.filename}')

vendor = Path(r'C:\Users\jinyu\Desktop\NTUT_icar\醫療機器人\AGV_parameter')
for name in ('TZAGV-DSB02-kh.STEP',
             'LD dual channel servo controller _485_ASCII communication protocol-2022-03-07.pdf',
             'LD dual channel Servo Drive-485-RTU Communication Protocol-2022-03-08.pdf',
             'LDS Series Dual-channel Servo Driver User Manual 20230406.pdf',
             'agv_move_sample_v6.zip', 'agv_read_encoder_v6.zip'):
    p = vendor / name
    save(p.read_bytes(), 'assets/vendor/' + name, 'external_vendor/' + name)
papers = Path(r'C:\Users\jinyu\Desktop\NTUT_icar\hybrid_A_star')
for name in ('Practical Search Techniques in Path Planning for Autonomous Driving.pdf',
             'IMPROVED HYBRID A-STAR ALGORITHM FOR PATH PLANNING IN (1).pdf'):
    save((papers / name).read_bytes(), 'docs/references/' + name, 'external_papers/' + name)

(ROOT / 'docs/supplement_manifest.json').write_text(
    json.dumps(entries, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print(json.dumps({'added': len(entries), 'bytes': sum(e['bytes'] for e in entries)}))
