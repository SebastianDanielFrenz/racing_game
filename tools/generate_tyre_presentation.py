"""Generate the garage UI's tyre presentation manifest from data/tyres/families.json.

There are no tyre files: tyres are built in memory by rg::TyreFamilies (core/src/tyre_spec.cpp) from families.json.
The only generated artefact is game/assets/components/tyres/catalog.json, which gives the UI each compound's
label/image/rated speed (the UI parses the tyre id itself: compound_width_aspectrRim).

Usage: python tools/generate_tyre_presentation.py          write the manifest
       python tools/generate_tyre_presentation.py --check  verify it, change nothing (exit 1 on a difference)
"""
from pathlib import Path
import json
import sys

root = Path(__file__).resolve().parents[1]
families_path = root / 'data/tyres/families.json'
manifest_path = root / 'game/assets/components/tyres/catalog.json'


def main():
    check = '--check' in sys.argv[1:]
    config = json.loads(families_path.read_text())
    if config.get('format') != 'racing_game.tyre_families/4':
        raise SystemExit('families.json must be racing_game.tyre_families/4')
    families = {item['id']: item for item in config['families']}
    cap = config['rule'].get('rated_speed_cap_kmh_up_to_rim_in', {'max_rim_in': 0, 'cap_kmh': 0})
    manifest = {
        'format': 'racing_game.tyre_presentation/1',
        'note': 'Presentation of the tyre compounds for the garage UI. A tyre id (compound_width_aspectrRim, e.g. road_265_35r20) '
                'is parsed by the UI; this file supplies the compound label/image and the rated-speed rule. '
                'Generated from data/tyres/families.json by tools/generate_tyre_presentation.py; do not edit.',
        'styles': {key: {'image': families[key]['image']} for key in ('road', 'track', 'drag', 'slick')},
        'families': families,
        'family_order': list(families),
        'rated_speed_cap': {'max_rim_in': cap['max_rim_in'], 'cap_kmh': cap['cap_kmh']},
    }
    text = json.dumps(manifest, indent=2) + '\n'
    current = manifest_path.read_bytes().replace(b'\r\n', b'\n') if manifest_path.exists() else b''
    differs = current != text.encode('utf-8')
    if check:
        print('tyre presentation check:', 'FAILED' if differs else 'ok', len(families), 'compounds')
        sys.exit(1 if differs else 0)
    if differs:
        manifest_path.write_bytes(text.encode('utf-8'))
    print('Tyre presentation manifest', 'written' if differs else 'unchanged', '-', len(families), 'compounds')


main()
