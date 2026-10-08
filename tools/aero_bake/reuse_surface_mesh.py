"""Reuse an independently checked, layer-free surface mesh for layer experiments."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shutil

def reuse(base,case):
    base,case=map(Path,(base,case))
    original=json.loads((base/'pilot_manifest.json').read_text())
    target=json.loads((case/'pilot_manifest.json').read_text())
    for key in ('stl_sha256','refinement','wheel_refinement','ground_z_m','wing_pose','wheel_regions'):
        if original.get(key)!=target.get(key):raise ValueError('base mesh differs: '+key)
    quality=(base/'log.checkMesh').read_text()
    if 'Mesh OK.' not in quality or 'Failed ' in quality:raise ValueError('base mesh quality did not pass')
    log=(base/'log.snappyHexMesh').read_text()
    if not log.rstrip().endswith('End'):raise ValueError('base meshing not complete')
    tables=log.split('overall thickness')
    if len(tables)<2:raise ValueError('missing achieved layer audit')
    rows=re.findall(r'^(?:body|hypercar|wheel_\w+|lowerWall)\s+\d+\s+([\d.eE+-]+)\s+',tables[-1],re.M)
    if not rows or any(float(v)!=0 for v in rows):raise ValueError('base mesh must have zero achieved prism layers')
    mesh=base/'constant/polyMesh';destination=case/'constant/polyMesh'
    if destination.exists():raise ValueError('target already has a mesh')
    files={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in mesh.iterdir() if p.is_file()}
    for name in ('points','faces','owner','neighbour','boundary'):
        if name not in files and name+'.gz' not in files:raise ValueError('incomplete base mesh')
    shutil.copytree(mesh,destination)
    for name,digest in files.items():
        if hashlib.sha256((destination/name).read_bytes()).hexdigest()!=digest or hashlib.sha256((mesh/name).read_bytes()).hexdigest()!=digest:
            raise ValueError('base mesh changed during copy')
    dictionary=case/'system/snappyHexMeshDict';s=dictionary.read_text()
    s=re.sub(r'castellatedMesh\s+true;', 'castellatedMesh false;',s)
    s=re.sub(r'snap\s+true;', 'snap false;',s);dictionary.write_text(s)
    launcher=case/'Allrun';s=launcher.read_text().replace('blockMesh >log.blockMesh 2>&1\n','');launcher.write_text(s)
    target['reused_surface_mesh']={'base_case':str(base),'base_manifest_sha256':hashlib.sha256((base/'pilot_manifest.json').read_bytes()).hexdigest(),'mesh_file_sha256':files,'achieved_base_layers':0,'quality_log_sha256':hashlib.sha256((base/'log.checkMesh').read_bytes()).hexdigest()}
    (case/'pilot_manifest.json').write_text(json.dumps(target,indent=2)+'\n')
    return target['reused_surface_mesh']

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('base');p.add_argument('case');a=p.parse_args()
    print(json.dumps(reuse(a.base,a.case),indent=2))
