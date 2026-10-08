"""Generate an exploratory OpenFOAM Foundation 14 hypercar mesh/flow case.

Run inside WSL with the official motorBikeSteady tutorial installed.
This static-wheel/rest-wing pilot must never be used to activate runtime maps.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import shlex

def generate(solid, output, tutorial, speed=50.0, refinement=3, workers=6):
    solid, output, tutorial = map(Path, (solid, output, tutorial))
    if not math.isfinite(speed) or speed <= 0 or refinement not in (2, 3, 4):
        raise ValueError('positive finite speed and refinement 2..4 required')
    if type(workers)!=int or workers not in (1,2,4,6,8):
        raise ValueError('workers must be 1,2,4,6 or 8')
    if output.exists():
        raise ValueError('output already exists; use a fresh case directory')
    manifest = json.loads((solid/'manifest.json').read_text())
    stl = solid/'hypercar_cfd_rest.stl'
    digest = hashlib.sha256(stl.read_bytes()).hexdigest()
    if digest != manifest['stl_sha256'] or not manifest['quality']['watertight']:
        raise ValueError('source hash mismatch or open surface')
    shutil.copytree(tutorial, output)
    geometry=output/'constant/geometry'
    geometry.mkdir(exist_ok=True)
    shutil.copyfile(stl, geometry/'hypercar.stl')
    def replace(path, old, new):
        p=output/path; text=p.read_text();
        if old not in text: raise ValueError(f'missing template token: {path}: {old}')
        p.write_text(text.replace(old,new))
    for p in output.rglob('*'):
        if p.is_file() and p.suffix not in ('.gz',):
            try: text=p.read_text()
            except UnicodeError: continue
            p.write_text(text.replace('motorBikeGroup','hypercarGroup').replace('motorBike','hypercar'))
    # ISO axes: incoming air and moving ground both travel toward -X.
    ground=manifest['bounds_iso_m'][0][2]-0.002
    verts=[(-25,-10,ground),(12,-10,ground),(12,10,ground),(-25,10,ground),
           (-25,-10,10),(12,-10,10),(12,10,10),(-25,10,10)]
    block=output/'system/blockMeshDict'; text=block.read_text()
    text=re.sub(r'vertices\s*\(.*?\);','vertices\n(\n'+'\n'.join(f'    ({x} {y} {z})' for x,y,z in verts)+'\n);',text,flags=re.S|re.M)
    text=text.replace('(20 8 8)','(74 40 21)')
    text=text.replace('(0 4 7 3)','(2 6 5 1)').replace('(2 6 5 1)\n        );\n    }\n    lowerWall','(0 4 7 3)\n        );\n    }\n    lowerWall')
    block.write_text(text)
    p=output/'0/include/initialConditions'
    p.write_text(f'flowVelocity (-{speed} 0 0);\npressure 0;\nturbulentKE {1.5*(speed*.01)**2};\nturbulentOmega {math.sqrt(1.5*(speed*.01)**2)/(.09**.25*.2)};\n')
    shutil.copyfile(output/'0/U.orig',output/'0/U')
    snappy=output/'system/snappyHexMeshDict'; text=snappy.read_text()
    text=text.replace('file "hypercar.obj"','file "hypercar.stl"').replace('addLayers       true','addLayers       false')
    text=re.sub(r'^    features\s*\(.*?\);','features ();',text,flags=re.S|re.M)
    text=text.replace('explicitFeatureSnap true','explicitFeatureSnap false').replace('implicitFeatureSnap false','implicitFeatureSnap true')
    text=text.replace('level (5 6)',f'level ({refinement} {refinement})').replace('level   4;',f'level   {max(1,refinement-1)};')
    text=text.replace('min (-1.0 -0.7 0.0)',f'min (-12 -2 {ground})').replace('max ( 8.0  0.7 2.5)','max ( 4 2 2)')
    text=text.replace('insidePoint (3.0001 3.0001 0.43)','insidePoint (8.0001 5.0001 3.0001)').replace('maxLocalCells 100000','maxLocalCells 2000000')
    snappy.write_text(text)
    quality=output/'system/meshQualityDict'
    quality.write_text(quality.read_text()+'\nmaxBoundarySkewness 4;\n')
    control=output/'system/controlDict'; text=control.read_text().replace('endTime         500','endTime         300')
    text+='\nfunctions\n{\n    loads\n    {\n        type forces;\n        libs ("libforces.so");\n        patches (hypercarGroup);\n        rho rhoInf;\n        rhoInf 1.225;\n        CofR (0 0 0);\n        writeControl timeStep;\n        writeInterval 1;\n    }\n}\n'
    control.write_text(text.split('\nfunctions\n')[0])
    (output/'system/functions').write_text('FoamFile { format ascii; class dictionary; object functions; }\n'+text.split('\nfunctions\n{\n')[1].rsplit('}',1)[0])
    decomposition=output/'system/decomposeParDict'
    text=decomposition.read_text()
    text=text.replace('numberOfSubdomains  6;',f'numberOfSubdomains  {workers};')
    grid={1:(1,1,1),2:(2,1,1),4:(2,2,1),6:(3,2,1),8:(4,2,1)}[workers]
    text=text.replace('(3 2 1)',f'({grid[0]} {grid[1]} {grid[2]})')
    decomposition.write_text(text)
    solve='foamRun >log.foamRun 2>&1' if workers==1 else f'decomposePar -force -copyZero >log.decomposePar 2>&1\nmpirun -np {workers} foamRun -parallel >log.foamRun 2>&1'
    (output/'Allrun').write_text('#!/bin/bash\nset -euo pipefail\ncd '+shlex.quote(str(output))+'\nblockMesh >log.blockMesh 2>&1\nsnappyHexMesh >log.snappyHexMesh 2>&1\ncheckMesh >log.checkMesh 2>&1\ngrep -q "Mesh OK" log.checkMesh\n'+solve+'\n')
    result={'format':'rg.openfoam-pilot/1','solver':'OpenFOAM Foundation 14/incompressibleFluid',
            'solver_workers':workers,'stl_sha256':digest,'speed_m_s':speed,'ground_z_m':ground,'refinement':refinement,
            'frame':manifest['frame'],'moment_origin_m':[0,0,0],
            'wing_pose':manifest['source_audit'].get('actuator_pose',{'wing_offset_deg':0,'wing_lift_m':0}),
            'validated':False,'runtime_map_eligible':False,
            'limitations':['static wheels','fixed wing pose only','2mm numerical ground gap',
                           'no prism layers in mesh pilot','no convergence study','sealed cooling paths']}
    (output/'pilot_manifest.json').write_text(json.dumps(result,indent=2)+'\n')
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--solid',required=True);p.add_argument('--output',required=True)
    p.add_argument('--tutorial',default='/opt/openfoam14/tutorials/incompressibleFluid/motorBikeSteady')
    p.add_argument('--speed',type=float,default=50);p.add_argument('--refinement',type=int,default=3)
    p.add_argument('--workers',type=int,default=6)
    a=p.parse_args();print(json.dumps(generate(a.solid,a.output,a.tutorial,a.speed,a.refinement,a.workers),indent=2))
