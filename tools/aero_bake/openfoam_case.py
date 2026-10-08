"""Generate an exploratory OpenFOAM Foundation 14 hypercar mesh/flow case.

Run inside WSL with the official motorBikeSteady tutorial installed.
These exploratory boundary-condition pilots cannot activate runtime maps.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
import shlex

def replace_block(text, key, replacement):
    match=re.search(r'^\s*'+re.escape(key)+r'\s*\{',text,re.M)
    if not match:raise ValueError('missing dictionary '+key)
    depth=1;end=match.end()
    while depth:
        if end>=len(text):raise ValueError('unclosed dictionary '+key)
        depth+=(text[end]=='{')-(text[end]=='}');end+=1
    return text[:match.start()]+'\n'+key+'\n{\n'+replacement+'\n}'+text[end:]

def wheel_velocity_boundary(region, speed):
    radius=region['radius_m']
    if not math.isfinite(radius) or radius<=0:raise ValueError('positive wheel radius required')
    if region['axis_iso'] != [0,1,0]:raise ValueError('unsupported wheel axis')
    center=region['origin_iso_m']
    if len(center)!=3 or not all(math.isfinite(v) for v in center):raise ValueError('finite wheel centre required')
    if not math.isfinite(speed) or speed<=0:raise ValueError('positive finite speed required')
    origin=' '.join(str(v) for v in center)
    # +Y rotation gives bottom-of-wheel velocity -X, matching moving ground.
    return f"{region['name']}Group {{ type rotatingWallVelocity; origin ({origin}); axis (0 1 0); omega {speed/radius}; value uniform (0 0 0); }}"

def generate(solid, output, tutorial, speed=50.0, refinement=3, workers=6, regions=None, layers=0, first_layer_m=.0006, iterations=300,wheel_refinement=None,layer_iterations=20):
    solid, output, tutorial = map(Path, (solid, output, tutorial))
    if not math.isfinite(speed) or speed <= 0 or refinement not in (2, 3, 4):
        raise ValueError('positive finite speed and refinement 2..4 required')
    if type(workers)!=int or workers not in (1,2,4,6,8):
        raise ValueError('workers must be 1,2,4,6 or 8')
    if type(layers)!=int or not 0<=layers<=12 or not math.isfinite(first_layer_m) or first_layer_m<=0:
        raise ValueError('layers must be 0..12 and first layer thickness positive')
    if type(iterations)!=int or iterations<100:raise ValueError('at least100 iterations required')
    if wheel_refinement is not None and (type(wheel_refinement)!=int or not refinement<=wheel_refinement<=6):raise ValueError('wheel refinement must be between body level and6')
    if type(layer_iterations)!=int or not 1<=layer_iterations<=100:raise ValueError('layer iterations must be1..100')
    if output.exists():
        raise ValueError('output already exists; use a fresh case directory')
    manifest = json.loads((solid/'manifest.json').read_text())
    stl = solid/'hypercar_cfd_rest.stl'
    digest = hashlib.sha256(stl.read_bytes()).hexdigest()
    if digest != manifest['stl_sha256'] or not manifest['quality']['watertight']:
        raise ValueError('source hash mismatch or open surface')
    region_manifest=None
    if regions is not None:
        regions=Path(regions);region_manifest=json.loads((regions/'regions.json').read_text())
        if (region_manifest['stl_sha256']!=digest or region_manifest['geometry_changed']
            or region_manifest['source_obj_sha256']!=manifest['source_audit']['obj_sha256']
            or region_manifest['vehicle_sha256']!=manifest['source_audit']['vehicle_sha256']):
            raise ValueError('region surface identity mismatch')
        if len(region_manifest['regions'])!=5 or {r['name'] for r in region_manifest['regions']}!={'body','wheel_FL','wheel_FR','wheel_RL','wheel_RR'}:
            raise ValueError('body and four wheel regions required')
        for row in region_manifest['regions']:
            source=regions/row['file']
            if source.resolve().parent!=regions.resolve() or hashlib.sha256(source.read_bytes()).hexdigest()!=row['sha256']:
                raise ValueError('region file identity mismatch')
    shutil.copytree(tutorial, output)
    geometry=output/'constant/geometry'
    geometry.mkdir(exist_ok=True)
    shutil.copyfile(stl, geometry/'hypercar.stl')
    if region_manifest:
        for row in region_manifest['regions']:shutil.copyfile(regions/row['file'],geometry/row['file'])
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
    if region_manifest:
        velocity=output/'0/U';text=velocity.read_text().replace('hypercarGroup','bodyGroup')
        boundaries='\n'.join(wheel_velocity_boundary(row,speed) for row in region_manifest['regions'] if row['name']!='body')
        text=text.replace('    #include "include/frontBackUpperPatches"',boundaries+'\n    #include "include/frontBackUpperPatches"')
        velocity.write_text(text)
    snappy=output/'system/snappyHexMeshDict'; text=snappy.read_text()
    text=text.replace('file "hypercar.obj"','file "hypercar.stl"').replace('addLayers       true','addLayers       false')
    text=re.sub(r'^    features\s*\(.*?\);','features ();',text,flags=re.S|re.M)
    text=text.replace('explicitFeatureSnap true','explicitFeatureSnap false').replace('implicitFeatureSnap false','implicitFeatureSnap true')
    text=text.replace('level (5 6)',f'level ({refinement} {refinement})').replace('level   4;',f'level   {max(1,refinement-1)};')
    text=text.replace('min (-1.0 -0.7 0.0)',f'min (-12 -2 {ground})').replace('max ( 8.0  0.7 2.5)','max ( 4 2 2)')
    text=text.replace('insidePoint (3.0001 3.0001 0.43)','insidePoint (8.0001 5.0001 3.0001)').replace('maxLocalCells 100000','maxLocalCells 2000000')
    if region_manifest:
        surfaces='';refinements=''
        for row in region_manifest['regions']:
            name=row['name']
            surfaces+=f'{name} {{ type triSurface; file "{row["file"]}"; }}\n'
            level=wheel_refinement if name!='body' and wheel_refinement is not None else refinement
            refinements+=f'{name} {{ level ({level} {level}); patchInfo {{ type wall; inGroups (hypercarGroup {name}Group); }} }}\n'
        surfaces+=f'refinementBox {{ type box; min (-12 -2 {ground}); max (4 2 2); }}'
        text=replace_block(text,'geometry',surfaces)
        text=replace_block(text,'refinementSurfaces',refinements)
    if layers:
        text=text.replace('addLayers       false','addLayers       true')
        pattern='(lowerWall|body|wheel_.*).*' if region_manifest else '(lowerWall|hypercar).*'
        text=replace_block(text,'layers',f'"{pattern}" {{ nSurfaceLayers {layers}; }}')
        text=text.replace('relativeSizes true','relativeSizes false').replace('expansionRatio 1.0','expansionRatio 1.3')
        text=text.replace('finalLayerThickness 0.3',f'firstLayerThickness {first_layer_m}').replace('minThickness 0.1',f'minThickness {first_layer_m*.2}')
    text=re.sub(r'nLayerIter\s+50;',f'nLayerIter {layer_iterations};',text)
    snappy.write_text(text)
    quality=output/'system/meshQualityDict'
    quality.write_text(quality.read_text()+'\nmaxBoundarySkewness 4;\n')
    control=output/'system/controlDict'; text=control.read_text().replace('endTime         500',f'endTime         {iterations}')
    text+='\nfunctions\n{\n    loads\n    {\n        type forces;\n        libs ("libforces.so");\n        patches (hypercarGroup);\n        rho rhoInf;\n        rhoInf 1.225;\n        CofR (0 0 0);\n        writeControl timeStep;\n        writeInterval 1;\n    }\n}\n'
    control.write_text(text.split('\nfunctions\n')[0])
    (output/'system/functions').write_text('FoamFile { format ascii; class dictionary; object functions; }\n'+text.split('\nfunctions\n{\n')[1].rsplit('}',1)[0])
    if layers:
        functions=output/'system/functions'
        functions.write_text(functions.read_text()+'\nwallResolution { type yPlus; libs ("libfieldFunctionObjects.so"); executeControl writeTime; writeControl writeTime; }\n')
    decomposition=output/'system/decomposeParDict'
    text=decomposition.read_text()
    text=text.replace('numberOfSubdomains  6;',f'numberOfSubdomains  {workers};')
    grid={1:(1,1,1),2:(2,1,1),4:(2,2,1),6:(3,2,1),8:(4,2,1)}[workers]
    text=text.replace('(3 2 1)',f'({grid[0]} {grid[1]} {grid[2]})')
    decomposition.write_text(text)
    solve='foamRun >log.foamRun 2>&1' if workers==1 else f'decomposePar -force -copyZero >log.decomposePar 2>&1\nmpirun -np {workers} foamRun -parallel >log.foamRun 2>&1'
    (output/'Allrun').write_text('#!/bin/bash\nset -euo pipefail\ncd '+shlex.quote(str(output))+'\nblockMesh >log.blockMesh 2>&1\nsnappyHexMesh >log.snappyHexMesh 2>&1\ncheckMesh -allGeometry -allTopology -meshQuality >log.checkMesh 2>&1\ngrep -q "Mesh OK" log.checkMesh\n'+solve+'\n')
    result={'format':'rg.openfoam-pilot/1','solver':'OpenFOAM Foundation 14/incompressibleFluid',
            'solver_workers':workers,'iterations':iterations,'layers':layers,'first_layer_m':first_layer_m if layers else None,
            'wheel_refinement':wheel_refinement,'layer_iterations':layer_iterations,'wheel_regions':region_manifest,'stl_sha256':digest,'speed_m_s':speed,'ground_z_m':ground,'refinement':refinement,
            'frame':manifest['frame'],'moment_origin_m':[0,0,0],
            'wing_pose':manifest['source_audit'].get('actuator_pose',{'wing_offset_deg':0,'wing_lift_m':0}),
            'validated':False,'runtime_map_eligible':False,
            'limitations':(['static wheels'] if not region_manifest else ['rotating-wall approximation; no resolved spoke motion','source-region seams require review'])+['fixed wing pose only','2mm numerical ground gap','no convergence study','sealed cooling paths']+(['no prism layers in mesh pilot'] if not layers else ['layer coverage and yPlus require review'])}
    (output/'pilot_manifest.json').write_text(json.dumps(result,indent=2)+'\n')
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--solid',required=True);p.add_argument('--output',required=True)
    p.add_argument('--tutorial',default='/opt/openfoam14/tutorials/incompressibleFluid/motorBikeSteady')
    p.add_argument('--speed',type=float,default=50);p.add_argument('--refinement',type=int,default=3)
    p.add_argument('--workers',type=int,default=6)
    p.add_argument('--regions');p.add_argument('--layers',type=int,default=0)
    p.add_argument('--first-layer-m',type=float,default=.0006);p.add_argument('--iterations',type=int,default=300)
    p.add_argument('--wheel-refinement',type=int);p.add_argument('--layer-iterations',type=int,default=20)
    a=p.parse_args();print(json.dumps(generate(a.solid,a.output,a.tutorial,a.speed,a.refinement,a.workers,a.regions,a.layers,a.first_layer_m,a.iterations,a.wheel_refinement,a.layer_iterations),indent=2))
