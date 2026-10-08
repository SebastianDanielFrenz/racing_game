"""Prepare a filled exterior approximation from geometry.py --vehicle --flow output.

Requires numpy, scipy, trimesh, scikit-image (development only). Does not run CFD.
Surface voxelization, small-seam closing, exterior flood fill, marching cubes.
"""
import argparse
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path
import numpy as np
from scipy import ndimage
from skimage import measure
import trimesh

def fill_exterior(surface,closing_voxels=1):
    if type(closing_voxels)!=int or not 0<=closing_voxels<=3:raise ValueError('closing must be 0..3 voxels')
    padding=closing_voxels+2
    matrix=np.pad(surface.astype(bool),padding)
    if closing_voxels:
        matrix=ndimage.binary_closing(matrix,structure=np.ones((3,3,3),dtype=bool),iterations=closing_voxels)
    return ndimage.binary_fill_holes(matrix),padding

def prepare(source,output,pitch=.006,closing_voxels=1,min_component_m3=1e-5):
    if not math.isfinite(pitch) or not .002<=pitch<=.02:raise ValueError('pitch must be 2..20 mm')
    if not math.isfinite(min_component_m3) or min_component_m3<=0:raise ValueError('positive component cutoff required')
    source=Path(source);output=Path(output);audit=json.loads((source/'audit.json').read_text())
    obj=source/'hypercar_rest_iso.obj'
    if not audit.get('flow_selection_sha256') or 'chassis body origin' not in audit['frame']:
        raise ValueError('requires source-bound exterior selection aligned to chassis')
    if hashlib.sha256(obj.read_bytes()).hexdigest()!=audit['obj_sha256']:raise ValueError('source OBJ changed')
    vertices=[];faces=[];body_mask=[];node=''
    for line in obj.read_text().splitlines():
        fields=line.split()
        if not fields:continue
        if fields[0]=='o':node=fields[1].rsplit('_primitive_',1)[0]
        if fields[0]=='v':vertices.append([float(v) for v in fields[1:]])
        if fields[0]=='f':
            faces.append([int(v)-1 for v in fields[1:]])
            body_mask.append(node in ('car_hyper','door_L','door_R','frunk','engine_cover'))
    mesh=trimesh.Trimesh(vertices=vertices,faces=faces,process=False)
    body_mask=np.asarray(body_mask,dtype=bool)
    print('Rasterizing',len(mesh.faces),'triangles at',pitch,'m',flush=True)
    # Same half-pitch surface sampling as trimesh's subdivide voxelizer, but
    # bounded batches avoid holding a completely subdivided car in memory.
    origin_index=np.floor(mesh.bounds[0]/pitch).astype(np.int64)
    shape=np.ceil(mesh.bounds[1]/pitch).astype(np.int64)-origin_index+1
    if np.prod(shape)>250_000_000:raise ValueError('voxel grid exceeds 250 million cells; choose coarser pitch')
    cache=source/f'surface_body_{pitch:.9g}.npz';surface=None;body_surface=None
    if cache.exists():
        with np.load(cache,allow_pickle=False) as stored:
            if stored['source_sha256'].item()==audit['obj_sha256'] and tuple(stored['surface'].shape)==tuple(shape):surface=stored['surface'];body_surface=stored['body_surface']
    if surface is None:
        surface=np.zeros(tuple(shape),dtype=bool)
        body_surface=np.zeros(tuple(shape),dtype=bool)
        for is_body,group in ((True,mesh.faces[body_mask]),(False,mesh.faces[~body_mask])):
            for start in range(0,len(group),16):
                triangles=mesh.vertices[group[start:start+16]]
                v,_=trimesh.remesh.subdivide_to_size(triangles.reshape((-1,3)),np.arange(triangles.size//3).reshape((-1,3)),max_edge=pitch*.5,max_iter=12)
                hit=np.round(v/pitch).astype(np.int64)-origin_index
                surface[hit[:,0],hit[:,1],hit[:,2]]=True
                if is_body:body_surface[hit[:,0],hit[:,1],hit[:,2]]=True
                if start%10240==0:print('Surface raster',is_body,start,'/',len(group),flush=True)
        np.savez_compressed(cache,surface=surface,body_surface=body_surface,source_sha256=audit['obj_sha256'])
    # The visual shell encloses thin material volumes, not a filled body. Fill
    # only central body columns bounded by its own floor/roof: never use wing,
    # wheel or splitter height and keep this core inboard of all wheel wells.
    lower=body_surface.argmax(axis=2);upper=shape[2]-1-body_surface[:,:,::-1].argmax(axis=2)
    present=body_surface.any(axis=2)
    y=(np.arange(shape[1])+origin_index[1])*pitch
    central=present & (np.abs(y)[None,:]<=.55)
    z=np.arange(shape[2])[None,None,:]
    core=central[:,:,None] & (z>=lower[:,:,None]) & (z<=upper[:,:,None])
    surface|=core;core_volume=float(core.sum()*pitch**3)
    filled,padding=fill_exterior(surface,closing_voxels)
    labels,count=ndimage.label(filled);sizes=np.bincount(labels.ravel())
    keep=sizes*pitch**3>=min_component_m3;keep[0]=False
    filled=keep[labels];volumes=sorted((sizes[1:][keep[1:]]*pitch**3).tolist(),reverse=True)
    removed=int(count-len(volumes));del labels
    print('Filled',float(filled.sum()*pitch**3),'m3,',len(volumes),'components; extracting surface',flush=True)
    field=ndimage.gaussian_filter(filled.astype(np.float32),sigma=.5)
    vertices,faces,_,_=measure.marching_cubes(field,.5,spacing=(pitch,pitch,pitch),allow_degenerate=False)
    vertices+=(origin_index-padding)*pitch
    solid=trimesh.Trimesh(vertices=vertices,faces=faces,process=True);solid.fix_normals(multibody=True)
    before_smoothing=solid.vertices.copy()
    # Suppress voxel stair-step roughness without the systematic shrinkage of
    # Laplacian-only smoothing. This modifies the CFD derivative, never the art.
    trimesh.smoothing.filter_taubin(solid,lamb=.5,nu=.53,iterations=16)
    smoothing_displacement=float(np.linalg.norm(solid.vertices-before_smoothing,axis=1).max())
    drift=float(np.max(np.abs(solid.bounds-mesh.bounds)))
    envelope_volume=float(np.prod(mesh.extents));volume=float(solid.volume)
    quality={'watertight':bool(solid.is_watertight),'winding_consistent':bool(solid.is_winding_consistent),
             'volume_m3':volume,'envelope_volume_m3':envelope_volume,'bounds_drift_m':drift,'components':len(volumes)}
    if not solid.is_watertight or not solid.is_winding_consistent or volume<envelope_volume*.15 or drift>pitch*3 or len(volumes)>32 or smoothing_displacement>pitch*3:
        raise ValueError('solid quality gate failed: '+json.dumps(quality))
    output.mkdir(parents=True,exist_ok=True);stl=output/'hypercar_cfd_rest.stl';solid.export(stl)
    report={'format':'rg.cfd-solid/1','status':'filled_exterior_approximation_requires_review','cfd_validated':False,
            'source_audit':audit,'versions':{name:importlib.metadata.version(name) for name in ('numpy','scipy','trimesh','scikit-image')},
            'voxel_m':pitch,'seam_closing_voxels':closing_voxels,'padding_voxels':padding,'min_component_m3':min_component_m3,
            'central_body_core_half_width_m':.55,'central_body_core_volume_m3':core_volume,'surface_smoothing_sigma_voxels':.5,
            'taubin_smoothing':{'lambda':.5,'nu':.53,'iterations':16,'max_displacement_m':smoothing_displacement},
            'discarded_small_components':removed,'component_volumes_m3':volumes,'quality':quality,
            'triangles':len(solid.faces),'frame':audit['frame'],'bounds_iso_m':solid.bounds.tolist(),
            'stl_sha256':hashlib.sha256(stl.read_bytes()).hexdigest(),
            'assumptions':['closed doors, fixed suspension; wing pose recorded in source audit','small panel seams sealed; enclosed volume filled',
                 'central body core filled only between body floor/roof within 0.55 m of centerline, excluding aero and wheels',
                 'half-voxel Gaussian surface regularization',
                 '16 Taubin smoothing iterations to suppress artificial voxel roughness',
                 'backed cooling grilles; no resolved radiator/duct airflow','thin details below voxel resolution may be lost'],
            'required_review':['shape/deviation review including intakes/exhaust/wing/tyres','resolution comparison',
                 'fluid domain, moving ground, rotating wheels and convergence before solver']}
    (output/'manifest.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print('CFD_SOLID',json.dumps(quality),flush=True)
    return report

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('input');parser.add_argument('output')
    parser.add_argument('--voxel-m',type=float,default=.006);parser.add_argument('--seam-closing-voxels',type=int,default=1)
    args=parser.parse_args();prepare(args.input,args.output,args.voxel_m,args.seam_closing_voxels)
