"""Partition the prepared CFD surface by source-bound stationary/wheel geometry.

Does not change triangles. Uses dense source voxel samples; near ties stay
stationary rather than rotating a fender. Region seams require visual review.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct
import numpy as np
from scipy.spatial import cKDTree
import trimesh

NAMES=('body','wheel_FL','wheel_FR','wheel_RL','wheel_RR')
STL=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])

def classify(points, trees, pitch):
    stationary=trees[0].query(points,workers=4)[0]
    wheel=np.stack([t.query(points,workers=4)[0] for t in trees[1:]],axis=1)
    nearest=wheel.argmin(axis=1);distance=wheel[np.arange(len(points)),nearest]
    chosen=(distance < stationary-.15*pitch)&(distance<=2*pitch)
    labels=np.where(chosen,nearest+1,0).astype(np.uint8)
    return labels, np.minimum(stationary,distance)

def prepare(source,solid,vehicle,output):
    source,solid,vehicle,output=map(Path,(source,solid,vehicle,output))
    audit=json.loads((source/'audit.json').read_text());manifest=json.loads((solid/'manifest.json').read_text())
    obj=source/'hypercar_rest_iso.obj';stl=solid/'hypercar_cfd_rest.stl'
    if hashlib.sha256(obj.read_bytes()).hexdigest()!=audit['obj_sha256'] or audit['obj_sha256']!=manifest['source_audit']['obj_sha256']:
        raise ValueError('source geometry identity mismatch')
    if hashlib.sha256(stl.read_bytes()).hexdigest()!=manifest['stl_sha256']:
        raise ValueError('prepared STL identity mismatch')
    vehicle_bytes=vehicle.read_bytes()
    if hashlib.sha256(vehicle_bytes).hexdigest()!=audit['vehicle_sha256']:
        raise ValueError('vehicle identity mismatch')
    description=json.loads(vehicle_bytes)
    pitch=manifest['voxel_m'];vertices=[];faces=[];groups=[];node=''
    for line in obj.read_text().splitlines():
        words=line.split()
        if not words:continue
        if words[0]=='o':node=words[1].rsplit('_primitive_',1)[0]
        elif words[0]=='v':vertices.append([float(v) for v in words[1:]])
        elif words[0]=='f':
            faces.append([int(v)-1 for v in words[1:]])
            groups.append(NAMES.index(node) if node in NAMES else 0)
    mesh=trimesh.Trimesh(vertices=vertices,faces=faces,process=False)
    origin=np.floor(mesh.bounds[0]/pitch).astype(np.int64)
    shape=np.ceil(mesh.bounds[1]/pitch).astype(np.int64)-origin+1
    cache=source/f'region_samples_{pitch:.9g}.npz'
    samples=None
    if cache.exists():
        with np.load(cache,allow_pickle=False) as stored:
            if stored['source_sha256'].item()==audit['obj_sha256']:
                samples=[stored[name] for name in NAMES]
    if samples is None:
        samples=[];groups=np.asarray(groups)
        for index,name in enumerate(NAMES):
            raster=np.zeros(tuple(shape),dtype=bool);group=mesh.faces[groups==index]
            if not len(group):raise ValueError('missing source region '+name)
            for start in range(0,len(group),16):
                triangles=mesh.vertices[group[start:start+16]]
                v,_=trimesh.remesh.subdivide_to_size(triangles.reshape((-1,3)),np.arange(triangles.size//3).reshape((-1,3)),max_edge=pitch*.5,max_iter=12)
                hit=np.round(v/pitch).astype(np.int64)-origin
                raster[hit[:,0],hit[:,1],hit[:,2]]=True
            samples.append((np.argwhere(raster)+origin)*pitch)
            print('REGION_SOURCE',name,len(samples[-1]),flush=True)
        np.savez_compressed(cache,source_sha256=audit['obj_sha256'],**dict(zip(NAMES,samples)))
    trees=[cKDTree(points) for points in samples]
    with stl.open('rb') as f:f.seek(80);count=struct.unpack('<I',f.read(4))[0]
    if stl.stat().st_size!=84+50*count:raise ValueError('expected exact binary STL')
    records=np.memmap(stl,dtype=STL,offset=84,shape=(count,),mode='r')
    labels=np.zeros(count,dtype=np.uint8);far=0;worst=0.
    far_area=0.;total_area=0.;far_min=np.full(3,np.inf);far_max=np.full(3,-np.inf)
    for start in range(0,count,50000):
        center=records['vertices'][start:start+50000].mean(axis=1)
        labels[start:start+len(center)],distance=classify(center,trees,pitch)
        distant=distance>4*pitch
        triangles=records['vertices'][start:start+len(center)]
        area=np.linalg.norm(np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]),axis=1)*.5
        total_area+=float(area.sum());far_area+=float(area[distant].sum())
        if distant.any():
            far_min=np.minimum(far_min,center[distant].min(axis=0));far_max=np.maximum(far_max,center[distant].max(axis=0))
        far+=int(distant.sum());worst=max(worst,float(distance.max()))
    output.mkdir(parents=True,exist_ok=True);regions=[]
    for index,name in enumerate(NAMES):
        selected=labels==index;n=int(selected.sum());target=output/(name+'.stl')
        if not n:raise ValueError('empty prepared region '+name)
        with target.open('wb') as f:
            f.write(name.encode().ljust(80,b'\0'));f.write(struct.pack('<I',n))
            for start in range(0,count,100000):f.write(records[start:start+100000][selected[start:start+100000]].tobytes())
        row={'name':name,'file':target.name,'triangles':n,'sha256':hashlib.sha256(target.read_bytes()).hexdigest()}
        if index:
            wheel=next(w for w in description['wheels'] if 'wheel_'+w['name']==name)
            row.update({'origin_iso_m':wheel['attachment_local'],'axis_iso':[0,1,0],'radius_m':wheel['wheel_radius']})
        regions.append(row)
    np.save(output/'triangle_regions.npy',labels,allow_pickle=False)
    result={'format':'rg.cfd-regions/1','stl_sha256':manifest['stl_sha256'],'source_obj_sha256':audit['obj_sha256'],
            'vehicle_sha256':audit['vehicle_sha256'],'triangles':count,'regions':regions,
            'geometry_changed':False,'assignment':'nearest dense source voxels; stationary wins within0.15 voxel',
            'sampling_pitch_m':pitch,'far_from_source_triangles':far,'maximum_source_distance_m':worst,
            'source_distance_audit':{'threshold_m':4*pitch,'far_area_m2':far_area,'total_area_m2':total_area,'far_area_fraction':far_area/total_area,'far_centroid_bounds_iso_m':[far_min.tolist(),far_max.tolist()] if far else None,'meaning':'distance to source voxel samples, not exact triangle distance; filled closure surfaces require explicit review'},
            'review_required':True,'validated':False,'frame':manifest['frame']}
    (output/'regions.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2));return result

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('source');p.add_argument('solid');p.add_argument('vehicle');p.add_argument('output')
    a=p.parse_args();prepare(a.source,a.solid,a.vehicle,a.output)
