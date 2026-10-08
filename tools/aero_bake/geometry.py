"""Export the supplied hypercar rest geometry for CFD preparation, never certification.

Restricted GLB reader following Khronos glTF 2.0: indexed/unindexed triangle
meshes, embedded buffer, node TRS/matrix. Unsupported geometry fails explicitly.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import struct

IDENTITY = [[float(i == j) for j in range(4)] for i in range(4)]

def multiply(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]

def transform(node):
    if "matrix" in node:
        return [[node["matrix"][j*4+i] for j in range(4)] for i in range(4)]
    x,y,z,w = node.get("rotation", [0,0,0,1])
    sx,sy,sz = node.get("scale", [1,1,1])
    tx,ty,tz = node.get("translation", [0,0,0])
    return [[(1-2*(y*y+z*z))*sx,2*(x*y-z*w)*sy,2*(x*z+y*w)*sz,tx],
            [2*(x*y+z*w)*sx,(1-2*(x*x+z*z))*sy,2*(y*z-x*w)*sz,ty],
            [2*(x*z-y*w)*sx,2*(y*z+x*w)*sy,(1-2*(x*x+y*y))*sz,tz], [0,0,0,1]]

def load_glb(path,pivots=None,flow=None,wing_offset_deg=0.0,wing_lift_m=0.0):
    if not all(math.isfinite(v) for v in (wing_offset_deg,wing_lift_m)):
        raise ValueError("nonfinite wing pose")
    posed_nodes=set()
    data=Path(path).read_bytes()
    magic,version,length=struct.unpack_from("<III",data)
    if magic!=0x46546c67 or version!=2 or length!=len(data):
        raise ValueError("expected complete GLB 2")
    chunks={};offset=12
    while offset<len(data):
        size,kind=struct.unpack_from("<II",data,offset);offset+=8
        if offset+size>len(data) or kind in chunks:raise ValueError("invalid GLB chunk")
        chunks[kind]=data[offset:offset+size];offset+=size
    doc=json.loads(chunks[0x4e4f534a]);blob=chunks[0x004e4942]
    if doc.get("extensionsRequired") or len(doc["buffers"])!=1 or "uri" in doc["buffers"][0]:
        raise ValueError("unsupported GLB extension/external buffers")
    def accessor(index):
        a=doc["accessors"][index]
        if "sparse" in a or a.get("normalized",False):raise ValueError("unsupported accessor")
        view=doc["bufferViews"][a["bufferView"]]
        fmt={5121:"B",5123:"H",5125:"I",5126:"f"}[a["componentType"]]
        n={"SCALAR":1,"VEC3":3}[a["type"]]
        item=struct.Struct("<"+fmt*n);stride=view.get("byteStride",item.size)
        if stride<item.size or view.get("buffer",0)!=0:raise ValueError("invalid buffer view")
        start=view.get("byteOffset",0)+a.get("byteOffset",0)
        end=start+(a["count"]-1)*stride+item.size
        if end>view.get("byteOffset",0)+view["byteLength"] or end>len(blob):raise ValueError("accessor outside view")
        return [item.unpack_from(blob,start+i*stride) for i in range(a["count"])]
    selected=None;seen=set()
    if flow is not None:
        selection=json.loads(Path(flow).read_text(encoding='utf-8-sig'))
        if selection['format']!='physics_sim.visual_flow_selection/1' or selection['source_glb_sha256']!=hashlib.sha256(data).hexdigest():
            raise ValueError('flow selection does not match source GLB')
        selected={}
        for row in selection['ranges']:
            start,count=row['first_triangle'],row['triangle_count']
            if type(start)!=int or type(count)!=int or start<0 or count<=0:raise ValueError('invalid flow triangle range')
            selected.setdefault((row['node'],row['material']),[]).append((start,start+count))
        for ranges in selected.values():
            ranges.sort()
            if any(a[1]>b[0] for a,b in zip(ranges,ranges[1:])):raise ValueError('overlapping flow ranges')
        if not selected:raise ValueError('empty flow selection')
    parts=[]
    def visit(index,parent):
        node=doc["nodes"][index]
        local_transform=transform(node)
        name=node.get("name","")
        if name in ("wing_flap","wing_lift") and (wing_offset_deg != 0 or wing_lift_m != 0):
            posed_nodes.add(name)
            joint=node.get("extras",{}).get("joint",{})
            value=math.radians(wing_offset_deg) if name=="wing_flap" else wing_lift_m
            if value<joint.get("min",0)-1e-12 or value>joint.get("max",0)+1e-12:
                raise ValueError("wing pose outside authored joint range")
            if name=="wing_flap":
                if joint.get("axis")!=[1,0,0]:raise ValueError("unsupported wing rotation axis")
                c,t=math.cos(value),math.sin(value)
                local_transform=multiply(local_transform,[[1,0,0,0],[0,c,-t,0],[0,t,c,0],[0,0,0,1]])
            else:
                if joint.get("axis")!=[0,1,0]:raise ValueError("unsupported wing lift axis")
                local_transform[1][3]+=value
        world=multiply(parent,local_transform)
        determinant=(world[0][0]*(world[1][1]*world[2][2]-world[1][2]*world[2][1])
                     -world[0][1]*(world[1][0]*world[2][2]-world[1][2]*world[2][0])
                     +world[0][2]*(world[1][0]*world[2][1]-world[1][1]*world[2][0]))
        if not math.isfinite(determinant) or abs(determinant)<1e-15:raise ValueError('singular/nonfinite node transform')
        if pivots is not None:
            pivots[node.get("name",f"node_{index}")]=(world[2][3],world[0][3],world[1][3])
        if "skin" in node:raise ValueError("skinned geometry requires posed export")
        if "mesh" in node:
            for pi,primitive in enumerate(doc["meshes"][node["mesh"]]["primitives"]):
                if primitive.get("mode",4)!=4 or primitive.get("targets") or primitive.get("extensions"):
                    raise ValueError("unsupported primitive/morph target")
                points=[]
                for p in accessor(primitive["attributes"]["POSITION"]):
                    v=[sum(world[i][k]*(*p,1)[k] for k in range(4)) for i in range(3)]
                    if not all(math.isfinite(c) for c in v):raise ValueError("nonfinite vertex")
                    points.append((v[2],v[0],v[1])) # supplied rig: gltf=(iso.y,iso.z,iso.x)
                indices=[v[0] for v in accessor(primitive["indices"])] if "indices" in primitive else list(range(len(points)))
                if len(indices)%3 or any(i>=len(points) for i in indices):raise ValueError("invalid triangle indices")
                triangles=[tuple(points[i] for i in indices[n:n+3]) for n in range(0,len(indices),3)]
                if determinant<0:triangles=[(a,c,b) for a,b,c in triangles]
                if selected is not None:
                    material=doc.get('materials',[])[primitive['material']]['name']
                    key=(node.get('name',f'node_{index}'),material)
                    if key not in selected:continue
                    ranges=selected[key];seen.add(key)
                    if ranges[-1][1]>len(triangles):raise ValueError('flow range exceeds final primitive')
                    triangles=[tri for a,b in ranges for tri in triangles[a:b]]
                parts.append((node.get("name",f"node_{index}")+f"_primitive_{pi}",triangles))
        for child in node.get("children",[]):visit(child,world)
    for root in doc["scenes"][doc.get("scene",0)]["nodes"]:visit(root,IDENTITY)
    if selected is not None and seen!=set(selected):raise ValueError('flow selection references absent node/material')
    if wing_offset_deg and "wing_flap" not in posed_nodes:raise ValueError("missing wing_flap joint")
    if wing_lift_m and "wing_lift" not in posed_nodes:raise ValueError("missing wing_lift joint")
    return parts

def cross(a,b):return (a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0])

def audit(triangles,tolerance):
    # Quantization is an audit approximation only; exported coordinates are unchanged.
    edges=Counter();directed=Counter();faces=Counter();degenerate=0;positions={}
    for tri in triangles:
        keys=[tuple(round(c/tolerance) for c in v) for v in tri]
        for key,point in zip(keys,tri):positions.setdefault(key,point)
        a,b,c=tri;normal=cross(tuple(b[i]-a[i] for i in range(3)),tuple(c[i]-a[i] for i in range(3)))
        if len(set(keys))<3 or sum(v*v for v in normal)<1e-24:degenerate+=1
        faces[tuple(sorted(keys))]+=1
        for u,v in zip(keys,keys[1:]+keys[:1]):edges[tuple(sorted((u,v)))]+=1;directed[(u,v)]+=1
    points=[p for tri in triangles for p in tri]
    boundary=sum(n==1 for n in edges.values());nonmanifold=sum(n>2 for n in edges.values())
    winding=sum(n==2 and (directed[(u,v)]!=1 or directed[(v,u)]!=1) for (u,v),n in edges.items())
    adjacency={}
    for (u,v),count in edges.items():
        if count==1:
            adjacency.setdefault(u,set()).add(v);adjacency.setdefault(v,set()).add(u)
    unseen=set(adjacency);components=[]
    while unseen:
        start=min(unseen);todo=[start];members=set()
        while todo:
            vertex=todo.pop()
            if vertex in members:continue
            members.add(vertex);unseen.discard(vertex)
            todo.extend(adjacency[vertex]-members)
        xyz=[positions[k] for k in members]
        components.append({'edges':sum(len(adjacency[k]) for k in members)//2,
                           'simple_closed_loop':all(len(adjacency[k])==2 for k in members),
                           'bounds_iso_m':{'min':[min(p[i] for p in xyz) for i in range(3)],'max':[max(p[i] for p in xyz) for i in range(3)]}})
    components.sort(key=lambda c:(-c['edges'],c['bounds_iso_m']['min']))
    return {"triangles":len(triangles),"bounds_iso_m":{"min":[min(p[i] for p in points) for i in range(3)],"max":[max(p[i] for p in points) for i in range(3)]},
            "boundary_edges":boundary,"nonmanifold_edges":nonmanifold,"inconsistent_winding_edges":winding,
            "duplicate_faces":sum(n-1 for n in faces.values()),"degenerate_triangles":degenerate,
            "boundary_components":components,
            "closed_edge_topology":not(boundary or nonmanifold or winding or degenerate),
            "cfd_ready":False,"note":"Edge topology cannot certify intersections, enclosed cavities, cooling paths or physical validity."}

def export(source,output,tolerance=1e-6,vehicle=None,flow=None,wing_offset_deg=0.0,wing_lift_m=0.0):
    if not math.isfinite(tolerance) or tolerance<=0:raise ValueError("positive weld tolerance required")
    pivots={};parts=load_glb(source,pivots,flow,wing_offset_deg,wing_lift_m);alignment=[0.0,0.0,0.0]
    vehicle_sha=None
    if vehicle is not None:
        vehicle_bytes=Path(vehicle).read_bytes();description=json.loads(vehicle_bytes)
        diffs=[]
        for wheel in description['wheels']:
            rest=pivots['susp_'+wheel['name']];attach=wheel['attachment_local']
            diffs.append([attach[i]-rest[i] for i in range(3)])
        if not diffs:raise ValueError('vehicle requires wheels for art alignment')
        alignment=[sum(d[i] for d in diffs)/len(diffs) for i in range(3)]
        if not all(math.isfinite(v) for v in alignment):raise ValueError('nonfinite vehicle alignment')
        # Body translation matches vehicle_visual.gd. Apply residual wheel bias
        # to every steer/wheel primitive, since the renderer biases suspension nodes.
        residuals={wheel['name']:[diffs[n][i]-alignment[i] for i in range(3)] for n,wheel in enumerate(description['wheels'])}
        adjusted=[]
        for name,triangles in parts:
            node=name.rsplit('_primitive_',1)[0]
            corner=next((key for key in residuals if node in ('steer_'+key,'wheel_'+key,'susp_'+key)),None)
            shift=[alignment[i]+(residuals[corner][i] if corner else 0) for i in range(3)]
            adjusted.append((name,[tuple(tuple(p[i]+shift[i] for i in range(3)) for p in tri) for tri in triangles]))
        parts=adjusted;vehicle_sha=hashlib.sha256(vehicle_bytes).hexdigest()
    output=Path(output);output.mkdir(parents=True,exist_ok=True)
    reports=[]
    with (output/"hypercar_rest_iso.obj").open("w",encoding="utf-8",newline="\n") as file:
        file.write("# Supplied visual rest pose, ISO axes. See audit.json for origin/alignment. NOT CFD-ready.\n")
        offset=1
        for name,triangles in parts:
            file.write("o "+name+"\n")
            for tri in triangles:
                for p in tri:file.write("v "+" ".join(format(c,".9g") for c in p)+"\n")
                file.write(f"f {offset} {offset+1} {offset+2}\n");offset+=3
            reports.append({"part":name,**audit(triangles,tolerance)})
    nodes={}
    for name,triangles in parts:nodes.setdefault(name.rsplit('_primitive_',1)[0],[]).extend(triangles)
    report={"format":"rg.cfd-geometry-audit/1","vehicle":"car_hyper","source_sha256":hashlib.sha256(Path(source).read_bytes()).hexdigest(),
            "obj_sha256":hashlib.sha256((output/"hypercar_rest_iso.obj").read_bytes()).hexdigest(),
            "frame":"ISO x forward,y left,z up; "+("chassis body origin" if vehicle else "rig ground origin, not chassis COM"),
            "body_alignment_iso_m":alignment,"vehicle_sha256":vehicle_sha,
            "flow_selection_sha256":hashlib.sha256(Path(flow).read_bytes()).hexdigest() if flow else None,
            "pose":f"closed doors, straight wheels, fixed suspension; wing pitch offset {wing_offset_deg:g} deg, lift {wing_lift_m:g} m",
            "actuator_pose":{"wing_offset_deg":wing_offset_deg,"wing_lift_m":wing_lift_m},
            "weld_tolerance_m":tolerance,"status":"requires_solid_preparation","parts":reports,
            "nodes":[{"node":name,**audit(triangles,tolerance)} for name,triangles in nodes.items()],
            "combined":audit([tri for _,triangles in parts for tri in triangles],tolerance)}
    (output/"audit.json").write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8")
    return report

if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("source");parser.add_argument("output");parser.add_argument("--weld-tolerance-m",type=float,default=1e-6)
    parser.add_argument('--vehicle',help='vehicle JSON for the same wheel-based chassis alignment as the visual model')
    parser.add_argument('--flow',help='source-bound exterior triangle selection; no hidden cabin/engine geometry')
    parser.add_argument("--wing-offset-deg",type=float,default=0.0)
    parser.add_argument("--wing-lift-m",type=float,default=0.0)
    args=parser.parse_args();report=export(args.source,args.output,args.weld_tolerance_m,args.vehicle,args.flow,args.wing_offset_deg,args.wing_lift_m)
    print(json.dumps(report["combined"],indent=2))
