"""Validate generated cockpit assets and byte-identical regeneration. No engine build.

python tools/cockpitgen/validate.py
"""
import hashlib
import json
import math
from pathlib import Path
import struct
import tempfile
from build import build, REPO


def validate(directory):
    raw=(directory/"cockpit_parts.glb").read_bytes()
    magic,version,total=struct.unpack_from("<III",raw)
    assert magic==0x46546c67 and version==2 and total==len(raw)
    size,kind=struct.unpack_from("<II",raw,12)
    assert kind==0x4e4f534a
    gltf=json.loads(raw[20:20+size])
    rig=json.loads((directory/"cockpit_parts.rig.json").read_text(encoding="utf-8"))
    spec=json.loads((REPO/"data/cockpits/parts_spec.json").read_text(encoding="utf-8"))
    nodes={n["name"]:n for n in rig["nodes"]}
    gn={n["name"]:n for n in gltf["nodes"]}
    assert len(nodes)==len(rig["nodes"])==len(gn)
    part_ids={p["id"] for p in rig["cockpit_parts"]["parts"]}
    required={b["id"] for b in spec["buttons"]}|{"start","epb","n2o","shifter_seq","shifter_h","shifter_sel","paddle_L","paddle_R","stalk_L","stalk_R","lights","horn","blank_cap"}
    assert part_ids==required,(part_ids,required)
    assert len(part_ids)==len(rig["cockpit_parts"]["parts"])
    parents={}
    for i,node in enumerate(gltf["nodes"]):
        assert node.get("rotation",[0,0,0,1])==[0,0,0,1]
        assert node.get("scale",[1,1,1])==[1,1,1]
        for child in node.get("children",[]):
            assert child not in parents,"multiple parents"
            parents[child]=i
    for p in rig["cockpit_parts"]["parts"]:
        assert nodes[p["root"]]["parent"]=="cockpit_parts"
        assert nodes[p["root"]]["pivot"]==[0,0,0]
    joints=hits=0
    for name,n in nodes.items():
        if "joint" in n:
            joints+=1
            j=n["joint"]
            assert j["min"]<=0<=j["max"]
            assert abs(sum(a*a for a in j["axis"])-1)<1e-9
            assert gn[name]["extras"]["joint"]["axis"]==[j["axis"][1],j["axis"][2],j["axis"][0]]
        if "interaction" in n:
            hits+=1
            h=n["interaction"]
            assert h["kind"] in ("poke","grab")
            assert all(v>0 for v in h["half_extents_m"])
            assert all(math.isfinite(v) for v in h["press_axis"])
            ancestor=n["parent"]
            assert ancestor in nodes and "joint" in nodes[ancestor]
            if "requires" in h: assert h["requires"]["node"] in nodes
    assert "interaction" not in nodes["part_blank_cap"]
    assert not any(n.startswith("hit_blank_cap") for n in nodes)
    assert all("lamp_tc_led"+str(i) in {m["name"] for m in gltf["materials"]} for i in range(5))
    materials={m["name"]:m for m in gltf["materials"]}
    for light in rig["lights"].values():
        for name in light["materials"]:
            assert name in materials
            assert materials[name].get("emissiveFactor",[0,0,0])==[0,0,0]
    model=build()
    tris=model.mb.triangle_count()
    assert tris<=spec["triangle_budget"]
    # Geometry validation is independent of export: rejects NaNs, missing nodes,
    # zero-area faces, bad indices and wrong-length normals.
    for (node,mat),prim in model.mb.prims.items():
        assert node in nodes and mat in materials
        assert len(prim.pos)==len(prim.nrm)==len(prim.uv)
        assert all(math.isfinite(v) for p in prim.pos for v in p)
        assert all(abs(sum(v*v for v in n)-1)<1e-5 for n in prim.nrm)
        for a,b,c in zip(prim.idx[::3],prim.idx[1::3],prim.idx[2::3]):
            assert max(a,b,c)<len(prim.pos)
            u=[prim.pos[b][i]-prim.pos[a][i] for i in range(3)]
            v=[prim.pos[c][i]-prim.pos[a][i] for i in range(3)]
            area=sum(q*q for q in (u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]))
            assert area>1e-24,(node,mat,"degenerate triangle")
    with tempfile.TemporaryDirectory(prefix="rg_cockpit_determinism_") as tmp:
        model.write(tmp)
        for filename in ("cockpit_parts.glb","cockpit_parts.rig.json"):
            assert (directory/filename).read_bytes()==(Path(tmp)/filename).read_bytes(),filename+" is stale or nondeterministic"
    print(f"PASS {len(part_ids)} parts, {joints} joints, {hits} interaction volumes, {tris}/{spec['triangle_budget']} triangles; deterministic GLB + rig")
    print("sha256",hashlib.sha256(raw).hexdigest())


if __name__=="__main__": validate(REPO/"data/models/cockpit_parts")
