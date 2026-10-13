"""Independent Blender import + contact sheet of the actual GLB, not a mockup.

blender -b --factory-startup --python-exit-code 1 -P tools/cockpitgen/render_sheet.py -- out/cockpit_parts_sheet.png
Optional second argument 'posed' shows moving joints, including open N2O guard.
"""
import json
import math
from pathlib import Path
import sys
import bpy
from mathutils import Vector,Quaternion

REPO=Path(__file__).resolve().parents[2]
args=sys.argv[sys.argv.index("--")+1:]
out=Path(args[0]); out=out if out.is_absolute() else REPO/out
posed=len(args)>1 and args[1]=="posed"
rig=json.loads((REPO/"data/models/cockpit_parts/cockpit_parts.rig.json").read_text(encoding="utf-8"))
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.gltf(filepath=str(REPO/"data/models/cockpit_parts/cockpit_parts.glb"))
objects={o.name:o for o in bpy.data.objects}
for n in rig["nodes"]: assert n["name"] in objects,n["name"]
triangle_count=sum(len(p.vertices)-2 for o in objects.values() if o.type=="MESH" for p in o.data.polygons)
spec=json.loads((REPO/"data/cockpits/parts_spec.json").read_text(encoding="utf-8"))
assert triangle_count<=spec["triangle_budget"],triangle_count
def iso(v): return Vector((v[1],-v[0],v[2]))
if posed:
    for n in rig["nodes"]:
        if "joint" not in n: continue
        j=n["joint"]; o=objects[n["name"]]
        if j["type"]=="prismatic": o.location+=iso(j["axis"])*j["max"]
        else:
            angle=j["max"] if n["name"]=="ctl_n2o_cover" else j["max"]*.65
            o.rotation_mode="QUATERNION"; o.rotation_quaternion=Quaternion(iso(j["axis"]),angle)

def material(name,color,metal=0):
    mat=bpy.data.materials.new(name); mat.diffuse_color=(*color,1);mat.use_nodes=True
    bsdf=mat.node_tree.nodes.get("Principled BSDF"); bsdf.inputs["Base Color"].default_value=(*color,1)
    bsdf.inputs["Metallic"].default_value=metal; bsdf.inputs["Roughness"].default_value=.5
    return mat
white=material("sheet_text",(.8,.85,.9)); floor=material("sheet_tile",(.022,.028,.038))
def text(label,location,size=.013):
    curve=bpy.data.curves.new(label,"FONT");curve.body=label;curve.align_x="CENTER";curve.size=size
    obj=bpy.data.objects.new(label,curve);bpy.context.collection.objects.link(obj);obj.location=location
    curve.materials.append(white)

# Independent framing per part: small buttons and large levers remain readable.
columns=5; spacing=.26; rows=math.ceil(len(rig["cockpit_parts"]["parts"])/columns)
for i,p in enumerate(rig["cockpit_parts"]["parts"]):
    root=objects[p["root"]]; root.parent=None
    # View from the driver (-X ISO), so the forward-facing legends read upright.
    root.rotation_mode="XYZ"; root.rotation_euler.z=math.pi
    cx=(i%columns-(columns-1)/2)*spacing; cy=((rows-1)/2-i//columns)*spacing
    bpy.context.view_layer.update()
    descendants=[root,*root.children_recursive]
    points=[o.matrix_world@Vector(c) for o in descendants if o.type=="MESH" for c in o.bound_box]
    low=Vector(tuple(min(p[k] for p in points) for k in range(3)))
    high=Vector(tuple(max(p[k] for p in points) for k in range(3)))
    scale=.12/max(high-low)
    root.scale=(scale,scale,scale)
    root.location=(cx-(low.x+high.x)/2*scale,cy-(low.y+high.y)/2*scale,.008-low.z*scale)
    bpy.ops.mesh.primitive_cube_add(size=1,location=(cx,cy,.001))
    tile=bpy.context.object; tile.dimensions=(.245,.245,.008);tile.data.materials.append(floor)
    bevel=tile.modifiers.new("tile edge","BEVEL");bevel.width=.006;bevel.segments=2
    text(p["title"],(cx,cy-.100,.008),.010)
    text(p["id"],(cx,cy-.117,.008),.007)

scene=bpy.context.scene; scene.render.engine="CYCLES";scene.cycles.samples=24
scene.render.resolution_x=2000;scene.render.resolution_y=1900;scene.render.resolution_percentage=100
scene.world=bpy.data.worlds.new("sheet_world");scene.world.use_nodes=True
scene.world.node_tree.nodes["Background"].inputs[0].default_value=(.09,.11,.15,1)
scene.world.node_tree.nodes["Background"].inputs[1].default_value=.6
for pos,energy,size in [((-.8,-.5,2),200,1.6),((1,.5,1.5),120,1.3)]:
    data=bpy.data.lights.new("softbox","AREA");data.energy=energy;data.shape="DISK";data.size=size
    obj=bpy.data.objects.new("softbox",data);bpy.context.collection.objects.link(obj);obj.location=pos
    obj.rotation_euler=(-obj.location).to_track_quat("-Z","Y").to_euler()
camera_data=bpy.data.cameras.new("sheet_camera");camera=bpy.data.objects.new("sheet_camera",camera_data)
bpy.context.collection.objects.link(camera);camera.location=(0,-.9,2.7)
camera.rotation_euler=(Vector((0,0,.03))-camera.location).to_track_quat("-Z","Y").to_euler()
camera_data.type="ORTHO";camera_data.ortho_scale=1.53;scene.camera=camera
scene.view_settings.view_transform="AgX"
out.parent.mkdir(parents=True,exist_ok=True);scene.render.filepath=str(out)
print("BLENDER_COCKPIT_IMPORT PASS",len(rig["cockpit_parts"]["parts"]),"parts",len(objects),"nodes",triangle_count,"triangles")
bpy.ops.render.render(write_still=True)
