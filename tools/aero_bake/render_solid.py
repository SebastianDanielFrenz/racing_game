"""Blender background review render of a gated CFD solid; never edits the visual model."""
import hashlib
import json
from pathlib import Path
import sys
import bpy
from mathutils import Vector

folder=Path(sys.argv[sys.argv.index('--')+1]);image=Path(sys.argv[sys.argv.index('--')+2])
manifest=json.loads((folder/'manifest.json').read_text());stl=folder/'hypercar_cfd_rest.stl'
if hashlib.sha256(stl.read_bytes()).hexdigest()!=manifest['stl_sha256']:raise ValueError('solid hash changed')
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.wm.stl_import(filepath=str(stl.resolve()),forward_axis='Y',up_axis='Z')
material=bpy.data.materials.new('CFD exterior approximation');material.diffuse_color=(.3,.46,.60,1)
material.use_nodes=True;bsdf=material.node_tree.nodes.get('Principled BSDF');bsdf.inputs['Base Color'].default_value=(.3,.46,.60,1);bsdf.inputs['Roughness'].default_value=.5
for obj in list(bpy.context.scene.objects):
    if obj.type=='MESH':obj.data.materials.append(material)
world=bpy.data.worlds.new('Review background');bpy.context.scene.world=world;world.use_nodes=True
world.node_tree.nodes['Background'].inputs[0].default_value=(.22,.22,.22,1);world.node_tree.nodes['Background'].inputs[1].default_value=.6
for position,power,size in [((4,-4,6),1600,5),((-3,4,5),1000,4)]:
    data=bpy.data.lights.new('Review light','AREA');data.energy=power;data.shape='DISK';data.size=size
    light=bpy.data.objects.new('Review light',data);bpy.context.collection.objects.link(light);light.location=position
    light.rotation_euler=(Vector((0,0,0))-light.location).to_track_quat('-Z','Y').to_euler()
camera_data=bpy.data.cameras.new('Review camera');camera=bpy.data.objects.new('Review camera',camera_data)
bpy.context.collection.objects.link(camera);camera.location=(6,-6,3)
camera.rotation_euler=(Vector((0,0,0))-camera.location).to_track_quat('-Z','Y').to_euler()
camera_data.type='ORTHO';camera_data.ortho_scale=6.4;bpy.context.scene.camera=camera
scene=bpy.context.scene;scene.render.engine='CYCLES';scene.cycles.device='CPU';scene.cycles.samples=16
scene.render.resolution_x=1280;scene.render.resolution_y=720;scene.render.resolution_percentage=100
scene.render.image_settings.file_format='PNG';scene.render.filepath=str(image.resolve())
bpy.ops.render.render(write_still=True)
