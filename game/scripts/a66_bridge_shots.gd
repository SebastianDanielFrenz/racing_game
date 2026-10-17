extends SceneTree
# Fixed-camera regression shots of the A66/B8 crossing and eastbound merge at Unterliederbach.
# godot --path game --script res://scripts/a66_bridge_shots.gd -- --out <absolute output directory>
# Uses a separate derived store under the output directory; source data remains read-only.
const SurfaceMaterials := preload("res://scripts/surface_materials.gd")
var view: Node
var repo := ProjectSettings.globalize_path("res://").trim_suffix("/").get_base_dir()
var out := ""
var cam: Camera3D
func _initialize(): run.call_deferred()
func local(e: float,n: float,z: float)->Vector3:return Vector3(-(n-5551546.78),z-110.0,-(e-466651.65))
func run():
	var args:=OS.get_cmdline_user_args()
	for i in range(args.size()):
		if args[i]=="--out" and i+1<args.size():out=args[i+1]
	if out=="":out=repo.path_join("out/a66_bridge_shots")
	DirAccess.make_dir_recursive_absolute(out)
	var config:Dictionary=JSON.parse_string(FileAccess.get_file_as_string(repo.path_join("data/world/world_config_rhein_main_corridor.json")))
	config.session_origin_utm.zone=int(config.session_origin_utm.zone)
	for field in ["prefetch_margin_tiles","prefetch_max_tiles","loader_workers","max_tile_fills_per_tick"]:
		if config.physics.has(field):config.physics[field]=int(config.physics[field])
	config.derived_store.dir=out.path_join("derived")
	config.surface_map=repo.path_join("data/world/landclass_surfaces.json")
	config.palette=repo.path_join("data/world/landclass_palette.json")
	var config_path:=out.path_join("world.json")
	var config_file:=FileAccess.open(config_path,FileAccess.WRITE)
	config_file.store_string(JSON.stringify(config));config_file.close()
	var world:=Node3D.new();root.add_child(world)
	var env:=Environment.new();env.background_mode=Environment.BG_COLOR;env.background_color=Color(.55,.6,.67);env.ambient_light_source=Environment.AMBIENT_SOURCE_COLOR;env.ambient_light_color=Color.WHITE;env.ambient_light_energy=.7
	var we:=WorldEnvironment.new();we.environment=env;world.add_child(we)
	var sun:=DirectionalLight3D.new();sun.rotation_degrees=Vector3(-48,-35,0);world.add_child(sun)
	view=ClassDB.instantiate("RgTerrainView");world.add_child(view)
	if not view.initialize(config_path):push_error(view.get_last_error());quit(1);return
	view.set_render_origin(Vector3(2651.65,-7453.22,110))
	view.set_view_lod({"view_distance_m":1500.0,"terrain_detail":1.0,"road_detail":"high"})
	var mat:=ShaderMaterial.new();mat.shader=load("res://shaders/terrain.gdshader");SurfaceMaterials.shared().apply_terrain(mat);view.set_material(mat.get_rid())
	view.set_upload_budget_per_frame(12);view.load_preview(2651.65,-7453.22)
	var roadmat:=ShaderMaterial.new();roadmat.shader=load("res://shaders/road.gdshader");SurfaceMaterials.shared().apply_road(roadmat)
	for xy in [Vector2i(455,5421),Vector2i(456,5421),Vector2i(455,5420),Vector2i(456,5420)]:
		var d:Dictionary=view.get_road_visual_tile(xy.x,xy.y,.018)
		if d.vertices.is_empty():continue
		var arrays:Array=[];arrays.resize(Mesh.ARRAY_MAX)
		arrays[Mesh.ARRAY_VERTEX]=d.vertices;arrays[Mesh.ARRAY_NORMAL]=d.normals;arrays[Mesh.ARRAY_TEX_UV]=d.uv;arrays[Mesh.ARRAY_TEX_UV2]=d.uv2;arrays[Mesh.ARRAY_COLOR]=d.colours;arrays[Mesh.ARRAY_INDEX]=d.indices
		var mesh:=ArrayMesh.new();mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES,arrays)
		var mi:=MeshInstance3D.new();mi.mesh=mesh;mi.material_override=roadmat;var o:Vector3=d.origin_session-Vector3(2651.65,-7453.22,110);mi.position=Vector3(-o.y,o.z,-o.x);world.add_child(mi)
	cam=Camera3D.new();world.add_child(cam);cam.far=2200;cam.fov=65
	for i in range(160):await process_frame
	DirAccess.make_dir_recursive_absolute(out)
	var shots=[ [Vector3(466625,5551485,114),Vector3(466680,5551560,115)], [Vector3(466430,5551430,121),Vector3(466750,5551600,120)], [Vector3(466660,5551440,155),Vector3(466650,5551570,113)], [Vector3(466760,5551570,111),Vector3(466730,5551605,114)], [Vector3(466651,5551547,111),Vector3(466700,5551580,113)] ]
	for i in range(shots.size()):
		cam.look_at_from_position(local(shots[i][0].x,shots[i][0].y,shots[i][0].z),local(shots[i][1].x,shots[i][1].y,shots[i][1].z))
		for f in range(12):await process_frame
		await RenderingServer.frame_post_draw
		root.get_texture().get_image().save_png(out+"/shot_%d.png"%i)
	print("A66_SHOTS "+out);view.release();quit()
