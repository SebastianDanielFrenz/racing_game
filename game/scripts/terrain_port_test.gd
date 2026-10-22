extends SceneTree
# Exercises the actual native bindings and the shipping building upload path on
# a pinned Unterliederbach tile. No player settings or shared Godot log changes.
var view: Node
var repo := ProjectSettings.globalize_path("res://").trim_suffix("/").get_base_dir()
var out := ""
func _initialize() -> void:
	run.call_deferred()
func fail(message: String) -> void:
	push_error(message)
	if view != null:
		view.release()
	quit(1)
func run() -> void:
	out = repo.path_join("out/terrain_port_probe")
	DirAccess.make_dir_recursive_absolute(out)
	var config: Dictionary = JSON.parse_string(FileAccess.get_file_as_string(repo.path_join("data/world/world_config_rhein_main_corridor.json")))
	var source_store := OS.get_environment("RG_TERRAIN_PORT_SOURCE_STORE")
	if source_store.is_empty():
		source_store = "S:/claude_code/geo2map_cache/rhein-main-corridor-r2"
	config.source_store.dir = source_store
	config.derived_store.dir = out.path_join("derived")
	config.surface_map = repo.path_join("data/world/landclass_surfaces.json")
	config.palette = repo.path_join("data/world/landclass_palette.json")
	config.session_origin_utm.zone = int(config.session_origin_utm.zone)
	for key in ["prefetch_margin_tiles","prefetch_max_tiles","loader_workers","max_tile_fills_per_tick"]:
		if config.physics.has(key):
			config.physics[key] = int(config.physics[key])
	var file := FileAccess.open(out.path_join("world.json"),FileAccess.WRITE)
	file.store_string(JSON.stringify(config))
	file.close()
	var world := Node3D.new()
	root.add_child(world)
	view = ClassDB.instantiate("RgTerrainView")
	world.add_child(view)
	if not view.initialize(out.path_join("world.json")):
		fail(view.get_last_error())
		return
	var start := Time.get_ticks_usec()
	var data: Dictionary = view.get_building_mesh_tile(455,5421,0.0,8.0,3.0,50.0)
	var elapsed := (Time.get_ticks_usec()-start)/1000.0
	if int(data.get("building_count",0)) == 0:
		fail("No buildings in the real-store fixture")
		return
	var uploader := preload("res://scripts/building_stream.gd").new()
	var vertex_count := 0
	for key in ["near","tall"]:
		var surfaces: Array = data[key]
		for surface: Dictionary in surfaces:
			var arrays: Array = surface.arrays
			vertex_count += arrays[0].size()
			if arrays[0].size() != arrays[1].size() or arrays[0].size() != arrays[2].size() or arrays[0].size() != arrays[3].size():
				fail("Mismatched native material arrays")
				return
			for p: Vector3 in arrays[0]:
				if not p.is_finite():
					fail("Non-finite building vertex")
					return
		var mesh: ArrayMesh = uploader._mesh(surfaces)
		if mesh != null:
			var instance := MeshInstance3D.new()
			instance.mesh = mesh
			world.add_child(instance)
	uploader.free()
	start = Time.get_ticks_usec()
	var road: Dictionary = view.get_road_visual_tile(455,5421,.018)
	var road_ms := (Time.get_ticks_usec()-start)/1000.0
	if not bool(road.get("native_paint_complete",false)):
		fail("Native paint exceeded its bounds or rejected the real-store mesh")
		return
	print("TERRAIN_PORT buildings=%d windows=%d unsupported_roofs=%d vertices=%d building_ms=%.3f paint_triangles=%d road_ms=%.3f" % [data.building_count,data.windows,data.unsupported_roofs,vertex_count,elapsed,road.paint_triangles,road_ms])
	print("TERRAIN_PORT models pitched=%d doors=%d chimneys=%d mapped_directions=%d supported_directions=%d" % [data.pitched_roofs,data.doors,data.chimneys,data.mapped_roof_directions,data.supported_roof_directions])
	if DisplayServer.get_name() != "headless":
		var environment := Environment.new()
		environment.background_mode = Environment.BG_COLOR
		environment.background_color = Color(.25,.32,.4)
		environment.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
		environment.ambient_light_color = Color.WHITE
		environment.ambient_light_energy = .55
		var we := WorldEnvironment.new()
		we.environment = environment
		world.add_child(we)
		var sun := DirectionalLight3D.new()
		sun.rotation_degrees = Vector3(-55,-35,0)
		world.add_child(sun)
		var camera := Camera3D.new()
		camera.far = 2500
		world.add_child(camera)
		camera.look_at_from_position(Vector3(-750,700,-750),Vector3(-512,100,-512))
		for i in range(8):
			await process_frame
		await RenderingServer.frame_post_draw
		root.get_texture().get_image().save_png(out.path_join("buildings.png"))
		# Close view of an actual mapped residential building: texture scale,
		# roof form and projecting facade models must be visible here.
		var footprints: Dictionary = view.get_building_tile(455,5421,0.0,8.0,3.0)
		for building: Dictionary in footprints.get("buildings",[]):
			if str(building.type) not in ["house","detached","semidetached_house","residential"] or building.outers.is_empty():
				continue
			var ring: PackedVector2Array = building.outers[0]
			var center := Vector2.ZERO
			for p: Vector2 in ring:
				center += p
			center /= ring.size()
			var target := Vector3(-center.y,float(building.base)+float(building.height)*.5,-center.x)
			camera.look_at_from_position(target+Vector3(20,14,24),target)
			for i in range(6):
				await process_frame
			await RenderingServer.frame_post_draw
			root.get_texture().get_image().save_png(out.path_join("building_close.png"))
			break
	view.release()
	print("TERRAIN_PORT_TEST ok")
	quit()
