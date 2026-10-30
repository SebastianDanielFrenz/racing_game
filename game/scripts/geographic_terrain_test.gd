extends SceneTree
# Optional real-data integration probe. Prepare an elevation store first and set
# RG_ELEVATION_SOURCE_STORE (legacy RG_GLO30_SOURCE_STORE also works).
# Scope, layer and UTM32 L2 coordinates have optional RG_ELEVATION_* overrides.
# Does not touch player configuration.
var view: Node
func _initialize() -> void:
	run.call_deferred()
func run() -> void:
	var store := OS.get_environment("RG_ELEVATION_SOURCE_STORE")
	if store.is_empty():
		store = OS.get_environment("RG_GLO30_SOURCE_STORE")
	if store.is_empty():
		print("GEOGRAPHIC_TERRAIN_TEST skipped: set RG_ELEVATION_SOURCE_STORE")
		quit()
		return
	var repo := ProjectSettings.globalize_path("res://").trim_suffix("/").get_base_dir()
	var output := repo.path_join("out/geographic_terrain_probe")
	DirAccess.make_dir_recursive_absolute(output)
	var config: Dictionary = JSON.parse_string(FileAccess.get_file_as_string(repo.path_join("data/world/world_config_rhein_main_corridor.json")))
	var scope := OS.get_environment("RG_ELEVATION_SOURCE_SCOPE")
	var layer := OS.get_environment("RG_ELEVATION_SOURCE_LAYER")
	if scope.is_empty():
		scope = "glo30-probe-v1"
	if layer.is_empty():
		layer = "g2m.src.elev.glo30"
	var tile_x := 455 if OS.get_environment("RG_ELEVATION_TILE_X").is_empty() else int(OS.get_environment("RG_ELEVATION_TILE_X"))
	var tile_y := 5403 if OS.get_environment("RG_ELEVATION_TILE_Y").is_empty() else int(OS.get_environment("RG_ELEVATION_TILE_Y"))
	var e := tile_x*1024+512
	var n := tile_y*1024+512
	var e0 := int(floor(float(e)/1000.0))*1000
	var n0 := int(floor(float(n)/1000.0))*1000
	config.source_store = {"dir":store,"scope":scope,"read_only":true,"elevation_layer":layer}
	config.derived_store = {"dir":output.path_join("derived"),"name":"tiles"}
	config.session_origin_utm = {"zone":32,"e0":e0,"n0":n0}
	config.spawn = {"e":e,"n":n,"yaw_deg":0}
	config.lod.max_distance_m = 400.0
	for key in ["prefetch_margin_tiles","prefetch_max_tiles","loader_workers","max_tile_fills_per_tick"]:
		if config.physics.has(key):
			config.physics[key] = int(config.physics[key])
	config.physics.road_surfaces.enabled = false
	config.buildings.enabled = false
	config.road_visuals.enabled = false
	config.surface_map = repo.path_join("data/world/landclass_surfaces.json")
	config.palette = repo.path_join("data/world/landclass_palette.json")
	var file := FileAccess.open(output.path_join("world.json"),FileAccess.WRITE)
	file.store_string(JSON.stringify(config))
	file.close()
	view = ClassDB.instantiate("RgTerrainView")
	root.add_child(view)
	if not view.initialize(output.path_join("world.json")) or not view.load_preview(float(e-e0),float(n-n0)):
		push_error(view.get_last_error())
		view.release()
		quit(1)
		return
	view.set_render_origin(Vector3(e-e0,n-n0,0))
	for i in range(20):
		await process_frame
	var vertices: int = view.get_total_vertex_count()
	print("GEOGRAPHIC_TERRAIN_TEST chunks=%d vertices=%d" % [view.get_chunk_count(),vertices])
	view.release()
	quit(0 if vertices > 0 else 1)
