extends SceneTree
# Car draw-call / LOD bench (perf, 2026-10-15): loads one car model the way vehicle_visual.gd does (GLB, car_lods.gd,
# car_mesh_merge.gd), lights it with a shadowed sun, and renders it from 8, 30 and 100 m. Prints per distance the root
# viewport's draw calls and primitives and saves a screenshot, so LOD switching and the merge can be checked without a
# Session:
#   godot --path game --script res://scripts/car_lod_bench.gd -- --model=data/models/pico/pico.glb --out=DIR [--tier=high]
# --tier=full / RG_CAR_MERGE=0 turn the two steps off for an A/B.
# Prints: RG_CAR_LOD_BENCH model=<file> distance_m=<d> draw_calls=<n> primitives=<n>

func _arg(name: String, fallback: String) -> String:
	for a in OS.get_cmdline_user_args():
		if a.begins_with("--%s=" % name):
			return a.substr(name.length() + 3)
	return fallback

func _initialize() -> void:
	var model_path := ProjectSettings.globalize_path("res://../" + _arg("model", "data/models/pico/pico.glb")).simplify_path()
	var out := _arg("out", "")
	var world := Node3D.new()
	root.add_child(world)
	var env := WorldEnvironment.new()
	env.environment = Environment.new()
	env.environment.background_mode = Environment.BG_COLOR
	env.environment.background_color = Color(0.45, 0.55, 0.7)
	env.environment.ambient_light_color = Color(0.5, 0.5, 0.55)
	env.environment.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	world.add_child(env)
	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-50, 30, 0)
	sun.shadow_enabled = true
	sun.directional_shadow_max_distance = 300.0
	world.add_child(sun)
	var ground := MeshInstance3D.new()
	var plane := PlaneMesh.new()
	plane.size = Vector2(400, 400)
	ground.mesh = plane
	world.add_child(ground)
	var doc := GLTFDocument.new()
	var state := GLTFState.new()
	if doc.append_from_file(model_path, state) != OK:
		print("RG_CAR_LOD_BENCH cannot load %s" % model_path)
		quit(1)
		return
	var model := doc.generate_scene(state) as Node3D
	world.add_child(model)
	print("RG_CAR_LOD_BENCH lod_parts=%d" % preload("res://scripts/car_lods.gd").attach(model, model_path, _arg("tier", "high")))
	if OS.get_environment("RG_CAR_MERGE") != "0":
		var cockpit := model_path.get_basename() + ".cockpit.json"
		print("RG_CAR_LOD_BENCH merge=%s" % str(preload("res://scripts/car_mesh_merge.gd").merge(model, model_path.get_basename() + ".rig.json", cockpit)))
	var cam := Camera3D.new()
	cam.fov = 40.0
	cam.far = 1000.0
	world.add_child(cam)
	for d in [8.0, 30.0, 100.0]:
		# The field of view narrows with distance so the car fills the same part of each picture (LOD by distance only).
		cam.look_at_from_position(Vector3(d * 0.6, d * 0.35, d * 0.75), Vector3(0, 0.6, 0))
		cam.fov = 40.0 * 8.0 / d
		for i in 4:
			await process_frame
		print("RG_CAR_LOD_BENCH model=%s distance_m=%.0f draw_calls=%d primitives=%d" % [model_path.get_file(), d,
			root.get_render_info(Viewport.RENDER_INFO_TYPE_VISIBLE, Viewport.RENDER_INFO_DRAW_CALLS_IN_FRAME) + root.get_render_info(Viewport.RENDER_INFO_TYPE_SHADOW, Viewport.RENDER_INFO_DRAW_CALLS_IN_FRAME),
			root.get_render_info(Viewport.RENDER_INFO_TYPE_VISIBLE, Viewport.RENDER_INFO_PRIMITIVES_IN_FRAME) + root.get_render_info(Viewport.RENDER_INFO_TYPE_SHADOW, Viewport.RENDER_INFO_PRIMITIVES_IN_FRAME)])
		if out != "":
			DirAccess.make_dir_recursive_absolute(out)
			root.get_texture().get_image().save_png(out.path_join("car_%03d.png" % int(d)))
	quit()
