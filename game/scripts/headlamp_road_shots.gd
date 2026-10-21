extends SceneTree
# Night road review shots of the headlamp beams (2026-10-21, "the light is not bright enough at night
# or goes far enough"). Needs a renderer and RgDriverActions (the GDExtension):
#   godot --path game --script res://scripts/headlamp_road_shots.gd -- --model <glb> --out <dir> [--tag t]
#       [--exposure E]   (default: headlights.json fallback_exposure = the night exposure)
# A flat dark-asphalt road (albedo 0.12) with white lane dashes every 12 m and 1 m posts every 25 m on
# both sides out to 200 m, a moonless sky, the car's own lamps through exterior_lamps.gd / headlights.gd.
# Shots: chase view with dipped beam (cabin.lights 1) and main beam (cabin.lights 2).
const LightExposure := preload("res://scripts/light_exposure.gd")

class FakeSim extends Node:
	var channels := {"cabin.lights": 1.0, "cabin.brake_lamp": 0.0, "brake": 0.0, "gear": 1.0}
	func get_control(c: String) -> float: return float(channels.get(c, 0.0))
	func get_vehicle_powertrain(_n: String) -> Dictionary: return {"gear": int(channels["gear"]), "engine_state": "Running"}

func _initialize() -> void:
	var args := OS.get_cmdline_user_args()
	var model_path := ""; var out := "."; var tag := "now"; var exposure := -1.0
	for i in range(args.size()):
		match args[i]:
			"--model": model_path = args[i + 1]
			"--out": out = args[i + 1]
			"--tag": tag = args[i + 1]
			"--exposure": exposure = float(args[i + 1])
	var world := Node3D.new(); root.add_child(world)
	var env := Environment.new()
	env.background_mode = Environment.BG_COLOR
	env.background_color = Color(0.004, 0.006, 0.012)
	env.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	env.ambient_light_color = Color(0.02, 0.025, 0.04); env.ambient_light_energy = 0.15
	var we := WorldEnvironment.new(); we.environment = env; world.add_child(we)
	var asphalt := StandardMaterial3D.new(); asphalt.albedo_color = Color(0.12, 0.12, 0.12); asphalt.roughness = 0.9
	var paint := StandardMaterial3D.new(); paint.albedo_color = Color(0.8, 0.8, 0.78); paint.roughness = 0.8
	var post_mat := StandardMaterial3D.new(); post_mat.albedo_color = Color(0.7, 0.7, 0.7)
	var ground := MeshInstance3D.new(); var pm := PlaneMesh.new(); pm.size = Vector2(60, 600); ground.mesh = pm
	ground.material_override = asphalt; ground.position = Vector3(0, 0, 250); world.add_child(ground)
	for z in range(6, 240, 12):
		for x in [-3.5, 0.0, 3.5]:
			var dash := MeshInstance3D.new(); var bm := BoxMesh.new(); bm.size = Vector3(0.15, 0.01, 3.0); dash.mesh = bm
			dash.material_override = paint; dash.position = Vector3(x, 0.005, z); world.add_child(dash)
	for z in range(25, 225, 25):
		for x in [-5.0, 5.0]:
			var post := MeshInstance3D.new(); var cm := BoxMesh.new(); cm.size = Vector3(0.12, 1.0, 0.12); post.mesh = cm
			post.material_override = post_mat; post.position = Vector3(x, 0.5, z); world.add_child(post)
	var doc := GLTFDocument.new(); var st := GLTFState.new()
	if doc.append_from_file(model_path, st) != OK:
		push_error("cannot load " + model_path); quit(2); return
	var model: Node3D = doc.generate_scene(st)
	var carroot := Node3D.new(); world.add_child(carroot); carroot.add_child(model)
	var sim := FakeSim.new(); root.add_child(sim)
	var car := model_path.get_file().get_basename()
	load("res://scripts/exterior_lamps.gd").attach_to(model, sim, car, model_path)
	LightExposure.exposure = exposure
	var cam := Camera3D.new(); cam.fov = 60; cam.far = 600; world.add_child(cam); cam.current = true
	cam.look_at_from_position(Vector3(0.0, 2.0, -5.5), Vector3(0, 0.3, 40))
	for level in [1, 2]:
		sim.channels["cabin.lights"] = float(level)
		for i in 8: await process_frame
		await RenderingServer.frame_post_draw
		var file := "%s/%s_%s_%s.png" % [out, tag, car, "dipped" if level == 1 else "main"]
		get_root().get_viewport().get_texture().get_image().save_png(file)
		print("shot ", file, " exposure ", LightExposure.current({"fallback_exposure": 0.0942}) if exposure > 0.0 else "fallback")
	quit()
