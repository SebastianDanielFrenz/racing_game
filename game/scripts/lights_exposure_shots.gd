extends SceneTree
# Day/dusk/night lamp review shots with the scene exposure (lights/exposure, 2026-10-15). Needs a
# renderer (xvfb works) and RgDriverActions (the GDExtension, or a stub for a Linux run):
#   godot --path game --rendering-driver opengl3 --script res://scripts/lights_exposure_shots.gd -- \
#       --states <json> --model <glb> --out <dir> [--tag after] [--legacy]
# <json>: the sky states rg_core computes for the shots (tests/unit/test_exposure.cpp
# "[exposure_shots]" writes it: sun/ambient/sky values, illuminance, exposure, auto_lights).
# For each state: the car from behind with the brakes on and the lamps the automatic headlamps
# chose ("rear"), and from the front with dipped beam forced on ("front"). The sky is applied as
# sky_controller.gd applies it (tuned look, environment exposure 1); the lamps go through the
# game's own exterior_lamps.gd / headlights.gd with light_exposure.gd set to the state's exposure.
# --legacy: no exposure is written (lamps at their fixed values) and the headlamps stay as the
# driver left them (off) - the behaviour before automatic headlamps, for "before" shots.
const LightExposure := preload("res://scripts/light_exposure.gd")

class FakeSim extends Node:
	var channels := {"cabin.lights": 0.0, "cabin.brake_lamp": 1.0, "brake": 1.0, "gear": 1.0}
	func get_control(c: String) -> float: return float(channels.get(c, 0.0))
	func get_vehicle_powertrain(_n: String) -> Dictionary: return {"gear": int(channels["gear"]), "engine_state": "Running"}

func _initialize() -> void:
	var args := OS.get_cmdline_user_args()
	var states_path := ""; var model_path := ""; var out := "/tmp"; var tag := "after"; var legacy := false
	for i in range(args.size()):
		match args[i]:
			"--states": states_path = args[i + 1]
			"--model": model_path = args[i + 1]
			"--out": out = args[i + 1]
			"--tag": tag = args[i + 1]
			"--legacy": legacy = true
	var states = JSON.parse_string(FileAccess.get_file_as_string(states_path))
	if not states is Array:
		push_error("no states in " + states_path)
		quit(2)
		return
	var world := Node3D.new(); root.add_child(world)
	var env := Environment.new()
	env.background_mode = Environment.BG_SKY
	env.sky = Sky.new()
	var sky_mat := ProceduralSkyMaterial.new()
	env.sky.sky_material = sky_mat
	env.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	var we := WorldEnvironment.new(); we.environment = env; world.add_child(we)
	var sun := DirectionalLight3D.new(); sun.shadow_enabled = true; world.add_child(sun)
	var ground := MeshInstance3D.new(); var pm := PlaneMesh.new(); pm.size = Vector2(800, 800); ground.mesh = pm
	var gm := StandardMaterial3D.new(); gm.albedo_color = Color(0.36, 0.36, 0.37); gm.roughness = 0.9; ground.material_override = gm
	world.add_child(ground)
	var doc := GLTFDocument.new(); var st := GLTFState.new()
	if doc.append_from_file(model_path, st) != OK:
		push_error("cannot load " + model_path); quit(2); return
	var model: Node3D = doc.generate_scene(st)
	var carroot := Node3D.new(); world.add_child(carroot); carroot.add_child(model)
	var sim := FakeSim.new(); root.add_child(sim)
	var car := model_path.get_file().get_basename()
	load("res://scripts/exterior_lamps.gd").attach_to(model, sim, car, model_path)
	var cam := Camera3D.new(); cam.fov = 55; cam.far = 600; world.add_child(cam); cam.current = true
	await process_frame
	for s in states:
		var top := _rgb(s["sky_top_rgb"]); var horizon := _rgb(s["sky_horizon_rgb"])
		sky_mat.sky_top_color = top; sky_mat.sky_horizon_color = horizon
		sky_mat.ground_horizon_color = horizon.darkened(0.2); sky_mat.ground_bottom_color = top.darkened(0.6)
		env.ambient_light_color = top.lerp(horizon, 0.5); env.ambient_light_energy = float(s["ambient_energy"])
		var el := deg_to_rad(maxf(float(s["sun_elevation_deg"]), 2.0))
		sun.basis = Basis.looking_at(-Vector3(cos(el) * 0.6, sin(el), cos(el) * 0.8), Vector3.UP)
		sun.light_energy = float(s["sun_energy"]); sun.light_color = _rgb(s["sun_rgb"]); sun.visible = sun.light_energy > 0.0
		LightExposure.exposure = -1.0 if legacy else float(s["exposure"])
		for view in ["rear", "front"]:
			var lights := 0.0 if legacy else float(s["auto_lights"])
			if view == "front":
				lights = 1.0
			sim.channels["cabin.lights"] = lights
			sim.channels["cabin.brake_lamp"] = 1.0 if view == "rear" else 0.0
			if view == "rear":
				cam.position = Vector3(1.2, 1.6, -7.5); cam.look_at(Vector3(0, 0.6, 0))
			else:
				cam.position = Vector3(-5.0, 2.0, 10.0); cam.look_at(Vector3(0, 0.4, 3.0))
			for i in 6: await process_frame
			await RenderingServer.frame_post_draw
			var file := "%s/%s_%s_%s_%s.png" % [out, tag, car, s["name"], view]
			get_root().get_viewport().get_texture().get_image().save_png(file)
			print("shot %s: %s %s lux=%s exposure=%s lights=%d" % [file, s["name"], s["clock"], str(snappedf(float(s["illuminance_lux"]), 0.001)), str(float(s["exposure"])), int(lights)])
	quit()

func _rgb(a: Array) -> Color:
	return Color(float(a[0]), float(a[1]), float(a[2]))
