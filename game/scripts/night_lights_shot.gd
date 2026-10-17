extends SceneTree
# Night lamp review shots (headless-less, needs a renderer; xvfb works):
#   godot --path game --rendering-driver opengl3 --script res://scripts/night_lights_shot.gd -- --car ID --out DIR [--tag before]
# Flat ground, dark sky, a car at the origin with its real lamp scripts; one shot each for dipped, main beam,
# brake (stronger red), reverse, seen from behind and from the front.
class FakeSim extends Node:
	var channels := {"cabin.lights": 0.0, "cabin.brake_lamp": 0.0, "gear": 1.0}
	func get_control(c: String) -> float: return float(channels.get(c, 0.0))
	func get_vehicle_powertrain(_n: String) -> Dictionary: return {"gear": int(channels["gear"]), "brake": channels["cabin.brake_lamp"]}

func _initialize() -> void:
	var args := OS.get_cmdline_user_args()
	var car := "kiri_86"; var out := "/tmp"; var tag := "after"
	for i in range(args.size()):
		if args[i] == "--car": car = args[i + 1]
		if args[i] == "--out": out = args[i + 1]
		if args[i] == "--tag": tag = args[i + 1]
	var world := Node3D.new(); root.add_child(world)
	var env := Environment.new()
	env.background_mode = Environment.BG_COLOR; env.background_color = Color(0.01, 0.015, 0.03)
	env.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR; env.ambient_light_color = Color(0.05, 0.06, 0.1); env.ambient_light_energy = 1.6
	var we := WorldEnvironment.new(); we.environment = env; world.add_child(we)
	var ground := MeshInstance3D.new(); var pm := PlaneMesh.new(); pm.size = Vector2(600, 600); ground.mesh = pm
	var gm := StandardMaterial3D.new(); gm.albedo_color = Color(0.32, 0.32, 0.33); gm.roughness = 0.9; ground.material_override = gm
	world.add_child(ground)
	var path := "res://../data/models/imported/%s/%s.glb" % [car, car]
	var doc := GLTFDocument.new(); var st := GLTFState.new()
	doc.append_from_file(ProjectSettings.globalize_path(path), st)
	var model: Node3D = doc.generate_scene(st)
	var carroot := Node3D.new(); world.add_child(carroot); carroot.add_child(model)
	var sim := FakeSim.new(); root.add_child(sim)
	var rig = JSON.parse_string(FileAccess.get_file_as_string(ProjectSettings.globalize_path(path).get_basename() + ".rig.json"))
	# Same entry point the game uses (exterior_lamps -> headlights), minus the RgDriverActions state lookup.
	var hl = load("res://scripts/headlights.gd").attach_to(model, sim, car, rig)
	var cam := Camera3D.new(); cam.fov = 60; cam.far = 400; world.add_child(cam); cam.current = true
	await process_frame
	var shots := [
		["dipped_front", 1.0, 0.0, 1.0, Vector3(-6, 6.0, -9), Vector3(1, 0.3, 7)],
		["main_front", 2.0, 0.0, 1.0, Vector3(-6, 6.0, -9), Vector3(1, 0.3, 9)],
		["tail_rear", 1.0, 0.0, 1.0, Vector3(5, 3.5, -10), Vector3(0, 0.3, -3)],
		["brake_rear", 1.0, 1.0, 1.0, Vector3(5, 3.5, -10), Vector3(0, 0.3, -3)],
		["reverse_rear", 1.0, 0.0, -1.0, Vector3(5, 3.5, -10), Vector3(0, 0.3, -3)],
	]
	for s in shots:
		sim.channels["cabin.lights"] = s[1]; sim.channels["cabin.brake_lamp"] = s[2]; sim.channels["gear"] = s[3]
		if hl.has_method("set_states"):
			hl.set_states({"tail_lamp": s[1] > 0.0, "brake_lamp": s[2] > 0.0, "reverse_lamp": s[3] < 0.0})
		cam.position = s[4]; cam.look_at(s[5])
		for i in 4: await process_frame
		await RenderingServer.frame_post_draw
		get_root().get_viewport().get_texture().get_image().save_png("%s/%s_%s_%s.png" % [out, tag, car, s[0]])
	quit()
