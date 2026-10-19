extends SceneTree
# Windowed screenshots of the hybrid dashboards (HUD tach gauge + digital cluster) on the real hybrid car, for the report:
#   out/dashboard_hybrid_hud_cluster.png   driving in D: two-needle tach, SoC bar
#   out/dashboard_hybrid_held.png          the car "not found" for a few frames: the cluster keeps its values, not N / 0 / 0
# godot --path game --script res://scripts/hybrid_dashboard_shot.gd   (not headless; closes itself)
func _initialize() -> void: run.call_deferred()
func wait_for(predicate: Callable, seconds: float = 8.0) -> bool:
	var deadline := Time.get_ticks_msec() + int(seconds * 1000)
	while not predicate.call():
		if Time.get_ticks_msec() > deadline:
			return false
		await process_frame
	return true
func shot(path: String) -> void:
	await RenderingServer.frame_post_draw
	root.get_viewport().get_texture().get_image().save_png(path)
	print("RG_SHOT ", path)
func run() -> void:
	var repo := ProjectSettings.globalize_path("res://..").simplify_path()
	var user_dir := repo.path_join("out/hybrid_shot_" + str(Time.get_ticks_usec()))
	DirAccess.make_dir_recursive_absolute(user_dir)
	var garage = ClassDB.instantiate("RgGarage")
	root.add_child(garage)
	garage.initialize(repo, user_dir, user_dir.path_join("work"), "")
	garage.begin_edit("car_hyper_n2o")
	garage.set_option("powertrain_install", "hybrid_battery")
	garage.save()
	var selected: Dictionary = garage.prepare_drive("car_hyper_n2o")
	var sim = ClassDB.instantiate("RgSimulation")
	sim.name = "Simulation"
	root.add_child(sim)
	sim.configure_dashboard(JSON.parse_string(FileAccess.get_file_as_string(repo.path_join("data/controls/dashboard.json"))))
	sim.set_vehicle_overrides(selected.chassis)
	sim.initialize(selected.vehicle_path, repo.path_join("data/surfaces/surfaces.json"))
	await wait_for(func(): return sim.get_init_status().get("state", "") in ["ready", "error"], 120.0)
	var vehicle: String = selected.sim_name
	sim.start()
	for channel in ["ignition", "electric.ready", "assist.auto_shift", "assist.auto_clutch", "assist.auto_blip"]:
		sim.set_control(channel, 1)
	sim.set_control("assist.drive_mode", 1)
	await wait_for(func(): return not sim.get_vehicle_powertrain(vehicle).is_empty(), 20.0)
	var bg := ColorRect.new()
	bg.color = Color(0.1, 0.12, 0.15)
	bg.set_anchors_preset(Control.PRESET_FULL_RECT)
	root.add_child(bg)
	var cluster: Control = load("res://scripts/digital_cluster.gd").new()
	cluster.simulation = sim
	cluster.vehicle_name = vehicle
	cluster.position = Vector2(20, 20)
	root.add_child(cluster)
	var gauge: Control = load("res://scripts/tach_gauge.gd").new()
	gauge.set("vehicle_name", vehicle)
	gauge.set("simulation_path", NodePath("/root/Simulation"))
	root.add_child(gauge)
	await create_timer(2.0).timeout
	sim.set_control("gear_request", 99.0)
	sim.set_control("throttle", 0.5)
	await create_timer(5.0).timeout
	await shot(repo.path_join("out/dashboard_hybrid_hud_cluster.png"))
	# The car cannot be found for a few frames (what made the dashboard read N / 0 km/h / 0 rpm): values are held.
	cluster.vehicle_name = "no such car"
	gauge.set("vehicle_name", "no such car")
	for i in range(20):
		await process_frame
	print("RG_SHOT held cluster speed=", cluster.speed, " gear=", cluster.gear, " rpm=", cluster.rpm, " misses=", cluster.feed.misses)
	await shot(repo.path_join("out/dashboard_hybrid_held.png"))
	sim.stop()
	quit(0)
