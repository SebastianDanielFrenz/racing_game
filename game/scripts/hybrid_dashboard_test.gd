extends SceneTree
# Hybrid dashboards (owner 2026-10-18): the real car_hyper_n2o + hybrid conversion driven in D through the garage path.
#  * the tachometer layout comes from core: ICE dial scaled to the ICE limiter (not the 20k motor), an inner motor scale whose
#    maximum is the traction motor's own max_rpm, the traction motor's rpm is NOT the max over all machines;
#  * the battery readout (percent, label, level) is published and drawn by the HUD gauge and the digital cluster;
#  * no dashboard reads an empty powertrain while the car drives in D (the feed counts misses).
const PowertrainFeed := preload("res://scripts/powertrain_feed.gd")
var failures := 0
func check(ok: bool, label: String) -> void:
	print("RG_HYBRID_DASH ", label, " ", ok)
	if not ok:
		failures += 1
func _initialize() -> void: run.call_deferred()
func wait_for(predicate: Callable, seconds: float = 8.0) -> bool:
	var deadline := Time.get_ticks_msec() + int(seconds * 1000)
	while not predicate.call():
		if Time.get_ticks_msec() > deadline:
			return false
		await process_frame
	return true
func run() -> void:
	var repo := ProjectSettings.globalize_path("res://..").simplify_path()
	var user_dir := repo.path_join("out/hybrid_dash_" + str(Time.get_ticks_usec()))
	DirAccess.make_dir_recursive_absolute(user_dir)
	var garage = ClassDB.instantiate("RgGarage")
	root.add_child(garage)
	if not garage.initialize(repo, user_dir, user_dir.path_join("work"), "").get("ok", false):
		check(false, "garage initialises"); quit(1); return
	check(garage.begin_edit("car_hyper_n2o").get("ok", false), "edit car_hyper_n2o")
	check(garage.set_option("powertrain_install", "hybrid_battery").get("accepted", false), "hybrid conversion accepted")
	check(garage.save().get("ok", false), "setup saved")
	var selected: Dictionary = garage.prepare_drive("car_hyper_n2o")
	if not selected.get("ok", false):
		check(false, "hybrid materialises " + str(selected)); quit(1); return
	print("RG_HYBRID_DASH vehicle_path=", selected.vehicle_path, " modified=", selected.modified, " swap=", selected.swap_lines)
	var sim = ClassDB.instantiate("RgSimulation")
	root.add_child(sim)
	sim.configure_dashboard(JSON.parse_string(FileAccess.get_file_as_string(repo.path_join("data/controls/dashboard.json"))))
	sim.set_vehicle_overrides(selected.chassis)
	if not sim.initialize(selected.vehicle_path, repo.path_join("data/surfaces/surfaces.json")):
		check(false, "initialise"); quit(1); return
	if not await wait_for(func(): return sim.get_init_status().get("state", "") in ["ready", "error"], 120.0) or sim.get_init_status().get("state", "") != "ready":
		check(false, "ready " + str(sim.get_init_status())); quit(1); return
	var vehicle: String = selected.sim_name
	sim.start()
	sim.set_control("ignition", 1)
	sim.set_control("electric.ready", 1)
	sim.set_control("assist.auto_shift", 1)
	sim.set_control("assist.auto_clutch", 1)
	sim.set_control("assist.auto_blip", 1)
	await wait_for(func(): return not sim.get_vehicle_powertrain(vehicle).is_empty(), 20.0)

	var info: Dictionary = sim.get_vehicle_gauge_info(vehicle)
	check(str(info.get("tach_kind", "")) == "hybrid", "tach_kind hybrid: " + str(info.get("tach_kind")))
	check(float(info.get("limiter_rpm", 0)) > 6000.0 and float(info.get("limiter_rpm", 0)) < 12000.0, "outer dial follows the ICE limiter, not the 20k motor: " + str(info.get("limiter_rpm")))
	check(float(info.get("motor_limit_rpm", 0)) == 20000.0, "motor limit from the motor description")
	check(float(info.get("motor_dial_max_rpm", 0)) == 20000.0 and float(info.get("motor_dial_tick_rpm", 0)) == 2000.0, "inner scale 0..20 x1000, ticks every 2")

	# Drive in D, WOT: feed every frame like a dashboard does.
	sim.set_control("assist.drive_mode", 2)
	await create_timer(2.0).timeout
	sim.set_control("gear_request", 99.0)
	sim.set_control("throttle", 1.0)
	var feed = PowertrainFeed.new("hybrid_dash_test")
	var max_ice := 0.0
	var max_motor := 0.0
	var seen_battery := false
	var frames := 0
	var t_end := Time.get_ticks_msec() + 9000
	while Time.get_ticks_msec() < t_end:
		await process_frame
		var pt: Dictionary = feed.read(sim, vehicle)
		frames += 1
		max_ice = maxf(max_ice, float(pt.get("rpm", 0.0)))
		max_motor = maxf(max_motor, float(pt.get("traction_motor_rpm", 0.0)))
		if bool(pt.get("battery_fitted", false)) and str(pt.get("battery_label", "")).begins_with("SoC ") and int(pt.get("battery_percent", 0)) > 0:
			seen_battery = true
	var pt_end: Dictionary = sim.get_vehicle_powertrain(vehicle)
	print("RG_HYBRID_DASH frames=", frames, " ice_max=", max_ice, " motor_max=", max_motor, " speed=", sim.get_vehicle_ground_speed_mps(vehicle) * 3.6, " gear=", pt_end.get("gear"), " label=", pt_end.get("battery_label"))
	check(seen_battery, "battery fitted/label/percent published")
	check(feed.misses == 0, "no empty powertrain reading while driving in D (misses %d)" % feed.misses)
	check(max_ice > 3000.0, "main rpm is the combustion engine's")
	check(max_ice <= float(info.get("limiter_rpm", 0)) + 300.0, "main rpm never reaches the motor's 20k scale")
	check(max_motor > 0.0 and max_motor <= 20000.0, "traction motor rpm published, within its limit")

	print("RG_HYBRID_DASH stage gauge")
	# The real HUD gauge and digital cluster parse and run a few frames on the live simulation.
	for script_path in ["res://scripts/tach_gauge.gd", "res://scripts/digital_cluster.gd"]:
		check(load(script_path).can_instantiate(), script_path + " parses")
	var gauge: Control = load("res://scripts/tach_gauge.gd").new()
	gauge.set("vehicle_name", vehicle)
	sim.name = "Simulation"
	gauge.set("simulation_path", NodePath("/root/Simulation"))
	root.add_child(gauge)
	var cluster: Control = load("res://scripts/digital_cluster.gd").new()
	cluster.simulation = sim
	cluster.vehicle_name = vehicle
	root.add_child(cluster)
	print("RG_HYBRID_DASH stage frames")
	for i in range(30):
		await process_frame
	check(str(gauge._gauge_info.get("tach_kind", "")) == "hybrid" and gauge._display_motor_rpm >= 0.0, "HUD gauge reads the hybrid layout")
	check(cluster.tach_kind == "hybrid" and cluster.motor_dial_max == 20000.0, "cluster reads the hybrid layout")
	check(cluster.rpm <= float(info.get("limiter_rpm", 0)) + 300.0, "cluster outer rpm is the ICE's")
	check(cluster.pt_now.get("battery_label", "") != "", "cluster has the battery readout")
	print("RG_HYBRID_DASH failures=", failures)
	sim.stop()
	quit(1 if failures else 0)
