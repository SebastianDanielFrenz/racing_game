extends SceneTree

func _initialize() -> void:
	_run.call_deferred()

func _run() -> void:
	for path in ["main", "input_map", "tach_gauge", "digital_cluster"]:
		var script = load("res://scripts/%s.gd" % path)
		if script == null or not script.can_instantiate():
			push_error("TC script parse failed: " + path)
			quit(1)
			return
	var sim = ClassDB.instantiate("RgSimulation")
	root.add_child(sim)
	var vehicle := OS.get_environment("RG_TC_SMOKE_VEHICLE")
	if vehicle.is_empty():
		vehicle = "car_sedan"
	var base := ProjectSettings.globalize_path("res://../external/physics_sim/data/")
	if not sim.initialize(base + "vehicles/" + vehicle + ".json", base + "surfaces/surfaces.json"):
		push_error("TC simulation initialization failed")
		quit(1)
		return
	var name: String = sim.get_vehicle_names()[0]
	var info: Dictionary = sim.get_vehicle_gauge_info(name)
	var ok: bool = "TC" in info.get("fitted_assists", [])
	sim.start()
	await create_timer(0.3).timeout
	var pt: Dictionary = sim.get_vehicle_powertrain(name)
	ok = ok and pt.get("assist_traction_control", false)
	sim.set_control("assist.traction_control", 0.0)
	await create_timer(0.2).timeout
	pt = sim.get_vehicle_powertrain(name)
	ok = ok and not pt.get("assist_traction_control", true) and pt.get("traction_torque_fraction", 0.0) == 1.0
	sim.stop()
	sim.queue_free()
	print("RG_TRACTION_CONTROL_TEST ", "PASS" if ok else "FAIL", " vehicle=", vehicle)
	quit(0 if ok else 1)
