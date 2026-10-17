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
	while str(sim.get_init_status().get("state", "")) == "loading":
		await process_frame
	if str(sim.get_init_status().get("state", "")) != "ready":
		push_error("TC simulation init failed: %s" % str(sim.get_init_status()))
		quit(1)
		return
	var name: String = sim.get_vehicle_names()[0]
	var info: Dictionary = sim.get_vehicle_gauge_info(name)
	var ok: bool = "TC" in info.get("fitted_assists", [])
	# The game sets no TC default: the channel starts at the car's own default (the Racing unit: On; a car with no unit
	# and no enabled traction_control block: Off). Switch it on first, as F4 would.
	sim.set_control("assist.traction_control", 1.0)
	sim.start()
	await create_timer(0.3).timeout
	var pt: Dictionary = sim.get_vehicle_powertrain(name)
	ok = ok and pt.get("assist_traction_control", false) and int(pt.get("traction_mode_effective", 0)) == 1
	# RgAssists cycles through the offered modes only
	var bits: int = int(pt.get("traction_modes_available_bits", 0))
	var next: int = RgAssists.next_traction_mode(bits, 1)
	ok = ok and (bits >> next) & 1 == 1
	sim.set_control("assist.traction_control", 0.0)
	await create_timer(0.2).timeout
	pt = sim.get_vehicle_powertrain(name)
	ok = ok and not pt.get("assist_traction_control", true) and pt.get("traction_torque_fraction", 0.0) == 1.0
	sim.stop()
	sim.queue_free()
	print("RG_TRACTION_CONTROL_TEST ", "PASS" if ok else "FAIL", " vehicle=", vehicle)
	quit(0 if ok else 1)
