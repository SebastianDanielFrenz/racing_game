extends SceneTree
# Focused Windows/GDExtension acceptance: materialise each gearbox and exercise
# the real physics selector/manual-mode seam. No real-map data or user saves.
var failures := 0
var checks := 0
var sim: Node
var actions: RefCounted
var vehicle := ""
func _initialize() -> void:
	run.call_deferred()
	watchdog.call_deferred()
func watchdog() -> void:
	await create_timer(180.0).timeout
	push_error("RG_AUTOMATIC_TEST timed out")
	quit(1)
func check(ok: bool, message: String) -> bool:
	checks += 1
	if not ok:
		failures += 1
		push_error("RG_AUTOMATIC_TEST " + message)
	return ok
func wait_for(predicate: Callable, seconds: float = 4.0) -> bool:
	var deadline := Time.get_ticks_msec()+int(seconds*1000)
	while not predicate.call():
		if Time.get_ticks_msec()>deadline: return false
		await process_frame
	return true
func snapshot() -> Dictionary:
	var pt: Dictionary = sim.get_vehicle_powertrain(vehicle)
	pt["speed_mps"] = sim.get_body_speed_mps("chassis")
	return pt
func channels() -> Dictionary:
	var result := {}
	for key in ["gear_request","park_lock","shift_up_count","shift_down_count","transmission.mode","assist.drive_mode"]:
		result[key] = sim.get_control(key)
	return result
func action(id: String) -> void:
	actions.submit(id,0,0.0,7) # Press, Script.
	for write in actions.apply(snapshot(),channels(),true):
		sim.set_control(write.channel,write.value)
func shift(up: int, down: int) -> void:
	var result: Dictionary = RgDriverActions.selector_shift(snapshot(),up,down)
	if result.change: sim.set_control("gear_request",result.gear_request)
	if int(result.park_request)>=0: sim.set_control("park_lock",result.park_request)
	if result.forward_counts:
		if up: sim.set_control("shift_up_count",sim.get_control("shift_up_count")+up)
		if down: sim.set_control("shift_down_count",sim.get_control("shift_down_count")+down)
func run() -> void:
	var repo := ProjectSettings.globalize_path("res://..").simplify_path()
	var user_dir := repo.path_join("out/automatic_acceptance_"+str(Time.get_ticks_usec()))
	DirAccess.make_dir_recursive_absolute(user_dir)
	var garage = ClassDB.instantiate("RgGarage")
	root.add_child(garage)
	if not check(garage.initialize(repo,user_dir,user_dir.path_join("work"),"").ok,"garage initialises"):
		garage.queue_free(); await process_frame; quit(1); return
	actions = ClassDB.instantiate("RgDriverActions")
	for box in ["auto_8at","dct_7","cvt_converter","cvt_clutch","hyper_7dct"]:
		var car_id := "car_hyper_n2o" if box == "hyper_7dct" else "car_sedan_rwd"
		check(garage.begin_edit(car_id).ok,box+": edit opens")
		var edited: Dictionary = garage.set_option("gearbox_install",box)
		if not check(edited.accepted and edited.ok,box+": install validates "+str(edited)):
			continue
		check(garage.save().ok,box+": setup saves")
		var selection: Dictionary = garage.prepare_drive(car_id)
		if not check(selection.ok,box+": drive materialises"):
			continue
		vehicle = selection.sim_name
		sim = ClassDB.instantiate("RgSimulation")
		root.add_child(sim)
		var overrides: Dictionary = selection.chassis.duplicate()
		overrides["engine_map_cache_dir"] = selection.engine_map_cache_dir
		sim.set_vehicle_overrides(overrides)
		if not check(sim.initialize(selection.vehicle_path,repo.path_join("data/surfaces/surfaces.json")),box+": simulation initialises"):
			sim.queue_free(); await process_frame; continue
		if not check(await wait_for(func(): return sim.get_init_status().get("state","") in ["ready","error"],120.0) and sim.get_init_status().get("state","")=="ready",box+": asynchronous initialization completes"):
			sim.queue_free(); await process_frame; continue
		sim.set_control("brake",1.0)
		sim.set_control("ignition",1.0)
		sim.set_control("starter",1.0)
		sim.start()
		check(await wait_for(func(): return snapshot().get("engine_state","")=="Running"),box+": engine starts")
		sim.set_control("starter",0.0)
		var kind := 3 if box.begins_with("cvt") else (2 if box in ["dct_7","hyper_7dct"] else 1)
		check(int(snapshot().get("transmission_kind",-1))==kind,box+": transmission kind published")
		action("park")
		check(await wait_for(func(): return bool(snapshot().get("park_lock_engaged",false))),box+": P engages")
		shift(1,0)
		check(await wait_for(func(): return int(snapshot().get("transmission_selector",0))<0),box+": P -> R")
		shift(1,0)
		check(await wait_for(func(): return int(snapshot().get("transmission_selector",-1))==0),box+": R -> N")
		shift(1,0)
		check(await wait_for(func(): return int(snapshot().get("transmission_selector",0))>0),box+": N -> D")
		if box == "hyper_7dct":
			check(bool(snapshot().get("drive_mode_fitted",false)), "hypercar DCT keeps driving modes fitted")
			for mode in [2,3,0,1]: # default Normal -> Sport -> Racing -> Eco -> Normal
				action("drive_mode")
				check(await wait_for(func(): return int(snapshot().get("drive_mode_effective",-1)) == mode), "M cycles hypercar DCT to mode " + str(mode))
				print("RG_AUTOMATIC_TEST hypercar drive mode=%s" % snapshot().get("drive_mode_name"))
		action("manual_mode")
		check(await wait_for(func(): return bool(snapshot().get("transmission_manual_mode",false))),box+": Y enters manual mode in physics")
		var same_tick := int(sim.get_step_count())
		check(await wait_for(func(): return int(sim.get_step_count())>same_tick+30),box+": manual mode persists across ticks")
		check(bool(snapshot().get("transmission_manual_mode",false)),box+": snapshot retains manual mode")
		action("manual_mode")
		check(await wait_for(func(): return not bool(snapshot().get("transmission_manual_mode",true))),box+": Y returns to automatic D")
		# Core uses measured speed, not an inferred gear or an absent snapshot field.
		var rolling := snapshot()
		rolling["speed_mps"] = 5.0
		var refused: Dictionary = RgDriverActions.selector_shift(rolling,0,2)
		check(bool(refused.refused),box+": reverse is refused while rolling")
		print("RG_AUTOMATIC_TEST box=%s selector=%s manual=%s" % [box,snapshot().get("transmission_selector"),snapshot().get("transmission_manual_mode")])
		sim.stop()
		sim.queue_free()
		await process_frame
	garage.cleanup()
	garage.queue_free()
	actions = null
	await process_frame
	print("RG_AUTOMATIC_TEST checks=%d failures=%d" % [checks,failures])
	quit(1 if failures else 0)
