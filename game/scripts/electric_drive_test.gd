extends SceneTree
class AudioTestDirector extends Node:
	var active_name := "chase"
	var camera: Camera3D
	func active_camera() -> Camera3D:
		return camera

var failures := 0
var checks := 0
func check(ok: bool,label: String) -> bool:
	print("RG_EV_TEST check: "+label)
	checks+=1
	if not ok:
		failures+=1
		push_error("RG_EV_TEST "+label)
	return ok
func _initialize() -> void: run.call_deferred()
func wait_for(predicate: Callable,seconds: float=8.0) -> bool:
	var deadline := Time.get_ticks_msec()+int(seconds*1000)
	while not predicate.call():
		if Time.get_ticks_msec()>deadline:return false
		await process_frame
	return true
func run() -> void:
	var repo := ProjectSettings.globalize_path("res://..").simplify_path()
	var user_dir := repo.path_join("out/ev_acceptance_"+str(Time.get_ticks_usec()))
	DirAccess.make_dir_recursive_absolute(user_dir)
	var garage = ClassDB.instantiate("RgGarage")
	root.add_child(garage)
	var init: Dictionary = garage.initialize(repo,user_dir,user_dir.path_join("work"),"")
	if not check(init.get("ok",false),"garage initialises "+str(init)):quit(1);return
	check(garage.begin_edit("car_electric").ok,"EV is selectable in garage")
	var selected: Dictionary = garage.prepare_drive("car_electric")
	if not check(selected.get("ok",false),"EV materialises with real part masses "+str(selected)):quit(1);return
	print("RG_EV_TEST chassis="+str(selected.chassis))
	var sim = ClassDB.instantiate("RgSimulation")
	root.add_child(sim)
	sim.set_vehicle_overrides(selected.chassis)
	if not check(sim.initialize(selected.vehicle_path,repo.path_join("data/surfaces/surfaces.json")),"EV initialises"):quit(1);return
	if not check(await wait_for(func():return sim.get_init_status().get("state","") in ["ready","error"]),"EV initialisation finishes"):quit(1);return
	if not check(sim.get_init_status().get("state","")=="ready","EV ready "+str(sim.get_init_status())):quit(1);return
	print("RG_EV_TEST initialisation ready")
	var vehicle: String = selected.sim_name
	print("RG_EV_TEST starting")
	sim.start()
	print("RG_EV_TEST started")
	check(await wait_for(func():return bool(sim.get_vehicle_powertrain(vehicle).get("electric_ready",false))),"electrical READY without starter")
	var pt: Dictionary = sim.get_vehicle_powertrain(vehicle)
	check(int(pt.get("electric_motor_count",0))==2,"both axle motors published")
	check(int(pt.get("transmission_kind",0))==4,"fixed electric transmission")
	check(str(pt.get("engine_state",""))=="Off","no fake running combustion engine")
	check(int(pt.get("traction_modes_available_bits",0))==31,"electric traction modes fitted")
	check(float(sim.get_vehicle_gauge_info(vehicle).get("limiter_rpm",0))==20000,"motor RPM gauge range")
	var shift: Dictionary = RgDriverActions.selector_shift(pt,1,0)
	check(bool(shift.change),"selector N to D")
	sim.set_control("gear_request",shift.gear_request)
	sim.set_control("throttle",1.0)
	check(await wait_for(func():return sim.get_body_speed_mps("chassis")>8,6),"accelerates from zero motor RPM")
	pt=sim.get_vehicle_powertrain(vehicle)
	check(float(pt.get("drive_power_kw",0))>0,"actual electric power reaches dashboard")
	check(float(pt.get("battery_soc",1))<0.9,"battery discharges")
	check(float(pt.get("motor_rpm",0))>0,"actual motor RPM")
	sim.set_control("throttle",0)
	sim.set_control("brake",1)
	check(await wait_for(func():return sim.get_body_speed_mps("chassis")<0.1,8),"EV brakes to standstill")
	var actions = ClassDB.instantiate("RgDriverActions")
	actions.configure_cabin(JSON.parse_string(FileAccess.get_file_as_string(repo.path_join("data/controls/cabin.json"))))
	pt=sim.get_vehicle_powertrain(vehicle)
	actions.submit("start_stop",0,0,7)
	for write in actions.apply(pt,{"electric.ready":1.0},true):sim.set_control(write.channel,write.value)
	check(await wait_for(func():return not sim.get_vehicle_powertrain(vehicle).get("electric_ready",true)),"start-stop disables electric READY")
	pt=sim.get_vehicle_powertrain(vehicle)
	actions.submit("start_stop",0,0,7)
	for write in actions.apply(pt,{"electric.ready":0.0},true):sim.set_control(write.channel,write.value)
	check(await wait_for(func():return sim.get_vehicle_powertrain(vehicle).get("electric_ready",false)),"start-stop enables READY without cranking")
	check(float(sim.get_control("starter"))==0,"no EV starter command")
	check(load("res://scripts/tach_gauge.gd").can_instantiate(),"EV HUD parses")
	var input: Node = load("res://scripts/input_map.gd").new()
	input.configure_vehicle(selected.vehicle_path)
	check(bool(input.get("_electric_drive")),"starter key maps to electric READY")
	input.free()
	var audio_script = load("res://scripts/vehicle_audio.gd")
	check(audio_script != null and audio_script.can_instantiate(), "vehicle audio parses")
	if "--ev-audio-test" in OS.get_cmdline_user_args():
		var test_director := AudioTestDirector.new()
		test_director.camera = Camera3D.new()
		root.add_child(test_director)
		test_director.add_child(test_director.camera)
		var audio: Node3D = audio_script.new()
		audio.director = test_director
		audio.simulation = sim
		audio.vehicle_name = vehicle
		root.add_child(audio)
		audio.set_process(false)
		audio._initialize_audio()
		check(audio.get_audio_diagnostics().get("electric_motor",false), "EV motor voice is connected")
		audio._update_motor_audio(false)
		audio.muted = true
		audio._update_motor_audio(false)
		audio.shutdown()
		audio.queue_free()
		test_director.queue_free()
	print("RG_EV_TEST checks=%d failures=%d" % [checks,failures])
	sim.stop();sim.queue_free();garage.queue_free();actions = null
	await process_frame
	quit(1 if failures else 0)
