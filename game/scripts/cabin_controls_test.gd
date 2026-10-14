extends SceneTree
class SilentSimulation extends Node:
	func is_running() -> bool: return false
	func get_step_count() -> int: return 0
	func stop_engine_audio() -> void: pass
var failures := 0
func check(value: bool, message: String) -> void:
	if not value:
		failures += 1
		push_error(message)
func _initialize() -> void: call_deferred("run")
func run() -> void:
	for path in ["input_map","main","vehicle_visual","vehicle_audio","exterior_lamps"]:
		check(load("res://scripts/"+path+".gd").can_instantiate(),path+" parses")
	var cfg: Dictionary = JSON.parse_string(FileAccess.get_file_as_string(ProjectSettings.globalize_path("res://../data/controls/cabin.json")))
	var actions := RgDriverActions.new()
	actions.configure_cabin(cfg)
	actions.submit("start_stop",RgDriverActions.PRESS,1,RgDriverActions.COCKPIT_MOUSE)
	var values := {"ignition":0.0}
	for write in actions.apply({"engine_state":"Stopped"},values,true): values[write["channel"]]=write["value"]
	for write in actions.advance({"engine_state":"Stopped"},values,true,0.1): values[write["channel"]]=write["value"]
	check(values["starter"]==1,"tap holds starter")
	for write in actions.advance({"engine_state":"Running"},values,true,0.1): values[write["channel"]]=write["value"]
	check(values["starter"]==0,"running stops auto crank")
	actions.submit("hazard",RgDriverActions.PRESS,1,RgDriverActions.PAD)
	actions.apply({},values,true)
	for write in actions.advance({},values,true,0): values[write["channel"]]=write["value"]
	check(values["cabin.indicator_left"]==1 and values["cabin.indicator_right"]==1,"hazards flash both sides")
	var silent := SilentSimulation.new()
	root.add_child(silent)
	var audio: Node = load("res://scripts/vehicle_audio.gd").new()
	audio.simulation=silent
	root.add_child(audio)
	var horn: AudioStreamPlayer3D = audio.get("_horn")
	check(horn != null,"generated horn player created")
	check(horn.stream is AudioStreamWAV and horn.stream.data.size()>0,"horn has generated PCM")
	check(horn.stream.loop_mode==AudioStreamWAV.LOOP_FORWARD,"horn loops while held")
	audio.queue_free(); silent.queue_free()
	var doc := GLTFDocument.new()
	var state := GLTFState.new()
	var path := ProjectSettings.globalize_path("res://../external/physics_sim/data/models/car_hyper/car_hyper.glb")
	check(doc.append_from_file(path,state)==OK,"real car loads")
	var car := doc.generate_scene(state)
	root.add_child(car)
	var lamps: Node = load("res://scripts/exterior_lamps.gd").attach_to(car,null,"car_hyper",path)
	check(lamps.lamps.size()>0,"rig lamp materials bound")
	lamps.update_lamps({},values)
	var lit := 0
	for lamp in lamps.lamps:
		if lamp["action"] in ["indicator_left","indicator_right"]:
			check(lamp["material"].emission_enabled,"actual indicator material glows")
			lit += 1
	check(lit>=2,"both real indicator materials present")
	lamps.update_lamps({"gear":-1},{"cabin.brake_lamp":1.0})
	for lamp in lamps.lamps:
		if lamp["action"] in ["brake_lamp","reverse_lamp"]: check(lamp["material"].emission_enabled,"brake/reverse glow")
	if "--capture" in OS.get_cmdline_user_args():
		lamps.update_lamps({},values)
		var camera := Camera3D.new()
		root.add_child(camera)
		camera.position=Vector3(3.5,1.9,-5)
		camera.fov=40
		var environment := WorldEnvironment.new()
		environment.environment=Environment.new()
		environment.environment.ambient_light_source=Environment.AMBIENT_SOURCE_COLOR
		environment.environment.ambient_light_color=Color(0.6,0.6,0.7)
		environment.environment.ambient_light_energy=0.6
		root.add_child(environment)
		camera.look_at(Vector3(0,0.65,0),Vector3.UP)
		camera.current=true
		var sun := DirectionalLight3D.new()
		root.add_child(sun)
		sun.rotation_degrees=Vector3(-40,-20,0)
		for i in range(8): await process_frame
		await RenderingServer.frame_post_draw
		root.get_texture().get_image().save_png(ProjectSettings.globalize_path("res://../out/c1_hazards.png"))
	car.queue_free()
	await process_frame
	print("RG_CABIN_CONTROLS_TEST failures=%d" % failures)
	quit(1 if failures else 0)
