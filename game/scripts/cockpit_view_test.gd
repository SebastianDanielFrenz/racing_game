extends SceneTree
class Sim extends Node:
	var steer := 0.25
	func get_control(channel: String) -> float: return steer if channel=="steer" else 0.0
	func get_vehicle_powertrain(_car: String) -> Dictionary: return {"engine_state":"Running"}
	func get_vehicle_gauge_info(_car: String) -> Dictionary: return {}
	func is_running() -> bool: return false
	func get_step_count() -> int: return 1
	func get_body_transform(_body: String) -> Transform3D: return Transform3D(Basis(Vector3.RIGHT,Vector3.FORWARD,Vector3.UP),Vector3.ZERO)
class Eye extends Node:
	func driver_eye_local() -> Vector3: return Vector3(-0.28,0.33,0.93)
class Look extends Node:
	var mouse := Vector2(240,313.333333)
	func get_camera_look_rate() -> Vector2: return Vector2.ZERO
	func consume_mouse_delta() -> Vector2:
		var value := mouse
		mouse=Vector2.ZERO
		return value
var failures := 0
func check(value: bool, label: String) -> void:
	if not value:
		failures+=1
		push_error(label)
func _initialize() -> void: call_deferred("run")
func run() -> void:
	for name in ["cockpit_rig","input_map","main","camera_director","vehicle_visual","drive_tour"]:
		check(load("res://scripts/"+name+".gd").can_instantiate(),name+" parses")
	var controls := RgControls.new()
	var directory := OS.get_cache_dir().path_join("rg_c2_%d" % OS.get_process_id())
	DirAccess.make_dir_recursive_absolute(directory)
	controls.initialize(ProjectSettings.globalize_path("res://../data"),directory)
	var adapter: Node = load("res://scripts/input_map.gd").new()
	adapter.controls=controls
	root.add_child(adapter)
	adapter.set_cockpit_mode(true)
	check(adapter.is_mouse_captured() and (DisplayServer.get_name()=="headless" or Input.mouse_mode==Input.MOUSE_MODE_CAPTURED),"cockpit enters captured and hidden")
	var click := InputEventMouseButton.new()
	click.button_index=MOUSE_BUTTON_MIDDLE
	click.pressed=true
	adapter._input(click)
	check(not adapter.is_mouse_captured() and Input.mouse_mode==Input.MOUSE_MODE_VISIBLE,"middle click releases")
	adapter._input(click)
	check(adapter.is_mouse_captured(),"second middle click captures")
	check(adapter.consume_log_marker()==0,"middle click does not log")
	adapter.set_world_input_active(false)
	adapter._input(click)
	check(not adapter.is_mouse_captured(),"menu prevents capture")
	adapter.set_world_input_active(true)
	check(adapter.is_mouse_captured(),"return to cockpit captures")
	adapter.set_test_snapshot({"keys":PackedStringArray(["Delete"])})
	adapter.poll();adapter.poll()
	check(adapter.consume_log_marker()==1,"Delete logs once per press")
	check(adapter.consume_log_marker()==0,"marker consumed once")
	adapter.set_cockpit_mode(false)
	check(not adapter.is_mouse_captured(),"leave cockpit releases")
	adapter.free();controls.free()
	var sim := Sim.new()
	root.add_child(sim)
	var visual: Node3D = load("res://scripts/vehicle_visual.gd").new()
	visual.simulation=sim
	visual.model_absolute_path=ProjectSettings.globalize_path("res://../external/physics_sim/data/models/car_hyper/car_hyper.glb")
	visual.transform=sim.get_body_transform("chassis")
	root.add_child(visual)
	await process_frame
	var cfg: Dictionary = JSON.parse_string(FileAccess.get_file_as_string(ProjectSettings.globalize_path("res://../data/controls/cabin.json")))
	visual._process(0.0)
	var angle: float = visual.get_steering_wheel_angle_rad()
	check(absf(angle-deg_to_rad(float(cfg["steering_wheel_lock_deg"])*sim.steer))<0.00001,"actual wheel angle is steer times configured lock")
	var wheel: Node3D = visual.find_child("steering_wheel",true,false)
	check(wheel.basis.is_equal_approx(Basis(visual._steering_axis,angle)*visual._steering_rest),"actual wheel basis follows rig axis")
	var rig: Node3D = load("res://scripts/cockpit_rig.gd").new()
	var eye := Eye.new()
	var look := Look.new()
	root.add_child(eye);root.add_child(look)
	rig.body_visuals=eye
	root.add_child(rig)
	rig.activate(Transform3D.IDENTITY)
	rig.update_rig(0.016,sim,look,true)
	check(absf(rig.look_pitch_deg+50)<0.001,"actual camera looks down 50 degrees")
	if "--capture" in OS.get_cmdline_user_args():
		root.size=Vector2i(1400,1000)
		DisplayServer.window_set_size(root.size)
		var environment := WorldEnvironment.new()
		environment.environment=Environment.new()
		environment.environment.ambient_light_source=Environment.AMBIENT_SOURCE_COLOR
		environment.environment.ambient_light_color=Color(0.7,0.75,0.85)
		environment.environment.ambient_light_energy=0.8
		root.add_child(environment)
		var light := OmniLight3D.new()
		light.position=Vector3(0.3,1.2,0)
		light.omni_range=3
		root.add_child(light)
		rig.camera.current=true
		for i in range(8): await process_frame
		await RenderingServer.frame_post_draw
		root.get_texture().get_image().save_png(ProjectSettings.globalize_path("res://../out/c2_cockpit_down.png"))
	visual.free();rig.free();eye.free();look.free();sim.free()
	print("RG_COCKPIT_VIEW_TEST failures=%d" % failures)
	quit(1 if failures else 0)
