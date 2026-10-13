extends SceneTree
var failures := 0
func check(value: bool, message: String) -> void:
	if not value:
		failures += 1
		push_error(message)
func _initialize() -> void:
	call_deferred("run")
func run() -> void:
	var actions := RgDriverActions.new()
	var snapshot := {"assist_fitted_bits":88,"traction_modes_available_bits":31,"traction_mode_effective":3,"drive_modes_available_bits":7,"drive_mode_effective":2,"launch_lamp":3,"aero_control_active":true}
	var channels := {"assist.traction_control":3.0,"assist.launch_control":1.0,"assist.drive_mode":1.0}
	actions.submit("traction_control",RgDriverActions.HOLD,1,RgDriverActions.KEYBOARD)
	actions.submit("traction_control",RgDriverActions.HOLD,1,RgDriverActions.KEYBOARD)
	actions.submit("toggle_launch_control",RgDriverActions.PRESS,1,RgDriverActions.VR_POKE)
	var writes: Array = actions.apply(snapshot,channels,true)
	check(writes.size()==3,"auto shift plus one TC edge and one LC press")
	check(writes[1]["channel"]=="assist.traction_control" and writes[1]["value"]==4,"TC current channel wins")
	check(writes[2]["value"]==0,"LC physics-written channel toggles off")
	check(channels["assist.traction_control"]==3,"binding does not mutate caller channel dictionary")
	check(actions.apply(snapshot,channels,false).is_empty(),"blocked frame has no writes")
	var tc: Dictionary = RgDriverActions.state("traction_control",snapshot,channels)
	check(tc["fitted"] and tc["value"]==3 and tc["label"]=="Sport","TC state binding")
	var dm: Dictionary = RgDriverActions.state("drive_mode",snapshot,channels)
	check(dm["label"]=="SPORT" and dm["lamp_segments"]==4,"drive mode state binding")
	var lc: Dictionary = RgDriverActions.state("toggle_launch_control",snapshot,channels)
	check(lc["blink"] and lc["label"]=="LC ARMED","LC state binding")
	check(RgDriverActions.state("toggle_aero",snapshot,channels)["value"]==1,"aero state binding")
	for path in ["main","input_map","hud","digital_cluster"]:
		var script := load("res://scripts/"+path+".gd") as GDScript
		check(script != null and script.can_instantiate(),path+" parses")
	# Real keyboard registry -> actual input adapter -> native action layer.
	var controls := RgControls.new()
	var user_dir := OS.get_cache_dir().path_join("rg_c0_%d" % OS.get_process_id())
	DirAccess.make_dir_recursive_absolute(user_dir)
	controls.initialize(ProjectSettings.globalize_path("res://../data"),user_dir)
	var adapter: Node = load("res://scripts/input_map.gd").new()
	adapter.controls = controls
	root.add_child(adapter)
	adapter.set_test_snapshot({"keys":PackedStringArray(["F4","F5","H","M","L","N"])})
	adapter.poll()
	adapter.poll() # a held key must never cause a second edge
	var actual: Array = adapter.driver_actions.apply(snapshot,{},true)
	check(actual.size()==6,"actual adapter emits six actions once each")
	var result: Dictionary = {}
	for write in actual: result[write["channel"]] = write["value"]
	check(result.get("assist.traction_control")==1,"F4 cycles through native layer")
	check(result.get("assist.auto_shift")==1 and adapter.get_auto_shift(),"F5 preference through native layer")
	check(result.get("assist.aero_control")==1 and result.get("assist.launch_control")==1,"H/L toggles through native layer")
	check(result.get("assist.drive_mode")==1 and result.get("nitrous_arm")==1,"M/N through native layer")
	adapter.set_test_snapshot({"keys":PackedStringArray()}); adapter.poll()
	adapter.set_test_snapshot({"keys":PackedStringArray(["F4"])}); adapter.poll()
	adapter.driver_actions.discard_pending()
	check(adapter.driver_actions.apply(snapshot,result,true).size()==1,"discarded edge cannot replay on unblock")
	adapter.free()
	controls.free()
	DirAccess.remove_absolute(user_dir)
	print("RG_DRIVER_ACTIONS_TEST failures=%d" % failures)
	quit(1 if failures else 0)
