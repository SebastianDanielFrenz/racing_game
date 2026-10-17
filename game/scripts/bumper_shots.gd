extends SceneTree
# game/scripts/bumper_shots.gd - one bumper-camera screenshot of a car in the
# flat world (needs a real renderer, e.g. xvfb-run):
#   godot --path game --script res://scripts/bumper_shots.gd -- --flat --vehicle car_gt70 --bumper-out out.png [--bumper-old]
# --bumper-old shows the old wheel-based eye (no model bounds) for comparison.
# Prints RG_BUMPER_SHOT with the eye in the model frame and the model's nose x
# (eye_x_model > nose_x = the camera is ahead of the body).
var main: Node

func _initialize() -> void:
	_run.call_deferred()

func _arg(name: String) -> String:
	var a := OS.get_cmdline_user_args()
	var i := a.find(name)
	return a[i + 1] if i >= 0 and i + 1 < a.size() else ""

func _run() -> void:
	var out := _arg("--bumper-out")
	var old := OS.get_cmdline_user_args().has("--bumper-old")
	main = load("res://scenes/main.tscn").instantiate()
	root.add_child(main)
	var began := Time.get_ticks_msec()
	var visuals: Node = null
	while true:
		await process_frame
		if Time.get_ticks_msec() - began > 120000:
			push_error("RG_BUMPER_SHOT timeout")
			quit(1)
			return
		if main.get("world_state") != "running":
			continue
		visuals = main.get_body_visuals()
		if visuals != null and visuals.vehicle_model_ok():
			break
	main.scripted_controls = {"throttle": 0.0, "brake": 1.0, "handbrake": 1.0, "steer": 0.0, "clutch": 0.0, "ignition": 1.0, "starter": 0.0}
	var rig: Node = main._director.rigs["bumper"]
	if old:
		rig.body_visuals = null
	main.set_drive_view("bumper")
	for i in range(90):
		await process_frame
	var key: Array = visuals.bumper_camera_inputs()
	var mount: Dictionary = rig._mount
	var eye_model: Vector3 = (mount.get("eye_local", Vector3.ZERO) as Vector3) - (key[1] as Vector3)
	var nose := NAN
	if str(key[0]) != "":
		var rj = JSON.parse_string(FileAccess.get_file_as_string(str(key[0])))
		nose = float(rj["overall_bounds"]["max"][0])
	print("RG_BUMPER_SHOT vehicle=%s mode=%s source=%s eye_x_model=%.3f eye_z_model=%.3f nose_x=%.3f ahead_m=%.3f" % [_arg("--vehicle"), "old" if old else "new", mount.get("source", "?"), eye_model.x, eye_model.z, nose, eye_model.x - nose])
	if out != "":
		var img := root.get_viewport().get_texture().get_image()
		img.save_png(out)
	quit(0)
