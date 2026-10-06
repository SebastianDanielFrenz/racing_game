extends Node
# game/scripts/controls_shots.gd - `--controls-shots <dir>` (needs a real window, not --headless;
# tools/controls_shots.ps1): drives the real shell to the controls screen with a simulated pad
# and saves the window as PNGs (PLAN.md R5b):
#   01_device_list.png            keyboard, mouse, a connected pad and a remembered (unplugged) one
#   02_action_list_keyboard.png   the keyboard's actions by group (a conflict is shown inline)
#   02_action_list_pad.png        the pad's actions
#   03_capture_prompt.png         "press the input you want to bind"
#   04_axis_tuning_monitor.png    a stick selected: tuning sliders over the live input monitor
#   05_calibration.png            the calibration prompt on a trigger

const GUID_A := "030000005e040000ea02000000000000"
const GUID_B := "030000004c050000c405000000000000"

var main: Node
var out_dir: String = ""

func _ready() -> void:
	_run.call_deferred()

func _frames(n: int) -> void:
	for _i in range(n):
		await get_tree().process_frame

func _seconds(s: float) -> void:
	var deadline := Time.get_ticks_msec() + int(s * 1000.0)
	while Time.get_ticks_msec() < deadline:
		await get_tree().process_frame

func _shot(name: String) -> void:
	await _frames(3)
	await RenderingServer.frame_post_draw
	var image := get_viewport().get_texture().get_image()
	var path := out_dir.path_join(name + ".png")
	var err := image.save_png(path)
	print("RG_CONTROLS_SHOT %s err=%d size=%s" % [path, err, image.get_size()])

func _key(keycode: Key) -> void:
	for pressed in [true, false]:
		var ev := InputEventKey.new()
		ev.keycode = keycode
		ev.physical_keycode = keycode
		ev.pressed = pressed
		Input.parse_input_event(ev)
		await _frames(3)

func _axes(values: Dictionary) -> PackedFloat32Array:
	var axes := PackedFloat32Array([0, 0, 0, 0, 0, 0])
	for k in values.keys():
		axes[int(k)] = float(values[k])
	return axes

func _run() -> void:
	DirAccess.make_dir_recursive_absolute(out_dir)
	await _seconds(0.5)
	main._apply_transition(main.get_shell().boot_finished())
	await _frames(5)
	var controls: Node = main.get_controls()
	var key_a: String = controls.joypad_connected(900, GUID_A, "Xbox Wireless Controller", 0x045e, 0x02ea, true)
	var key_b: String = controls.joypad_connected(901, GUID_B, "Wireless Controller", 0x054c, 0x05c4, true)
	# make the second pad "remembered": customise it, then unplug it
	controls.bind_binding(key_b, "handbrake", 0, {"type": "joy_button", "index": 1}, true)
	controls.joypad_disconnected(901)
	main._apply_transition(main.get_shell().menu_item("settings"))
	await _frames(5)
	main._apply_transition(main.get_shell().menu_item("controls"))
	await _seconds(0.6)
	var ui: CanvasLayer = main.get_shell_ui()
	var cui: Control = ui.controls_screen()
	cui.select_device("keyboard")
	await _seconds(0.3)
	await _shot("01_device_list")
	# a conflict: the brake on the throttle's key
	controls.bind_binding("keyboard", "brake", 0, {"type": "key", "key": "W"}, false)
	cui.refresh()
	await _seconds(0.3)
	await _shot("02_action_list_keyboard")
	controls.reset_action("keyboard", "brake")
	cui.select_device(key_a)
	await _seconds(0.3)
	await _shot("02_action_list_pad")
	# the capture prompt
	cui.inject_pad(900, PackedInt32Array(), _axes({}))
	cui.begin_capture("throttle", 0, true)
	await _seconds(0.3)
	await _shot("03_capture_prompt")
	await _key(KEY_ESCAPE)
	# the stick: tuning + live monitor
	var steer_idx := -1
	for r in controls.get_rows(key_a):
		if str(r["action"]) == "steer":
			for b in r["bindings"]:
				if bool(b["analogue"]) and steer_idx < 0:
					steer_idx = int(b["list_index"])
					cui.get_hook("bind:steer:%d" % int(r["sign"])).focus_entered.emit()
	await _seconds(0.2)
	var dz: HSlider = cui.get_hook("tune:deadzone:%d" % steer_idx)
	if dz != null:
		dz.value = 0.15
	cui.inject_pad(900, PackedInt32Array([0]), _axes({0: 0.62, 5: 0.35}))
	await _seconds(0.4)
	await _shot("04_axis_tuning_monitor")
	# calibration of the right trigger
	cui.inject_pad(900, PackedInt32Array(), _axes({}))
	var thr_idx := -1
	for r in controls.get_rows(key_a):
		if str(r["action"]) == "throttle":
			thr_idx = int(r["bindings"][0]["list_index"])
	cui.get_hook("bind:throttle:0").focus_entered.emit()
	await _frames(3)
	var cal: Button = cui.get_hook("calibrate:%d" % thr_idx)
	if cal != null:
		cal.pressed.emit()
		for v in [0.3, 0.7, 1.0, 0.6]:
			cui.inject_pad(900, PackedInt32Array(), _axes({5: v}))
			await _frames(3)
		await _shot("05_calibration")
	print("RG_CONTROLS_SHOT done")
	get_tree().quit(0)
