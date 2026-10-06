extends Node
# game/scripts/controls_test.gd - `--controls-test` / `--controls-verify` (tools/smoke_test.ps1 -Controls):
# the headless acceptance flow of the control configuration (PLAN.md R5b). It drives the real screen like a
# player (Button.pressed.emit(), synthetic key events through Input.parse_input_event, an injected pad
# snapshot for simulated joypads) and asserts what rg::Controls / controls_ui.gd / input_map.gd did.
#
# Phase "write" (`--controls-test`): main menu -> Settings -> Controls; every default keyboard binding name
# survives the engine's key-name round trip; rebind throttle on the KEYBOARD by capture (synthetic key press),
# Esc cancels a capture, a conflict is shown inline and resolved, a second binding is added; a simulated
# pad (SDL GUID A) gets its own profile: capture a pad BUTTON for throttle, tune a stick (deadzone, invert),
# calibrate a trigger through the screen's own calibration flow, the live monitor follows the injected axes;
# a second pad of the SAME GUID shares the first one's setup, a pad of ANOTHER GUID does not inherit it; the
# input map really uses the new bindings (W no longer throttles, U does, pad button 0 does on pad A only);
# reset action / reset device; Esc leaves the screen and the file is written (rg.controls/1); a second RgControls
# reading the same file sees every choice; a broken file gives defaults + .bak and never an error.
#
# Phase "verify" (`--controls-verify`, a NEW Godot process on the same --shell-user-dir): the choices of the
# write phase were loaded at boot, before any input was read.
#
# Every check prints "RG_CONTROLS_TEST ok: ..." or "... FAIL: ..."; the last line is "RG_CONTROLS_TEST PASS checks=N"
# or "... FAIL failures=N". Run with `--shell-user-dir <dir>`.

const TIMEOUT_S := 120.0
const GUID_A := "030000005e040000ea02000000000000"
const GUID_B := "030000004c050000c405000000000000"

var main: Node
var verify_phase := false
var _checks := 0
var _failures := 0
var controls: Node
var input_map: Node

func _ready() -> void:
	_run.call_deferred()

func _check(condition: bool, what: String) -> bool:
	_checks += 1
	if condition:
		print("RG_CONTROLS_TEST ok: %s" % what)
	else:
		_failures += 1
		print("RG_CONTROLS_TEST FAIL: %s" % what)
	return condition

func _finish() -> void:
	if _failures == 0:
		print("RG_CONTROLS_TEST PASS checks=%d" % _checks)
		get_tree().quit(0)
	else:
		print("RG_CONTROLS_TEST FAIL failures=%d checks=%d" % [_failures, _checks])
		get_tree().quit(1)

func _wait_until(condition: Callable, timeout_s: float = 10.0) -> bool:
	var deadline := Time.get_ticks_msec() + int(timeout_s * 1000.0)
	while not condition.call():
		if Time.get_ticks_msec() > deadline:
			return false
		await get_tree().process_frame
	return true

func _frames(n: int) -> void:
	for i in range(n):
		await get_tree().process_frame

func _screen() -> String:
	return main.shell_screen()

func _press(id: String) -> bool:
	var ui: CanvasLayer = main.get_shell_ui()
	var button: Button = ui.get_button(id)
	if button == null:
		_check(false, "button '%s' exists on screen '%s' (has %s)" % [id, _screen(), ", ".join(ui.button_ids())])
		return false
	button.pressed.emit()
	return true

func _send_key(keycode: Key, pressed: bool) -> void:
	var ev := InputEventKey.new()
	ev.keycode = keycode
	ev.physical_keycode = keycode
	ev.pressed = pressed
	Input.parse_input_event(ev)

func _tap_key(keycode: Key, hold_frames: int = 4) -> void:
	_send_key(keycode, true)
	await _frames(hold_frames)
	_send_key(keycode, false)
	await _frames(2)

func _row(device: String, action: String, sign: int = 0) -> Dictionary:
	for r in controls.get_rows(device):
		if str(r["action"]) == action and int(r["sign"]) == sign:
			return r
	return {}

func _texts(device: String, action: String, sign: int = 0) -> PackedStringArray:
	var out := PackedStringArray()
	for b in _row(device, action, sign).get("bindings", []):
		out.append(str(b["text"]))
	return out

func _has_key(device: String, action: String, key: String, sign: int = 0) -> bool:
	for b in _row(device, action, sign).get("bindings", []):
		if str(b["type"]) == "key" and str(b["key"]) == key:
			return true
	return false

func _snap_keys(keys: Array) -> Dictionary:
	return {"keys": PackedStringArray(keys)}

func _snap_pad(slot: int, buttons: Array, axes: Array) -> Dictionary:
	return {"pads": [{"slot": slot, "buttons": PackedInt32Array(buttons), "axes": PackedFloat32Array(axes)}]}

func _run() -> void:
	var guard := get_tree().create_timer(TIMEOUT_S)
	guard.timeout.connect(func():
		print("RG_CONTROLS_TEST FAIL: timeout on screen '%s'" % main.shell_screen())
		get_tree().quit(1))
	controls = main.get_controls()
	input_map = main.get_input_map()
	if not _check(controls != null and controls.is_ready(), "RgControls is ready"):
		_finish()
		return
	_check(await _wait_until(func(): return _screen() == "main_menu", 10.0), "the boot splash ends and the main menu follows")
	if verify_phase:
		await _verify()
	else:
		await _write()
	_finish()

# ---------------------------------------------------------------------------------------------------------------

func _write() -> void:
	var ui: CanvasLayer = main.get_shell_ui()
	_check(str(controls.get_load_message()) == "", "a fresh user dir loads without a message")

	# every default keyboard binding name survives the engine's key-name round trip
	var bad := PackedStringArray()
	var key_count := 0
	for r in controls.get_rows("keyboard"):
		for b in r["bindings"]:
			if str(b["type"]) == "key":
				key_count += 1
				var code: int = OS.find_keycode_from_string(str(b["key"]))
				if code == 0 or OS.get_keycode_string(code as Key) != str(b["key"]):
					bad.append(str(b["key"]))
	_check(key_count > 20 and bad.is_empty(), "%d default keyboard bindings round-trip as engine key names (bad: %s)" % [key_count, ", ".join(bad)])
	_check(_has_key("keyboard", "throttle", "W"), "the default keyboard throttle is W")

	# ---- the screen is reachable from the main menu's settings ----
	_press("settings")
	_check(_screen() == "settings", "Settings opens (screen '%s')" % _screen())
	_press("controls")
	_check(_screen() == "controls", "Settings -> Controls opens the controls screen (screen '%s')" % _screen())
	await _frames(3)
	var cui: Control = ui.controls_screen()
	if not _check(cui != null, "the controls screen exists"):
		return
	_check(cui.get_hook("device:keyboard") != null and cui.get_hook("device:mouse") != null, "the device list shows the keyboard and the mouse")
	_check(cui.selected_device() == "keyboard", "the keyboard is selected first")
	_check(cui.get_hook("bind:throttle:0") != null and cui.get_hook("bind:steer:-1") != null, "the action list has a row per action / side of an axis")
	_check(input_map.suspended, "the input map is suspended while the screen is up")

	# ---- keyboard rebind by capture ----
	cui.get_hook("bind:throttle:0").pressed.emit()
	_check(cui.is_capturing(), "pressing a binding button starts a capture")
	await _tap_key(KEY_ESCAPE)
	_check(not cui.is_capturing() and _screen() == "controls", "Esc cancels the capture and stays on the screen")
	_check(_has_key("keyboard", "throttle", "W"), "a cancelled capture changes nothing")
	cui.get_hook("bind:throttle:0").pressed.emit()
	await _frames(2)
	await _tap_key(KEY_U)
	_check(not cui.is_capturing(), "a key press ends the capture")
	_check(_has_key("keyboard", "throttle", "U") and not _has_key("keyboard", "throttle", "W"), "throttle on the keyboard is now U (not W): %s" % ", ".join(_texts("keyboard", "throttle")))
	_check(controls.is_dirty(), "the controls are marked changed")
	# the input map follows at once (it is suspended while this screen is up: lift that for the probe)
	input_map.suspended = false
	input_map.set_test_snapshot(_snap_keys(["U"]))
	await _frames(3)
	var thr_u: float = input_map.get_throttle()
	input_map.set_test_snapshot(_snap_keys(["W"]))
	await _frames(3)
	var thr_w: float = input_map.get_throttle()
	input_map.set_test_snapshot({})
	input_map.suspended = true
	_check(thr_u > 0.9 and thr_w < 0.1, "the input map throttles on U (%.2f) and no longer on W (%.2f)" % [thr_u, thr_w])
	# conflict: brake onto U too
	cui.get_hook("bind:brake:0").pressed.emit()
	await _frames(2)
	await _tap_key(KEY_U)
	_check(controls.get_conflicts("keyboard").size() > 0, "binding U to the brake as well is reported as a conflict")
	_check(cui.status_text().contains("also used by"), "the screen says so inline: '%s'" % cui.status_text())
	_check(bool(_row("keyboard", "brake").get("conflict", false)) and bool(_row("keyboard", "throttle").get("conflict", false)), "both rows are flagged")
	cui.get_hook("reset:brake:0").pressed.emit()
	await _frames(2)
	_check(controls.get_conflicts("keyboard").size() == 0 and _has_key("keyboard", "brake", "S"), "reset action resolves the conflict (brake back on S)")
	# a second binding
	cui.get_hook("add:handbrake:0").pressed.emit()
	await _frames(2)
	await _tap_key(KEY_B)
	_check(_has_key("keyboard", "handbrake", "B") and _has_key("keyboard", "handbrake", "Space"), "'+' adds a second handbrake binding: %s" % ", ".join(_texts("keyboard", "handbrake")))
	cui.get_hook("clear:handbrake:0").pressed.emit()
	await _frames(2)
	_check(_row("keyboard", "handbrake").get("bindings", []).is_empty(), "Clear removes every binding of the row")
	cui.get_hook("reset:handbrake:0").pressed.emit()
	await _frames(2)
	_check(_has_key("keyboard", "handbrake", "Space"), "Reset brings the default back")

	# ---- a simulated pad of GUID A gets its own profile ----
	var key_a: String = controls.joypad_connected(900, GUID_A, "Xbox Wireless Controller", 0x045e, 0x02ea, true)
	await _frames(3)
	_check(key_a == "joy:%s#1" % GUID_A, "a pad is keyed by its SDL GUID and ordinal (%s)" % key_a)
	_check(cui.get_hook("device:" + key_a) != null, "the pad appears in the device list")
	cui.get_hook("device:" + key_a).pressed.emit()
	await _frames(2)
	_check(cui.selected_device() == key_a, "selecting the pad shows its profile")
	_check(controls.get_devices().any(func(d): return str(d["key"]) == key_a and bool(d["connected"])), "the pad is listed as connected")
	cui.inject_pad(900, PackedInt32Array(), PackedFloat32Array([0, 0, 0, 0, 0, 0]))
	cui.get_hook("bind:throttle:0").pressed.emit()
	await _frames(2)
	_check(cui.is_capturing(), "a capture starts for the pad")
	cui.inject_pad(900, PackedInt32Array([0]), PackedFloat32Array([0, 0, 0, 0, 0, 0]))
	await _frames(4)
	cui.inject_pad(900, PackedInt32Array(), PackedFloat32Array([0, 0, 0, 0, 0, 0]))
	await _frames(2)
	var pad_row := _row(key_a, "throttle")
	_check(not cui.is_capturing() and pad_row.get("bindings", []).size() == 1 and str(pad_row["bindings"][0]["type"]) == "joy_button" and int(pad_row["bindings"][0]["index"]) == 0,
		"pad button A is now the pad's throttle: %s" % ", ".join(_texts(key_a, "throttle")))
	_check(bool(controls.get_devices().filter(func(d): return str(d["key"]) == key_a)[0]["customised"]), "the pad's profile is customised")
	_check(_has_key("keyboard", "throttle", "U"), "the keyboard profile is untouched by the pad edit")
	# the input map: pad A button 0 throttles, a snapshot of another slot does not
	input_map.suspended = false
	input_map.set_test_snapshot(_snap_pad(900, [0], [0, 0, 0, 0, 0, 0]))
	await _frames(3)
	var pad_thr: float = input_map.get_throttle()
	input_map.set_test_snapshot({})
	input_map.suspended = true

	# a second pad of the same GUID shares the setup, a pad of another GUID does not
	var key_a2: String = controls.joypad_connected(901, GUID_A, "Xbox Wireless Controller", 0x045e, 0x02ea, true)
	var key_b: String = controls.joypad_connected(902, GUID_B, "PS4 Controller", 0x054c, 0x05c4, true)
	_check(key_a2 == "joy:%s#2" % GUID_A and key_b == "joy:%s#1" % GUID_B, "same GUID gives #2, another GUID its own key (%s, %s)" % [key_a2, key_b])
	_check(str(_row(key_a2, "throttle")["bindings"][0]["type"]) == "joy_button", "a second pad of the same GUID starts from the first one's setup")
	var b_throttle: Array = _row(key_b, "throttle")["bindings"]
	_check(not b_throttle.is_empty() and str(b_throttle[0]["type"]) == "joy_axis", "a pad of another GUID does not inherit it (its default: %s)" % ", ".join(_texts(key_b, "throttle")))
	input_map.suspended = false
	input_map.set_test_snapshot(_snap_pad(902, [0], [0, 0, 0, 0, 0, 0]))
	await _frames(3)
	var other_thr: float = input_map.get_throttle()
	input_map.set_test_snapshot({})
	input_map.suspended = true
	_check(pad_thr > 0.9 and other_thr < 0.1, "pad A's button throttles (%.2f) while the same button on the other GUID does not (%.2f)" % [pad_thr, other_thr])

	# ---- axis tuning on pad A's stick ----
	await _frames(2)
	cui.get_hook("device:" + key_a).pressed.emit()
	await _frames(2)
	var steer_rows: Array = []
	for r in controls.get_rows(key_a):
		if str(r["action"]) == "steer":
			steer_rows.append(r)
	var steer_idx := -1
	var steer_row: Dictionary = {}
	for r in steer_rows:
		for b in r["bindings"]:
			if bool(b["analogue"]) and steer_idx < 0:
				steer_idx = int(b["list_index"])
				steer_row = r
	if _check(steer_idx >= 0, "the pad's steer row has an analogue binding"):
		cui.get_hook("bind:steer:%d" % int(steer_row["sign"])).focus_entered.emit()
		await _frames(2)
		var dz: HSlider = cui.get_hook("tune:deadzone:%d" % steer_idx)
		var inv: CheckBox = cui.get_hook("invert:%d" % steer_idx)
		if _check(dz != null and inv != null, "the detail panel offers deadzone and invert sliders"):
			dz.value = 0.3
			inv.button_pressed = true
			await _frames(2)
			var tuned: Dictionary = controls.get_binding(key_a, "steer", steer_idx)
			_check(absf(float(tuned["deadzone"]) - 0.3) < 1e-4 and bool(tuned["invert"]), "deadzone 0.3 and invert reached the profile (%.2f, %s)" % [float(tuned["deadzone"]), str(tuned["invert"])])
			# the live monitor follows the injected stick
			var axis := int(tuned["index"])
			var axes := PackedFloat32Array([0, 0, 0, 0, 0, 0])
			axes[axis] = 0.8
			cui.inject_pad(900, PackedInt32Array(), axes)
			await _frames(3)
			var shown := false
			for entry in cui._mon_mapped:
				if (entry["box"] as Control).visible and str((entry["name"] as Label).text).begins_with("Steer"):
					shown = true
			_check(shown, "the live monitor lists the steering the game receives for the injected axis")
			var raw_bar: ProgressBar = cui._mon_raw[axis]["bar"]
			_check(absf(raw_bar.value - 0.8) < 0.01, "the raw axis bar shows 0.80 (%.2f)" % raw_bar.value)
			var mon: Array = controls.monitor_device(key_a, _snap_pad(900, [], axes))
			var steer_value := 0.0
			for m in mon:
				if str(m["action"]) == "steer":
					steer_value = float(m["value"])
			_check(steer_value < -0.1, "inverted, tuned steering reaches the game as %.2f (inverted, deadzone applied)" % steer_value)
			cui.inject_pad(900, PackedInt32Array(), PackedFloat32Array([0, 0, 0, 0, 0, 0]))

	# ---- calibration of a trigger through the screen's own flow (pad B's default throttle axis) ----
	cui.get_hook("device:" + key_b).pressed.emit()
	await _frames(2)
	var thr_binding: Dictionary = _row(key_b, "throttle")["bindings"][0]
	var thr_index := int(thr_binding["list_index"])
	var thr_axis := int(thr_binding["index"])
	cui.inject_pad(902, PackedInt32Array(), PackedFloat32Array([0, 0, 0, 0, 0, 0]))
	cui.get_hook("bind:throttle:0").focus_entered.emit()
	await _frames(2)
	var cal_button: Button = cui.get_hook("calibrate:%d" % thr_index)
	if _check(cal_button != null, "an axis binding offers Calibrate"):
		cal_button.pressed.emit()
		await _frames(2)
		_check(cui.is_calibrating() and cui.get_hook("calib_apply").disabled, "calibration starts, Apply waits for travel")
		for v in [0.2, 0.5, 0.9, 1.0, 0.5, 0.0]:
			var ax := PackedFloat32Array([0, 0, 0, 0, 0, 0])
			ax[thr_axis] = v
			cui.inject_pad(902, PackedInt32Array(), ax)
			await _frames(2)
		_check(not cui.get_hook("calib_apply").disabled, "Apply is enabled once the axis travelled")
		cui.get_hook("calib_apply").pressed.emit()
		await _frames(2)
		_check(not cui.is_calibrating() and bool(controls.get_binding(key_b, "throttle", thr_index)["calibrated"]), "applying stores the calibration in the profile")
	cui.clear_pad_injection()
	cui.get_hook("device:" + key_b).pressed.emit()
	await _frames(1)

	# ---- reset device (two presses) ----
	var reset_btn: Button = cui.get_hook("reset_device")
	reset_btn.pressed.emit()
	_check(_row(key_b, "throttle")["bindings"][0]["calibrated"] == true, "the first press of 'Reset device' only arms it")
	reset_btn.pressed.emit()
	await _frames(2)
	_check(_row(key_b, "throttle")["bindings"][0]["calibrated"] == false, "the second press resets the device to its defaults")

	# ---- leaving: Esc backs out and the file is written ----
	await _tap_key(KEY_ESCAPE)
	_check(_screen() == "settings", "Esc leaves the controls screen (screen '%s')" % _screen())
	_check(not input_map.suspended, "the input map is live again")
	var path: String = str(controls.get_file_path())
	_check(FileAccess.file_exists(path), "user://controls.json was written (%s)" % path)
	var parsed = JSON.parse_string(FileAccess.get_file_as_string(path))
	_check(parsed is Dictionary and str(parsed.get("format", "")) == "rg.controls/1", "its format is rg.controls/1")
	_check(str(FileAccess.get_file_as_string(path)).contains(GUID_A), "the pad's GUID is in the file")
	_check(not FileAccess.file_exists(path + ".tmp"), "no temporary file is left behind")

	# ---- a second instance reads the file ----
	var other: Node = ClassDB.instantiate("RgControls")
	add_child(other)
	var report: Dictionary = other.initialize(main._rg_data_path(""), main._user_dir())
	_check(bool(report["ok"]) and bool(report["load_ok"]) and not bool(report["file_missing"]), "a new RgControls loads the saved file")
	var other_rows: Array = other.get_rows("keyboard")
	var other_throttle := ""
	for r in other_rows:
		if str(r["action"]) == "throttle":
			other_throttle = str(r["bindings"][0]["key"])
	_check(other_throttle == "U", "the new instance has throttle on U")
	var key_a_again: String = other.joypad_connected(5, GUID_A, "Xbox Wireless Controller", 0x045e, 0x02ea, true)
	var found_button := false
	for r in other.get_rows(key_a_again):
		if str(r["action"]) == "throttle" and not r["bindings"].is_empty() and str(r["bindings"][0]["type"]) == "joy_button":
			found_button = true
	_check(found_button, "the same pad plugged into the new instance gets its configuration back (key %s)" % key_a_again)
	other.queue_free()

	# ---- a broken file never stops the game ----
	var broken_dir: String = str(main._user_dir()).path_join("broken_controls")
	DirAccess.make_dir_recursive_absolute(broken_dir)
	var f := FileAccess.open(broken_dir.path_join("controls.json"), FileAccess.WRITE)
	f.store_string("{ this is not json")
	f.close()
	var broken: Node = ClassDB.instantiate("RgControls")
	add_child(broken)
	var broken_report: Dictionary = broken.initialize(main._rg_data_path(""), broken_dir)
	_check(bool(broken_report["ok"]) and not bool(broken_report["load_ok"]) and str(broken_report["message"]) != "", "a broken file is reported ('%s')" % str(broken_report["message"]))
	_check(FileAccess.file_exists(broken_dir.path_join("controls.json.bak")), "the broken file is kept as controls.json.bak")
	var w_default := false
	for r in broken.get_rows("keyboard"):
		if str(r["action"]) == "throttle":
			w_default = str(r["bindings"][0]["key"]) == "W"
	_check(w_default, "the defaults are in force after a broken file")
	broken.queue_free()

	# the pad B / pad A edits stay in memory for the real file: keep the pad customised for the verify phase
	controls.save(true)

# ---------------------------------------------------------------------------------------------------------------

func _verify() -> void:
	_check(str(controls.get_load_message()) == "", "the file of the previous run loaded without a message (%s)" % str(controls.get_load_message()))
	_check(FileAccess.file_exists(str(controls.get_file_path())), "controls.json exists at boot")
	_check(_has_key("keyboard", "throttle", "U") and not _has_key("keyboard", "throttle", "W"), "the keyboard throttle on U survived the restart")
	input_map.set_test_snapshot(_snap_keys(["U"]))
	await _frames(3)
	var thr: float = input_map.get_throttle()
	input_map.set_test_snapshot({})
	_check(thr > 0.9, "the input map read the file before the first poll (throttle on U = %.2f)" % thr)
	var key_a: String = controls.joypad_connected(900, GUID_A, "Xbox Wireless Controller", 0x045e, 0x02ea, true)
	_check(key_a == "joy:%s#1" % GUID_A, "the pad is recognised by its GUID after the restart")
	var pad_row := _row(key_a, "throttle")
	_check(pad_row.get("bindings", []).size() == 1 and str(pad_row["bindings"][0]["type"]) == "joy_button", "its own configuration (button as throttle) is back")
	input_map.set_test_snapshot(_snap_pad(900, [0], [0, 0, 0, 0, 0, 0]))
	await _frames(3)
	var pad_thr: float = input_map.get_throttle()
	input_map.set_test_snapshot({})
	_check(pad_thr > 0.9, "pressing that button throttles (%.2f)" % pad_thr)
	var key_b: String = controls.joypad_connected(902, GUID_B, "PS4 Controller", 0x054c, 0x05c4, true)
	_check(str(_row(key_b, "throttle")["bindings"][0]["type"]) == "joy_axis", "a pad of another GUID is on its defaults")
	var listed := false
	var disconnected_listed := false
	controls.joypad_disconnected(900)
	for d in controls.get_devices():
		if str(d["key"]) == key_a and not bool(d["connected"]):
			disconnected_listed = true
		if str(d["key"]) == key_a:
			listed = true
	_check(listed and disconnected_listed, "an unplugged pad stays listed as not connected")
	# the screen lists the remembered device
	var ui: CanvasLayer = main.get_shell_ui()
	_press("settings")
	_press("controls")
	await _frames(3)
	var cui: Control = ui.controls_screen()
	_check(cui != null and cui.get_hook("device:" + key_a) != null, "the controls screen lists the remembered pad")
	_check(cui != null and cui.get_hook("bind:throttle:0") != null, "...and the keyboard actions")
	if cui != null:
		cui.get_hook("device:" + key_a).pressed.emit()
		await _frames(2)
		_check(cui.get_hook("bind:throttle:0").disabled, "a pad that is not connected cannot be captured (its Bind button is disabled)")
