extends Node
# game/scripts/shell_flow_test.gd - `--shell-test` (tools/smoke_test.ps1 -Shell):
# the headless UI flow test of the game shell (PLAN.md R5 acceptance). It drives
# the real screens the way a player does - presses the real buttons
# (Button.pressed.emit()), injects a real Esc key event - and asserts what
# rg::ShellFlow / main.gd did:
#
#   boot -> main menu -> settings (change a value, Back saves, a second RgShell
#   reloads it) -> back -> credits -> back -> Free roam -> spawn picker -> flat
#   world -> loading -> drive -> Esc pause (sim paused, Resume) -> Reset car ->
#   pause -> Main menu (world torn down: sim stopped, nodes released) -> Free
#   roam AGAIN in the same process (runs, same node count afterwards) -> main
#   menu -> Quit.
#
# With `--shell-real` (tools/smoke_test.ps1 -Shell adds it when a geo2map store
# exists) a third round uses the real world: Free roam -> world_spawn preset ->
# Esc during the loading screen (cancel -> main menu, world unloaded) -> again
# -> the real world loads and drives -> pause -> main menu -> unloaded.
#
# Every check prints "RG_SHELL_TEST ok: ..." or "RG_SHELL_TEST FAIL: ..."; the
# last line is "RG_SHELL_TEST PASS checks=N" or "RG_SHELL_TEST FAIL failures=N"
# (the smoke asserts on it; exit code 1 on a failure). Run with
# `--shell-user-dir <dir>` so the settings the test writes never touch the real
# user://.

const TIMEOUT_S := 60.0
const TIMEOUT_REAL_S := 400.0
const EXPECTED_MAIN_ITEMS := ["free_roam", "settings", "credits", "quit"]
const EXPECTED_PAUSE_ITEMS := ["resume", "reset_car", "settings", "main_menu"]

var main: Node
var _checks := 0
var _failures := 0
var _started_ms := 0

func _ready() -> void:
	_started_ms = Time.get_ticks_msec()
	_run.call_deferred()

func _check(condition: bool, what: String) -> bool:
	_checks += 1
	if condition:
		print("RG_SHELL_TEST ok: %s" % what)
	else:
		_failures += 1
		print("RG_SHELL_TEST FAIL: %s" % what)
	return condition

func _finish() -> void:
	if _failures == 0:
		print("RG_SHELL_TEST PASS checks=%d" % _checks)
	else:
		print("RG_SHELL_TEST FAIL failures=%d checks=%d" % [_failures, _checks])
		get_tree().quit(1)

# Wait (frames) until `condition` holds; false on timeout.
func _wait_until(condition: Callable, timeout_s: float = 20.0) -> bool:
	var deadline := Time.get_ticks_msec() + int(timeout_s * 1000.0)
	while not condition.call():
		if Time.get_ticks_msec() > deadline:
			return false
		await get_tree().process_frame
	return true

func _wait_seconds(seconds: float) -> void:
	var deadline := Time.get_ticks_msec() + int(seconds * 1000.0)
	while Time.get_ticks_msec() < deadline:
		await get_tree().process_frame

func _screen() -> String:
	return main.shell_screen()

func _press(id: String) -> bool:
	var ui: CanvasLayer = main.get_shell_ui()
	var button: Button = ui.get_button(id)
	if not _check(button != null, "button '%s' exists on screen '%s' (has %s)" % [id, _screen(), ", ".join(ui.button_ids())]):
		return false
	button.pressed.emit()
	return true

func _press_escape() -> void:
	var down := InputEventKey.new()
	down.keycode = KEY_ESCAPE
	down.physical_keycode = KEY_ESCAPE
	down.pressed = true
	Input.parse_input_event(down)
	var up := InputEventKey.new()
	up.keycode = KEY_ESCAPE
	up.physical_keycode = KEY_ESCAPE
	up.pressed = false
	Input.parse_input_event(up)

func _count_nodes(root: Node) -> int:
	var n := 1
	for child in root.get_children():
		n += _count_nodes(child)
	return n

func _sorted_ids(ui: CanvasLayer) -> Array:
	var ids: Array = []
	for id in ui.button_ids():
		ids.append(str(id))
	ids.sort()
	return ids

func _label_texts(node: Node, into: Array) -> void:
	if node is Label:
		into.append((node as Label).text)
	for child in node.get_children():
		_label_texts(child, into)

# Free roam -> flat world -> Drive. Returns whether the Drive screen was reached.
func _start_flat_world(round_name: String) -> bool:
	var ui: CanvasLayer = main.get_shell_ui()
	if not _press("free_roam"):
		return false
	_check(_screen() == "spawn_picker", "%s: Free roam opens the spawn picker" % round_name)
	var spawn_buttons: Array = []
	for id in ui.button_ids():
		if str(id).begins_with("spawn:"):
			spawn_buttons.append(str(id))
	_check(spawn_buttons.has("spawn:flat"), "%s: the spawn picker offers the flat world" % round_name)
	_check(spawn_buttons.size() >= 3, "%s: the spawn picker lists the presets too (%s)" % [round_name, ", ".join(spawn_buttons)])
	if not _press("spawn:flat"):
		return false
	_check(_screen() == "loading", "%s: picking a spawn shows the loading screen (is '%s')" % [round_name, _screen()])
	var reached: bool = await _wait_until(func(): return _screen() == "drive")
	if not _check(reached, "%s: the flat world finishes loading and the flow reaches drive (screen '%s')" % [round_name, _screen()]):
		return false
	_check(main.world_state == "running" and main.world_kind == "flat", "%s: world_state running / kind flat" % round_name)
	var sim: Node = main._simulation
	_check(sim.is_running(), "%s: the sim thread runs" % round_name)
	var steps0: int = sim.get_step_count()
	await _wait_seconds(0.4)
	_check(sim.get_step_count() > steps0, "%s: the sim ticks (%d -> %d)" % [round_name, steps0, sim.get_step_count()])
	_check(main._hud.visible, "%s: the HUD is shown while driving" % round_name)
	return true

func _real_world_round(node_count_before: int) -> void:
	var ui: CanvasLayer = main.get_shell_ui()
	var sim: Node = main._simulation
	# (a) cancel a real-world load with Esc on the loading screen
	if not _press("free_roam"):
		return
	var preset: Button = ui.get_button("spawn:world_spawn")
	if not _check(preset != null and not preset.disabled, "real: the world_spawn preset is available"):
		return
	_press("spawn:world_spawn")
	_check(_screen() == "loading", "real: picking a preset shows the loading screen (is '%s')" % _screen())
	await _wait_seconds(0.5)
	_press_escape()
	_check(await _wait_until(func(): return _screen() == "main_menu", 10.0), "real: Esc on the loading screen returns to the main menu (is '%s')" % _screen())
	_check(main.world_state == "none", "real: the cancelled load left no world (state '%s')" % main.world_state)
	_check(not sim.is_running(), "real: the sim is not running after the cancel")
	await _wait_seconds(0.3)
	_check(_count_nodes(main) == node_count_before, "real: node count after the cancel equals the start (%d vs %d)" % [_count_nodes(main), node_count_before])
	# (b) load it for real
	_press("free_roam")
	_press("spawn:world_spawn")
	var reached: bool = await _wait_until(func(): return _screen() == "drive", 300.0)
	if not _check(reached, "real: the real world loads and the flow reaches drive (screen '%s', state '%s')" % [_screen(), main.world_state]):
		return
	_check(main.world_kind == "real_world" and main.world_state == "running", "real: world_kind real_world, running")
	_check(sim.is_terrain_mode(), "real: the session is in terrain mode")
	var steps0: int = sim.get_step_count()
	await _wait_seconds(1.0)
	_check(sim.get_step_count() > steps0, "real: the sim ticks (%d -> %d)" % [steps0, sim.get_step_count()])
	var pose: Dictionary = sim.get_chassis_session_pose()
	print("RG_SHELL_TEST real spawn pose x=%.1f y=%.1f z=%.2f yaw=%.1f" % [pose["x"], pose["y"], pose["z"], pose["yaw_deg"]])
	_press_escape()
	await _wait_until(func(): return _screen() == "pause", 5.0)
	_press("main_menu")
	_check(_screen() == "main_menu" and main.world_state == "none" and not sim.is_running(), "real: Main menu unloads the real world")
	await _wait_seconds(0.5)
	_check(main._roads == null and main._buildings == null, "real: the road and building streams are gone")
	_check(_count_nodes(main) == node_count_before, "real: node count after the real world equals the start (%d vs %d)" % [_count_nodes(main), node_count_before])

func _run() -> void:
	var real: bool = "--shell-real" in OS.get_cmdline_user_args()
	var guard := get_tree().create_timer(TIMEOUT_REAL_S if real else TIMEOUT_S)
	guard.timeout.connect(func():
		print("RG_SHELL_TEST FAIL: timeout on screen '%s'" % main.shell_screen())
		get_tree().quit(1))

	var shell: Node = main.get_shell()
	var ui: CanvasLayer = main.get_shell_ui()

	# ---- boot ----
	_check(shell != null and ui != null, "RgShell and the shell UI exist")
	_check(_screen() == "boot", "the first screen is the boot splash (is '%s')" % _screen())
	var boot_texts: Array = []
	_label_texts(ui, boot_texts)
	var attribution: String = str(shell.get_attribution_line())
	_check(attribution != "" and boot_texts.has(attribution), "the boot splash carries the attribution line")
	_check(await _wait_until(func(): return _screen() == "main_menu", 10.0), "the boot splash ends by itself and the main menu follows")

	# ---- main menu ----
	var expected_main: Array = EXPECTED_MAIN_ITEMS.duplicate()
	expected_main.sort()
	_check(_sorted_ids(ui) == expected_main, "main menu is exactly Free roam | Settings | Credits | Quit (has %s)" % ", ".join(ui.button_ids()))
	_check(not main._hud.visible, "no HUD on the main menu")
	_check(main.world_state == "none" or main.world_state == "", "no world is loaded on the main menu (state '%s')" % main.world_state)

	# ---- settings: change, save by leaving, reload ----
	_press("settings")
	_check(_screen() == "settings", "Settings opens")
	var fov_control: Control = ui.get_setting_control("camera.fov_deg")
	if _check(fov_control is HSlider, "the field-of-view setting is a slider"):
		(fov_control as HSlider).value = 88.0 # value_changed -> RgShell.set_setting
	_check(is_equal_approx(float(shell.get_setting("camera.fov_deg")), 88.0), "the slider change reached the settings (camera.fov_deg = %s)" % shell.get_setting("camera.fov_deg"))
	var volume_control: Control = ui.get_setting_control("audio.master_volume")
	if _check(volume_control is HSlider, "the master volume setting is a slider"):
		(volume_control as HSlider).value = 0.5
	_check(shell.settings_dirty(), "the settings are marked changed before leaving the screen")
	_press("back")
	_check(_screen() == "main_menu", "Back returns to the main menu")
	_check(not shell.settings_dirty(), "leaving the settings screen saved them")
	var settings_path: String = str(shell.get_settings_path())
	_check(FileAccess.file_exists(settings_path), "the settings file exists (%s)" % settings_path)
	var reload: Node = ClassDB.instantiate("RgShell")
	reload.initialize(main._rg_data_path(""), main._user_dir(), main._world_config_path())
	_check(is_equal_approx(float(reload.get_setting("camera.fov_deg")), 88.0), "a fresh RgShell reads the saved field of view back (%s)" % reload.get_setting("camera.fov_deg"))
	_check(is_equal_approx(float(reload.get_setting("audio.master_volume")), 0.5), "... and the master volume (%s)" % reload.get_setting("audio.master_volume"))
	reload.free()

	# ---- credits ----
	_press("credits")
	_check(_screen() == "credits", "Credits opens")
	var credits_texts: Array = []
	_label_texts(ui, credits_texts)
	_check(int(shell.get_credits_entry_count()) > 0 and credits_texts.size() > 5, "the credits screen lists the entries (%d entries, %d labels)" % [shell.get_credits_entry_count(), credits_texts.size()])
	_press("back")
	_check(_screen() == "main_menu", "Back returns from Credits to the main menu")

	# ---- free roam, round 1 ----
	var node_count_before: int = _count_nodes(main)
	if await _start_flat_world("round 1"):
		# pause with a real Esc key event
		_press_escape()
		_check(await _wait_until(func(): return _screen() == "pause", 5.0), "Esc pauses (screen '%s')" % _screen())
		var pause_ids := _sorted_ids(ui)
		var expected_pause: Array = EXPECTED_PAUSE_ITEMS.duplicate()
		expected_pause.sort()
		_check(pause_ids == expected_pause, "pause menu is exactly Resume | Reset car | Settings | Main menu (has %s)" % ", ".join(pause_ids))
		var sim: Node = main._simulation
		_check(sim.is_paused(), "the simulation is paused")
		# the pause takes effect on the sim thread: let a tick in flight finish first
		await _wait_seconds(0.3)
		var paused_steps: int = sim.get_step_count()
		await _wait_seconds(0.4)
		_check(sim.get_step_count() == paused_steps, "no ticks run while paused (%d -> %d)" % [paused_steps, sim.get_step_count()])
		_press("resume")
		_check(_screen() == "drive" and not sim.is_paused(), "Resume returns to drive and runs the sim")
		await _wait_seconds(0.3)
		_check(sim.get_step_count() > paused_steps, "ticks advance again after Resume")
		# pause -> settings -> back returns to the pause menu
		_press_escape()
		await _wait_until(func(): return _screen() == "pause", 5.0)
		_press("settings")
		_check(_screen() == "settings", "Settings opens from the pause menu")
		_press("back")
		_check(_screen() == "pause", "Back from Settings returns to the pause menu (is '%s')" % _screen())
		_press("reset_car")
		_check(_screen() == "drive" and not sim.is_paused(), "Reset car resumes the drive")
		# pause -> main menu: the world goes away
		_press_escape()
		await _wait_until(func(): return _screen() == "pause", 5.0)
		_press("main_menu")
		_check(_screen() == "main_menu", "Main menu is reached from the pause menu")
		_check(main.world_state == "none", "the world state is none after leaving (is '%s')" % main.world_state)
		_check(not sim.is_running(), "the sim thread is stopped")
		_check(not sim.is_paused(), "the pause flag is cleared with the world")
		_check(not main._hud.visible, "the HUD is hidden again")
		await _wait_seconds(0.2)
		_check(_count_nodes(main) == node_count_before, "no node is left behind after the world is gone (%d before, %d after)" % [node_count_before, _count_nodes(main)])

	# ---- free roam, round 2: the same process again ----
	if _screen() == "main_menu" and await _start_flat_world("round 2"):
		_press_escape()
		await _wait_until(func(): return _screen() == "pause", 5.0)
		_press("main_menu")
		_check(_screen() == "main_menu" and not main._simulation.is_running(), "round 2: back at the main menu with the sim stopped")
		await _wait_seconds(0.2)
		_check(_count_nodes(main) == node_count_before, "round 2: node count equals the start (%d vs %d)" % [_count_nodes(main), node_count_before])

	# ---- free roam, round 3: the real world through the menu (--shell-real) ----
	if real and _screen() == "main_menu":
		await _real_world_round(node_count_before)

	# ---- quit ----
	if _screen() == "main_menu":
		_check(_press("quit"), "Quit is pressed")
		_check(shell.get_screen() == "quit", "the flow reaches the quit screen")
	var expected_log := ["boot>main_menu", "main_menu>settings", "settings>main_menu", "main_menu>credits", "credits>main_menu", "main_menu>spawn_picker"]
	var transitions: PackedStringArray = main.shell_log
	var prefix_ok := transitions.size() >= expected_log.size()
	for i in range(mini(expected_log.size(), transitions.size())):
		if transitions[i] != expected_log[i]:
			prefix_ok = false
	_check(prefix_ok, "the transition log starts as expected (%s)" % ", ".join(transitions))
	_check(transitions.size() > 0 and transitions[transitions.size() - 1] == "main_menu>quit", "the log ends with main_menu>quit")
	_finish()
