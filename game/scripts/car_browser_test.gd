extends Node
# game/scripts/car_browser_test.gd - `--car-browser-test` (tools/smoke_test.ps1 -CarBrowser):
# the headless acceptance flow of the car browser and the in-world car switch (PLAN.md R6c).
# It drives the real screens like a player (Button.pressed.emit(), injected key events) and
# asserts what rg::CarBrowser / main.gd did:
#
#   main menu -> Garage -> the browser lists every catalog car, the grid has 2-4 rows, the arrow
#   keys move the focus, grouping by body type yields several category headers, the AWD filter
#   leaves exactly the AWD cars (checked against the stats, not against the browser's own list)
#   and persists in the settings, F opens the filter panel and Esc closes it before leaving ->
#   Free roam (flat world) -> the car is moved away from the spawn -> Esc -> Change car -> the
#   browser -> another car (a preset) -> the new car stands where the old one stood (within
#   0.5 m, physics read back), is at rest, carries its own physics file and its weight on the
#   wheels -> Main menu: the node count is what it was before the browser opened.
#
# With `--car-browser-big` (a synthetic catalog of hundreds of cars, --catalog) it also asserts
# that only a window of the tiles exists as nodes while every car is listed.
#
# Every check prints "RG_CAR_BROWSER_TEST ok: ..." or "... FAIL: ..."; the last line is
# "RG_CAR_BROWSER_TEST PASS checks=N" or "... FAIL failures=N". Run with `--shell-user-dir <dir>`.

const TIMEOUT_S := 420.0
const WORLD_LOAD_S := 240.0

var main: Node
var big := false
var _checks := 0
var _failures := 0

func _ready() -> void:
	_run.call_deferred()

func _check(condition: bool, what: String) -> bool:
	_checks += 1
	if condition:
		print("RG_CAR_BROWSER_TEST ok: %s" % what)
	else:
		_failures += 1
		print("RG_CAR_BROWSER_TEST FAIL: %s" % what)
	return condition

func _finish() -> void:
	if _failures == 0:
		print("RG_CAR_BROWSER_TEST PASS checks=%d" % _checks)
	else:
		print("RG_CAR_BROWSER_TEST FAIL failures=%d checks=%d" % [_failures, _checks])
		get_tree().quit(1)

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

func _key(keycode: Key) -> void:
	for pressed in [true, false]:
		var ev := InputEventKey.new()
		ev.keycode = keycode
		ev.physical_keycode = keycode
		ev.pressed = pressed
		Input.parse_input_event(ev)

func _count_nodes(root: Node) -> int:
	var n := 1
	for child in root.get_children():
		n += _count_nodes(child)
	return n

func _wheel_load_sum() -> float:
	var sim: Node = main._simulation
	var total := 0.0
	for i in range(int(sim.get_vehicle_wheel_count(main.VEHICLE_NAME))):
		total += float(sim.get_wheel_load_n(main.VEHICLE_NAME, i))
	return total

func _run() -> void:
	var guard := get_tree().create_timer(TIMEOUT_S)
	guard.timeout.connect(func():
		print("RG_CAR_BROWSER_TEST FAIL: timeout on screen '%s'" % main.shell_screen())
		get_tree().quit(1))

	var ui: CanvasLayer = main.get_shell_ui()
	var garage: Node = main.get_garage()
	var sim: Node = main._simulation
	_check(await _wait_until(func(): return _screen() == "main_menu", 10.0), "the boot splash ends and the main menu follows")
	await _wait_seconds(0.3)
	var nodes_before: int = _count_nodes(main)
	var all_cars: Array = garage.get_vehicles()
	_check(all_cars.size() >= 8, "the catalog lists %d cars (bases and presets)" % all_cars.size())

	# ---- the browser from the main menu ----
	_press("garage")
	_check(_screen() == "vehicle_select", "Garage opens the car browser (screen '%s')" % _screen())
	await _wait_seconds(0.5)
	var browser: Control = ui.browser()
	if not _check(browser != null, "the browser exists"):
		_finish()
		return
	var listed: PackedStringArray = browser.listed_ids()
	_check(listed.size() == all_cars.size(), "every catalog car is listed (%d of %d)" % [listed.size(), all_cars.size()])
	_check(browser.rows() >= 2 and browser.rows() <= 4, "the grid has 2-4 rows (%d)" % browser.rows())
	_check(browser.instantiated_tile_count() > 0 and browser.instantiated_tile_count() <= listed.size(), "tiles exist for the visible part (%d)" % browser.instantiated_tile_count())
	if big:
		_check(listed.size() >= 150, "the synthetic catalog has hundreds of cars (%d)" % listed.size())
		_check(browser.instantiated_tile_count() < 70, "only a window of the %d tiles exists as nodes (%d)" % [listed.size(), browser.instantiated_tile_count()])
	var first_focus: String = browser.focused_id()
	_check(first_focus != "", "a car is focused (%s)" % first_focus)

	# arrow keys move the focus
	var moved := false
	for k in [KEY_RIGHT, KEY_DOWN, KEY_LEFT, KEY_UP]:
		var before: String = browser.focused_id()
		_key(k)
		await get_tree().process_frame
		if browser.focused_id() != before:
			moved = true
	_check(moved, "the arrow keys move the focus (now %s)" % browser.focused_id())
	var cell_before: Dictionary = garage.browser_get_focus_cell()
	_key(KEY_PAGEDOWN)
	await get_tree().process_frame
	_check(garage.browser_get_focus_cell() != cell_before or listed.size() < 2, "Page Down jumps to another category or column")
	if big:
		# the focus can reach far cars while the node count stays small
		for i in range(60):
			_key(KEY_RIGHT)
		await _wait_seconds(0.3)
		_check(browser.instantiated_tile_count() < 70, "after scrolling far right still a window of tiles (%d)" % browser.instantiated_tile_count())

	# grouping by body type gives several categories
	browser.set_group_by("body_type")
	var view: Dictionary = garage.browser_get_view(60, 0)
	var headers: Array = view.get("headers", [])
	_check(headers.size() >= 2, "grouping by body type shows %d category headers" % headers.size())
	browser.set_group_by("drive_layout")

	# the AWD filter leaves exactly the AWD cars
	var expected_awd: Array = []
	for v in all_cars:
		var st: Dictionary = v["stats"]
		if bool(st.get("ok", false)) and str(st["layout"]) == "AWD":
			expected_awd.append(str(v["id"]))
	browser.toggle_filter("layout", "AWD")
	var after_filter: PackedStringArray = browser.listed_ids()
	var got_awd: Array = Array(after_filter)
	got_awd.sort()
	expected_awd.sort()
	_check(expected_awd.size() > 0 and got_awd == expected_awd, "the AWD filter leaves exactly the AWD cars (%s)" % str(got_awd))
	var visible_ok := true
	for id in browser._tiles.keys():
		if not got_awd.has(id):
			visible_ok = false
	_check(visible_ok, "no tile of a filtered-out car is shown")
	_check(str(main._shell.get_setting("browser.filter_layouts")) == "AWD", "the filter is stored in the settings (%s)" % main._shell.get_setting("browser.filter_layouts"))
	browser.toggle_filter("layout", "AWD")
	_check(browser.listed_ids().size() == all_cars.size(), "removing the filter lists every car again")

	# the filter panel: F opens it, Esc closes it first, a second Esc leaves
	_key(KEY_F)
	await get_tree().process_frame
	_check(browser.panel_open(), "F opens the filter panel")
	_check(browser.get_control("group:body_type") != null and browser.get_control("sort:power") != null, "the panel offers group and sort choices")
	_key(KEY_ESCAPE)
	await get_tree().process_frame
	_check(not browser.panel_open() and _screen() == "vehicle_select", "Esc closes the panel and stays in the browser (screen '%s')" % _screen())
	_key(KEY_ESCAPE)
	await get_tree().process_frame
	_check(_screen() == "main_menu", "a second Esc returns to the main menu (screen '%s')" % _screen())
	await _wait_seconds(0.3)
	_check(_count_nodes(main) == nodes_before, "node count back after the browser (%d vs %d)" % [_count_nodes(main), nodes_before])

	# ---- in-world car switch ----
	_press("free_roam")
	_press("spawn:flat")
	var reached: bool = await _wait_until(func(): return _screen() == "drive", WORLD_LOAD_S)
	if not _check(reached, "the flat world loads (screen '%s')" % _screen()):
		_finish()
		return
	await _wait_seconds(2.0)
	var old_name: String = main.VEHICLE_NAME
	var old_vehicle: String = str(main.get_drive_selection()["vehicle_id"])
	sim.relocate_vehicle(31.0, 12.0, 40.0)
	await _wait_seconds(2.5)
	var pose_before: Dictionary = sim.get_chassis_session_pose()
	print("RG_CAR_BROWSER_TEST pose before x=%.2f y=%.2f yaw=%.1f" % [pose_before["x"], pose_before["y"], pose_before["yaw_deg"]])
	_check(absf(float(pose_before["x"]) - 31.0) < 1.5, "the car was moved away from the spawn (x %.2f)" % float(pose_before["x"]))

	_key(KEY_ESCAPE)
	_check(await _wait_until(func(): return _screen() == "pause", 5.0), "Esc pauses")
	_check(ui.button_ids().has("change_car"), "the pause menu offers Change car (has %s)" % ", ".join(ui.button_ids()))
	_check(ui.button_ids().has("garage"), "the pause menu keeps Garage (respawn)")
	_press("change_car")
	_check(_screen() == "change_car", "Change car opens the browser (screen '%s')" % _screen())
	await _wait_seconds(0.4)
	var pick := "car_sedan_awd_rally"
	if garage.get_vehicle(pick).is_empty():
		for v in garage.get_vehicles():
			if str(v["id"]) != old_vehicle and str(v["id"]) != "car_hyper":
				pick = str(v["id"])
				break
	_check(ui.browser().focus_car(pick), "the picked car (%s) can be focused" % pick)
	_press("choose")
	var back_in_world: bool = await _wait_until(func(): return _screen() == "drive" and main.world_state == "running", WORLD_LOAD_S)
	if not _check(back_in_world, "the world runs again with the new car (screen '%s', state '%s')" % [_screen(), main.world_state]):
		_finish()
		return
	await _wait_seconds(2.0)
	var sel: Dictionary = main.get_drive_selection()
	_check(str(sel["vehicle_id"]) == pick, "the drive selection is the picked car (%s)" % sel["vehicle_id"])
	_check(main.VEHICLE_NAME == str(garage.get_vehicle(pick)["sim_name"]), "the simulation carries the new car (%s, was %s)" % [main.VEHICLE_NAME, old_name])
	var pose_after: Dictionary = sim.get_chassis_session_pose()
	print("RG_CAR_BROWSER_TEST pose after x=%.2f y=%.2f yaw=%.1f" % [pose_after["x"], pose_after["y"], pose_after["yaw_deg"]])
	var dist := Vector2(float(pose_after["x"]) - float(pose_before["x"]), float(pose_after["y"]) - float(pose_before["y"])).length()
	_check(dist < 0.5, "the new car stands where the old one stood (%.3f m apart)" % dist)
	var yaw_diff := fmod(absf(float(pose_after["yaw_deg"]) - float(pose_before["yaw_deg"])) + 180.0, 360.0) - 180.0
	_check(absf(yaw_diff) < 3.0, "... with the same heading (%.2f deg apart)" % yaw_diff)
	_check(sim.get_body_speed_mps("chassis") < 1.0, "the new car is at rest (%.2f m/s)" % sim.get_body_speed_mps("chassis"))
	var mass: float = float(sel["chassis"]["mass_kg"])
	var load_n := _wheel_load_sum()
	_check(absf(load_n - mass * 9.81) < 0.15 * mass * 9.81, "physics: the wheels carry the new car's weight (%.0f N vs %.0f N)" % [load_n, mass * 9.81])
	_check(int(sim.get_vehicle_wheel_count(main.VEHICLE_NAME)) == 4, "physics: four wheels")
	var steps0: int = sim.get_step_count()
	await _wait_seconds(0.5)
	_check(sim.get_step_count() > steps0, "the sim ticks")
	_check(main.get_garage_scene() == null and main.get_thumbnails() == null, "no garage scene or thumbnail renderer while driving")

	# ---- leaving: nothing left behind ----
	_key(KEY_ESCAPE)
	await _wait_until(func(): return _screen() == "pause", 5.0)
	_press("main_menu")
	await _wait_seconds(0.5)
	_check(_screen() == "main_menu" and main.world_state == "none", "Main menu: no world")
	_check(_count_nodes(main) == nodes_before, "no node is left behind (%d before, %d after)" % [nodes_before, _count_nodes(main)])
	_check(garage.get_work_file_count() == 0, "no materialised work file is left (%d)" % garage.get_work_file_count())
	_finish()
	get_tree().quit()
