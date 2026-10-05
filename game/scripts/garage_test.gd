extends Node
# game/scripts/garage_test.gd - `--garage-test` (tools/smoke_test.ps1 -Garage):
# the headless acceptance flow of the garage (PLAN.md R6). It drives the real
# screens like a player (Button.pressed.emit(), real slider controls, an injected
# Esc key) and asserts what rg::Garage / main.gd did:
#
#   main menu -> Free roam -> flat world with the STOCK hyper car: the wheel
#   suspension compression is read from the physics -> Esc -> Main menu ->
#   Garage -> vehicle select (cars, stats) -> Configure -> Wheels tab: the front
#   springs slider to its maximum -> an INVALID gear-ratio set (non-monotonic) is
#   rejected with the loader's message, Save/Drive are disabled, an out-of-range
#   value is rejected too -> back to valid -> Save -> a second RgGarage reads the
#   saved setup -> Drive -> spawn picker -> flat world: the same wheel readings
#   show the stiffer front springs (a physics value, not the UI) -> Esc ->
#   Garage (respawn) -> the sedan -> Drive -> the sedan runs -> Main menu: no
#   world, no garage node, no materialised work file left.
#
# Every check prints "RG_GARAGE_TEST ok: ..." or "RG_GARAGE_TEST FAIL: ..."; the
# last line is "RG_GARAGE_TEST PASS checks=N" or "RG_GARAGE_TEST FAIL failures=N".
# Run with `--shell-user-dir <dir>` so settings and setups never touch user://.

const TIMEOUT_S := 420.0
const WORLD_LOAD_S := 240.0

var main: Node
var _checks := 0
var _failures := 0

func _ready() -> void:
	_run.call_deferred()

func _check(condition: bool, what: String) -> bool:
	_checks += 1
	if condition:
		print("RG_GARAGE_TEST ok: %s" % what)
	else:
		_failures += 1
		print("RG_GARAGE_TEST FAIL: %s" % what)
	return condition

func _finish() -> void:
	if _failures == 0:
		print("RG_GARAGE_TEST PASS checks=%d" % _checks)
	else:
		print("RG_GARAGE_TEST FAIL failures=%d checks=%d" % [_failures, _checks])
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

func _press_escape() -> void:
	for pressed in [true, false]:
		var ev := InputEventKey.new()
		ev.keycode = KEY_ESCAPE
		ev.physical_keycode = KEY_ESCAPE
		ev.pressed = pressed
		Input.parse_input_event(ev)

func _count_nodes(root: Node) -> int:
	var n := 1
	for child in root.get_children():
		n += _count_nodes(child)
	return n

# Mean suspension compression of the front / rear wheels from the physics snapshot.
func _compression() -> Dictionary:
	var sim: Node = main._simulation
	var name: String = main.VEHICLE_NAME
	var front := 0.0
	var rear := 0.0
	var nf := 0
	var nr := 0
	for i in range(int(sim.get_vehicle_wheel_count(name))):
		var c: float = sim.get_wheel_compression(name, i)
		if sim.get_wheel_is_front(name, i):
			front += c
			nf += 1
		else:
			rear += c
			nr += 1
	return {"front": front / maxf(nf, 1), "rear": rear / maxf(nr, 1)}

# Free roam -> flat world -> drive; `settle_s` lets the car come to rest.
func _drive_flat(round_name: String, settle_s: float) -> bool:
	if not _press("free_roam"):
		return false
	if not _press("spawn:flat"):
		return false
	var reached: bool = await _wait_until(func(): return _screen() == "drive", WORLD_LOAD_S)
	if not _check(reached, "%s: the flat world loads and the flow reaches drive (screen '%s')" % [round_name, _screen()]):
		return false
	return await _after_world_load(round_name, settle_s)

func _after_world_load(round_name: String, settle_s: float) -> bool:
	var sim: Node = main._simulation
	_check(main.world_state == "running" and sim.is_running(), "%s: the world runs" % round_name)
	var steps0: int = sim.get_step_count()
	await _wait_seconds(settle_s)
	_check(sim.get_step_count() > steps0, "%s: the sim ticks (%d -> %d)" % [round_name, steps0, sim.get_step_count()])
	return true

func _pause_to_main_menu() -> void:
	_press_escape()
	await _wait_until(func(): return _screen() == "pause", 5.0)
	_press("main_menu")
	_check(_screen() == "main_menu" and main.world_state == "none", "Main menu from the pause menu: no world (screen '%s', state '%s')" % [_screen(), main.world_state])

func _set_slider(id: String, value: float) -> void:
	var control: Control = main.get_shell_ui().get_option_control(id)
	if _check(control is HSlider, "option control '%s' is a slider" % id):
		(control as HSlider).value = value # value_changed -> RgGarage.set_option

func _run() -> void:
	var guard := get_tree().create_timer(TIMEOUT_S)
	guard.timeout.connect(func():
		print("RG_GARAGE_TEST FAIL: timeout on screen '%s'" % main.shell_screen())
		get_tree().quit(1))

	var ui: CanvasLayer = main.get_shell_ui()
	var garage: Node = main.get_garage()
	var sim: Node = main._simulation
	_check(garage != null, "RgGarage exists")
	_check(await _wait_until(func(): return _screen() == "main_menu", 10.0), "the boot splash ends and the main menu follows")
	_check(ui.button_ids().has("garage"), "the main menu offers Garage (has %s)" % ", ".join(ui.button_ids()))
	await _wait_seconds(0.3)
	var nodes_before: int = _count_nodes(main)
	var vehicles: Array = garage.get_vehicles()
	_check(vehicles.size() >= 2, "the catalog lists %d cars" % vehicles.size())
	var hyper_id := "car_hyper"
	var sedan_id := "car_sedan"
	_check(str(garage.get_selected_id()) == hyper_id, "the saved choice is the catalog default (%s)" % garage.get_selected_id())
	_check(garage.get_work_file_count() == 0, "no work file exists at the start")

	# ---- stock drive: the reference physics reading ----
	if not await _drive_flat("stock", 4.0):
		_finish()
		return
	_check(main.VEHICLE_NAME == str(garage.get_vehicle(hyper_id)["sim_name"]), "stock: the hyper car drives (%s)" % main.VEHICLE_NAME)
	_check(not bool(main.get_drive_selection()["modified"]), "stock: the car carries no setup")
	var stock: Dictionary = _compression()
	print("RG_GARAGE_TEST stock compression front=%.5f rear=%.5f" % [stock["front"], stock["rear"]])
	_check(float(stock["front"]) > 0.0005 and float(stock["rear"]) > 0.0005, "stock: the suspension carries the car (front %.5f m, rear %.5f m)" % [stock["front"], stock["rear"]])
	await _pause_to_main_menu()
	await _wait_seconds(0.3)
	_check(_count_nodes(main) == nodes_before, "stock: node count back at the start (%d vs %d)" % [_count_nodes(main), nodes_before])

	# ---- main menu -> Garage -> vehicle select ----
	_press("garage")
	_check(_screen() == "vehicle_select", "Garage opens the vehicle select (is '%s')" % _screen())
	_check(main.get_garage_scene() != null, "the garage scene exists")
	await _wait_seconds(0.5)
	for v in vehicles:
		_check(ui.get_button("veh:%s" % v["id"]) != null, "vehicle select lists %s" % v["id"])
	var stats: Dictionary = garage.get_vehicle(hyper_id)["stats"]
	_check(bool(stats["ok"]) and float(stats["mass_kg"]) > 500.0 and float(stats["peak_power_kw"]) > 100.0 and int(stats["gear_count"]) >= 5, "the stats come from the data (power %.0f kW, mass %.0f kg, %d gears)" % [stats["peak_power_kw"], stats["mass_kg"], stats["gear_count"]])
	_press("veh:%s" % sedan_id)
	_check(ui.highlighted_vehicle() == sedan_id, "highlighting the sedan shows it (%s)" % ui.highlighted_vehicle())
	_press("veh:%s" % hyper_id)
	_check(ui.highlighted_vehicle() == hyper_id, "highlighting the hyper car shows it (%s)" % ui.highlighted_vehicle())
	_press("choose")
	_check(_screen() == "configurator", "Configure opens the configurator (is '%s')" % _screen())
	_check(str(garage.get_edit_id()) == hyper_id, "the edit session is the hyper car (%s)" % garage.get_edit_id())
	_check(not garage.is_dirty(), "a fresh edit session is clean")
	var save_button: Button = ui.get_button("save")
	_check(save_button != null and save_button.disabled, "Save is disabled while nothing changed")

	# ---- a valid change through the real control ----
	_press("area:wheels")
	await _wait_seconds(0.3)
	_check(ui.active_area() == "wheels", "the Wheels tab is active")
	_set_slider("spring_front", 1.3)
	_check(garage.is_dirty(), "moving the front spring slider marks the setup changed")
	_check(bool(garage.get_validation()["ok"]), "the change passes the physics loader (%s)" % garage.get_validation()["message"])
	_check(not save_button.disabled, "Save is enabled for a valid change")

	# ---- an invalid change is rejected with the loader's message ----
	_press("area:engine")
	await _wait_seconds(0.3)
	_set_slider("gear_ratios:0", 0.8)
	_check(bool(garage.get_validation()["ok"]), "a lower first gear alone is still valid")
	_set_slider("gear_ratios:1", 1.2)
	var verdict: Dictionary = garage.get_validation()
	_check(not bool(verdict["ok"]) and str(verdict["message"]) != "", "a non-monotonic gear set is rejected (message: %s)" % verdict["message"])
	_check(ui.status_text().begins_with("Cannot save"), "the configurator shows the rejection ('%s')" % ui.status_text())
	_check(ui.get_button("save").disabled and ui.get_button("drive").disabled, "Save and Drive are disabled for an invalid setup")
	var saved_bad: Dictionary = garage.save()
	_check(not bool(saved_bad["ok"]), "saving the invalid setup fails (%s)" % saved_bad.get("error", ""))
	# an out-of-range value is rejected too (not clamped silently)
	_set_slider("gear_ratios:1", 1.0)
	_set_slider("gear_ratios:0", 1.0)
	_check(bool(garage.get_validation()["ok"]), "putting the gears back makes the setup valid again")
	var out_of_range: Dictionary = garage.set_option("final_drive", 3.0)
	_check(not bool(out_of_range["ok"]) and str(out_of_range["validation_message"]) != "", "final drive x3.0 is outside the whitelisted range and rejected (%s)" % out_of_range["validation_message"])
	garage.set_option("final_drive", 1.0)
	_check(bool(garage.get_validation()["ok"]), "final drive back to stock is valid")
	await _wait_seconds(0.1)

	# ---- save, persistence ----
	_check(_press("save"), "Save is pressed")
	_check(not garage.is_dirty() and bool(garage.get_vehicle(hyper_id)["has_setup"]), "the setup is saved")
	var reload: Node = ClassDB.instantiate("RgGarage")
	var repo: String = ProjectSettings.globalize_path("res://").path_join("..").simplify_path()
	var report: Dictionary = reload.initialize(repo, main._user_dir(), main._user_dir().path_join("garage_work_check"))
	_check(bool(report.get("ok", false)), "a second RgGarage initialises (%s)" % report.get("error", ""))
	_check(bool(reload.get_vehicle(hyper_id)["has_setup"]), "a fresh RgGarage finds the saved setup of the hyper car")
	var begun: Dictionary = reload.begin_edit(hyper_id)
	var spring: Variant = reload.get_options()
	var found_spring := false
	for opt in spring:
		if str(opt["id"]) == "spring_front":
			found_spring = true
			_check(is_equal_approx(float(opt["value"]), 1.3), "... and reads the front springs back as x1.3 (%s)" % opt["value"])
	_check(found_spring, "the reloaded edit session lists the front spring option (begin_edit ok=%s)" % begun.get("ok", false))
	reload.free()

	# ---- drive the configured car ----
	_press("drive")
	_check(_screen() == "spawn_picker", "Drive opens the spawn picker like Free roam (is '%s')" % _screen())
	_check(main.get_garage_scene() == null, "the garage scene is gone once the drive starts")
	_press("spawn:flat")
	var reached: bool = await _wait_until(func(): return _screen() == "drive", WORLD_LOAD_S)
	_check(reached, "the configured car's world loads (screen '%s')" % _screen())
	if reached and await _after_world_load("configured", 4.0):
		var sel: Dictionary = main.get_drive_selection()
		_check(bool(sel["modified"]), "configured: the drive carries a setup (%s)" % sel["vehicle_path"])
		_check(garage.get_work_file_count() > 0, "configured: the setup was materialised (%d work files)" % garage.get_work_file_count())
		var tuned: Dictionary = _compression()
		print("RG_GARAGE_TEST configured compression front=%.5f rear=%.5f" % [tuned["front"], tuned["rear"]])
		var front_ratio: float = float(tuned["front"]) / float(stock["front"])
		var rear_ratio: float = float(tuned["rear"]) / float(stock["rear"])
		_check(front_ratio < 0.9, "physics: the stiffer front springs compress less (front %.3f x stock)" % front_ratio)
		_check(absf(rear_ratio - 1.0) < 0.06, "physics: the rear axle is unchanged (rear %.3f x stock)" % rear_ratio)

		# ---- pause -> Garage (respawn) -> another car ----
		_press_escape()
		_check(await _wait_until(func(): return _screen() == "pause", 5.0), "Esc pauses")
		_check(ui.button_ids().has("garage"), "the pause menu offers Garage (respawn)")
		_press("garage")
		_check(_screen() == "vehicle_select" and main.get_garage_scene() != null, "pause -> Garage opens the vehicle select (is '%s')" % _screen())
		await _wait_seconds(0.5)
		_press("veh:%s" % sedan_id)
		_press("choose")
		_check(_screen() == "configurator" and str(garage.get_edit_id()) == sedan_id, "the sedan's configurator opens (is '%s')" % _screen())
		_press("drive")
		var respawned: bool = await _wait_until(func(): return _screen() == "drive" and main.world_state == "running", WORLD_LOAD_S)
		_check(respawned, "the respawn with the sedan reaches drive (screen '%s', state '%s')" % [_screen(), main.world_state])
		if respawned:
			var sedan_sim: String = str(garage.get_vehicle(sedan_id)["sim_name"])
			_check(main.VEHICLE_NAME == sedan_sim and sim.is_running(), "the sedan drives now (%s)" % main.VEHICLE_NAME)
			_check(not bool(main.get_drive_selection()["modified"]), "the sedan runs stock (no setup saved for it)")
			_check(main.get_garage_scene() == null, "no garage scene while driving")
			await _wait_seconds(1.0)
			var steps0: int = sim.get_step_count()
			await _wait_seconds(0.5)
			_check(sim.get_step_count() > steps0, "the respawned car's sim ticks")
			await _pause_to_main_menu()

	# ---- everything released ----
	await _wait_seconds(0.4)
	_check(not sim.is_running(), "the sim is stopped at the main menu")
	_check(garage.get_work_file_count() == 0, "no materialised work file is left (%d)" % garage.get_work_file_count())
	_check(_count_nodes(main) == nodes_before, "no node is left behind after garage and drives (%d before, %d after)" % [nodes_before, _count_nodes(main)])
	_press("garage")
	await _wait_seconds(0.4)
	_check(main.get_garage_scene() != null and _count_nodes(main) > nodes_before, "the garage scene adds nodes while open (%d)" % _count_nodes(main))
	_press("back")
	_check(_screen() == "main_menu", "Back from the vehicle select returns to the main menu (is '%s')" % _screen())
	await _wait_seconds(0.4)
	_check(main.get_garage_scene() == null, "the garage scene is removed")
	_check(_count_nodes(main) == nodes_before, "no garage node is left after Back (%d before, %d after)" % [nodes_before, _count_nodes(main)])
	_finish()
