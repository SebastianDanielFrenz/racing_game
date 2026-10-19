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

# The working value of a garage option, failing fast and loudly when the option is not offered (an empty
# Dictionary from ShellUI._garage_option would otherwise raise a script error and leave the test to time out).
func _installed_option_value(id: String) -> String:
	var opt: Dictionary = main.get_shell_ui()._garage_option(id)
	if not _check(not opt.is_empty(), "garage option '%s' is available (unavailable or unknown options come back empty)" % id):
		return "<option '%s' unavailable>" % id
	return str(opt.get("value", "<no value>"))

func _press_escape() -> void:
	for pressed in [true, false]:
		var ev := InputEventKey.new()
		ev.keycode = KEY_ESCAPE
		ev.physical_keycode = KEY_ESCAPE
		ev.pressed = pressed
		Input.parse_input_event(ev)
	await _wait_seconds(0.1)

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
	await _press_escape()
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
	var listed: PackedStringArray = ui.browser().listed_ids()
	for v in vehicles:
		_check(listed.has(str(v["id"])), "the car browser lists %s" % v["id"])
	var stats: Dictionary = garage.get_vehicle(hyper_id)["stats"]
	_check(bool(stats["ok"]) and float(stats["mass_kg"]) > 500.0 and float(stats["peak_power_kw"]) > 100.0 and int(stats["gear_count"]) >= 5, "the stats come from the data (power %.0f kW, mass %.0f kg, %d gears)" % [stats["peak_power_kw"], stats["mass_kg"], stats["gear_count"]])
	ui.browser().focus_car(sedan_id)
	_check(ui.highlighted_vehicle() == sedan_id, "highlighting the sedan shows it (%s)" % ui.highlighted_vehicle())
	ui.browser().focus_car(hyper_id)
	_check(ui.highlighted_vehicle() == hyper_id, "highlighting the hyper car shows it (%s)" % ui.highlighted_vehicle())
	_press("choose")
	_check(_screen() == "configurator", "Configure opens the configurator (is '%s')" % _screen())
	_check(str(garage.get_edit_id()) == hyper_id, "the edit session is the hyper car (%s)" % garage.get_edit_id())
	_check(not garage.is_dirty(), "a fresh edit session is clean")
	_check(await _wait_until(func(): return not garage.get_dyno()["busy"], 120.0), "initial real dyno finishes in background")
	var dyno_initial: Dictionary = garage.get_dyno()
	_check(dyno_initial["points"].size() == 33 and str(dyno_initial["error"]).is_empty(), "initial dyno has 33 real engine samples")
	await _wait_seconds(0.2)
	_check(ui._garage_dyno.points.size() == 33, "bottom-left chart receives native dyno points")
	var dyno_revision: int = int(dyno_initial["revision"])
	garage.set_option("brake_force",1.05)
	_check(int(garage.get_dyno()["revision"]) == dyno_revision and not garage.get_dyno()["busy"], "brake change does not recalculate engine dyno")
	garage.reset_all()

	# ---- limit advisor: an extreme boost target is allowed and flagged, never clamped ----
	_check(bool(garage.get_advice().get("available", false)), "the limit advisor is available (%s)" % str(garage.get_advice().get("error", "")))
	_check(await _wait_until(func(): var a: Dictionary = garage.get_advice(); return not a["busy"] and a["have_result"], 180.0), "advisor finishes for the stock car")
	_check(str(garage.get_advice()["error"]).is_empty(), "advisor: stock car check has no error (%s)" % str(garage.get_advice()["error"]))
	var edit: Dictionary = garage.set_option("turbo_boost_target", 8.0)
	_check(bool(edit["accepted"]), "an extreme turbo boost target is accepted, not clamped")
	await _wait_seconds(0.5)
	_check(ui.advice_status_text().contains("checking") or not bool(garage.get_advice()["busy"]), "the setup check shows a checking state (%s)" % ui.advice_status_text())
	_check(await _wait_until(func(): var a: Dictionary = garage.get_advice(); return not a["busy"] and a["have_result"] and not a["items"].is_empty(), 180.0), "advisor reports items for the extreme boost target")
	var boost_item := {}
	for item in garage.get_advice()["items"]:
		if str(item["kind"]) == "BoostBelowTarget" or str(item["kind"]) == "WastegateSaturated" or str(item["kind"]) == "TurboRotorCap":
			boost_item = item
			break
	_check(not boost_item.is_empty() and str(boost_item["title"]) != "" and boost_item["lines"].size() >= 2, "a turbo item with a title and lines: %s" % str(boost_item.get("title", "")))
	print("RG_GARAGE_TEST advice item: ", str(boost_item.get("text", "")).replace("
", " | "))
	await _wait_seconds(0.4)
	_check(ui.advice_item_count() == garage.get_advice()["items"].size(), "the setup screen draws every advice item (%d)" % ui.advice_item_count())
	garage.reset_all()
	await _wait_until(func(): var a: Dictionary = garage.get_advice(); return not a["busy"], 180.0)

	var save_button: Button = ui.get_button("save")
	_check(save_button != null and save_button.disabled, "Save is disabled while nothing changed")
	_check(ui.get_button("back") == null and ui.get_button("drive") == null, "garage has no Back or Drive buttons")
	await _press_escape()
	_check(_screen() == "configurator" and ui.get_button("garage_exit:drive") != null, "Esc at top level opens exit choices")
	var cancel := InputEventJoypadButton.new()
	cancel.button_index = JOY_BUTTON_B
	cancel.pressed = true
	Input.parse_input_event(cancel)
	cancel = cancel.duplicate()
	cancel.pressed = false
	Input.parse_input_event(cancel)
	await _wait_seconds(0.1)
	_check(_screen() == "configurator" and ui.get_button("garage_exit:drive") == null, "controller B cancels the exit prompt")

	# Real category navigation, independent tyre compound/dimension installs.
	_press("tile:Wheels")
	_press("tile:All")
	_press("tile:Compounds")
	for family in ["cord","belted","touring","economy","sport","road","track","slick","drag"]:
		_check(ui.get_button("tyre:"+family) != null, "compound card exists: " + family)
	await _wait_seconds(0.1)
	_check(get_viewport().gui_get_focus_owner() == ui.get_button("tyre:road"), "installed compound has initial focus")
	var before_widths: Vector2 = ui.garage_wheel_widths()
	_press("tyre:economy")
	_check(garage.is_dirty() and bool(garage.get_validation()["ok"]), "All compounds installs both valid tyres")
	_check(ui.garage_wheel_widths() == before_widths, "compound install preserves both axle widths")
	for opt in garage.get_options():
		if str(opt["id"]) in ["tyre_front","tyre_rear"]:
			_check(str(ui._tyre_presentation(str(opt["value"])).get("family")) == "economy", "All applies economy to " + str(opt["id"]))
	await _press_escape()
	_check(ui.get_button("tile:Dimensions") != null and ui.get_button("garage_exit:drive") == null, "B/Esc returns exactly one level")
	_press("tile:Dimensions")
	_check(ui.get_button("tyre:235") == null, "All widths excludes front-only body fitments")
	_press("tyre:305")
	_check(ui.garage_wheel_widths() == Vector2(0.305,0.305), "All dimensions sets both axle widths")
	_press("reset_all")
	_check(not garage.is_dirty(), "reset restores both original axle tyres")
	await _press_escape()
	await _press_escape()
	_press("tile:Front")
	_press("tile:Compounds")
	_press("tyre:track")
	await _back_to_root()
	_press("tile:Wheels")
	_press("tile:All")
	_press("tile:Dimensions")
	_press("tyre:295")
	var families: Dictionary = {}
	for opt in garage.get_options():
		if str(opt["id"]) in ["tyre_front","tyre_rear"]: families[str(opt["id"])] = ui._tyre_presentation(str(opt["value"])).get("family")
	_check(families.get("tyre_front") == "track" and families.get("tyre_rear") == "road", "All dimensions preserves split compounds")
	_press("reset_all")
	await _back_to_root()
	await _check_upgrade_layout()
	_press("tile:Engine")
	_press("tile:Turbo")
	_check(ui.get_button("part:turbo_install:hyper_single") == null, "Engine turbo lists configured dual type only")
	_press("part:turbo_install:hyper_twin_large")
	_check(garage.is_dirty() and bool(garage.get_validation()["ok"]), "dual turbo upgrade validates")
	_press("part:turbo_install:stock")
	_check(not garage.is_dirty(), "stock turbo restores clean configuration")
	await _back_to_root()
	_press("tile:Swaps")
	_press("tile:Forced induction")
	_press("induction:NA")
	_check(ui._garage_path.size() == 2, "NA installs without opening a submenu")
	_check(bool(garage.get_validation()["ok"]), "NA conversion passes native loader")
	_check(_installed_option_value("turbo_install") == "hyper_na", "NA induction persists in working setup")
	_press("induction:Single turbo")
	_check(ui._garage_path.size() == 2, "single turbo installs without opening a submenu")
	_check(bool(garage.get_validation()["ok"]), "single forced-induction swap validates")
	_press("induction:Twin turbo")
	_check(_installed_option_value("turbo_install") == "stock", "twin swap restores original hardware")
	_press("induction:Single turbo")
	await _back_to_root()
	_press("tile:Engine")
	_press("tile:Turbo")
	_check(ui.get_button("part:turbo_install:hyper_single_compact") != null and ui.get_button("part:turbo_install:hyper_twin_large") == null, "turbo upgrades follow installed single type")
	_press("reset_all")
	await _back_to_root()
	_press("tile:Swaps")
	_press("tile:Drivetrain")
	var layout_option: Dictionary = ui._garage_option("drive_layout")
	_check(not layout_option.is_empty(), "drivetrain swap options exist")
	for part in layout_option.get("parts", []):
		_check(ui.get_button("part:drive_layout:" + str(part["id"])) != null, "drivetrain swap card exists: " + str(part["id"]))
	# ---- gearbox swap: the hypercar takes only boxes rated for its V8 ----
	await _back_to_root()
	_press("tile:Drivetrain")
	_press("tile:Gearbox")
	var gb_option: Dictionary = ui._garage_option("gearbox_install")
	_check(not gb_option.is_empty(), "the gearbox option is offered")
	var gb_soon: Array = []
	for section in gb_option.get("coming_soon", []): gb_soon.append(str(section))
	_check(gb_soon.is_empty(), "every gearbox section has entries, none is coming soon (%s)" % ", ".join(gb_soon))
	_check(ui.get_button("part:gearbox_install:stock") != null and ui.get_button("part:gearbox_install:straight_cut_6sq") != null, "the stock box and the straight-cut box are listed for the hypercar")
	_check(ui.get_button("part:gearbox_install:heavy_duty_6mt") == null and ui.get_button("part:gearbox_install:econ_5mt") == null, "boxes rated below the V8's torque are not listed")
	_press("part:gearbox_install:straight_cut_6sq")
	_check(garage.is_dirty() and bool(garage.get_validation()["ok"]), "the straight-cut gearbox validates")
	_check(_installed_option_value("gearbox_install") == "straight_cut_6sq", "the swapped gearbox is the working value")
	_press("part:gearbox_install:stock")
	_check(not garage.is_dirty(), "stock gearbox restores the clean configuration")
	_press("reset_all")
	# ---- a valid change through the real control ----
	_press("garage:tuning")
	_check(ui.get_button("garage:tuning").button_pressed and not ui.get_button("garage:upgrades").button_pressed, "active mode highlight follows Tuning")
	_press("tile:Wheels")
	await _wait_seconds(0.3)
	_check(ui.active_area() == "wheels", "the Wheels tab is active")
	_set_slider("spring_front", 1.3)
	_check(garage.is_dirty(), "moving the front spring slider marks the setup changed")
	_check(bool(garage.get_validation()["ok"]), "the change passes the physics loader (%s)" % garage.get_validation()["message"])
	_check(not save_button.disabled, "Save is enabled for a valid change")

	# ---- an invalid change is rejected with the loader's message ----
	await _back_to_root()
	_press("tile:Engine")
	await _wait_seconds(0.3)
	_set_slider("gear_ratios:0", 0.8)
	_check(bool(garage.get_validation()["ok"]), "a lower first gear alone is still valid")
	_set_slider("gear_ratios:1", 1.2)
	var verdict: Dictionary = garage.get_validation()
	_check(not bool(verdict["ok"]) and str(verdict["message"]) != "", "a non-monotonic gear set is rejected (message: %s)" % verdict["message"])
	_check(ui.status_text().begins_with("Cannot save"), "the configurator shows the rejection ('%s')" % ui.status_text())
	await _back_to_root()
	await _press_escape()
	_check(ui.get_button("save").disabled and ui.get_button("garage_exit:drive").disabled and ui.get_button("garage_exit:selection").disabled, "invalid setup disables both save-and-exit choices")
	await _press_escape()
	var saved_bad: Dictionary = garage.save()
	_check(not bool(saved_bad["ok"]), "saving the invalid setup fails (%s)" % saved_bad.get("error", ""))
	_press("tile:Engine")
	# an out-of-range value is rejected too (not clamped silently)
	_set_slider("gear_ratios:1", 1.0)
	_set_slider("gear_ratios:0", 1.0)
	_check(bool(garage.get_validation()["ok"]), "putting the gears back makes the setup valid again")
	var out_of_range: Dictionary = garage.set_option("final_drive", 3.0)
	_check(not bool(out_of_range["ok"]) and str(out_of_range["validation_message"]) != "", "final drive x3.0 is outside the whitelisted range and rejected (%s)" % out_of_range["validation_message"])
	garage.set_option("final_drive", 1.0)
	_check(bool(garage.get_validation()["ok"]), "final drive back to stock is valid")
	await _wait_seconds(0.1)

	# ---- keyboard entry boxes (garage numeric inputs, 2026-10-13) ----
	await _numeric_entry_checks()

	# ---- save, persistence ----
	_check(_press("save"), "Save is pressed")
	_check(not garage.is_dirty() and bool(garage.get_vehicle(hyper_id)["has_setup"]), "the setup is saved")
	# Save-and-selection also handles unsaved changes, then preserves them on reopen.
	garage.set_option("brake_force",1.05)
	_check(garage.is_dirty(), "setup is dirty before save-and-selection")
	await _back_to_root()
	await _press_escape()
	_press("garage_exit:selection")
	_check(_screen() == "vehicle_select", "save-and-selection returns to the car browser")
	ui.browser().focus_car(hyper_id)
	_press("choose")
	_check(not garage.is_dirty(), "reopening the saved car is clean")
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
	garage.set_option("brake_force",1.1)
	_check(garage.is_dirty(), "setup is dirty before save-and-drive")
	await _press_escape()
	_check(get_viewport().gui_get_focus_owner() == ui.get_button("garage_exit:drive"), "exit prompt focuses save-and-drive")
	for pressed in [true,false]:
		var confirm := InputEventJoypadButton.new()
		confirm.button_index = JOY_BUTTON_A
		confirm.pressed = pressed
		Input.parse_input_event(confirm)
	await _wait_seconds(0.1)
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
		await _press_escape()
		_check(await _wait_until(func(): return _screen() == "pause", 5.0), "Esc pauses")
		_check(ui.button_ids().has("garage"), "the pause menu offers Garage (respawn)")
		_press("garage")
		_check(_screen() == "vehicle_select" and main.get_garage_scene() != null, "pause -> Garage opens the vehicle select (is '%s')" % _screen())
		await _wait_seconds(0.5)
		ui.browser().focus_car(sedan_id)
		_press("choose")
		_check(_screen() == "configurator" and str(garage.get_edit_id()) == sedan_id, "the sedan's configurator opens (is '%s')" % _screen())
		_press("garage:upgrades")
		_press("tile:Swaps")
		_press("tile:Engine")
		var i4_key := "default:i4_2l_na"
		var v8_key := "default:hyper_v8_one1_audio"
		_check(ui.get_button("part:engine_install:" + i4_key) != null, "sedan inline-four engine card exists")
		_press("part:engine_install:" + i4_key)
		_check(garage.is_dirty() and bool(garage.get_validation()["ok"]), "sedan engine swap installs and validates")
		_press("part:engine_install:stock")
		_check(not garage.is_dirty(), "original sedan engine restores stock")
		# G1: the engine list is one list (defaults + user engines) and a legacy id of the first version still works
		var engine_keys: Array = []
		for engine in garage.get_engines():
			engine_keys.append(str(engine["key"]))
		_check(engine_keys.size() >= 6 and engine_keys.has(v8_key), "the garage lists the catalog engines (%d)" % engine_keys.size())
		_check(bool(garage.set_option("engine_install", "sedan_i4")["accepted"]) and str(garage.get_options().filter(func(o): return str(o["id"]) == "engine_install")[0]["value"]) == i4_key, "a legacy engine id is accepted and stored as a key")
		garage.reset_option("engine_install")
		# the hyper V8 into the sedan: the swap package derives fuel, clutch, shift points and weight
		_check(ui.get_button("part:engine_install:" + v8_key) != null, "the twin-turbo V8 card exists on the sedan")
		_press("part:engine_install:" + v8_key)
		_check(garage.is_dirty() and bool(garage.get_validation()["ok"]), "the V8 swap into the sedan installs and validates")
		var swap_info: Dictionary = garage.get_swap_info()
		_check(PackedStringArray(swap_info["lines"]).size() > 0, "the swap lists what it changed: %s" % [swap_info["lines"]])
		_check(is_equal_approx(float(swap_info["mass_delta_kg"]), 10.0), "the chassis-mass delta is the catalog difference (%s kg)" % swap_info["mass_delta_kg"])
		var swapped_mass: Dictionary = garage.get_mass_breakdown(sedan_id)
		_check(bool(swapped_mass["ok"]) and is_equal_approx(float(swapped_mass["total_kg"]), float(swapped_mass["chassis_kg"]) + float(swapped_mass["physics_carried_kg"])), "the swapped mass includes the parts and separately carried fuel")
		_check(ui.find_child("SwapSummary", true, false) != null, "the engine page shows the swap summary")
		await _back_to_root()
		await _press_escape()
		_press("garage_exit:drive")
		var respawned: bool = await _wait_until(func(): return _screen() == "drive" and main.world_state == "running", WORLD_LOAD_S)
		_check(respawned, "the respawn with the sedan reaches drive (screen '%s', state '%s')" % [_screen(), main.world_state])
		if respawned:
			var sedan_sim: String = str(garage.get_vehicle(sedan_id)["sim_name"])
			_check(main.VEHICLE_NAME == sedan_sim and sim.is_running(), "the sedan drives now (%s)" % main.VEHICLE_NAME)
			var sedan_sel: Dictionary = main.get_drive_selection()
			_check(bool(sedan_sel["modified"]), "the sedan carries the V8 swap")
			_check(str(sedan_sel["engine_title"]) == "Twin-turbo V8 5.1", "the drive names the installed engine ('%s')" % sedan_sel["engine_title"])
			_check(is_equal_approx(float(sedan_sel["chassis"]["mass_kg"]), float(swapped_mass["chassis_kg"])), "the drive's chassis mass is the configured part tree minus carried contents (%s kg)" % sedan_sel["chassis"]["mass_kg"])
			_check(main.get_garage_scene() == null, "no garage scene while driving")
			await _wait_seconds(1.0)
			var steps0: int = sim.get_step_count()
			await _wait_seconds(0.5)
			_check(sim.get_step_count() > steps0, "the respawned car's sim ticks")
			# readbacks of the swapped car: limiter of the new engine, clutch capacity as derived, rpm reaches the limiter in 2nd
			var v8_info: Dictionary = {}
			for engine in garage.get_engines():
				if str(engine["key"]) == v8_key:
					v8_info = engine
			var gauge: Dictionary = sim.get_vehicle_gauge_info(main.VEHICLE_NAME)
			_check(is_equal_approx(float(gauge["limiter_rpm"]), float(v8_info["limiter_rpm"])), "physics: the sedan's limiter is the V8's (%s rpm)" % gauge["limiter_rpm"])
			var derived_clutch: float = ceilf(1.5 * float(v8_info["torque_nm"]) / 10.0) * 10.0
			_check(is_equal_approx(float(gauge["clutch_capacity_nm"]), derived_clutch), "physics: the clutch capacity is the derived 1.5 x peak torque (%s, derived %s N*m)" % [gauge["clutch_capacity_nm"], derived_clutch])
			await _rev_in_second_gear(float(gauge["limiter_rpm"]))
			# put the sedan back to stock for the persistence checks below
			garage.begin_edit(sedan_id)
			garage.reset_all()
			garage.save()
			garage.discard()
			await _pause_to_main_menu()

	# ---- back to the hyper car through the garage: the same node count as after the stock drive ----
	_press("garage")
	ui.browser().focus_car(hyper_id)
	_press("choose")
	_check(str(garage.get_edit_id()) == hyper_id and bool(garage.get_vehicle(hyper_id)["has_setup"]), "the hyper car's saved setup is still there")
	await _press_escape()
	_press("garage_exit:drive")
	_press("spawn:flat")
	if await _wait_until(func(): return _screen() == "drive", WORLD_LOAD_S):
		await _after_world_load("hyper again", 1.0)
		_check(bool(main.get_drive_selection()["modified"]), "the saved setup is applied again (persistence across drives)")
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
	_press("quit")
	_finish()

# Full throttle in manual 1st then 2nd (assist auto-shift off): the engine reaches the limiter of the installed engine.
func _rev_in_second_gear(limiter_rpm: float) -> void:
	var sim: Node = main._simulation
	main.scripted_controls = {"throttle": 1.0, "brake": 0.0, "handbrake": 0.0, "steer": 0.0, "clutch": 0.0, "ignition": 1.0,
		"starter": 0.0, "assist.auto_shift": 0.0, "assist.auto_clutch": 1.0, "assist.auto_blip": 1.0}
	# start the engine (it begins Stalled/Off in neutral): starter on, throttle off, until it runs
	main.scripted_controls["throttle"] = 0.0
	main.scripted_controls["starter"] = 1.0
	var running: bool = await _wait_until(func(): return str(sim.get_vehicle_powertrain(main.VEHICLE_NAME)["engine_state"]) == "Running", 15.0)
	main.scripted_controls["starter"] = 0.0
	_check(running, "the V8 in the sedan starts (%s)" % sim.get_vehicle_powertrain(main.VEHICLE_NAME)["engine_state"])
	await _wait_seconds(1.0)
	main.scripted_controls["throttle"] = 1.0
	sim.set_control("shift_up_count", sim.get_control("shift_up_count") + 1)
	_check(await _wait_until(func(): return int(sim.get_vehicle_powertrain(main.VEHICLE_NAME)["gear"]) == 1, 10.0), "the sedan with the V8 selects 1st")
	await _wait_seconds(1.5)
	sim.set_control("shift_up_count", sim.get_control("shift_up_count") + 1)
	_check(await _wait_until(func(): return int(sim.get_vehicle_powertrain(main.VEHICLE_NAME)["gear"]) == 2, 10.0), "... and 2nd")
	var peak_holder: Array = [0.0] # a lambda captures plain floats by value; an Array is shared
	var reached: bool = await _wait_until(func():
		var pt: Dictionary = sim.get_vehicle_powertrain(main.VEHICLE_NAME)
		peak_holder[0] = maxf(float(peak_holder[0]), float(pt["rpm"]))
		return int(pt["gear"]) == 2 and (float(peak_holder[0]) >= limiter_rpm - 200.0 or bool(pt["limiter"])), 25.0)
	var peak: float = float(peak_holder[0])
	var last: Dictionary = sim.get_vehicle_powertrain(main.VEHICLE_NAME)
	_check(reached, "physics: the rpm reaches the new limiter in 2nd gear (peak %.0f of %.0f rpm; now gear %s, %s, clutch %s, throttle %s)" % [peak, limiter_rpm, last.get("gear"), last.get("engine_state"), last.get("clutch_engagement"), sim.get_control("throttle")])
	_check(peak <= limiter_rpm + 400.0, "... and does not pass it by much (peak %.0f rpm)" % peak)
	main.scripted_controls = {}

# Types `text` into the value box of an option as a player does (focus, text, Enter).
func _type_entry(id: String, text: String, index: int = 0) -> void:
	var box: LineEdit = main.get_shell_ui().get_option_entry(id, index)
	if not _check(box != null, "option '%s' has a value box (has %s)" % [id, ", ".join(main.get_shell_ui().option_control_ids())]):
		return
	box.grab_focus()
	box.text = text
	box.text_submitted.emit(text)

func _option_value(id: String) -> Variant:
	var garage: Node = main.get_garage()
	for opt in garage.get_options():
		if str(opt["id"]) == id:
			return opt["value"]
	return null

func _option_modified(id: String) -> bool:
	var garage: Node = main.get_garage()
	for opt in garage.get_options():
		if str(opt["id"]) == id:
			return bool(opt["modified"])
	return false

func _option_stock(id: String) -> float:
	var garage: Node = main.get_garage()
	for opt in garage.get_options():
		if str(opt["id"]) == id:
			return float(opt["stock_numbers"][0])
	return 1.0

func _entry_text(id: String, index: int = 0) -> String:
	var box: LineEdit = main.get_shell_ui().get_option_entry(id, index)
	return box.text if box != null else "<no box>"

# The value boxes: typing, both-way sync with the slider, the log mapping at both ends, rejection with a reason and the
# take-back of a value the physics loader refuses. Runs on the hyper car's Engine page; leaves every option at stock.
func _numeric_entry_checks() -> void:
	var ui: CanvasLayer = main.get_shell_ui()
	var garage: Node = main.get_garage()
	_check(ui.active_area() == "engine", "the Engine page is open for the entry boxes (is '%s')" % ui.active_area())

	# -- rev limiter: 2000..20000 rpm on a log slider, absolute rpm in the box --
	var stock_rpm := _option_stock("engine_rev_limit")
	_check(stock_rpm > 1000.0, "the rev limiter's stock value is the engine's limit (%.0f rpm)" % stock_rpm)
	_check(_entry_text("engine_rev_limit") == "%d" % int(round(stock_rpm)), "the rev limiter box shows the stock limit in rpm ('%s')" % _entry_text("engine_rev_limit"))
	var slider: HSlider = ui.get_option_control("engine_rev_limit")
	_check(slider != null and is_equal_approx(slider.min_value, 0.0) and is_equal_approx(slider.max_value, 1.0), "the rev limiter slider runs over the log position 0..1")
	var entry_info: Dictionary = {}
	for opt in garage.get_options():
		if str(opt["id"]) == "engine_rev_limit":
			entry_info = opt["entry"]
	_check(is_equal_approx(float(entry_info.get("lo", 0.0)), 2000.0) and is_equal_approx(float(entry_info.get("hi", 0.0)), 20000.0) and str(entry_info.get("mapping", "")) == "log", "the range comes from the option data: 2000 to 20000 rpm, log (%s)" % entry_info)

	_type_entry("engine_rev_limit", "12000")
	_check(is_equal_approx(float(_option_value("engine_rev_limit")) * stock_rpm, 12000.0), "typing 12000 stores %.1f rpm" % (float(_option_value("engine_rev_limit")) * stock_rpm))
	_check(_entry_text("engine_rev_limit") == "12000", "... and the box keeps 12000 ('%s')" % _entry_text("engine_rev_limit"))
	_check(absf(slider.value - log(12000.0 / 2000.0) / log(10.0)) < 0.002, "... and the slider moved to the log position of 12000 rpm (%.4f)" % slider.value)
	_check(garage.is_dirty() and bool(garage.get_validation()["ok"]), "12000 rpm is a valid setup (%s)" % garage.get_validation()["message"])

	slider.value = 0.5 # dragging the slider updates the box: the geometric middle of 2000..20000
	_check(_entry_text("engine_rev_limit") == "%d" % int(round(sqrt(2000.0 * 20000.0))), "the slider's middle is the geometric mean in the box ('%s')" % _entry_text("engine_rev_limit"))

	_type_entry("engine_rev_limit", "20000")
	_check(is_equal_approx(float(_option_value("engine_rev_limit")) * stock_rpm, 20000.0) and is_equal_approx(slider.value, 1.0), "typing 20000 reaches the top end of the slider")
	_check(bool(garage.get_validation()["ok"]), "20000 rpm loads in the physics (%s)" % garage.get_validation()["message"])
	# the setup check (limit advisor) warns about what 20000 rpm does, it never refuses the value
	_check(await _wait_until(func(): var a: Dictionary = garage.get_advice(); return not a["busy"] and a["have_result"] and not a["items"].is_empty(), 180.0), "the setup check reports items for 20000 rpm")
	var flagged := false
	for item in garage.get_advice()["items"]:
		if str(item["id"]).begins_with("MeanPistonSpeed") or str(item["id"]).begins_with("MapGridEdge"):
			flagged = true
	_check(flagged, "20000 rpm is flagged by the setup check (piston speed / data range), not refused")

	# a refused value is taken back and the reason is shown
	var before := float(_option_value("engine_rev_limit"))
	_type_entry("engine_rev_limit", "20001")
	_check(is_equal_approx(float(_option_value("engine_rev_limit")), before) and _entry_text("engine_rev_limit") == "20000", "20001 rpm is rejected: the box shows 20000 again ('%s')" % _entry_text("engine_rev_limit"))
	var note: Label = ui._option_notes.get("engine_rev_limit", null)
	_check(note != null and note.visible and note.text.contains("2000 to 20000 rpm"), "... with the allowed range as the reason ('%s')" % (note.text if note != null else "no note"))
	_type_entry("engine_rev_limit", "1999")
	_check(is_equal_approx(float(_option_value("engine_rev_limit")), before) and note.visible, "1999 rpm is rejected too")
	_type_entry("engine_rev_limit", "fast")
	_check(is_equal_approx(float(_option_value("engine_rev_limit")), before) and note.text.contains("not a number"), "text that is no number is rejected ('%s')" % note.text)
	# the lowest limit: inside the range; the physics loader decides (the hypercar's launch control needs rpm headroom)
	_type_entry("engine_rev_limit", "2000")
	var typed_2000 := is_equal_approx(float(_option_value("engine_rev_limit")) * stock_rpm, 2000.0)
	if typed_2000:
		_check(bool(garage.get_validation()["ok"]), "2000 rpm typed and kept: the setup is valid")
	else:
		_check(is_equal_approx(float(_option_value("engine_rev_limit")), before) and note.visible and not note.text.is_empty(), "2000 rpm refused by the loader is taken back with its reason ('%s')" % note.text)
	print("garage_test: 2000 rpm typed on this car -> kept=%s, note='%s'" % [typed_2000, note.text])
	slider.value = 0.0 # the slider end is always reachable; its verdict is the setup status
	_check(is_equal_approx(float(_option_value("engine_rev_limit")) * stock_rpm, 2000.0), "the slider's low end is 2000 rpm (%.1f)" % (float(_option_value("engine_rev_limit")) * stock_rpm))
	_check(_entry_text("engine_rev_limit") == "2000", "... and the box says 2000 ('%s')" % _entry_text("engine_rev_limit"))
	print("garage_test: 2000 rpm by slider -> valid=%s message='%s'" % [garage.get_validation()["ok"], garage.get_validation()["message"]])
	_type_entry("engine_rev_limit", "%d" % int(round(stock_rpm)))
	_check(not _option_modified("engine_rev_limit"), "typing the stock limit is no change")
	_check(bool(garage.get_validation()["ok"]), "the stock limit is valid again (%s)" % garage.get_validation()["message"])

	# -- boost target: 0.1..30 bar, log --
	if ui.get_option_entry("turbo_boost_target") != null:
		var stock_bar := _option_stock("turbo_boost_target")
		_type_entry("turbo_boost_target", "2.5")
		_check(is_equal_approx(float(_option_value("turbo_boost_target")) * stock_bar, 2.5), "typing 2.5 bar stores %.3f bar" % (float(_option_value("turbo_boost_target")) * stock_bar))
		_type_entry("turbo_boost_target", "30")
		_check(is_equal_approx(float(_option_value("turbo_boost_target")) * stock_bar, 30.0), "30 bar is accepted (the data's upper end)")
		_type_entry("turbo_boost_target", "31")
		_check(is_equal_approx(float(_option_value("turbo_boost_target")) * stock_bar, 30.0), "31 bar is rejected")
		_type_entry("turbo_boost_target", "0.1")
		_check(is_equal_approx(float(_option_value("turbo_boost_target")) * stock_bar, 0.1), "0.1 bar is accepted (the data's lower end)")
		_type_entry("turbo_boost_target", "%.3f" % stock_bar)
		_check(not _option_modified("turbo_boost_target"), "typing the stock boost is no change")
	else:
		_check(false, "the hyper car's Engine page has a boost target box (has %s)" % ", ".join(ui.option_control_ids()))

	# -- gear ratios: one box per gear in the absolute ratio --
	var gears: PackedFloat64Array = _option_value("gear_ratios")
	var stock_gear := _option_stock("gear_ratios")
	_check(ui.get_option_entry("gear_ratios", gears.size() - 1) != null, "every gear has a value box (%d gears)" % gears.size())
	_type_entry("gear_ratios", "%.3f" % (stock_gear * 1.1), 0)
	_check(absf(float(_option_value("gear_ratios")[0]) - 1.1) < 0.001, "typing 110 %% of the first gear's ratio stores x1.1 (%.4f)" % float(_option_value("gear_ratios")[0]))
	_type_entry("gear_ratios", "%.3f" % (stock_gear * 5.0), 0)
	_check(absf(float(_option_value("gear_ratios")[0]) - 1.1) < 0.001, "a first-gear ratio far outside its range is rejected and taken back")
	garage.reset_option("gear_ratios")

	# -- a plain scale (x factor) and the brake bias (percent), on the Wheels page --
	await _back_to_root()
	_press("tile:Wheels")
	await _wait_seconds(0.3)
	_type_entry("spring_front", "1.25")
	_check(is_equal_approx(float(_option_value("spring_front")), 1.25), "a plain scale takes a typed factor (x1.25)")
	_check(_entry_text("spring_front") == "1.25", "... shown as the factor ('%s')" % _entry_text("spring_front"))
	_type_entry("spring_front", "x1.3")
	_check(is_equal_approx(float(_option_value("spring_front")), 1.3), "a typed 'x1.3' is read as 1.3")
	_type_entry("spring_front", "4")
	_check(is_equal_approx(float(_option_value("spring_front")), 1.3), "a factor outside the range is rejected")
	var bias_before := float(_option_value("brake_bias"))
	_type_entry("brake_bias", "%.1f" % (bias_before * 100.0 + 2.0))
	_check(absf(float(_option_value("brake_bias")) - (bias_before + 0.02)) < 0.0006, "the brake bias box takes percent (%.3f -> %.3f)" % [bias_before, float(_option_value("brake_bias"))])
	_type_entry("brake_bias", "99")
	_check(absf(float(_option_value("brake_bias")) - (bias_before + 0.02)) < 0.0006, "a brake bias far from stock is rejected")
	var any_box: LineEdit = ui.get_option_entry("spring_front")
	_check(any_box != null and any_box.focus_mode == Control.FOCUS_CLICK, "value boxes are click-focus only (the pad and Tab skip them)")
	garage.reset_option("brake_bias")
	await _back_to_root()
	_press("tile:Engine")
	await _wait_seconds(0.3)

func _back_to_root() -> void:
	var ui: CanvasLayer = main.get_shell_ui()
	while not ui._garage_path.is_empty():
		await _press_escape()

# Upgrades layout (owner 2026-10-18): the category tiles and the Chassis / Weight reduction tiles at two window sizes. Every
# tile's name label lies inside its tile, no two tiles overlap, no other label sits on a tile, and a highlighted (focused)
# tile shows at most two badges, the same ones RgGarage.get_part_badges returns, red when worse, green when better.
const UPGRADE_LAYOUT_SIZES := [Vector2i(1280, 720), Vector2i(1920, 1080)]

func _tiles_under(node: Node, out: Array) -> void:
	for child in node.get_children():
		if child is Button and child.name.begins_with("tile:") or child is Button and child.has_node("Name"):
			out.append(child)
		else:
			_tiles_under(child, out)

func _labels_under(node: Node, out: Array) -> void:
	for child in node.get_children():
		if child is Label and child.is_visible_in_tree():
			out.append(child)
		_labels_under(child, out)

# Returns the geometry findings for the tiles currently drawn in the option box.
func _upgrade_layout_report(ui: CanvasLayer) -> Dictionary:
	var report := {"tiles": 0, "overflow": 0, "overlap": 0, "label_on_tile": 0}
	var tiles: Array = []
	_tiles_under(ui._option_box, tiles)
	var rects: Array = []
	for tile in tiles:
		report["tiles"] += 1
		var rect := Rect2(tile.global_position, tile.size)
		rects.append(rect)
		var name_label: Label = tile.get_node_or_null("Name")
		if name_label != null and not rect.encloses(Rect2(name_label.global_position, name_label.size)):
			report["overflow"] += 1
	for i in range(rects.size()):
		for j in range(i + 1, rects.size()):
			if rects[i].intersects(rects[j]):
				report["overlap"] += 1
	var labels: Array = []
	_labels_under(ui._option_box, labels)
	for label in labels:
		var inside_tile := false
		for tile in tiles:
			if tile.is_ancestor_of(label):
				inside_tile = true
		if inside_tile or label.size.x <= 0.0 or label.size.y <= 0.0:
			continue
		var label_rect := Rect2(label.global_position, label.size)
		for rect in rects:
			if label_rect.intersects(rect):
				report["label_on_tile"] += 1
				break
	return report

func _check_upgrade_layout() -> void:
	var ui: CanvasLayer = main.get_shell_ui()
	var pages := [[], ["tile:Chassis", "tile:Weight reduction"]]
	for size in UPGRADE_LAYOUT_SIZES:
		get_window().size = size
		for page in pages:
			await _back_to_root()
			for step in page:
				_press(step)
			await get_tree().process_frame
			await get_tree().process_frame
			var report := _upgrade_layout_report(ui)
			var where := "%s %s" % [str(size), " / ".join(page) if not page.is_empty() else "root"]
			if not page.is_empty():
				_check(int(report["tiles"]) > 0, "upgrades %s: tiles are drawn (%d)" % [where, report["tiles"]])
			_check(int(report["overflow"]) == 0, "upgrades %s: no tile label leaves its tile (%d)" % [where, report["overflow"]])
			_check(int(report["overlap"]) == 0, "upgrades %s: no two tiles overlap (%d)" % [where, report["overlap"]])
			_check(int(report["label_on_tile"]) == 0, "upgrades %s: no other label sits on a tile (%d)" % [where, report["label_on_tile"]])
			print("RG_GARAGE_TEST info: upgrades %s viewport %s tiles %d overflow %d overlap %d label_on_tile %d" % [where, str(get_viewport().get_visible_rect().size), report["tiles"], report["overflow"], report["overlap"], report["label_on_tile"]])
	# Highlight badges: focus each weight tile; the row holds at most two badges, and each badge text is the core's answer.
	await _back_to_root()
	_press("tile:Chassis")
	_press("tile:Weight reduction")
	await get_tree().process_frame
	var tiles: Array = []
	_tiles_under(ui._option_box, tiles)
	var with_badges := 0
	for tile in tiles:
		var row: HBoxContainer = tile.get_node_or_null("Badges")
		if row == null:
			continue
		tile.grab_focus()
		await get_tree().process_frame
		var texts: Array = []
		for badge in row.get_children():
			texts.append(str(badge.text))
		_check(texts.size() <= 2, "a highlighted tile shows at most two badges (%d)" % texts.size())
		var expected: Array = []
		for b in ui.garage.get_part_badges(str(tile.get_meta("badge_option", "")), str(tile.get_meta("badge_part", ""))):
			expected.append(str(b["text"]))
		_check(texts == expected, "highlighted badges equal the core answer (%s vs %s)" % [", ".join(texts), ", ".join(expected)])
		with_badges += texts.size()
	_check(with_badges > 0, "the weight page shows at least one badge on a highlighted tile (%d)" % with_badges)
	await _back_to_root()
