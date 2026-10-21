extends Node
# game/scripts/garage_shots.gd - `--garage-shots <dir>` (needs a real window, not
# --headless): drives the real shell to the garage and saves the window as PNGs:
#   vehicle_select.png, overview.png, turntable_rotated.png, edit_wheels.png,
#   edit_engine.png, edit_rear.png (the PLAN.md R6 acceptance screenshots).
# Optional `--garage-shots-vehicle <id>` picks the car (default: the catalog default)
# and `--garage-shots-prefix <p>` prefixes the file names.

var main: Node
var out_dir: String = ""
var vehicle: String = ""
var prefix: String = ""

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
	_stage = "shot " + name
	await RenderingServer.frame_post_draw
	var image := get_viewport().get_texture().get_image()
	var path := out_dir.path_join(prefix + name + ".png")
	var err := image.save_png(path)
	print("RG_GARAGE_SHOT %s err=%d size=%s" % [path, err, image.get_size()])

func _press(id: String) -> void:
	var ui: CanvasLayer = main.get_shell_ui()
	var button: Button = ui.get_button(id)
	if button == null:
		print("RG_GARAGE_SHOT FAIL no button %s (has %s)" % [id, ", ".join(ui.button_ids())])
		return
	button.pressed.emit()

# A shots run can never hang (the upgrades walker once sat ~40 min with idle CPU): every run ends after this many seconds.
const SHOTS_TIMEOUT_S := 420.0
var _stage := "start"

func _run() -> void:
	var guard := get_tree().create_timer(SHOTS_TIMEOUT_S * (5.0 if OS.get_cmdline_user_args().has("--hint-shots-only") else 1.0))
	guard.timeout.connect(func():
		print("RG_GARAGE_SHOT FAIL timeout in stage '%s'" % _stage)
		get_tree().quit(1))
	DirAccess.make_dir_recursive_absolute(out_dir)
	await _seconds(0.5)
	main._apply_transition(main.get_shell().boot_finished())
	await _frames(5)
	main._apply_transition(main.get_shell().menu_item("garage"))
	await _frames(5)
	if vehicle != "":
		main.get_shell_ui().browser().focus_car(vehicle)
	await _seconds(2.5)
	await _shot("vehicle_select")
	if OS.get_cmdline_user_args().has("--damage-shots-only"):
		var ui: CanvasLayer = main.get_shell_ui()
		var g = ui.garage
		var damage: Dictionary = g.get_damage(vehicle)
		if not bool(damage.get("any_damaged", false)):
			push_error("RG_DAMAGE_UI missing isolated damage fixture")
			get_tree().quit(1)
			return
		await _shot("damage_browser")
		var repair: Button = ui.browser().get_control("repair")
		if repair == null:
			push_error("RG_DAMAGE_UI repair button missing")
			get_tree().quit(1)
			return
		repair.pressed.emit()
		await _frames(5)
		if bool(g.get_damage(vehicle).get("any_damaged", false)):
			push_error("RG_DAMAGE_UI repair did not clear saved damage")
			get_tree().quit(1)
			return
		await _shot("damage_repaired")
		main._on_escape()
		await _frames(2)
		_press("settings")
		await _frames(2)
		if main.get_shell_ui().get_setting_control("difficulty.mechanical_damage") == null:
			push_error("RG_DAMAGE_UI difficulty toggle missing")
			get_tree().quit(1)
			return
		print("RG_DAMAGE_UI PASS repair persisted and difficulty toggle exists")
		get_tree().quit()
		return
	_press("choose")
	await _seconds(2.0)
	var dyno_deadline := Time.get_ticks_msec() + 120000
	while main.get_shell_ui().garage.get_dyno().get("busy",false) and Time.get_ticks_msec() < dyno_deadline:
		await _frames(1)
	await _seconds(0.25)
	if OS.get_cmdline_user_args().has("--clutch-shots-only"):
		# A dual-clutch gearbox swapped in: the Drivetrain tiles (Clutch enabled) and the "Gearbox clutches" page at x2.5.
		var cui: CanvasLayer = main.get_shell_ui()
		cui.garage.set_option("gearbox_install", "hyper_7dct")
		cui._draw_garage_options()
		_press("tile:Drivetrain")
		await _seconds(0.5)
		await _shot("drivetrain_tiles_dct")
		_press("tile:Clutch")
		await _seconds(0.5)
		_press("clutch:2.5")
		await _seconds(0.5)
		await _shot("gearbox_clutches_page")
		get_tree().quit()
		return
	if OS.get_cmdline_user_args().has("--advice-shots-only"):
		await _advice_shots()
		get_tree().quit()
		return
	if OS.get_cmdline_user_args().has("--dyno-shots-only"):
		await _dyno_shots()
		get_tree().quit()
		return
	if OS.get_cmdline_user_args().has("--upgrades-shots-only"):
		await _upgrade_shots()
		get_tree().quit()
		return
	if OS.get_cmdline_user_args().has("--hint-shots-only"):
		await _hint_shots()
		get_tree().quit()
		return
	if OS.get_cmdline_user_args().has("--spec-shots-only"):
		await _spec_shots()
		get_tree().quit()
		return
	if OS.get_cmdline_user_args().has("--strip-shots-only"):
		await _strip_shots()
		get_tree().quit()
		return
	await _shot("overview")
	_press("tile:Chassis")
	_press("tile:Weight")
	await _seconds(0.5)
	await _shot("part_mass")
	main._on_escape()
	_press("tile:Weight reduction")
	await _seconds(0.5)
	await _shot("weight_reduction")
	for i in range(2): main._on_escape()
	if OS.get_cmdline_user_args().has("--mass-shots-only"):
		get_tree().quit()
		return
	# The setup check: an extreme (owner-like) turbo setup is allowed and flagged. Only for cars that have those options.
	var shell_garage = main.get_shell_ui().garage
	var extreme: Dictionary = shell_garage.set_option("turbo_boost_target", 8.600083819796385)
	if bool(extreme.get("accepted", false)):
		shell_garage.set_option("clutch_capacity", 8.0)
		shell_garage.set_option("turbo_install_n2o", "hyper_twin_large_n2o")
		main.get_shell_ui()._draw_garage_options()
		var advice_deadline := Time.get_ticks_msec() + 180000
		await _seconds(0.6)
		while shell_garage.get_advice().get("busy", false) and Time.get_ticks_msec() < advice_deadline:
			await _frames(1)
		await _seconds(0.6)
		await _shot("setup_check")
		shell_garage.reset_all()
		main.get_shell_ui()._draw_garage_options()
		await _seconds(0.3)
	await _seconds(3.0)
	await _shot("turntable_rotated")
	_press("tile:Wheels")
	_press("tile:All")
	_press("tile:Compounds")
	await _seconds(0.5)
	await _shot("wheel_compounds")
	main._on_escape()
	_press("tile:Dimensions")
	await _seconds(0.5)
	await _shot("wheel_dimensions")
	for i in range(3): main._on_escape()
	_press("tile:Engine")
	_press("tile:Turbo")
	await _seconds(1.5)
	await _shot("turbo_parts")
	for i in range(2): main._on_escape()
	_press("tile:Swaps")
	_press("tile:Forced induction")
	await _seconds(0.5)
	await _shot("induction_types")
	for i in range(2): main._on_escape()
	main._on_escape()
	await _seconds(0.25)
	await _shot("garage_exit")
	print("RG_GARAGE_SHOT done")
	get_tree().quit()

# `--dyno-shots-only`: the garage dyno chart for the stock car, then (cars with the powertrain conversion) the battery hybrid,
# the fuel-cell EV and a dual-clutch hybrid (no curve, the reason shown): dyno_<conversion>.png.
func _wait_dyno() -> void:
	var g = main.get_shell_ui().garage
	await _seconds(0.6)
	var deadline := Time.get_ticks_msec() + 180000
	while g.get_dyno().get("busy",false) and Time.get_ticks_msec() < deadline:
		await _frames(1)
	await _seconds(0.6)
	var d: Dictionary = g.get_dyno()
	print("RG_DYNO_SHOT topology=%s caption='%s' reason='%s' error='%s' points=%d" % [d.get("topology",""), d.get("caption",""),
		d.get("unsupported_reason",""), d.get("error",""), d.get("points",[]).size()])

func _dyno_shots() -> void:
	var g = main.get_shell_ui().garage
	await _wait_dyno()
	await _shot("dyno_stock")
	for install in [["hybrid_battery", ""], ["fuel_cell_ev", ""], ["hybrid_battery", "dct_7"]]:
		var reply: Dictionary = g.set_option("powertrain_install", install[0])
		if not bool(reply.get("accepted", false)):
			print("RG_DYNO_SHOT skip %s: %s" % [install[0], reply])
			continue
		var name: String = install[0]
		if install[1] != "":
			var box: Dictionary = g.set_option("gearbox_install", install[1])
			print("RG_DYNO_SHOT gearbox %s accepted=%s" % [install[1], box.get("accepted", false)])
			name += "_" + install[1]
		main.get_shell_ui()._draw_garage_options()
		await _wait_dyno()
		await _shot("dyno_" + name)
		g.reset_all()
	main.get_shell_ui()._draw_garage_options()

# `--advice-shots-only`: the setup check of the car as the user dir's saved setup leaves it (setup_check_<car>.png); the
# status line and every note are printed as RG_ADVICE_SHOT lines.
func _advice_shots() -> void:
	var ui: CanvasLayer = main.get_shell_ui()
	var g = ui.garage
	var deadline := Time.get_ticks_msec() + 180000
	await _seconds(0.6)
	while Time.get_ticks_msec() < deadline:
		var a: Dictionary = g.get_advice()
		if not bool(a.get("busy", false)) and (bool(a.get("have_result", false)) or str(a.get("error", "")) != ""):
			break
		await _frames(1)
	await _seconds(0.6)
	print("RG_ADVICE_SHOT status: %s" % ui.advice_status_text())
	for item in g.get_advice().get("items", []):
		print("RG_ADVICE_SHOT item sev=%d %s" % [int(item.get("severity", 0)), str(item.get("title", ""))])
	await _shot("setup_check_" + vehicle)
	# Open the first driveline note (clutch, then gearbox) to show a note's details.
	for kind in ["ClutchCapacity", "GearboxInputTorque"]:
		for item in g.get_advice().get("items", []):
			if str(item.get("kind", "")) == kind:
				_press("advice_note:" + str(item.get("id", "")))
				await _seconds(0.4)
				await _shot("setup_check_" + vehicle + "_open")
				return

# Render the real showroom model from the driver's seat, before and after removing cabin parts.
func _strip_shots() -> void:
	var scene: Node = main.get_garage_scene()
	scene.set_process(false)
	main.get_shell_ui().hide()
	var holder: Node3D = scene.get("_cars")[vehicle]["real"]
	var eye: Node3D = holder.find_child("socket_driver_eye", true, false)
	var seat: Node3D = holder.find_child("seat_passenger", true, false)
	var camera: Camera3D = scene.camera_node()
	camera.h_offset = 0.0
	camera.fov = 85.0
	camera.global_position = eye.global_position + Vector3(0, 0.12, 0)
	camera.look_at(seat.global_position + Vector3(0, 0.25, 0), Vector3.UP)
	await _frames(4)
	await RenderingServer.frame_post_draw
	scene.capture_image().save_png(out_dir.path_join("cabin_stock.png"))
	var g = main.get_shell_ui().garage
	for option in ["trim_passenger_seat", "trim_rear_seats", "trim_audio", "trim_door_trim", "trim_carpets", "trim_sound_deadening"]:
		var result: Dictionary = g.set_option(option, "removed")
		print("RG_STRIP_SHOT %s %s" % [option, result])
	var fitted: Dictionary = g.get_working_colours()
	scene.set_model_options(Array(fitted.get("model_options", [])), Array(fitted.get("hidden_nodes", [])))
	await _frames(4)
	await RenderingServer.frame_post_draw
	scene.capture_image().save_png(out_dir.path_join("cabin_stripped.png"))
	print("RG_STRIP_SHOT done")

# Upgrades screens (owner 2026-10-18): every Upgrades page two levels deep, at each window size, into out_dir as
# upgrades_<width>_<page path>.png. Pages are walked from the root; a tile that is missing on a page is skipped.
const UPGRADE_SHOT_SIZES := [Vector2i(1280, 720), Vector2i(1920, 1080)]

func _back_to_root_shots(ui: CanvasLayer) -> void:
	while not ui._garage_path.is_empty():
		main._on_escape()
		await _frames(2)

func _slug(text: String) -> String:
	var re := RegEx.new()
	re.compile("[^A-Za-z0-9]+")
	return re.sub(text, "_", true).trim_prefix("_").trim_suffix("_")

func _walk_upgrades(ui: CanvasLayer, path: Array, depth: int, visited: Dictionary) -> void:
	var key := " / ".join(path)
	if visited.has(key):
		return
	visited[key] = true
	await _back_to_root_shots(ui)
	for step in path:
		_press(step)
		await _frames(3)
	var tiles: Array = []
	for id in ui.button_ids():
		var text := str(id)
		if text.begins_with("tile:") and not text.begins_with("tile:tune") and not text.begins_with("tile:Tune"):
			tiles.append(text)
	for size in UPGRADE_SHOT_SIZES:
		get_window().size = size
		await _seconds(0.6)
		await _page_shots(ui, "upgrades_%d_%s" % [size.x, _slug(key) if not key.is_empty() else "root"])
		if key.ends_with("tile:Weight reduction"):
			await _badge_shots(ui, "upgrades_%d_weight_badge" % size.x)
	if depth < 2:
		for id in tiles:
			await _walk_upgrades(ui, path + [id], depth + 1, visited)

func _upgrade_shots() -> void:
	var ui: CanvasLayer = main.get_shell_ui()
	await _walk_upgrades(ui, [], 0, {})
	await _back_to_root_shots(ui)

# The option page, whole: the setup-check panel is hidden for the shot (it would leave the page ~150 px of scroll area at
# 720 p), then the page is shot at scroll offsets of one view height each (at most 4), names <name>_1, _2, ...
func _page_shots(ui: CanvasLayer, name: String) -> void:
	var scroll: ScrollContainer = ui._option_box.get_parent() if ui._option_box != null else null
	if scroll == null:
		await _shot(name)
		return
	if ui._advice_box != null and is_instance_valid(ui._advice_box):
		ui._advice_box.visible = false
	await _seconds(0.3)
	var view := maxf(scroll.size.y, 1.0)
	var total: float = ui._option_box.size.y
	var pages := clampi(int(ceil(total / view)), 1, 4)
	for i in range(pages):
		scroll.scroll_vertical = int(i * (view - 40.0))
		await _frames(3)
		await _shot("%s_%d" % [name, i + 1] if pages > 1 else name)
	scroll.scroll_vertical = 0
	if ui._advice_box != null and is_instance_valid(ui._advice_box):
		ui._advice_box.visible = true

# Upgrade HINTS (owner 2026-10-19) on every category page: all hint rows of a page are shown at once (not only the focused
# tile's) after the core finished the figures, one shot set per page, plus a log line per tile with the badge texts.
# `--hint-shots-only`; at most HINT_TILES_PER_PAGE tiles per page are asked (a swap page lists dozens of engines).
const HINT_TILES_PER_PAGE := 8
const HINT_PAGES := [
	["tyres_compounds", ["tile:Wheels", "tile:All", "tile:Compounds"]],
	["tyres_dimensions", ["tile:Wheels", "tile:All", "tile:Dimensions"]],
	["swaps_engine", ["tile:Swaps", "tile:Engine"]],
	["swaps_fuel", ["tile:Swaps", "tile:Fuel"]],
	["swaps_forced_induction", ["tile:Swaps", "tile:Forced induction"]],
	["swaps_drivetrain", ["tile:Swaps", "tile:Drivetrain"]],
	["engine_turbo", ["tile:Engine", "tile:Turbo"]],
	["engine_n2o", ["tile:Engine", "tile:N2O"]],
	["engine_internals", ["tile:Engine", "tile:Internals"]],
	["engine_cooling", ["tile:Engine", "tile:Cooling"]],
	["drivetrain_gearbox", ["tile:Drivetrain", "tile:Gearbox"]],
	["chassis_weight_reduction", ["tile:Chassis", "tile:Weight reduction"]],
]

func _hint_shots() -> void:
	var ui: CanvasLayer = main.get_shell_ui()
	for page in HINT_PAGES:
		var name := str(page[0])
		_stage = "hints " + name
		await _back_to_root_shots(ui)
		for step in page[1]:
			_press(step)
			await _frames(3)
		await _seconds(0.5)
		var tiles: Array = []
		_collect_tiles(ui._option_box, tiles)
		tiles = tiles.filter(func(t): return not t.has_node("Installed")) # the installed part has no hints
		if tiles.size() > HINT_TILES_PER_PAGE:
			tiles = tiles.slice(0, HINT_TILES_PER_PAGE)
		var rows: Array = []
		for tile in tiles:
			var row: Container = tile.get_node_or_null("Badges")
			if row == null:
				continue
			rows.append([tile, row])
			# The request, then wait until the core finished it (the newest request cancels the previous one's work).
			ui._fill_badges(row, str(tile.get_meta("badge_option")), tile.get_meta("badge_edits"))
			var t0 := Time.get_ticks_msec()
			ui.garage.wait_for_hints(600000)
			print("RG_HINT_SHOT %s wait %.2f s" % [name, (Time.get_ticks_msec() - t0) / 1000.0])
		for pair in rows:
			var tile: Button = pair[0]
			var row: Container = pair[1]
			ui._fill_badges(row, str(tile.get_meta("badge_option")), tile.get_meta("badge_edits"))
			row.visible = true
			var texts: Array = []
			for chip in row.get_children():
				texts.append(str(chip.text))
			var label: Label = tile.get_node_or_null("Name")
			print("RG_HINT_SHOT %s tile '%s' edits %s -> [%s]" % [name, label.text if label != null else "?", str(tile.get_meta("badge_edits")), " | ".join(texts)])
		await _seconds(0.3)
		if rows.is_empty():
			print("RG_HINT_SHOT %s no tiles with hints" % name)
		await _page_shots(ui, "hints_" + name)
		print("RG_HINT_SHOT stats %s" % str(ui.garage.get_hint_stats()))
	await _back_to_root_shots(ui)

# The left specification pane after swaps (owner 2026-10-19: it must show the configuration being edited, not the stock car).
# `--spec-shots-only`: the stock pane, then after each swap of SPEC_SWAPS[vehicle] the figures once the core and the dyno finished,
# with a swap page whose tiles show their hints (the installed tile none).
const SPEC_SWAPS := {
	"car_hyper": [["single_turbo", {"turbo_install": "hyper_single_compact"}, ["tile:Swaps", "tile:Forced induction"]]],
	"car_sedan": [
		["v8", {"engine_install": "default:ventura_v8_5l0_na"}, ["tile:Swaps", "tile:Engine"]],
		["hybrid", {"powertrain_install": "hybrid_battery"}, ["tile:Swaps", "tile:Powertrain", "tile:Hybrid"]],
	],
}

func _spec_wait(ui: CanvasLayer) -> void:
	ui.garage.wait_for_hints(600000)
	var deadline := Time.get_ticks_msec() + 300000
	await _seconds(0.6)
	while bool(ui.garage.get_dyno().get("busy", false)) and Time.get_ticks_msec() < deadline:
		await _frames(2)
	ui.garage.wait_for_hints(600000)
	await _seconds(1.0)

func _spec_shots() -> void:
	var ui: CanvasLayer = main.get_shell_ui()
	await _back_to_root_shots(ui)
	await _spec_wait(ui)
	print("RG_SPEC stock: %s" % str(ui.garage.get_working_spec()))
	await _shot("spec_stock")
	var vehicle := str(OS.get_cmdline_user_args()[OS.get_cmdline_user_args().find("--garage-shots-vehicle") + 1]) if OS.get_cmdline_user_args().has("--garage-shots-vehicle") else ""
	for swap in SPEC_SWAPS.get(vehicle, []):
		_stage = "spec " + str(swap[0])
		await _back_to_root_shots(ui)
		var edits: Dictionary = swap[1]
		for id in edits:
			ui._edit_option(str(id), edits[id])
		await _spec_wait(ui)
		print("RG_SPEC after %s: %s" % [str(swap[0]), str(ui.garage.get_working_spec())])
		for step in swap[2]:
			_press(step)
			await _frames(3)
		await _seconds(0.5)
		var tiles: Array = []
		_collect_tiles(ui._option_box, tiles)
		if tiles.size() > HINT_TILES_PER_PAGE:
			tiles = tiles.slice(0, HINT_TILES_PER_PAGE)
		for tile in tiles:
			var row: Container = tile.get_node_or_null("Badges")
			if row != null:
				ui._fill_badges(row, str(tile.get_meta("badge_option")), tile.get_meta("badge_edits"))
				ui.garage.wait_for_hints(600000) # the newest request cancels the previous one's work: one tile at a time
		var texts := []
		for tile in tiles:
			var row: Container = tile.get_node_or_null("Badges")
			if row == null:
				continue
			ui._fill_badges(row, str(tile.get_meta("badge_option")), tile.get_meta("badge_edits"))
			row.visible = true
			var chips := []
			for chip in row.get_children():
				chips.append(str(chip.text))
			var label: Label = tile.get_node_or_null("Name")
			print("RG_SPEC tile '%s'%s -> [%s]" % [label.text if label != null else "?", " (installed)" if tile.has_node("Installed") else "", " | ".join(chips)])
		await _seconds(0.5)
		await _shot("spec_after_" + str(swap[0]))
	await _back_to_root_shots(ui)

# One heavier (red) and one lighter (green) weight tile highlighted by focus, the badge at its bottom middle.
func _badge_shots(ui: CanvasLayer, name: String) -> void:
	var scroll: ScrollContainer = ui._option_box.get_parent()
	if ui._advice_box != null and is_instance_valid(ui._advice_box):
		ui._advice_box.visible = false
	await _seconds(0.3)
	var want := {"red": true, "green": true}
	var tiles: Array = []
	_collect_tiles(ui._option_box, tiles)
	for tile in tiles:
		for badge in ui.garage.get_hints(str(tile.get_meta("badge_option")), tile.get_meta("badge_edits"))["badges"]:
			var colour := "red" if bool(badge["worse"]) else "green"
			if not want.get(colour, false):
				continue
			want[colour] = false
			scroll.ensure_control_visible(tile)
			await _frames(2)
			tile.grab_focus()
			await _frames(3)
			await _shot("%s_%s" % [name, colour])
	if ui._advice_box != null and is_instance_valid(ui._advice_box):
		ui._advice_box.visible = true

func _collect_tiles(node: Node, out: Array) -> void:
	for child in node.get_children():
		if child is Button and child.has_meta("badge_option"):
			out.append(child)
		else:
			_collect_tiles(child, out)
