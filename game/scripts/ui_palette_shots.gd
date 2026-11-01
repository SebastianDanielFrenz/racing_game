extends Node
# game/scripts/ui_palette_shots.gd - `--ui-palette-shots <dir>` with `--flat --vehicle car_hyper_n2o` (needs a real window):
# walks the real screens of one running game (HUD, cockpit cluster, the language's components over the scene, pause, settings,
# controls, car browser, configurator)
# and, on each screen, swaps the palette at runtime without rebuilding the screen, one PNG per screen and palette:
#   <screen>__<palette>.png
# Before the design language existed this ran with the one palette "today" (the pictures the `racing` palette is compared
# with); the script keeps that path, it only asks for the theme layer when the file is there.

const THEME_PATH := "res://scripts/ui/ui_theme.gd"
const TIMEOUT_S := 300.0

var main: Node
var out_dir: String = ""
var _ui_theme: Variant = null
var _cluster: Control
var _cluster_layer: CanvasLayer

func _ready() -> void:
	_run.call_deferred()

func _frames(n: int) -> void:
	for _i in range(n):
		await get_tree().process_frame

func _seconds(s: float) -> void:
	var deadline := Time.get_ticks_msec() + int(s * 1000.0)
	while Time.get_ticks_msec() < deadline:
		await get_tree().process_frame

func _palettes() -> Array:
	if _ui_theme == null:
		return ["today"]
	return Array(_ui_theme.palette_ids())

# Every palette on the screen that stands now: nothing is rebuilt between two pictures.
func _shots(screen_name: String) -> void:
	for palette: String in _palettes():
		if _ui_theme != null:
			var error: String = _ui_theme.set_palette(palette)
			if error != "":
				print("RG_UI_SHOT FAIL %s: %s" % [palette, error])
		await _frames(4)
		await RenderingServer.frame_post_draw
		var path := out_dir.path_join("%s__%s.png" % [screen_name, palette])
		var err := get_viewport().get_texture().get_image().save_png(path)
		print("RG_UI_SHOT %s err=%d" % [path, err])

func _press(id: String) -> bool:
	var ui: CanvasLayer = main.get_shell_ui()
	var button: Button = ui.get_button(id)
	if button == null:
		print("RG_UI_SHOT FAIL no button %s (has %s)" % [id, ", ".join(ui.button_ids())])
		return false
	button.pressed.emit()
	return true

func _run() -> void:
	var guard := get_tree().create_timer(TIMEOUT_S)
	guard.timeout.connect(func():
		print("RG_UI_SHOT FAIL timeout")
		get_tree().quit(1))
	DirAccess.make_dir_recursive_absolute(out_dir)
	if ResourceLoader.exists(THEME_PATH):
		_ui_theme = load(THEME_PATH)
	var simulation: Node = main.get_simulation()
	while not (simulation.is_running() and simulation.get_step_count() > 240):
		await _frames(2)
	await _seconds(2.0)
	await _shots("hud")
	# The cockpit's digital cluster, drawn flat over the HUD (in the car it is a screen in the dashboard).
	_cluster_layer = CanvasLayer.new()
	_cluster_layer.layer = 20
	add_child(_cluster_layer)
	_cluster = load("res://scripts/digital_cluster.gd").new()
	_cluster.simulation = simulation
	_cluster.vehicle_name = main.VEHICLE_NAME
	_cluster.position = Vector2(448, 300)
	_cluster_layer.add_child(_cluster)
	await _seconds(0.5)
	await _shots("cluster")
	_cluster_layer.queue_free()
	if _ui_theme != null:
		await _components()
	main._on_escape()
	await _seconds(0.5)
	print("RG_UI_SHOT pause buttons: %s" % ", ".join(main.get_shell_ui().button_ids()))
	await _shots("pause")
	if await _walk_settings():
		main._on_escape()
		await _seconds(0.5)
	if _press("garage"):
		await _seconds(3.0)
		await _shots("car_browser")
		if _press("choose"):
			await _seconds(2.0)
			var deadline := Time.get_ticks_msec() + 120000
			while main.get_shell_ui().garage.get_dyno().get("busy", false) and Time.get_ticks_msec() < deadline:
				await _frames(2)
			await _seconds(1.0)
			await _shots("configurator")
	print("RG_UI_SHOT done")
	get_tree().quit(0)

# A white shape on transparent, the way a tab's icon arrives.
func _icon(points: int) -> ImageTexture:
	var image := Image.create(44, 44, false, Image.FORMAT_RGBA8)
	for x in range(44):
		for y in range(44):
			var at := Vector2(x - 21.5, y - 21.5)
			var edge := 16.0 * (1.0 + 0.3 * cos(points * at.angle()))
			if absf(at.length() - edge) < 2.0:
				image.set_pixel(x, y, Color.WHITE)
	return ImageTexture.create_from_image(image)

# A striped picture, the way a tile's thumbnail arrives: the palette must leave it alone.
func _picture(a: Color, b: Color) -> ImageTexture:
	var image := Image.create(240, 150, false, Image.FORMAT_RGBA8)
	for x in range(240):
		for y in range(150):
			image.set_pixel(x, y, a if int((x + y) / 30.0) % 2 == 0 else b)
	return ImageTexture.create_from_image(image)

# Panel, key prompts, dial, tab strip and tiles standing on the running scene, in the places of the owner's two sketches.
func _components() -> void:
	var layer := CanvasLayer.new()
	layer.layer = 21
	add_child(layer)
	var panel: Control = load("res://scripts/ui/ui_panel.gd").new()
	panel.position = Vector2(28, 28)
	var column := VBoxContainer.new()
	var prompt: Script = load("res://scripts/ui/ui_key_prompt.gd")
	column.add_child(prompt.new("L", "Turn on lights"))
	column.add_child(prompt.new("Q", "Hitch"))
	var unavailable: Node = prompt.new("Space", "Fold the arms")
	unavailable.set_available(false)
	column.add_child(unavailable)
	panel.add_child(column)
	layer.add_child(panel)
	var dial: Control = load("res://scripts/ui/ui_gauge.gd").new()
	dial.position = Vector2(700, 380)
	dial.size = Vector2(300, 300)
	dial.max_value = 2800.0
	dial.major_step = 400.0
	dial.figure_divisor = 100.0
	dial.value = 1900.0
	dial.gear = "4"
	dial.readout = "18"
	dial.caption = "km/h"
	layer.add_child(dial)
	var red_dial: Control = load("res://scripts/ui/ui_gauge.gd").new()
	red_dial.position = Vector2(500, 500)
	red_dial.size = Vector2(180, 180)
	red_dial.max_value = 8000.0
	red_dial.major_step = 1000.0
	red_dial.figure_divisor = 1000.0
	red_dial.value = 7600.0
	red_dial.gear = "N"
	red_dial.alarm = true
	layer.add_child(red_dial)
	var tabs: Control = load("res://scripts/ui/ui_tab_strip.gd").new()
	tabs.position = Vector2(760, 28)
	layer.add_child(tabs)
	tabs.add_tab(&"shop", _icon(5), "Shop")
	tabs.add_tab(&"fields", _icon(4), "Fields")
	tabs.add_tab(&"machines", _icon(6), "Machines")
	var plain: Button = tabs.add_tab(&"money", null, "Money")
	tabs.select(&"fields")
	plain.grab_focus()
	var grid := GridContainer.new()
	grid.columns = 3
	grid.position = Vector2(28, 320)
	var tile: Script = load("res://scripts/ui/ui_image_tile.gd")
	grid.add_child(tile.new(_picture(Color("#6f8f3a"), Color("#c9b458")), "Tractor"))
	var hovered: Control = tile.new(_picture(Color("#3a6f8f"), Color("#58c9b4")))
	grid.add_child(hovered)
	var muted: Control = tile.new(_picture(Color("#8f3a6f"), Color("#c958b4")), "Locked")
	muted.set_available(false)
	grid.add_child(muted)
	layer.add_child(grid)
	hovered.grab_focus()
	await _seconds(0.3)
	await _shots("components")
	layer.queue_free()

func _walk_settings() -> bool:
	if not _press("settings"):
		return false
	await _seconds(0.5)
	await _shots("settings")
	if _press("controls"):
		await _seconds(0.6)
		await _shots("controls")
		main._on_escape()
		await _seconds(0.4)
	return true
