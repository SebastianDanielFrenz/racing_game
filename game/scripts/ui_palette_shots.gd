extends Node
# game/scripts/ui_palette_shots.gd - `--ui-palette-shots <dir>` with `--flat --vehicle car_hyper_n2o` (needs a real window):
# walks the real screens of one running game (HUD, cockpit cluster, pause, settings, controls, car browser, configurator)
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
