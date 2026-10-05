extends Node
# game/scripts/car_browser_shots.gd - `--car-browser-shots <dir>` (needs a real window, not
# --headless; tools/car_browser_shots.ps1): drives the real shell to the car browser and saves
# the window as PNGs (PLAN.md R6c):
#   browser_by_body_type.png   the browser grouped by body type, thumbnails rendered
#   browser_filter_panel.png   the filter/sort panel open
#   browser_big_catalog.png    (with --car-browser-big and --catalog <synthetic>) hundreds of cars
#   change_car_in_world.png    the car after an in-world "Change car"
# With --car-browser-big only the big-catalog shot is taken. Waits until the thumbnail renderer
# has nothing pending so the pictures show real previews.

var main: Node
var out_dir: String = ""
var big := false

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
	await RenderingServer.frame_post_draw
	var image := get_viewport().get_texture().get_image()
	var path := out_dir.path_join(name + ".png")
	var err := image.save_png(path)
	print("RG_CAR_BROWSER_SHOT %s err=%d size=%s" % [path, err, image.get_size()])

func _press(id: String) -> void:
	var ui: CanvasLayer = main.get_shell_ui()
	var button: Button = ui.get_button(id)
	if button == null:
		print("RG_CAR_BROWSER_SHOT FAIL no button %s (has %s)" % [id, ", ".join(ui.button_ids())])
		return
	button.pressed.emit()

func _wait_thumbs(timeout_s: float) -> void:
	var deadline := Time.get_ticks_msec() + int(timeout_s * 1000.0)
	while Time.get_ticks_msec() < deadline:
		var t: Node = main.get_thumbnails()
		if t != null and t.pending_count() == 0:
			await _frames(6)
			if t.pending_count() == 0:
				return
		await get_tree().process_frame

func _key(keycode: Key) -> void:
	for pressed in [true, false]:
		var ev := InputEventKey.new()
		ev.keycode = keycode
		ev.physical_keycode = keycode
		ev.pressed = pressed
		Input.parse_input_event(ev)

func _run() -> void:
	DirAccess.make_dir_recursive_absolute(out_dir)
	await _seconds(0.5)
	main._apply_transition(main.get_shell().boot_finished())
	await _frames(5)
	main._apply_transition(main.get_shell().menu_item("garage"))
	await _frames(10)
	var ui: CanvasLayer = main.get_shell_ui()
	var browser: Control = ui.browser()
	print("RG_CAR_BROWSER_SHOT sizes browser=%s parent=%s window=%s" % [browser.size, browser.get_parent().size, get_viewport().get_visible_rect().size])
	if big:
		browser.set_group_by("none")
		await _seconds(1.0)
		await _wait_thumbs(60.0)
		await _shot("browser_big_catalog")
		for i in range(40):
			_key(KEY_RIGHT)
		await _seconds(1.2)
		await _wait_thumbs(60.0)
		await _shot("browser_big_catalog_scrolled")
		print("RG_CAR_BROWSER_SHOT tiles=%d listed=%d" % [browser.instantiated_tile_count(), browser.listed_ids().size()])
		print("RG_CAR_BROWSER_SHOT done")
		get_tree().quit()
		return
	browser.set_group_by("body_type")
	await _seconds(1.0)
	await _wait_thumbs(60.0)
	await _shot("browser_by_body_type")
	browser.open_panel()
	await _seconds(0.6)
	await _shot("browser_filter_panel")
	browser.close_panel()
	browser.set_group_by("drive_layout")
	# the in-world switch: Free roam -> flat world, then pause -> Change car
	_key(KEY_ESCAPE)
	await _frames(5)
	main._apply_transition(main.get_shell().menu_item("free_roam"))
	await _frames(5)
	main._apply_transition(main.get_shell().spawn_picked("flat"))
	var deadline := Time.get_ticks_msec() + 120000
	while not (main.shell_screen() == "drive" and main.world_state == "running") and Time.get_ticks_msec() < deadline:
		await get_tree().process_frame
	await _seconds(2.5)
	_key(KEY_ESCAPE)
	await _seconds(0.6)
	_press("change_car")
	await _seconds(0.8)
	await _wait_thumbs(60.0)
	browser = ui.browser()
	browser.focus_car("car_sedan_awd_rally")
	await _seconds(0.6)
	await _shot("change_car_browser")
	_press("choose")
	deadline = Time.get_ticks_msec() + 120000
	while not (main.shell_screen() == "drive" and main.world_state == "running") and Time.get_ticks_msec() < deadline:
		await get_tree().process_frame
	await _seconds(3.0)
	await _shot("change_car_in_world")
	print("RG_CAR_BROWSER_SHOT done")
	get_tree().quit()
