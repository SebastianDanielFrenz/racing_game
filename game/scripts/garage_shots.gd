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

func _run() -> void:
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
	_press("choose")
	await _seconds(2.0)
	await _shot("overview")
	await _seconds(3.0)
	await _shot("turntable_rotated")
	_press("area:wheels")
	await _seconds(2.4)
	await _shot("upgrade_wheels")
	_press("tyre:family:track")
	await _seconds(0.5)
	await _shot("tyre_widths")
	_press("garage:tuning")
	await _seconds(0.5)
	await _shot("edit_wheels")
	_press("area:engine")
	await _seconds(2.4)
	await _shot("edit_engine")
	_press("garage:upgrades")
	await _seconds(0.5)
	await _shot("upgrade_engine")
	main._on_escape()
	await _seconds(0.25)
	await _shot("garage_exit")
	main._on_escape()
	_press("garage:tuning")
	_press("area:rear")
	await _seconds(2.4)
	await _shot("edit_rear")
	print("RG_GARAGE_SHOT done")
	get_tree().quit()
