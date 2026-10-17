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
	var dyno_deadline := Time.get_ticks_msec() + 120000
	while main.get_shell_ui().garage.get_dyno().get("busy",false) and Time.get_ticks_msec() < dyno_deadline:
		await _frames(1)
	await _seconds(0.25)
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
