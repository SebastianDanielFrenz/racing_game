extends CanvasLayer
# Seat translation in chassis ISO axes; never modifies physical car geometry.
var body_visuals: Node
var vehicle_name := "car_hyper"
const DEFAULT := Vector3(0.20, 0.0, 0.0)
const SAVE_PATH := "user://seat_positions.cfg"
var _panel: PanelContainer
var _sliders: Array[HSlider] = []
var _labels: Array[Label] = []
var _config := ConfigFile.new()

func _ready() -> void:
	layer = 30
	_config.load(SAVE_PATH)
	_panel = PanelContainer.new()
	_panel.position = Vector2(24, 320)
	_panel.custom_minimum_size = Vector2(420, 0)
	add_child(_panel)
	var box := VBoxContainer.new()
	box.add_theme_constant_override("separation", 12)
	_panel.add_child(box)
	var title := Label.new()
	title.text = "Driver seat / eye position"
	box.add_child(title)
	var names := ["Fore / aft (+ toward wheel)", "Left / right (+ left)", "Height (+ up)"]
	var lower := [-0.20, -0.15, -0.20]
	var upper := [0.45, 0.15, 0.20]
	var saved: Variant = _config.get_value(vehicle_name, "offset", DEFAULT)
	var offset: Vector3 = saved if saved is Vector3 and saved.is_finite() else DEFAULT
	for axis in range(3):
		var label := Label.new()
		box.add_child(label)
		_labels.append(label)
		var slider := HSlider.new()
		slider.min_value = lower[axis]
		slider.max_value = upper[axis]
		slider.step = 0.005
		slider.custom_minimum_size = Vector2(380, 28)
		slider.value = clampf(offset[axis], lower[axis], upper[axis])
		slider.tooltip_text = names[axis]
		box.add_child(slider)
		_sliders.append(slider)
		slider.value_changed.connect(func(_value: float): _apply())
	var note := Label.new()
	note.text = "Live adjustment; saved per car. Brake held while open.\nF6 / Escape closes. Tab and arrows also adjust sliders."
	box.add_child(note)
	var reset := Button.new()
	reset.text = "Reset seat"
	reset.pressed.connect(_reset)
	box.add_child(reset)
	var close := Button.new()
	close.text = "Close"
	close.pressed.connect(func(): _set_open(false))
	box.add_child(close)
	_apply(false)
	_panel.hide()

func is_open() -> bool:
	return _panel != null and _panel.visible

func _apply(save := true) -> void:
	var offset := Vector3(_sliders[0].value, _sliders[1].value, _sliders[2].value)
	body_visuals.seat_offset = offset
	var names := ["Fore / aft", "Left / right", "Height"]
	for axis in range(3):
		_labels[axis].text = "%s: %+.1f cm" % [names[axis], offset[axis] * 100.0]
	if save:
		_config.set_value(vehicle_name, "offset", offset)
		var error := _config.save(SAVE_PATH)
		if error != OK:
			push_warning("Seat settings could not be saved: %s" % error_string(error))

func _reset() -> void:
	for axis in range(3):
		_sliders[axis].set_value_no_signal(DEFAULT[axis])
	_apply()

func _set_open(open: bool) -> void:
	_panel.visible = open
	if open:
		add_to_group("seat_adjustment_open")
		Input.mouse_mode = Input.MOUSE_MODE_VISIBLE
		_sliders[0].grab_focus()
	else:
		remove_from_group("seat_adjustment_open")
		get_viewport().gui_release_focus()
		# Leaving the cursor visible avoids a stale mouse-capture flag; RMB resumes look.
		Input.mouse_mode = Input.MOUSE_MODE_VISIBLE

func _input(event: InputEvent) -> void:
	if event is InputEventKey and event.pressed and not event.echo:
		if event.keycode == KEY_F6 or (is_open() and event.keycode == KEY_ESCAPE):
			_set_open(not is_open())
			get_viewport().set_input_as_handled()

