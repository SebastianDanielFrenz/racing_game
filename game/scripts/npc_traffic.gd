extends Node3D
var simulation: Node
var _actors := {}
var _panel: PanelContainer
var _status: Label
var _controls: Array[HSlider] = []
var _labels: Array[Label] = []
var _configured_session := -1
var _settings := ConfigFile.new()
var _canvas: CanvasLayer

func _ready() -> void:
 _settings.load("user://traffic.cfg")
 _canvas = CanvasLayer.new()
 _canvas.layer = 31
 add_child(_canvas)
 _panel = PanelContainer.new()
 _panel.position = Vector2(24, 310)
 _canvas.add_child(_panel)
 var box := VBoxContainer.new()
 box.add_theme_constant_override("separation", 10)
 _panel.add_child(box)
 var title := Label.new()
 title.text = "NPC traffic (F7 / Escape closes)"
 box.add_child(title)
 var names := ["Density / lane-km", "Spawn radius (m)", "Minimum spawn distance (m)", "Grip factor (dry=1, rain/snow/ice lower)"]
 var defaults := [4.0, 600.0, 100.0, 1.0]
 var minimum := [0.0, 200.0, 30.0, 0.05]
 var maximum := [30.0, 1200.0, 500.0, 1.0]
 for i in range(4):
  var label := Label.new()
  label.text = names[i]
  box.add_child(label)
  _labels.append(label)
  var slider := HSlider.new()
  slider.min_value = minimum[i]
  slider.max_value = maximum[i]
  slider.step = 0.05 if i == 3 else (0.5 if i == 0 else 10.0)
  slider.custom_minimum_size = Vector2(460, 28)
  slider.value = float(_settings.get_value("traffic", str(i), defaults[i]))
  slider.tooltip_text = names[i]
  box.add_child(slider)
  _controls.append(slider)
  slider.value_changed.connect(func(_value: float): _configure())
 _status = Label.new()
 box.add_child(_status)
 var note := Label.new()
 note.text = "Cars and trucks use OSM destinations. Zero density removes\ntraffic after it leaves view. Changes are saved. Driving stays active."
 box.add_child(note)
 var close := Button.new()
 close.text = "Close"
 close.pressed.connect(func(): _toggle(false))
 box.add_child(close)
 _panel.hide()

func _configure() -> void:
 if _controls.size() != 4 or simulation == null:
  return
 _controls[2].set_value_no_signal(minf(_controls[2].value, _controls[1].value - 30.0))
 _controls[2].max_value = _controls[1].value - 30.0
 simulation.configure_traffic(_controls[0].value, _controls[1].value, _controls[2].value, _controls[3].value)
 for i in range(4):
  _settings.set_value("traffic", str(i), _controls[i].value)
  _labels[i].text = "%s: %.2f" % [_controls[i].tooltip_text, _controls[i].value]
 _settings.save("user://traffic.cfg")

func _car(id: int) -> Node3D:
 var root := Node3D.new()
 # Native root uses ISO axes: forward X, left Y, up Z.
 var color := Color.from_hsv(fmod(float(id)*0.173,1.0),0.55,0.75)
 for part in [[Vector3.ZERO, Vector3(4.6,2.0,0.8),color], [Vector3(-0.25,0,0.6),Vector3(2.4,1.8,0.7),Color(0.12,0.2,0.25)]]:
  var mesh := MeshInstance3D.new()
  var shape := BoxMesh.new()
  shape.size = part[1]
  mesh.mesh = shape
  mesh.position = part[0]
  var material := StandardMaterial3D.new()
  material.albedo_color = part[2]
  mesh.material_override = material
  root.add_child(mesh)
 for x in [-1.45,1.45]:
  for y in [-1.05,1.05]:
   var wheel := MeshInstance3D.new()
   var cylinder := CylinderMesh.new()
   cylinder.top_radius = 0.35
   cylinder.bottom_radius = 0.35
   cylinder.height = 0.2
   wheel.mesh = cylinder
   wheel.position = Vector3(x,y,-0.45)
   var rubber := StandardMaterial3D.new()
   rubber.albedo_color = Color(0.035,0.035,0.035)
   wheel.material_override = rubber
   root.add_child(wheel)
 return root

func _process(_delta: float) -> void:
 if simulation == null:
  return
 var ticks: int = simulation.get_step_count()
 if ticks > 0:
  if ticks < _configured_session or _configured_session < 0:
   _configure()
  _configured_session = ticks
 else:
  _configured_session = -1
 var state: Dictionary = simulation.get_traffic_state()
 var live := {}
 var visible_ids: Array = []
 var camera := get_viewport().get_camera_3d()
 for actor: Dictionary in state.get("actors", []):
  var id := int(actor["id"])
  live[id] = true
  if not _actors.has(id):
   var model: Node3D
   if actor["truck"]:
    model = preload("res://scripts/npc_truck.gd").new()
    model.managed_externally = true
   else:
    model = _car(id)
   add_child(model)
   _actors[id] = model
  var model: Node3D = _actors[id]
  model.global_transform = actor["transform"]
  if camera != null:
   # Keep actors with any representative body corner in view, not just their center.
   var in_view := camera.is_position_in_frustum(model.global_position)
   var half_length := 6.6 if actor["truck"] else 2.4
   for x in [-half_length, half_length]:
    for y in [-1.3, 1.3]:
     for z in [-1.8, 1.8]:
      in_view = in_view or camera.is_position_in_frustum(model.global_transform * Vector3(x,y,z))
   if in_view:
    visible_ids.append(id)
 for id in _actors.keys():
  if not live.has(id):
   _actors[id].queue_free()
   _actors.erase(id)
 simulation.set_visible_traffic(visible_ids)
 _status.text = "%d vehicles | %s" % [_actors.size(),str(state.get("message", ""))]

func _toggle(open: bool) -> void:
 _panel.visible = open
 if open:
  add_to_group("traffic_settings_open") # Mouse/UI capture only; driving remains live.
  Input.mouse_mode = Input.MOUSE_MODE_VISIBLE
  _controls[0].grab_focus()
 else:
  remove_from_group("traffic_settings_open")
  get_viewport().gui_release_focus()

func _input(event: InputEvent) -> void:
 if event is InputEventKey and event.pressed and not event.echo:
  if event.keycode == KEY_F7 or (_panel.visible and event.keycode == KEY_ESCAPE):
   _toggle(not _panel.visible)
   get_viewport().set_input_as_handled()
