extends Node3D
var simulation: Node
var _batches := {false: [], true: []}
var _visibility_elapsed := 0.0
var _panel: PanelContainer
var _status: Label
var _controls: Array[HSlider] = []
var _labels: Array[Label] = []
var _configured_session := -1
var _settings := ConfigFile.new()
var _canvas: CanvasLayer

func _ready() -> void:
 _settings.load("user://traffic.cfg")
 if int(_settings.get_value("traffic", "scale_version", 0)) < 1:
  _settings.set_value("traffic", "0", maxf(120.0, float(_settings.get_value("traffic", "0", 120.0))))
  _settings.set_value("traffic", "1", maxf(1200.0, float(_settings.get_value("traffic", "1", 1200.0))))
  _settings.set_value("traffic", "4", 2048.0)
  _settings.set_value("traffic", "scale_version", 1)
 _build_batches()
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
 var names := ["Density / lane-km", "Spawn radius (m)", "Minimum spawn distance (m)", "Grip factor (dry=1, rain/snow/ice lower)", "Maximum active NPCs"]
 var defaults := [120.0, 1200.0, 100.0, 1.0, 2048.0]
 var minimum := [0.0, 200.0, 30.0, 0.05, 0.0]
 var maximum := [1000.0, 3000.0, 500.0, 1.0, 4096.0]
 for i in range(5):
  var label := Label.new()
  label.text = names[i]
  box.add_child(label)
  _labels.append(label)
  var slider := HSlider.new()
  slider.min_value = minimum[i]
  slider.max_value = maximum[i]
  slider.step = 0.05 if i == 3 else (1.0 if i == 0 or i == 4 else 10.0)
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
 if _controls.size() != 5 or simulation == null:
  return
 _controls[2].set_value_no_signal(minf(_controls[2].value, _controls[1].value - 30.0))
 _controls[2].max_value = _controls[1].value - 30.0
 simulation.configure_traffic(_controls[0].value, _controls[1].value, _controls[2].value, _controls[3].value, int(_controls[4].value))
 for i in range(5):
  _settings.set_value("traffic", str(i), _controls[i].value)
  _labels[i].text = "%s: %.2f" % [_controls[i].tooltip_text, _controls[i].value]
 _settings.save("user://traffic.cfg")

func _part(truck: bool, size: Vector3, at: Vector3, color: Color, rubber := false) -> void:
 var node := MultiMeshInstance3D.new()
 var mesh: Mesh
 if rubber:
  var cylinder := CylinderMesh.new()
  cylinder.top_radius = size.x
  cylinder.bottom_radius = size.x
  cylinder.height = size.y
  mesh = cylinder
 else:
  var box := BoxMesh.new()
  box.size = size
  mesh = box
 var material := StandardMaterial3D.new()
 material.albedo_color = color
 material.vertex_color_use_as_albedo = true
 material.roughness = 0.75
 mesh.material = material
 var multi := MultiMesh.new()
 multi.transform_format = MultiMesh.TRANSFORM_3D
 multi.use_colors = true
 multi.mesh = mesh
 node.multimesh = multi
 add_child(node)
 _batches[truck].append({"multi": multi, "local": Transform3D(Basis.IDENTITY, at)})

func _build_batches() -> void:
 _part(false, Vector3(4.6,2,0.8), Vector3.ZERO, Color.WHITE)
 _part(false, Vector3(2.4,1.8,0.7), Vector3(-0.25,0,0.6), Color(0.12,0.2,0.25))
 for x in [-1.45,1.45]:
  for y in [-1.05,1.05]:
   _part(false, Vector3(0.35,0.2,0), Vector3(x,y,-0.45), Color(0.04,0.04,0.04), true)
 _part(true, Vector3(10.5,2.5,3.1), Vector3(-1.25,0,0), Color(0.82,0.84,0.88))
 _part(true, Vector3(2.7,2.5,2.4), Vector3(5.15,0,-0.3), Color(0.12,0.3,0.64))
 _part(true, Vector3(0.04,2.15,0.8), Vector3(6.52,0,0.2), Color(0.08,0.16,0.22))
 _part(true, Vector3(12.8,2.25,0.35), Vector3(0,0,-1.25), Color(0.1,0.12,0.14))
 for x in [-5.1,-3.9,3.8,5.4]:
  for y in [-1.28,1.28]:
   _part(true, Vector3(0.5,0.3,0), Vector3(x,y,-1.7), Color(0.035,0.035,0.035), true)

 for y in [-0.95,0.95]:
  _part(true, Vector3(0.04,0.35,0.2), Vector3(6.52,y,-0.8), Color(1,0.95,0.75))
  _part(true, Vector3(0.04,0.3,0.2), Vector3(-6.52,y,-0.8), Color(0.9,0.035,0.025))

func _draw_batch(truck: bool, actors: Array) -> void:
 for part: Dictionary in _batches[truck]:
  var multi: MultiMesh = part["multi"]
  if actors.size() > multi.instance_count:
   var capacity := 1
   while capacity < actors.size():
    capacity *= 2
   multi.instance_count = capacity
  multi.visible_instance_count = actors.size()
 for i in range(actors.size()):
  var actor: Dictionary = actors[i]
  var pose: Transform3D = actor["transform"]
  var color := Color.WHITE if truck else Color.from_hsv(fmod(float(actor["id"])*0.173,1.0),0.55,0.75)
  for part: Dictionary in _batches[truck]:
   var multi: MultiMesh = part["multi"]
   multi.set_instance_transform(i, pose * part["local"])
   multi.set_instance_color(i, color)

func _process(delta: float) -> void:
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
 var cars: Array = []
 var trucks: Array = []
 var visible_ids: Array = []
 _visibility_elapsed += delta
 var report_visibility := _visibility_elapsed >= 0.1
 var camera := get_viewport().get_camera_3d()
 for actor: Dictionary in state.get("actors", []):
  if actor["truck"]:
   trucks.append(actor)
  else:
   cars.append(actor)
  if report_visibility and camera != null:
   var pose: Transform3D = actor["transform"]
   var in_view := camera.is_position_in_frustum(pose.origin)
   if not in_view:
    var half := 6.6 if actor["truck"] else 2.4
    for x in [-half,half]:
     in_view = in_view or camera.is_position_in_frustum(pose * Vector3(x,0,0))
   if in_view:
    visible_ids.append(actor["id"])
 _draw_batch(false,cars)
 _draw_batch(true,trucks)
 if report_visibility:
  simulation.set_visible_traffic(visible_ids)
  _visibility_elapsed = 0.0
 _status.text = "%d active / %d target (cap %d) | %d queued" % [cars.size()+trucks.size(), int(state.get("target",0)), int(state.get("maximum",2048)), int(state.get("queued",0))]

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
