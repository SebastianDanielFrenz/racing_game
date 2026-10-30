extends CanvasLayer
# R4 navigation HUD (thin): the minimap and the next manoeuvres. rg_core's
# RgNavigation (rg/route_planner.h, rg/map_view.h) builds the routing graph,
# plans routes, tracks progress and serves the map geometry; this layer only
# forwards the car pose, draws and prints. It follows the world itself: it
# attaches while a real world runs and releases as soon as it does not, so
# main.gd only creates it. A destination comes from the F10 address search
# ("Set as route destination") or set_destination().

const WEATHER_CLEARANCE := 244.0 # weather_widget.gd moves this far from the screen edge, left of the minimap

var main: Node
var config_path := ""
var _nav: Node
var _minimap: Control
var _label: Label
var _attached := false
var _failed := false
var _lod_choice: Dictionary = {}
var _destination: Variant = null
var _destination_label := ""

func _ready() -> void:
	layer = 15
	_nav = ClassDB.instantiate("RgNavigation")
	_nav.name = "RgNavigation"
	add_child(_nav)
	_minimap = Control.new()
	_minimap.set_script(load("res://scripts/minimap.gd"))
	_minimap.anchor_left = 1.0
	_minimap.anchor_right = 1.0
	_minimap.offset_left = -236.0
	_minimap.offset_right = -16.0
	_minimap.offset_top = 16.0
	_minimap.offset_bottom = 236.0
	add_child(_minimap)
	_label = Label.new()
	_label.anchor_left = 1.0
	_label.anchor_right = 1.0
	_label.offset_left = -236.0
	_label.offset_right = -16.0
	_label.offset_top = 240.0
	_label.autowrap_mode = TextServer.AUTOWRAP_WORD_SMART
	add_child(_label)
	_set_shown(false)
	if main != null and main.get("_teleport_dialog") != null and main._teleport_dialog.has_signal("destination_chosen"):
		main._teleport_dialog.destination_chosen.connect(set_destination)

func set_view_lod(choice: Dictionary) -> void:
	_lod_choice = choice
	_nav.set_view_lod(choice)

func set_destination(point: Vector2, label: String = "") -> void:
	_destination = point
	_destination_label = label
	_nav.clear_route()
	_minimap.set_route(PackedVector2Array())
	_plan()

func _set_shown(shown: bool) -> void:
	_minimap.visible = shown
	_label.visible = shown
	if main != null and main.get("_weather_widget") != null:
		main._weather_widget.set_right_margin(WEATHER_CLEARANCE if shown else main._weather_widget.RIGHT_MARGIN)

func _plan() -> void:
	if not _attached or _destination == null:
		return
	var pose: Dictionary = main._simulation.get_chassis_session_pose()
	if pose.is_empty():
		return
	var yaw := deg_to_rad(float(pose.yaw_deg))
	var serial: int = _nav.plan_route(float(pose.x), float(pose.y), cos(yaw), sin(yaw), _destination.x, _destination.y)
	print("RG_ROUTE request serial=%d to=%s (%s)" % [serial, _destination, _destination_label])

var _guidance_wait_s := 0.0 # guidance text and map requests at 10 Hz (perf, 2026-10-15); the car marker every frame

func _process(delta: float) -> void:
	var running: bool = main != null and main.world_state == "running" and main.world_kind == "real_world"
	if not running:
		if _attached or _failed:
			_nav.release()
			_attached = false
			_failed = false
			_set_shown(false)
		return
	if _failed:
		return
	var pose: Dictionary = main._simulation.get_chassis_session_pose()
	if pose.is_empty():
		return
	var car := Vector2(float(pose.x), float(pose.y))
	if not _attached:
		if not _nav.initialize_shared(main._simulation, config_path, car.x, car.y):
			push_warning("Navigation: %s" % _nav.get_last_error())
			_failed = true
			return
		_attached = true
		_nav.set_view_lod(_lod_choice)
		_minimap.radius_m = float(_nav.get_minimap_radius_m())
		_set_shown(true)
		_plan()
	var focus := car
	var drone: Dictionary = main._simulation.get_fpv_state()
	if drone.is_empty():
		_minimap.set_car(car, float(pose.yaw_deg))
	else:
		focus = _drone_session_xy(drone)
		_minimap.set_drone(focus, rad_to_deg(float(drone.yaw_rad)), car, float(pose.yaw_deg))
	_guidance_wait_s -= delta
	if _guidance_wait_s > 0.0:
		return
	_guidance_wait_s = 0.1
	_nav.request_map(focus.x, focus.y, _minimap.radius_m)
	if _nav.poll_map():
		_minimap.set_map(_nav.get_map())
	if _nav.poll_route():
		var route: Dictionary = _nav.get_route()
		_minimap.set_route(route.get("polyline", PackedVector2Array()))
		print("RG_ROUTE %s length_m=%.0f duration_s=%.0f maneuvers=%d %s" % [route.status, float(route.length_m), float(route.duration_s), route.maneuvers.size(), route.message])
	_label.text = _guidance_text(_nav.get_progress(car.x, car.y))

# get_fpv_state's position is relative to the render origin, in Godot axes: session x = -z, session y = -x.
func _drone_session_xy(drone: Dictionary) -> Vector2:
	var origin: Vector3 = main._simulation.get_render_origin_session()
	var position: Vector3 = drone.position
	return Vector2(origin.x - position.z, origin.y - position.x)

func _guidance_text(progress: Dictionary) -> String:
	if not bool(progress.get("valid", false)):
		var graph: Dictionary = _nav.get_graph_status()
		if str(graph.get("state", "")) == "building":
			return "Map: loading roads..."
		return "" if _destination == null else "No route"
	if bool(progress.off_route):
		return "Off route (%.0f m)" % float(progress.off_route_m)
	var lines: Array = []
	for m in progress.next:
		var road: String = str(m.ref) if str(m.name).is_empty() else str(m.name)
		var text := "%s %.0f m: %s" % ["In" if lines.is_empty() else "then", float(m.distance_m), str(m.kind).replace("_", " ")]
		if int(m.exit_number) > 0:
			text += " (exit %d)" % int(m.exit_number)
		if not road.is_empty():
			text += " - " + road
		lines.append(text)
	lines.append("%.1f km to go" % (float(progress.remaining_m) / 1000.0))
	return "\n".join(lines)
