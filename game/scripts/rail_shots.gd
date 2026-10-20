extends SceneTree
# Railways (RACE-028) screenshots at the owner spot (UTM32 E467577.92, N5550232.49): the rail bridge over the road.
# Run windowed (needs the rail world config: RG_WORLD_CONFIG=data/world/world_config_rhein_main_hessen_rail.json):
#   godot --path game --script res://scripts/rail_shots.gd
# It finds the crossing from the meshes (car on the road at the spot, rail mesh above it), takes the shots
# into out/rail_shots/ and drives a car through the opening in both directions (RG_RAIL_SHOTS_EXPECT=blocked: a run with the
# naive collision config must NOT get through, the recorded sabotage).
const SPOT_E := 467577.92
const SPOT_N := 5550232.49
var main: Node
var camera: Camera3D
const start_back_m := 26.0 # the drive-through starts this far before the crossing (45 m hit an unrelated obstacle on the SE side)
func _initialize() -> void:
	_run.call_deferred()
func _godot(p: Vector3, origin: Vector3) -> Vector3:
	var d := p - origin
	return Vector3(-d.y, d.z, -d.x)
# Principal axis (unit, in the E/N plane) of points.
func _axis(points: Array) -> Vector2:
	var mean := Vector2.ZERO
	for p: Vector2 in points:
		mean += p
	mean /= maxf(1.0, float(points.size()))
	var sxx := 0.0
	var sxy := 0.0
	var syy := 0.0
	for p: Vector2 in points:
		var d := p - mean
		sxx += d.x * d.x
		sxy += d.x * d.y
		syy += d.y * d.y
	var angle := 0.5 * atan2(2.0 * sxy, sxx - syy)
	return Vector2(cos(angle), sin(angle))
# Session-frame (E, N, up) points of a mesh dictionary (the Godot-axis vertices of get_*_tile) within `radius` of c.
func _points(data: Dictionary, c: Vector2, radius: float) -> Array:
	var out: Array = []
	if data.is_empty():
		return out
	var o: Vector3 = data.origin_session
	for v: Vector3 in data.vertices:
		var p := Vector3(o.x - v.z, o.y - v.x, v.y)
		if Vector2(p.x, p.y).distance_to(c) <= radius:
			out.append(p)
	return out
func _shot(name: String, eye: Vector3, target: Vector3, origin: Vector3, directory: String) -> void:
	camera.position = _godot(eye, origin)
	camera.look_at(_godot(target, origin), Vector3.UP)
	await process_frame
	await process_frame
	await RenderingServer.frame_post_draw
	var path := directory.path_join(name + ".png")
	var result := root.get_texture().get_image().save_png(path)
	print("RG_RAIL_SHOTS file=%s eye=%s target=%s result=%d" % [path, str(eye), str(target), result])
func _ground(sim: Node, p: Vector2, origin: Vector3) -> float:
	var h: Variant = sim.get_camera_ground_height(_godot(Vector3(p.x, p.y, 0.0), origin))
	if h == null:
		return NAN
	# get_camera_ground_height is in Godot y (render frame): back to session up
	return float(h) + origin.z
func _run() -> void:
	main = load("res://scenes/main.tscn").instantiate()
	root.add_child(main)
	var began := Time.get_ticks_msec()
	while main.world_state != "running":
		if main.world_state == "failed" or Time.get_ticks_msec() - began > 900000:
			push_error("RG_RAIL_SHOTS world startup failed")
			quit(1)
			return
		await process_frame
	main.scripted_controls = {"throttle": 0.0, "brake": 1.0, "handbrake": 1.0, "steer": 0.0}
	main.get_director().set_process(false)
	camera = Camera3D.new()
	camera.fov = 65
	camera.near = .2
	camera.far = 25000
	main.add_child(camera)
	camera.current = true
	var sim: Node = main.get_simulation()
	var view: Node = main.get_world_view()
	var info: Dictionary = view.get_rail_info()
	print("RG_RAIL_SHOTS rail_info=%s" % str(info))
	if not bool(info.get("enabled", false)):
		push_error("RG_RAIL_SHOTS the world has no rail layer (RG_WORLD_CONFIG must be the _rail config)")
		quit(1)
		return
	var directory := OS.get_environment("RG_RAIL_SHOTS_OUT") if OS.get_environment("RG_RAIL_SHOTS_OUT") != "" else ProjectSettings.globalize_path("res://../out/rail_shots")
	DirAccess.make_dir_recursive_absolute(directory)
	var utm_origin := Vector2(464000.0, 5559000.0)
	var spot := Vector2(SPOT_E, SPOT_N) - utm_origin
	var before: int = sim.get_streaming_status().get("relocations", 0)
	sim.relocate_vehicle(spot.x, spot.y, 150)
	while int(sim.get_streaming_status().get("relocations", 0)) == before:
		await process_frame
	var origin: Vector3 = sim.get_render_origin_session()
	view.set_render_origin(origin)
	view.update_focus(spot.x, spot.y)
	for frame in range(30):
		await process_frame
	while not view.is_stream_idle():
		await process_frame
	# Hide the HUD, minimap and widgets: the shots are about the geometry.
	for child in main.get_children():
		if child is CanvasLayer:
			child.visible = false
	# The crossing from the car: it stands on the road at the spot (the road axis comes from the ribbon around it), the
	# rail bridge is the rail mesh above it.
	var car: Vector3 = main.chassis_session_position()
	var car_xy := Vector2(car.x, car.y)
	var tx := int(floor(SPOT_E / 1024.0))
	var ty := int(floor(SPOT_N / 1024.0))
	var rail: Dictionary = view.get_rail_tile(tx, ty)
	print("RG_RAIL_SHOTS rail_tile=%d,%d vertices=%d car=%s" % [tx, ty, rail.get("vertices", PackedVector3Array()).size(), str(car)])
	var above: Array = []
	for p: Vector3 in _points(rail, car_xy, 45.0):
		if p.z > car.z + 1.5:
			above.append(p)
	if above.is_empty():
		push_error("RG_RAIL_SHOTS no rail mesh above the car within 45 m")
		quit(1)
		return
	var nearest: Vector3 = above[0]
	for p: Vector3 in above:
		if Vector2(p.x, p.y).distance_to(car_xy) < Vector2(nearest.x, nearest.y).distance_to(car_xy):
			nearest = p
	var centre := Vector2.ZERO
	var near_count := 0
	var rail_z_max := -1e9
	var rail_xy: Array = []
	for p: Vector3 in above:
		var xy := Vector2(p.x, p.y)
		if xy.distance_to(Vector2(nearest.x, nearest.y)) <= 8.0:
			centre += xy
			near_count += 1
			rail_z_max = maxf(rail_z_max, p.z)
		if xy.distance_to(Vector2(nearest.x, nearest.y)) <= 30.0:
			rail_xy.append(xy)
	centre /= float(near_count)
	var rail_axis := _axis(rail_xy)
	var road: Dictionary = view.get_road_visual_tile(tx, ty, 0.0)
	var road_xy: Array = []
	for p: Vector3 in _points(road, car_xy, 16.0):
		if absf(p.z - car.z) < 1.5:
			road_xy.append(Vector2(p.x, p.y))
	var road_axis := _axis(road_xy) if road_xy.size() > 4 else Vector2(-rail_axis.y, rail_axis.x)
	# Orient the road axis so that it points from the car toward the crossing.
	if road_axis.dot(centre - car_xy) < 0.0:
		road_axis = -road_axis
	var deck_underside := rail_z_max - 0.35 - 0.9
	var road_at_centre := _ground(sim, centre, origin)
	print("RG_RAIL_SHOTS crossing=%s rail_axis=%s road_axis=%s road_points=%d rail_top=%.2f underside=%.2f car_z=%.2f ground_at_crossing=%.2f clearance_over_car_z=%.2f" % [str(centre + utm_origin), str(rail_axis), str(road_axis), road_xy.size(), rail_z_max, deck_underside, car.z, road_at_centre, deck_underside - car.z])
	var road_here := car.z
	# 1: standing on the road 55 m before the crossing, looking along it through the opening.
	var back := centre - road_axis * 55.0
	await _shot("01_road_through_opening", Vector3(back.x, back.y, maxf(_ground(sim, back, origin), road_here) + 1.8), Vector3(centre.x, centre.y, road_here + 3.0), origin, directory)
	# 2: from the side: 45 m along the road, 25 m off to the side, 5 m up, looking at the deck centre.
	var side := centre - road_axis * 45.0 + rail_axis * 25.0
	await _shot("02_side", Vector3(side.x, side.y, _ground(sim, side, origin) + 5.0), Vector3(centre.x, centre.y, deck_underside + 1.0), origin, directory)
	# 3, 4: along the rail from both sides, 4 m above the rail top: ballast, rails, parapets on the approach and the deck.
	var along := centre - rail_axis * 60.0
	await _shot("03_along_the_rail", Vector3(along.x, along.y, rail_z_max + 4.0), Vector3(centre.x, centre.y, rail_z_max), origin, directory)
	var along2 := centre + rail_axis * 60.0
	await _shot("04_along_the_rail_back", Vector3(along2.x, along2.y, rail_z_max + 4.0), Vector3(centre.x, centre.y, rail_z_max), origin, directory)
	# 5: from above at an angle, the whole crossing.
	var high := centre - road_axis * 35.0 + rail_axis * 35.0
	await _shot("05_overview", Vector3(high.x, high.y, rail_z_max + 25.0), Vector3(centre.x, centre.y, road_here + 2.0), origin, directory)
	# The deck must not block the road: put the car on the road 45 m before the crossing, aimed along it, drive through.
	var expect := OS.get_environment("RG_RAIL_SHOTS_EXPECT")
	var yaw := rad_to_deg(atan2(road_axis.y, road_axis.x))
	var passed_count := 0
	for direction: float in [1.0, -1.0]:
		var s := centre - road_axis * start_back_m * direction
		var heading := yaw + (0.0 if direction > 0.0 else 180.0)
		before = sim.get_streaming_status().get("relocations", 0)
		sim.relocate_vehicle(s.x, s.y, heading)
		while int(sim.get_streaming_status().get("relocations", 0)) == before:
			await process_frame
		main.scripted_controls = {"throttle": 0.0, "steer": 0.0, "brake": 0.0, "handbrake": 0.0, "clutch": 0.0, "ignition": 1.0, "assist.auto_shift": 1.0}
		for frame in range(240):
			await process_frame
		var start_car: Vector3 = main.chassis_session_position()
		main.scripted_controls = {"throttle": 0.5, "steer": 0.0, "brake": 0.0, "handbrake": 0.0, "clutch": 0.0, "ignition": 1.0, "assist.auto_shift": 1.0}
		var t0 := Time.get_ticks_msec()
		var passed := false
		var along_road := 0.0
		while Time.get_ticks_msec() - t0 < 25000:
			await process_frame
			var now_car: Vector3 = main.chassis_session_position()
			along_road = (Vector2(now_car.x, now_car.y) - centre).dot(road_axis) * direction
			if along_road > 30.0:
				passed = true
				break
		var car_end: Vector3 = main.chassis_session_position()
		print("RG_RAIL_SHOTS drive_through direction=%d passed=%s start=%s end=%s along_road_m=%.1f elapsed_ms=%d" % [int(direction), str(passed), str(start_car), str(car_end), along_road, Time.get_ticks_msec() - t0])
		main.scripted_controls = {"throttle": 0.0, "brake": 1.0, "handbrake": 1.0, "steer": 0.0}
		passed_count += 1 if passed else 0
	var verdict := "passed_both" if passed_count == 2 else ("blocked" if passed_count == 0 else "passed_one")
	var ok := (expect == "blocked" and passed_count < 2) or (expect != "blocked" and passed_count == 2)
	print("RG_RAIL_SHOTS result=%s verdict=%s expect=%s" % ["ok" if ok else "fail", verdict, expect if expect != "" else "passed_both"])
	quit(0 if ok else 2)
