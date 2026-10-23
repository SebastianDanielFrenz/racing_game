extends Node3D
# Placement and procedural appearance are generated in geo2map. This script only streams/uploads meshes.
var view: Node
var simulation: Node
var director: Node
var settings: Dictionary = {}
var utm_origin := Vector2.ZERO
var _thread := Thread.new()
var _mutex := Mutex.new()
var _wake := Semaphore.new()
var _stop := false
var _results: Array = []
var _tiles: Dictionary = {}
# Which tiles to build at which tier, show, downgrade and drop: geo2map's
# RingStreamer (full within ordinary_distance_m + 725 m, skyline-only within
# skyline_distance_m + 725 m; downgraded / evicted beyond the distance + 2048 m).
var _ring := RgRingStreamer.new()
var _materials := preload("res://scripts/building_materials.gd").shared()
var _placed_origin := Vector3.ZERO
func _ready() -> void:
	process_priority = 100
	settings["ordinary_distance_m"] = clampf(float(settings.get("ordinary_distance_m",2200.0)),256.0,10000.0)
	settings["skyline_distance_m"] = clampf(float(settings.get("skyline_distance_m",40000.0)),settings.ordinary_distance_m,80000.0)
	settings["skyline_min_height_m"] = clampf(float(settings.get("skyline_min_height_m",50.0)),10.0,1000.0)
	settings["fallback_height_m"] = clampf(float(settings.get("fallback_height_m",8.0)),1.0,100.0)
	settings["storey_height_m"] = clampf(float(settings.get("storey_height_m",3.0)),1.0,10.0)
	_ring.configure({"full_m":float(settings.ordinary_distance_m),"far_m":float(settings.skyline_distance_m),"empty_keeps_resident":false})
	_thread.start(_worker)
func shutdown() -> void:
	_mutex.lock()
	_stop = true
	_mutex.unlock()
	_wake.post()
	if _thread.is_started():
		_thread.wait_to_finish()
	set_process(false)
func _exit_tree() -> void:
	shutdown()
const FrameProbe := preload("res://scripts/frame_probe.gd")
func _process(_delta: float) -> void:
	var probe_t0 := Time.get_ticks_usec()
	_process_body(_delta)
	FrameProbe.add("building_stream", Time.get_ticks_usec() - probe_t0)
func _process_body(_delta: float) -> void:
	var rig: Node = director.active_rig()
	if rig == null:
		return
	var cam: Vector3 = view.godot_to_session(rig.camera.global_position)
	var led: Vector2 = director.render_focus_session() if director.render_focus_valid() else Vector2(cam.x, cam.y)
	var focus := Vector3(led.x, led.y, cam.z)
	var velocity: Vector2 = director.render_velocity_session() if director.render_focus_valid() else Vector2.ZERO
	var absolute := Vector2(focus.x, focus.y) + utm_origin
	if _ring.update(absolute.x, absolute.y, velocity.x, velocity.y):
		_wake.post()
	var skyline := float(settings.get("skyline_distance_m", 40000.0))
	rig.camera.far = maxf(rig.camera.far, skyline + 3000.0)
	_mutex.lock()
	var result: Dictionary = {}
	if not _results.is_empty():
		result = _results.pop_front()
	_mutex.unlock()
	if not result.is_empty():
		_upload(result)
		_wake.post()
	for change: Dictionary in _ring.frame(absolute.x, absolute.y):
		var key := Vector2i(int(change.x), int(change.y))
		if not _tiles.has(key):
			continue
		var tile: Dictionary = _tiles[key]
		if change.evicted:
			tile.node.queue_free()
			_tiles.erase(key)
			continue
		if change.downgraded:
			tile.near.mesh = null
		tile.near.visible = change.near
		tile.tall.visible = change.far
	# A new tile is placed in _upload; every tile moves only when the render origin changes (no per-frame loop over the
	# resident tiles, which reach the 40 km skyline ring).
	var origin: Vector3 = simulation.get_render_origin_session()
	if origin != _placed_origin:
		_placed_origin = origin
		for tile: Dictionary in _tiles.values():
			_place(tile)
func _place(tile: Dictionary) -> void:
	var s: Vector3 = tile.origin - _placed_origin
	tile.node.position = Vector3(-s.y, s.z, -s.x)
func _worker() -> void:
	while true:
		_mutex.lock()
		var stop := _stop
		var job: Dictionary = {}
		if not stop and _results.size()<4:
			job = _ring.next_job()
		_mutex.unlock()
		if stop:
			return
		if job.is_empty():
			_wake.wait()
			continue
		var threshold := float(settings.get("skyline_min_height_m",50.0))
		var data: Dictionary = view.get_building_mesh_tile(int(job.x),int(job.y),0.0 if job.full else threshold,float(settings.get("fallback_height_m",8.0)),float(settings.get("storey_height_m",3.0)),threshold)
		var near_arrays: Array = data.get("near",_empty_arrays())
		var tall_arrays: Array = data.get("tall",_empty_arrays())
		# RenderingServer calls (including tangent readback) can wait for the main
		# thread. Build outside the queue mutex: the main thread polls that mutex.
		var near_mesh := _mesh(near_arrays)
		var tall_mesh := _mesh(tall_arrays)
		_mutex.lock()
		_results.append({"key":Vector2i(int(job.x),int(job.y)),"full":bool(job.full),"origin":data.get("origin_session",Vector3.ZERO),"near":near_mesh,"tall":tall_mesh})
		_mutex.unlock()
func _empty_arrays() -> Array:
	return []
func _mesh(surfaces: Array) -> ArrayMesh:
	if surfaces.is_empty():
		return null
	var mesh := ArrayMesh.new()
	for record: Dictionary in surfaces:
		var arrays: Array = record.arrays
		if arrays[0].is_empty():
			continue
		var surface: Array = []
		surface.resize(Mesh.ARRAY_MAX)
		surface[Mesh.ARRAY_VERTEX] = arrays[0]
		surface[Mesh.ARRAY_NORMAL] = arrays[1]
		surface[Mesh.ARRAY_COLOR] = arrays[2]
		surface[Mesh.ARRAY_TEX_UV] = arrays[3]
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES,surface)
		mesh.surface_set_material(mesh.get_surface_count()-1,_materials[int(record.material)])
	if mesh.get_surface_count() == 0:
		return null
	mesh.regen_normal_maps()
	return mesh
func _upload(result: Dictionary) -> void:
	var key: Vector2i = result.key
	var has_content: bool = result.near != null or result.tall != null
	var accepted: int = _ring.accept(key.x, key.y, bool(result.full), has_content)
	if accepted==0:
		return
	if _tiles.has(key):
		_tiles[key].node.queue_free()
		_tiles.erase(key)
	if accepted!=2:
		return
	var near_mesh: ArrayMesh = result.near
	var tall_mesh: ArrayMesh = result.tall
	var root := Node3D.new()
	var near_node := MeshInstance3D.new()
	var tall_node := MeshInstance3D.new()
	near_node.mesh = near_mesh
	tall_node.mesh = tall_mesh
	near_node.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	tall_node.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	root.add_child(near_node)
	root.add_child(tall_node)
	add_child(root)
	_tiles[key] = {"node":root,"near":near_node,"tall":tall_node,"origin":result.origin,"full":result.full}
	_place(_tiles[key])
