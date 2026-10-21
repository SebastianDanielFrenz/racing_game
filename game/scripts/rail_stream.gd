extends Node3D
# Railways (RACE-028): bridge deck + parapets over the road, ballast bed + rails on the ground, from the served
# g2m.rail.geom layer. The mesh is built in core (rg/rail_meshes.h, numbers in data/world/rail.json); this node only
# streams level-2 tiles like road_stream.gd: a worker asks RgTerrainView.get_rail_tile for the tiles geo2map's
# RingStreamer wants, the main thread attaches them. Vertex colours carry the materials (deck, parapet, ballast, rail).
var view: Node
var simulation: Node
var director: Node
var utm_origin := Vector2.ZERO
var _thread := Thread.new()
var _mutex := Mutex.new()
var _wake := Semaphore.new()
var _stop := false
var _results: Array = []
var _tiles: Dictionary = {}
var _ring := RgRingStreamer.new()
var _material := StandardMaterial3D.new()
var _placed_origin := Vector3.ZERO
var _distance_m := 2500.0
func _ready() -> void:
	process_priority = 101
	var info: Dictionary = view.get_rail_info()
	if not bool(info.get("enabled", false)):
		set_process(false)
		return
	_distance_m = clampf(float(info.get("view_distance_m", 2500.0)), 512.0, 6000.0)
	_ring.configure({"full_m": _distance_m, "empty_keeps_resident": true})
	_material.vertex_color_use_as_albedo = true
	_material.roughness = 0.9
	_material.metallic = 0.0
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
func _process(_delta: float) -> void:
	var rig: Node = director.active_rig()
	if rig == null or not _thread.is_started():
		return
	var cam: Vector3 = view.godot_to_session(rig.camera.global_position)
	var led: Vector2 = director.render_focus_session() if director.render_focus_valid() else Vector2(cam.x, cam.y)
	var velocity: Vector2 = director.render_velocity_session() if director.render_focus_valid() else Vector2.ZERO
	var absolute := led + utm_origin
	if _ring.update(absolute.x, absolute.y, velocity.x, velocity.y):
		_wake.post()
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
		if change.evicted:
			_tiles[key].node.queue_free()
			_tiles.erase(key)
		else:
			_tiles[key].node.visible = change.near
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
		if not stop and _results.size() < 2:
			job = _ring.next_job()
		_mutex.unlock()
		if stop:
			return
		if job.is_empty():
			_wake.wait()
			continue
		var data: Dictionary = view.get_rail_tile(int(job.x), int(job.y))
		data["key"] = Vector2i(int(job.x), int(job.y))
		data["mesh"] = _make_mesh(data)
		_mutex.lock()
		_results.append(data)
		_mutex.unlock()
func _upload(data: Dictionary) -> void:
	var key: Vector2i = data.key
	var has_content: bool = data.get("mesh") != null
	if _ring.accept(key.x, key.y, true, has_content) != 2:
		return
	if _tiles.has(key):
		_tiles[key].node.queue_free()
	var node := MeshInstance3D.new()
	node.mesh = data.mesh
	add_child(node)
	_tiles[key] = {"node": node, "origin": data.origin_session}
	_place(_tiles[key])
func _make_mesh(data: Dictionary) -> ArrayMesh:
	if data.get("vertices", PackedVector3Array()).is_empty():
		return null
	var arrays: Array = []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = data.vertices
	arrays[Mesh.ARRAY_NORMAL] = data.normals
	arrays[Mesh.ARRAY_COLOR] = data.colours
	arrays[Mesh.ARRAY_INDEX] = data.indices
	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, arrays)
	mesh.surface_set_material(0, _material)
	return mesh
