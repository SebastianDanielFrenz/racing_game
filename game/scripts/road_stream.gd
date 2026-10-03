extends Node3D
# CPU road geometry on a worker; rendering resources on the main thread.
var view: Node
var simulation: Node
var director: Node
var settings: Dictionary = {}
var utm_origin := Vector2.ZERO
var _thread := Thread.new()
var _mutex := Mutex.new()
var _wake := Semaphore.new()
var _stop := false
var _jobs: Array = []
var _results: Array = []
var _known: Dictionary = {}
var _tiles: Dictionary = {}
var _center := Vector2i(2147483647,2147483647)
var _material := ShaderMaterial.new()
func _ready() -> void:
	process_priority = 101
	settings["distance_m"] = clampf(float(settings.get("distance_m",2500.0)),512.0,6000.0)
	settings["surface_lift_m"] = clampf(float(settings.get("surface_lift_m",.018)),.005,.04)
	_material.shader = preload("res://shaders/road.gdshader")
	_material.set_shader_parameter("line_width_m",clampf(float(settings.get("line_width_m",.15)),.08,.3))
	_thread.start(_worker)
func shutdown() -> void:
	_mutex.lock()
	_stop = true
	_jobs.clear()
	_mutex.unlock()
	_wake.post()
	if _thread.is_started():
		_thread.wait_to_finish()
	set_process(false)
func _exit_tree() -> void:
	shutdown()
func _process(_delta: float) -> void:
	var rig: Node = director.active_rig()
	if rig==null:
		return
	var focus: Vector3 = view.godot_to_session(rig.camera.global_position)
	var absolute := Vector2(focus.x,focus.y)+utm_origin
	var center := Vector2i(floori(absolute.x/1024.0),floori(absolute.y/1024.0))
	if center!=_center:
		_schedule(center,absolute)
		_center = center
	_mutex.lock()
	var result: Dictionary = {}
	if not _results.is_empty():
		result = _results.pop_front()
	_mutex.unlock()
	if not result.is_empty():
		_upload(result)
		_wake.post()
	var origin: Vector3 = simulation.get_render_origin_session()
	for key in _tiles.keys():
		var tile: Dictionary = _tiles[key]
		var s: Vector3 = tile.origin-origin
		tile.node.position = Vector3(-s.y,s.z,-s.x)
		var distance: float = (Vector2(tile.origin.x+512,tile.origin.y+512)-Vector2(focus.x,focus.y)).length()
		tile.node.visible = distance<=float(settings.distance_m)+725.0
		if distance>float(settings.distance_m)+2048.0:
			tile.node.queue_free()
			_tiles.erase(key)
			_known.erase(key)
func _schedule(center: Vector2i,focus: Vector2) -> void:
	var radius := ceili(float(settings.distance_m)/1024.0)+1
	var jobs: Array = []
	for y in range(center.y-radius,center.y+radius+1):
		for x in range(center.x-radius,center.x+radius+1):
			var key := Vector2i(x,y)
			var distance := (Vector2(x*1024+512,y*1024+512)-focus).length()
			if distance<=float(settings.distance_m)+725.0 and not _known.has(key):
				jobs.append({"key":key,"distance":distance})
	jobs.sort_custom(func(a: Dictionary,b: Dictionary) -> bool: return a.distance>b.distance)
	_mutex.lock()
	_jobs = jobs
	_mutex.unlock()
	_wake.post()
func _worker() -> void:
	while true:
		_mutex.lock()
		var stop := _stop
		var job: Dictionary = {}
		if not stop and not _jobs.is_empty() and _results.size()<2:
			job = _jobs.pop_back()
		_mutex.unlock()
		if stop:
			return
		if job.is_empty():
			_wake.wait()
			continue
		var data: Dictionary = view.get_road_visual_tile(job.key.x,job.key.y,float(settings.surface_lift_m))
		data["key"] = job.key
		_mutex.lock()
		_results.append(data)
		_mutex.unlock()
func _upload(data: Dictionary) -> void:
	var key: Vector2i = data.key
	_known[key] = true
	if data.get("vertices",PackedVector3Array()).is_empty():
		return
	if _tiles.has(key):
		_tiles[key].node.queue_free()
	var arrays: Array = []
	arrays.resize(Mesh.ARRAY_MAX)
	arrays[Mesh.ARRAY_VERTEX] = data.vertices
	arrays[Mesh.ARRAY_NORMAL] = data.normals
	arrays[Mesh.ARRAY_TEX_UV] = data.uv
	arrays[Mesh.ARRAY_TEX_UV2] = data.uv2
	arrays[Mesh.ARRAY_COLOR] = data.colours
	arrays[Mesh.ARRAY_INDEX] = data.indices
	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES,arrays)
	mesh.surface_set_material(0,_material)
	var node := MeshInstance3D.new()
	node.mesh = mesh
	node.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	add_child(node)
	_tiles[key] = {"node":node,"origin":data.origin_session}
