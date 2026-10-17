extends Node3D
const SurfaceMaterials := preload("res://scripts/surface_materials.gd")
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
var _results: Array = []
var _tiles: Dictionary = {}
# Which tiles to build, show and drop: geo2map's RingStreamer (one tier: built
# within distance_m + 725 m, dropped beyond + 2048 m, nearest first, behind a
# moving car last; a tile built empty stays known).
var _ring := RgRingStreamer.new()
var _material := ShaderMaterial.new()
# Streaming statistics (RG_ROAD_STREAM): the focus/velocity of the last frame.
var _velocity := Vector2.ZERO
var _camera_abs := Vector2.ZERO
var _tiles_built_total := 0
var _placed_origin := Vector3.ZERO # the render origin every resident tile is placed against
func _ready() -> void:
	process_priority = 101
	settings["distance_m"] = clampf(float(settings.get("distance_m",2500.0)),512.0,6000.0)
	settings["surface_lift_m"] = clampf(float(settings.get("surface_lift_m",.018)),.005,.04)
	_ring.configure({"full_m":float(settings.distance_m),"empty_keeps_resident":true})
	_material.shader = preload("res://shaders/road.gdshader")
	SurfaceMaterials.shared().apply_road(_material)
	_material.set_shader_parameter("line_width_m",clampf(float(settings.get("line_width_m",.15)),.08,.3))
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
	FrameProbe.add("road_stream", Time.get_ticks_usec() - probe_t0)
func _process_body(_delta: float) -> void:
	var rig: Node = director.active_rig()
	if rig==null:
		return
	# The streaming focus is led by the car's velocity while the rig follows the
	# car (camera_director.gd, rg/render_focus.h); the drop-off distances below
	# stay relative to it.
	var cam: Vector3 = view.godot_to_session(rig.camera.global_position)
	var led: Vector2 = director.render_focus_session() if director.render_focus_valid() else Vector2(cam.x,cam.y)
	var focus := Vector3(led.x,led.y,cam.z)
	_velocity = director.render_velocity_session() if director.render_focus_valid() else Vector2.ZERO
	_camera_abs = Vector2(cam.x,cam.y)+utm_origin
	var absolute := Vector2(focus.x,focus.y)+utm_origin
	if _ring.update(absolute.x,absolute.y,_velocity.x,_velocity.y):
		_wake.post()
	_mutex.lock()
	var result: Dictionary = {}
	if not _results.is_empty():
		result = _results.pop_front()
	_mutex.unlock()
	if not result.is_empty():
		_upload(result)
		_wake.post()
	for change: Dictionary in _ring.frame(absolute.x,absolute.y):
		var key := Vector2i(int(change.x),int(change.y))
		if not _tiles.has(key):
			continue
		if change.evicted:
			_tiles[key].node.queue_free()
			_tiles.erase(key)
		else:
			_tiles[key].node.visible = change.near
	# A new tile is placed in _upload; every tile moves only when the render origin changes (no per-frame tile loop).
	var origin: Vector3 = simulation.get_render_origin_session()
	if origin != _placed_origin:
		_placed_origin = origin
		for tile: Dictionary in _tiles.values():
			_place(tile)
func _place(tile: Dictionary) -> void:
	var s: Vector3 = tile.origin-_placed_origin
	tile.node.position = Vector3(-s.y,s.z,-s.x)
func _worker() -> void:
	while true:
		_mutex.lock()
		var stop := _stop
		var job: Dictionary = {}
		if not stop and _results.size()<2:
			job = _ring.next_job()
		_mutex.unlock()
		if stop:
			return
		if job.is_empty():
			_wake.wait()
			continue
		var data: Dictionary = view.get_road_visual_tile(int(job.x),int(job.y),float(settings.surface_lift_m))
		data["key"] = Vector2i(int(job.x),int(job.y))
		# Packing the arrays into a mesh (format, compression, AABB) is the 20-50 ms part of a tile: here, not in the
		# main thread's _process (RenderingServer queues the GPU part for the main thread). Attaching it there is cheap.
		data["mesh"] = _make_mesh(data)
		_mutex.lock()
		_results.append(data)
		_mutex.unlock()
func _upload(data: Dictionary) -> void:
	var key: Vector2i = data.key
	_tiles_built_total += 1
	var has_content: bool = not data.get("vertices",PackedVector3Array()).is_empty()
	if _ring.accept(key.x,key.y,true,has_content)!=2:
		return
	if _tiles.has(key):
		_tiles[key].node.queue_free()
	var node := MeshInstance3D.new()
	node.mesh = data.mesh # built on the worker (_make_mesh)
	node.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	add_child(node)
	_tiles[key] = {"node":node,"origin":data.origin_session}
	_place(_tiles[key])
func _make_mesh(data: Dictionary) -> ArrayMesh:
	if data.get("vertices",PackedVector3Array()).is_empty():
		return null
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
	return mesh

# {pending (queued + built but not yet uploaded), oldest_age_s (longest a wanted
# tile has waited), ribbon_ahead_m (distance along `direction` - session XY, any
# length - from the car until the first ribbon tile that is neither built nor
# known empty), tiles (resident), built_total}. Meant for a 1 Hz log line.
func get_stats(car_session: Vector2,direction: Vector2) -> Dictionary:
	_mutex.lock()
	var pending := _ring.pending_jobs()+_results.size()
	_mutex.unlock()
	var oldest := _ring.oldest_wanted_age_s()
	var known := _ring.known_tiles()
	var absolute := car_session+utm_origin
	var ahead := RgRenderFocus.grid_coverage_ahead(known,1024.0,absolute.x,absolute.y,direction.x,direction.y,float(settings.distance_m),64.0)
	return {"pending":pending,"oldest_age_s":oldest,"ribbon_ahead_m":ahead,"tiles":_tiles.size(),"built_total":_tiles_built_total}
