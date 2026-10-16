extends Node3D
# OSM placement is independent of the replaceable appearance layer.
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
var _material := StandardMaterial3D.new()
var _placed_origin := Vector3.ZERO # the render origin every resident tile is placed against
func _ready() -> void:
	process_priority = 100
	settings["ordinary_distance_m"] = clampf(float(settings.get("ordinary_distance_m",2200.0)),256.0,10000.0)
	settings["skyline_distance_m"] = clampf(float(settings.get("skyline_distance_m",40000.0)),settings.ordinary_distance_m,80000.0)
	settings["skyline_min_height_m"] = clampf(float(settings.get("skyline_min_height_m",50.0)),10.0,1000.0)
	settings["fallback_height_m"] = clampf(float(settings.get("fallback_height_m",8.0)),1.0,100.0)
	settings["storey_height_m"] = clampf(float(settings.get("storey_height_m",3.0)),1.0,10.0)
	_ring.configure({"full_m":float(settings.ordinary_distance_m),"far_m":float(settings.skyline_distance_m),"empty_keeps_resident":false})
	_material.vertex_color_use_as_albedo = true
	_material.roughness = 1.0
	_material.cull_mode = BaseMaterial3D.CULL_DISABLED
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
		var data: Dictionary = view.get_building_tile(int(job.x),int(job.y),0.0 if job.full else threshold,float(settings.get("fallback_height_m",8.0)),float(settings.get("storey_height_m",3.0)))
		var near_arrays := _empty_arrays()
		var tall_arrays := _empty_arrays()
		for building: Dictionary in data.get("buildings",[]):
			_tessellate(building,tall_arrays if float(building.height)>=threshold else near_arrays)
		_mutex.lock()
		# The meshes (format, compression, AABB) are built here, not on the main thread; _upload only attaches them.
		_results.append({"key":Vector2i(int(job.x),int(job.y)),"full":bool(job.full),"origin":data.get("origin_session",Vector3.ZERO),"near":_mesh(near_arrays),"tall":_mesh(tall_arrays)})
		_mutex.unlock()
func _empty_arrays() -> Array:
	return [PackedVector3Array(),PackedVector3Array(),PackedColorArray()]
func _vertex(arrays: Array,point: Vector2,z: float,normal: Vector3,colour: Color) -> void:
	arrays[0].append(Vector3(-point.y,z,-point.x))
	arrays[1].append(normal)
	arrays[2].append(colour)
func _tessellate(building: Dictionary,arrays: Array) -> void:
	var seed_value := posmod(int(building.id),19)/19.0
	var colour := Color(.55+seed_value*.15,.53+seed_value*.13,.49+seed_value*.12)
	if Color.html_is_valid(str(building.colour)):
		colour = Color.html(str(building.colour))
	var top := float(building.base)+float(building.height)
	var bottom := float(building.bottom)
	var rings: Array = building.outers.duplicate()
	rings.append_array(building.inners)
	for ring_index in rings.size():
		var ring: PackedVector2Array = rings[ring_index]
		var area := 0.0
		for i in ring.size():
			var a := ring[i]
			var b := ring[(i+1)%ring.size()]
			area += a.x*b.y-b.x*a.y
		var facing := (1.0 if area>=0 else -1.0)*(1.0 if ring_index<building.outers.size() else -1.0)
		for i in ring.size():
			var a := ring[i]
			var b := ring[(i+1)%ring.size()]
			if a.distance_squared_to(b)<.0001:
				continue
			var normal := Vector3(-(b.y-a.y),0,-(b.x-a.x)).cross(Vector3.UP).normalized()*facing
			for v: Array in [[a,bottom],[b,bottom],[b,top],[a,bottom],[b,top],[a,top]]:
				_vertex(arrays,v[0],v[1],normal,colour)
	for outer: PackedVector2Array in building.outers:
		var holes: Array = []
		for inner: PackedVector2Array in building.inners:
			if Geometry2D.is_point_in_polygon(inner[0],outer):
				holes.append(inner)
		if not holes.is_empty():
			_roof_with_holes(outer,holes,top,colour.darkened(.25),arrays)
			continue
		var triangles := Geometry2D.triangulate_polygon(outer)
		for index in triangles:
			_vertex(arrays,outer[index],top,Vector3.UP,colour.darkened(.25))
func _roof_with_holes(outer: PackedVector2Array,holes: Array,top: float,colour: Color,arrays: Array) -> void:
	# Horizontal slabs use an even/odd fill: exact concave contours and courtyards,
	# without unconstrained triangulation crossing a boundary or covering a hole.
	var rings: Array = [outer]
	rings.append_array(holes)
	var ys: Array[float] = []
	for ring: PackedVector2Array in rings:
		for p in ring:
			ys.append(p.y)
	ys.sort()
	for slab in range(ys.size()-1):
		var lo := ys[slab]
		var hi := ys[slab+1]
		if hi-lo < .00001:
			continue
		var mid := (lo+hi)*.5
		var crossings: Array = []
		for ring: PackedVector2Array in rings:
			for i in ring.size():
				var a := ring[i]
				var b := ring[(i+1)%ring.size()]
				if mid <= minf(a.y,b.y) or mid >= maxf(a.y,b.y):
					continue
				var slope := (b.x-a.x)/(b.y-a.y)
				crossings.append({"mid":a.x+(mid-a.y)*slope,"lo":a.x+(lo-a.y)*slope,"hi":a.x+(hi-a.y)*slope})
		crossings.sort_custom(func(a: Dictionary,b: Dictionary) -> bool: return a.mid<b.mid)
		for i in range(0,crossings.size()-1,2):
			var left: Dictionary = crossings[i]
			var right: Dictionary = crossings[i+1]
			var a := Vector2(left.lo,lo)
			var b := Vector2(right.lo,lo)
			var c := Vector2(right.hi,hi)
			var d := Vector2(left.hi,hi)
			for point: Vector2 in [a,b,c,a,c,d]:
				_vertex(arrays,point,top,Vector3.UP,colour)
func _mesh(arrays: Array) -> ArrayMesh:
	if arrays[0].is_empty():
		return null
	var surface: Array = []
	surface.resize(Mesh.ARRAY_MAX)
	surface[Mesh.ARRAY_VERTEX] = arrays[0]
	surface[Mesh.ARRAY_NORMAL] = arrays[1]
	surface[Mesh.ARRAY_COLOR] = arrays[2]
	var mesh := ArrayMesh.new()
	mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES,surface)
	mesh.surface_set_material(0,_material)
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
