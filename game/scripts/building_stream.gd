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
var _jobs: Array = []
var _results: Array = []
var _known: Dictionary = {}
var _tiles: Dictionary = {}
var _center := Vector2i(2147483647, 2147483647)
var _material := StandardMaterial3D.new()
func _ready() -> void:
	process_priority = 100
	settings["ordinary_distance_m"] = clampf(float(settings.get("ordinary_distance_m",2200.0)),256.0,10000.0)
	settings["skyline_distance_m"] = clampf(float(settings.get("skyline_distance_m",40000.0)),settings.ordinary_distance_m,80000.0)
	settings["skyline_min_height_m"] = clampf(float(settings.get("skyline_min_height_m",50.0)),10.0,1000.0)
	settings["fallback_height_m"] = clampf(float(settings.get("fallback_height_m",8.0)),1.0,100.0)
	settings["storey_height_m"] = clampf(float(settings.get("storey_height_m",3.0)),1.0,10.0)
	_material.vertex_color_use_as_albedo = true
	_material.roughness = 1.0
	_material.cull_mode = BaseMaterial3D.CULL_DISABLED
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
	if rig == null:
		return
	var focus: Vector3 = view.godot_to_session(rig.camera.global_position)
	var absolute := Vector2(focus.x, focus.y) + utm_origin
	var center := Vector2i(floori(absolute.x/1024.0), floori(absolute.y/1024.0))
	if center != _center:
		_schedule(center, absolute)
		_center = center
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
	var origin: Vector3 = simulation.get_render_origin_session()
	var ordinary := float(settings.get("ordinary_distance_m", 2200.0))
	for key in _tiles.keys():
		var tile: Dictionary = _tiles[key]
		var s: Vector3 = tile.origin - origin
		tile.node.position = Vector3(-s.y, s.z, -s.x)
		var distance: float = (Vector2(tile.origin.x+512, tile.origin.y+512)-Vector2(focus.x,focus.y)).length()
		tile.near.visible = distance <= ordinary + 725.0
		tile.tall.visible = distance <= skyline + 725.0
		if distance > skyline + 2048.0:
			tile.node.queue_free()
			_tiles.erase(key)
			_known.erase(key)
		elif tile.full and distance > ordinary + 2048.0:
			tile.near.mesh = null
			tile.full = false
			_known[key] = false
func _schedule(center: Vector2i, focus: Vector2) -> void:
	var ordinary := float(settings.get("ordinary_distance_m", 2200.0))
	var skyline := float(settings.get("skyline_distance_m", 40000.0))
	var radius := ceili(skyline/1024.0)+1
	var jobs: Array = []
	for y in range(center.y-radius, center.y+radius+1):
		for x in range(center.x-radius, center.x+radius+1):
			var key := Vector2i(x,y)
			var distance := (Vector2(x*1024+512,y*1024+512)-focus).length()
			if distance > skyline+725:
				continue
			var full := distance <= ordinary+725
			if _known.has(key) and (not full or bool(_known[key])):
				continue
			jobs.append({"key":key,"full":full,"distance":distance})
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
		if not stop and not _jobs.is_empty() and _results.size()<4:
			job = _jobs.pop_back()
		_mutex.unlock()
		if stop:
			return
		if job.is_empty():
			_wake.wait()
			continue
		var threshold := float(settings.get("skyline_min_height_m",50.0))
		var data: Dictionary = view.get_building_tile(job.key.x,job.key.y,0.0 if job.full else threshold,float(settings.get("fallback_height_m",8.0)),float(settings.get("storey_height_m",3.0)))
		var near_arrays := _empty_arrays()
		var tall_arrays := _empty_arrays()
		for building: Dictionary in data.get("buildings",[]):
			_tessellate(building,tall_arrays if float(building.height)>=threshold else near_arrays)
		_mutex.lock()
		_results.append({"key":job.key,"full":job.full,"origin":data.get("origin_session",Vector3.ZERO),"near":near_arrays,"tall":tall_arrays})
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
	if _tiles.has(key) and bool(_tiles[key].full) and not bool(result.full):
		return
	_known[key] = result.full
	if _tiles.has(key):
		_tiles[key].node.queue_free()
		_tiles.erase(key)
	var near_mesh := _mesh(result.near)
	var tall_mesh := _mesh(result.tall)
	if near_mesh==null and tall_mesh==null:
		return
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
