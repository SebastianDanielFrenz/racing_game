extends Node
# game/scripts/car_thumbnails.gd - lazily rendered preview images of the cars for the car
# browser (PLAN.md R6c). Rendering only: which image a car needs (its cache key: model +
# paint + rim + render version) comes from rg_core (RgGarage.browser_get_car -> thumbnail_key,
# rg::thumbnail_key), this script draws and caches it.
#
#   request(key, model_path, paint, rim) -> Texture2D or null
#     Memory cache -> disk cache (user://car_thumbs/<key>.png) -> render queue. Never blocks:
#     a miss returns null (the tile shows a placeholder) and `thumbnail_ready(key)` fires when
#     the picture exists. One picture is rendered per frame at most, so a screen full of
#     tiles fills in over a few frames without a hitch.
#
# The renderer is its own SubViewport with its own World3D (a small studio: key light,
# ambient, plain background) so it never touches the garage scene or the driving world.
# A car model is loaded once per model file and recoloured per request (the sedan presets all
# share one model). In a headless run there is no renderer: requests stay null and the
# placeholder stays, which is the documented headless behaviour.
#
# Bump RENDER_VERSION together with rg::kThumbnailRenderVersion (car_browser.h) whenever the
# camera, lights or paint shader below change - the version is part of the cache key.

signal thumbnail_ready(key: String)

const SIZE := Vector2i(384, 216)
const CACHE_DIR := "user://car_thumbs"
const PALETTE_ROUGHNESS := 0.3

var _viewport: SubViewport
var _world_root: Node3D
var _camera: Camera3D
var _holder: Node3D
var _models: Dictionary = {}     # model path -> Node3D (the loaded glTF scene, kept hidden)
var _textures: Dictionary = {}   # key -> Texture2D
var _queue: Array = []           # [{key, model_path, paint, rim}]
var _queued: Dictionary = {}     # key -> true
var _busy := false
var _enabled := true
var rendered_count := 0          # pictures rendered this session (tests)
var disk_hits := 0

func _ready() -> void:
	_enabled = DisplayServer.get_name() != "headless"
	DirAccess.make_dir_recursive_absolute(ProjectSettings.globalize_path(CACHE_DIR))
	if _enabled:
		_build_studio()

func is_enabled() -> bool:
	return _enabled

func pending_count() -> int:
	return _queue.size() + (1 if _busy else 0)

func has_texture(key: String) -> bool:
	return _textures.has(key)

# Number of nodes this renderer owns (the node-count checks add it to what the browser owns).
func node_count() -> int:
	return _count_nodes(self)

func _count_nodes(n: Node) -> int:
	var c := 1
	for child in n.get_children():
		c += _count_nodes(child)
	return c

func request(key: String, model_path: String, paint: String, rim: String) -> Texture2D:
	if _textures.has(key):
		return _textures[key]
	var path := _disk_path(key)
	if FileAccess.file_exists(path):
		var img := Image.load_from_file(path)
		if img != null and not img.is_empty():
			var tex := ImageTexture.create_from_image(img)
			_textures[key] = tex
			disk_hits += 1
			return tex
	if _enabled and not _queued.has(key):
		_queued[key] = true
		_queue.append({"key": key, "model_path": model_path, "paint": paint, "rim": rim})
	return null

func _disk_path(key: String) -> String:
	return CACHE_DIR + "/" + key.validate_filename() + ".png"

func _process(_delta: float) -> void:
	if not _enabled or _busy or _queue.is_empty():
		return
	_render_next()

func _render_next() -> void:
	_busy = true
	var job: Dictionary = _queue.pop_front()
	var key: String = job["key"]
	var model := _model_for(str(job["model_path"]))
	if model == null:
		_queued.erase(key)
		_busy = false
		return
	for path in _models:
		(_models[path] as Node3D).visible = (_models[path] == model)
	_apply_colours(model, Color.html(str(job["paint"])), Color.html(str(job["rim"])))
	_frame_camera(model)
	_viewport.render_target_update_mode = SubViewport.UPDATE_ONCE
	await RenderingServer.frame_post_draw
	var img := _viewport.get_texture().get_image()
	if img != null and not img.is_empty():
		img.save_png(ProjectSettings.globalize_path(_disk_path(key)))
		_textures[key] = ImageTexture.create_from_image(img)
		rendered_count += 1
		thumbnail_ready.emit(key)
	_queued.erase(key)
	_busy = false

# ---- the studio ---------------------------------------------------------------

func _build_studio() -> void:
	_viewport = SubViewport.new()
	_viewport.name = "ThumbViewport"
	_viewport.size = SIZE
	_viewport.own_world_3d = true
	_viewport.msaa_3d = Viewport.MSAA_4X
	_viewport.transparent_bg = false
	_viewport.render_target_update_mode = SubViewport.UPDATE_DISABLED
	add_child(_viewport)
	_world_root = Node3D.new()
	_world_root.name = "ThumbWorld"
	_viewport.add_child(_world_root)
	var env := Environment.new()
	env.background_mode = Environment.BG_COLOR
	env.background_color = Color(0.10, 0.115, 0.14)
	env.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	env.ambient_light_color = Color(0.62, 0.66, 0.74)
	env.ambient_light_energy = 0.9
	var we := WorldEnvironment.new()
	we.environment = env
	_world_root.add_child(we)
	var key_light := DirectionalLight3D.new()
	key_light.rotation_degrees = Vector3(-38.0, -35.0, 0.0)
	key_light.light_energy = 1.35
	_world_root.add_child(key_light)
	var rim_light := DirectionalLight3D.new()
	rim_light.rotation_degrees = Vector3(-20.0, 150.0, 0.0)
	rim_light.light_energy = 0.55
	_world_root.add_child(rim_light)
	var floor_mesh := MeshInstance3D.new()
	var plane := PlaneMesh.new()
	plane.size = Vector2(40, 40)
	floor_mesh.mesh = plane
	var floor_mat := StandardMaterial3D.new()
	floor_mat.albedo_color = Color(0.13, 0.145, 0.17)
	floor_mat.roughness = 0.85
	floor_mesh.material_override = floor_mat
	_world_root.add_child(floor_mesh)
	_holder = Node3D.new()
	_holder.name = "Holder"
	_holder.rotation.y = PI # glTF forward is +Z; the garage turns models the same way
	_world_root.add_child(_holder)
	_camera = Camera3D.new()
	_camera.fov = 32.0
	_camera.current = true
	_world_root.add_child(_camera)

func _model_for(path: String) -> Node3D:
	if _models.has(path):
		return _models[path]
	if path == "" or not FileAccess.file_exists(path):
		push_warning("car_thumbnails.gd: model not found at '%s'" % path)
		return null
	var doc := GLTFDocument.new()
	var state := GLTFState.new()
	if doc.append_from_file(path, state) != OK:
		push_warning("car_thumbnails.gd: cannot load %s" % path)
		return null
	var scene: Node = doc.generate_scene(state)
	if scene == null or not scene is Node3D:
		return null
	var model := scene as Node3D
	_holder.add_child(model)
	_unique_materials(model)
	_models[path] = model
	return model

# Camera on a three-quarter front view, distance from the model's bounds.
func _frame_camera(model: Node3D) -> void:
	var box := _bounds(model)
	var centre := _holder.global_transform * (box.position + box.size * 0.5)
	var radius := maxf(box.size.length() * 0.5, 0.5)
	var dist := radius / sin(deg_to_rad(_camera.fov) * 0.5) * 0.92
	var dir := Vector3(0.62, 0.30, -0.72).normalized() # front-right-above (the car faces -Z in the holder frame)
	_camera.position = centre + dir * dist
	_camera.look_at(centre, Vector3.UP)

func _bounds(model: Node3D) -> AABB:
	var have := false
	var out := AABB()
	for node in _mesh_instances(model):
		var mi := node as MeshInstance3D
		if mi.mesh == null:
			continue
		var xf := Transform3D.IDENTITY
		var n: Node = mi
		while n != null and n != _holder:
			if n is Node3D:
				xf = (n as Node3D).transform * xf
			n = n.get_parent()
		var box: AABB = xf * mi.mesh.get_aabb()
		out = box if not have else out.merge(box)
		have = true
	return out if have else AABB(Vector3(-2, 0, -1), Vector3(4, 1.5, 2))

# ---- paint (the same roles garage_scene.gd recolours: materials named paint* / rim*) --------------

func _mesh_instances(root: Node) -> Array:
	var out: Array = []
	var stack: Array = [root]
	while not stack.is_empty():
		var n: Node = stack.pop_back()
		if n is MeshInstance3D:
			out.append(n)
		for c in n.get_children():
			stack.append(c)
	return out

func _unique_materials(model: Node) -> void:
	for node in _mesh_instances(model):
		var mi := node as MeshInstance3D
		if mi.mesh == null:
			continue
		for s in range(mi.mesh.get_surface_count()):
			var src: Material = mi.get_active_material(s)
			if src == null:
				continue
			var copy := src.duplicate() as Material
			copy.resource_name = src.resource_name
			mi.set_surface_override_material(s, copy)

func _apply_colours(model: Node3D, paint: Color, rim: Color) -> void:
	for node in _mesh_instances(model):
		var mi := node as MeshInstance3D
		if mi.mesh == null:
			continue
		for s in range(mi.mesh.get_surface_count()):
			var std := mi.get_surface_override_material(s) as StandardMaterial3D
			if std == null:
				continue
			var n := std.resource_name.to_lower()
			if n == "paint" or n.begins_with("paint"):
				std.vertex_color_use_as_albedo = false
				std.albedo_texture = null
				std.albedo_color = paint
				std.metallic = 0.55
				std.roughness = PALETTE_ROUGHNESS
				std.clearcoat_enabled = true
				std.clearcoat = 1.0
				std.clearcoat_roughness = 0.06
			elif n.begins_with("rim"):
				std.vertex_color_use_as_albedo = false
				std.albedo_texture = null
				std.albedo_color = rim
				std.metallic = 0.9
				std.roughness = 0.25
