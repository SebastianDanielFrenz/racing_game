extends CanvasLayer
# game/scripts/garage_scene.gd - the garage/showroom 3D set (PLAN.md R6, 11.6).
# A dedicated scene in its own World3D (a SubViewport shown full-screen on a layer
# below the shell UI), so it never mixes with the driving world's lights, sky or
# camera, and so removing it removes every node it created. The set is built from
# data (data/garage/garage_set.json through RgGarage.get_set(): room, turntable,
# lights, softboxes, environment); the camera pose and the turntable angle come
# from rg::GarageCamera through RgGarage.get_camera() every frame - this script
# decides nothing about where the camera goes.
#
# Look: a dark studio room (cyclorama wall, ceiling, glossy floor) around a
# turntable; key/rim/fill spot lights plus emissive softbox panels. The same
# softboxes are painted into a procedural sky shader so the car's paint reflects
# them (the compatibility renderer has no screen-space or probe reflections); the
# floor shows a mirrored copy of the car on a render layer lit by mirrored lights.
# Everything is primitive geometry and own materials: no asset is imported but the
# car model itself (read-only, loaded as the driving scene loads it).
#
# Frames: the room is built in Godot axes (RgGarage already converted the set's
# ISO positions); the turntable top is y = 0, the floor y = -turntable height. The
# car model is a glTF in the model's own axes: rotating it PI about Y puts its
# forward axis on -Z, exactly the ISO -> Godot conversion RgGarage uses.

const LAYER_REAL := 1
const LAYER_MIRROR := 2
const PALETTE_ROUGHNESS := 0.3

var garage: Node # RgGarage, set by main.gd before add_child
var vehicle_id: String = ""
var vehicle_info: Dictionary = {}

# A TextureRect showing the SubViewport, not a stretching SubViewportContainer:
# under the root content_scale_factor (ui_scale.gd) a container would render
# the car at canvas size (1/factor of the physical pixels) and blur it.
var _container: TextureRect
var _viewport: SubViewport
var _world_root: Node3D
var _camera: Camera3D
var _car_pivot: Node3D
var _mirror_pivot: Node3D
var _cars: Dictionary = {} # vehicle id -> {real: Node3D, mirror: Node3D, bounds_min, bounds_max}
var _paint: Color = Color(0.5, 0.5, 0.55)
var _rim: Color = Color(0.7, 0.7, 0.75)
var _set: Dictionary = {}
var _built: bool = false
var _models_loaded: int = 0
# The shell panel covers one side of the screen: the camera shifts sideways so the car
# sits in the free part ("left" / "right" / "" and the panel's width in pixels).
var panel_side: String = ""
var panel_px: float = 0.0
var _panel_shift: float = 0.0 # smoothed fraction of the view width
var _panel_frac: float = 0.0  # smoothed share of the screen the panel covers

func _ready() -> void:
	layer = 20
	_container = TextureRect.new()
	_container.name = "Container"
	_container.expand_mode = TextureRect.EXPAND_IGNORE_SIZE
	_container.stretch_mode = TextureRect.STRETCH_SCALE
	_container.set_anchors_preset(Control.PRESET_FULL_RECT)
	_container.mouse_filter = Control.MOUSE_FILTER_IGNORE
	add_child(_container)
	_viewport = SubViewport.new()
	_viewport.name = "Viewport"
	_viewport.own_world_3d = true
	_viewport.msaa_3d = Viewport.MSAA_4X
	_viewport.positional_shadow_atlas_size = 4096
	_viewport.handle_input_locally = false
	_viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS
	add_child(_viewport)
	_container.texture = _viewport.get_texture()
	_container.resized.connect(_fit_viewport)
	_fit_viewport()
	_world_root = Node3D.new()
	_world_root.name = "GarageWorld"
	_viewport.add_child(_world_root)
	_build_set()

# Render at physical pixels: the rect's canvas size times the root's content scale.
func _fit_viewport() -> void:
	var px := (_container.size * get_tree().root.content_scale_factor).round()
	_viewport.size = Vector2i(maxi(int(px.x), 1), maxi(int(px.y), 1))

func _exit_tree() -> void:
	_cars.clear()

# ---- public ------------------------------------------------------------------

func is_built() -> bool:
	return _built

# The car browser covers the whole screen: nothing to draw behind it (PLAN.md R6c).
func set_rendering(on: bool) -> void:
	if _viewport != null:
		_viewport.render_target_update_mode = SubViewport.UPDATE_ALWAYS if on else SubViewport.UPDATE_DISABLED
	set_process(on)

func models_loaded() -> int:
	return _models_loaded

# Number of nodes this scene owns (the acceptance test compares the tree's node count
# before opening and after closing; this is the same number for the garage alone).
func node_count() -> int:
	return _count_nodes(self)

func _count_nodes(n: Node) -> int:
	var c := 1
	for child in n.get_children():
		c += _count_nodes(child)
	return c

# Shows the car (loads its model on first use). `info` is RgGarage.get_vehicle()'s
# dictionary. Returns false when the model could not be loaded (the turntable stays empty).
func show_vehicle(info: Dictionary) -> bool:
	vehicle_info = info
	vehicle_id = str(info.get("id", ""))
	for id in _cars:
		_cars[id]["real"].visible = (id == vehicle_id)
		_cars[id]["mirror"].visible = (id == vehicle_id)
	if not _cars.has(vehicle_id):
		if not _load_car(info):
			return false
	_paint = Color.html(str(info.get("paint", "#8a8f98")))
	_rim = Color.html(str(info.get("rim", "#c4c8cf")))
	_apply_colours()
	var car: Dictionary = _cars[vehicle_id]
	garage.setup_camera(vehicle_id, car["bounds_min"], car["bounds_max"], true)
	return true

# Paint/rim of the working setup (visual only). "" keeps the colour.
func set_colours(paint_hex: String, rim_hex: String) -> void:
	if paint_hex != "":
		_paint = Color.html(paint_hex)
	if rim_hex != "":
		_rim = Color.html(rim_hex)
	_apply_colours()
	_apply_tyre_widths()

func _apply_tyre_widths() -> void:
	var owner := get_parent()
	if not owner.has_method("get_shell_ui"):
		return
	var ui: Node = owner.get_shell_ui()
	if ui == null or not ui.has_method("garage_wheel_widths"):
		return
	var widths: Vector2 = ui.garage_wheel_widths()
	for id in _cars:
		if not _cars[id]["real"].visible:
			continue
		for instance in [_cars[id]["real"],_cars[id]["mirror"]]:
			for corner in ["FL","FR","RL","RR"]:
				var wheel: Node3D = instance.find_child("wheel_"+corner,true,false)
				if wheel == null:
					continue
				if not wheel.has_meta("stock_width_scale"):
					wheel.set_meta("stock_width_scale",wheel.scale.x)
				var front: bool = str(corner).begins_with("F")
				var reference: float = (0.265 if front else 0.325) if str(garage.get_vehicle(str(id)).get("model_path", "")).get_file() == "car_hyper.glb" else 0.225
				var width: float = widths.x if front else widths.y
				wheel.scale.x = float(wheel.get_meta("stock_width_scale")) * width / reference if width > 0.0 else float(wheel.get_meta("stock_width_scale"))

func go_to_area(area_id: String, instant: bool = false) -> void:
	garage.camera_go_to(area_id, instant)

func camera_node() -> Camera3D:
	return _camera

# Snapshot of the garage viewport (tests / screenshots). null while it has no image yet.
func capture_image() -> Image:
	var tex := _viewport.get_texture()
	return tex.get_image() if tex != null else null

# ---- per frame ---------------------------------------------------------------

func _process(delta: float) -> void:
	if garage == null or not _built:
		return
	garage.camera_update(delta)
	var cam: Dictionary = garage.get_camera()
	if cam.is_empty():
		return
	var pos: Vector3 = cam["position"]
	var look: Vector3 = cam["look_at"]
	_camera.fov = float(cam["fov_deg"])
	_camera.position = pos
	if pos.distance_to(look) > 0.001:
		_camera.look_at(look, Vector3.UP)
	# shift the view so the car centres in the part of the screen the panel leaves free
	var size := _container.size # canvas units, like panel_px
	var target_shift := 0.0
	var panel_widths := Vector2.ZERO
	var owner := get_parent()
	if owner.has_method("get_shell_ui"):
		var ui: Node = owner.get_shell_ui()
		if ui != null and ui.has_method("garage_panel_widths"):
			panel_widths = ui.garage_panel_widths()
	if panel_widths != Vector2.ZERO and size.x > 1.0:
		target_shift = (panel_widths.y - panel_widths.x) * 0.5 / size.x
	elif panel_side != "" and size.x > 1.0:
		target_shift = (panel_px * 0.5 / size.x) * (-1.0 if panel_side == "left" else 1.0)
	_panel_shift = lerpf(_panel_shift, target_shift, clampf(delta * 6.0, 0.0, 1.0))
	var target_frac := (panel_px / size.x) if (panel_side != "" and size.x > 1.0) else 0.0
	if panel_widths != Vector2.ZERO:
		target_frac = (panel_widths.x + panel_widths.y) / maxf(size.x,1.0)
	_panel_frac = lerpf(_panel_frac, target_frac, clampf(delta * 6.0, 0.0, 1.0))
	# back off so the car still fits the free part of the screen
	pos = look + (pos - look) / maxf(1.0 - _panel_frac * (0.65 if panel_widths != Vector2.ZERO else 0.5), 0.25)
	_camera.position = pos
	var dist := pos.distance_to(look)
	var view_width := 2.0 * dist * tan(deg_to_rad(_camera.fov) * 0.5) * (size.x / maxf(size.y, 1.0))
	# a sideways camera move of +x shifts the scene left on screen: a left panel
	# (scene should move right) needs the camera to move left
	_camera.h_offset = _panel_shift * view_width
	var yaw := float(cam["car_yaw_rad"])
	_car_pivot.rotation.y = yaw
	_mirror_pivot.rotation.y = yaw

# ---- set ---------------------------------------------------------------------

func _hex(s: Variant, fallback: Color = Color.WHITE) -> Color:
	var text := str(s)
	return Color.html(text) if text.is_valid_html_color() else fallback

func _build_set() -> void:
	_set = garage.get_set()
	if _set.is_empty():
		push_warning("garage_scene.gd: the garage set could not be read")
		return
	var room: Dictionary = _set["room"]
	var turntable: Dictionary = _set["turntable"]
	var env_data: Dictionary = _set["environment"]
	var radius := float(room["radius_m"])
	var height := float(room["height_m"])
	var tt_r := float(turntable["radius_m"])
	var tt_h := float(turntable["height_m"])
	var floor_y := -tt_h
	var accent := _hex(room["accent_colour"], Color(0.9, 0.45, 0.17))

	# environment: procedural sky carrying the softboxes (what the paint reflects)
	var environment := Environment.new()
	environment.background_mode = Environment.BG_SKY
	environment.sky = _make_sky()
	environment.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	environment.ambient_light_color = _hex(env_data["ambient_colour"], Color(0.22, 0.25, 0.3))
	environment.ambient_light_energy = float(env_data["ambient_energy"])
	environment.reflected_light_source = Environment.REFLECTION_SOURCE_SKY
	environment.tonemap_mode = Environment.TONE_MAPPER_ACES
	environment.tonemap_exposure = float(env_data["exposure"])
	environment.tonemap_white = 6.0
	environment.glow_enabled = true
	environment.glow_intensity = 0.5
	environment.glow_bloom = 0.04
	environment.glow_hdr_threshold = 1.2
	var world_env := WorldEnvironment.new()
	world_env.name = "Environment"
	world_env.environment = environment
	_world_root.add_child(world_env)

	# floor (glossy, semi-transparent over a dark base so the mirrored car shows through)
	var reflectivity := clampf(float(env_data["floor_reflectivity"]), 0.0, 1.0)
	var base := MeshInstance3D.new()
	base.name = "FloorBase"
	var base_mesh := CylinderMesh.new()
	base_mesh.top_radius = radius
	base_mesh.bottom_radius = radius
	base_mesh.height = 0.05
	base_mesh.radial_segments = 96
	base_mesh.rings = 1
	base.mesh = base_mesh
	base.position = Vector3(0, floor_y - 2.6, 0)
	var base_mat := StandardMaterial3D.new()
	base_mat.albedo_color = Color(0.01, 0.01, 0.012)
	base_mat.roughness = 1.0
	base.material_override = base_mat
	base.layers = LAYER_REAL
	_world_root.add_child(base)

	var floor_mi := MeshInstance3D.new()
	floor_mi.name = "Floor"
	var floor_mesh := CylinderMesh.new()
	floor_mesh.top_radius = radius
	floor_mesh.bottom_radius = radius
	floor_mesh.height = 0.02
	floor_mesh.radial_segments = 96
	floor_mesh.rings = 1
	floor_mi.mesh = floor_mesh
	floor_mi.position = Vector3(0, floor_y - 0.01, 0)
	var floor_mat := StandardMaterial3D.new()
	floor_mat.albedo_color = Color(_hex(room["floor_colour"], Color(0.08, 0.08, 0.09)), 1.0 - 0.75 * reflectivity)
	floor_mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	floor_mat.metallic = 0.0
	floor_mat.metallic_specular = 0.5
	floor_mat.roughness = 0.3
	floor_mi.material_override = floor_mat
	floor_mi.layers = LAYER_REAL
	_world_root.add_child(floor_mi)

	# the turntable: a disc proud of the floor with an accent ring
	var disc := MeshInstance3D.new()
	disc.name = "Turntable"
	var disc_mesh := CylinderMesh.new()
	disc_mesh.top_radius = tt_r
	disc_mesh.bottom_radius = tt_r + 0.02
	disc_mesh.height = tt_h
	disc_mesh.radial_segments = 96
	disc_mesh.rings = 1
	disc.mesh = disc_mesh
	disc.position = Vector3(0, -tt_h * 0.5, 0)
	var disc_mat := StandardMaterial3D.new()
	disc_mat.albedo_color = Color(_hex(turntable["colour"], Color(0.14, 0.15, 0.17)), 1.0 - 0.7 * reflectivity)
	disc_mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	disc_mat.metallic = 0.3
	disc_mat.roughness = 0.14
	disc.material_override = disc_mat
	disc.layers = LAYER_REAL
	_world_root.add_child(disc)
	var ring := MeshInstance3D.new()
	ring.name = "TurntableRing"
	var ring_mesh := TorusMesh.new()
	ring_mesh.inner_radius = tt_r - 0.12
	ring_mesh.outer_radius = tt_r - 0.06
	ring_mesh.rings = 96
	ring_mesh.ring_segments = 4
	ring.mesh = ring_mesh
	ring.scale = Vector3(1, 0.04, 1)
	ring.position = Vector3(0, 0.003, 0)
	var rim_colour := _hex(turntable["rim_colour"], accent)
	var ring_mat := StandardMaterial3D.new()
	ring_mat.albedo_color = rim_colour
	ring_mat.emission_enabled = true
	ring_mat.emission = rim_colour
	ring_mat.emission_energy_multiplier = 2.4
	ring_mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	ring.material_override = ring_mat
	ring.layers = LAYER_REAL
	_world_root.add_child(ring)
	# a few radial ticks on the turntable top, so its spin reads
	for i in range(24):
		var tick := MeshInstance3D.new()
		tick.name = "Tick%d" % i
		var tick_mesh := BoxMesh.new()
		tick_mesh.size = Vector3(0.05, 0.004, 0.32 if i % 6 == 0 else 0.16)
		tick.mesh = tick_mesh
		var a := TAU * float(i) / 24.0
		tick.position = Vector3(sin(a) * (tt_r - 0.34), 0.002, cos(a) * (tt_r - 0.34))
		tick.rotation.y = a
		tick.material_override = ring_mat
		tick.layers = LAYER_REAL
		_world_root.add_child(tick)

	# the room: cyclorama wall, ceiling, accent strips
	var wall := MeshInstance3D.new()
	wall.name = "Wall"
	var wall_mesh := CylinderMesh.new()
	wall_mesh.top_radius = radius
	wall_mesh.bottom_radius = radius
	wall_mesh.height = height + tt_h
	wall_mesh.radial_segments = 96
	wall_mesh.rings = 1
	wall_mesh.cap_top = true
	wall_mesh.cap_bottom = false
	wall.mesh = wall_mesh
	wall.position = Vector3(0, (height - tt_h) * 0.5, 0)
	var wall_shader := Shader.new()
	wall_shader.code = """
shader_type spatial;
render_mode unshaded, cull_front;
uniform vec3 wall_colour = vec3(0.1, 0.11, 0.14);
uniform vec3 glow_colour = vec3(0.9, 0.45, 0.17);
uniform float wall_height = 7.5;
void fragment() {
	float h = clamp(UV.y, 0.0, 1.0);
	float pillar = smoothstep(0.46, 0.5, abs(fract(UV.x * 24.0) - 0.5));
	vec3 base = wall_colour * mix(2.4, 0.5, pow(h, 0.7));
	base *= 1.0 - 0.45 * pillar;
	base += glow_colour * 0.05 * exp(-h * 7.0);
	ALBEDO = base;
}
"""
	var wall_mat := ShaderMaterial.new()
	wall_mat.shader = wall_shader
	var wc := _hex(room["wall_colour"], Color(0.1, 0.11, 0.14))
	wall_mat.set_shader_parameter("wall_colour", Vector3(wc.r, wc.g, wc.b))
	wall_mat.set_shader_parameter("glow_colour", Vector3(accent.r, accent.g, accent.b))
	wall.material_override = wall_mat
	wall.layers = LAYER_REAL
	_world_root.add_child(wall)
	for strip_y in [1.4, height - 0.7]:
		var strip := MeshInstance3D.new()
		strip.name = "AccentStrip"
		var strip_mesh := CylinderMesh.new()
		var strip_r := radius - 0.02
		strip_mesh.top_radius = strip_r
		strip_mesh.bottom_radius = strip_r
		strip_mesh.height = 0.06
		strip_mesh.radial_segments = 96
		strip_mesh.rings = 1
		strip_mesh.cap_top = false
		strip_mesh.cap_bottom = false
		strip.mesh = strip_mesh
		strip.position = Vector3(0, float(strip_y), 0)
		var strip_mat := StandardMaterial3D.new()
		strip_mat.albedo_color = accent
		strip_mat.emission_enabled = true
		strip_mat.emission = accent
		strip_mat.emission_energy_multiplier = 1.8
		strip_mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
		strip_mat.cull_mode = BaseMaterial3D.CULL_FRONT
		strip.material_override = strip_mat
		strip.layers = LAYER_REAL
		_world_root.add_child(strip)

	# lights (real) and their mirrored twins for the mirror car
	for l in _set["lights"]:
		_add_light(l, false)
		if reflectivity > 0.0:
			_add_light(l, true)

	# softboxes: emissive panels in the room (the same ones the sky shader paints)
	for b in _set["softboxes"]:
		_add_softbox(b)

	# car holders: the real car and its mirrored twin below the floor
	_car_pivot = Node3D.new()
	_car_pivot.name = "CarPivot"
	_world_root.add_child(_car_pivot)
	_mirror_pivot = Node3D.new()
	_mirror_pivot.name = "MirrorPivot"
	_mirror_pivot.scale = Vector3(1, -1, 1)
	_mirror_pivot.visible = reflectivity > 0.0
	_world_root.add_child(_mirror_pivot)

	_camera = Camera3D.new()
	_camera.name = "Camera"
	_camera.current = true
	_camera.near = 0.05
	_camera.far = 80.0
	_camera.cull_mask = LAYER_REAL | LAYER_MIRROR
	_world_root.add_child(_camera)
	_built = true

func _add_light(l: Dictionary, mirrored: bool) -> void:
	var kind := str(l["kind"])
	var light: Light3D
	if kind == "omni":
		var omni := OmniLight3D.new()
		omni.omni_range = float(l["range_m"])
		light = omni
	else:
		var spot := SpotLight3D.new()
		spot.spot_range = float(l["range_m"])
		spot.spot_angle = float(l["angle_deg"])
		spot.spot_angle_attenuation = 0.8
		light = spot
	light.name = ("Mirror" if mirrored else "Light_") + str(l["id"])
	light.light_color = _hex(l["colour"])
	light.light_energy = float(l["energy"]) * (0.55 if mirrored else 1.0)
	light.shadow_enabled = bool(l["shadow"]) and not mirrored
	light.shadow_bias = 0.06
	light.shadow_normal_bias = 2.5
	light.shadow_blur = 1.6
	light.light_cull_mask = LAYER_MIRROR if mirrored else LAYER_REAL
	var pos: Vector3 = l["position"]
	_world_root.add_child(light)
	if mirrored:
		pos.y = -pos.y
	light.position = pos
	if kind != "omni":
		var target: Vector3 = l["target"]
		if mirrored:
			target.y = -target.y
		var up := Vector3.UP if abs((target - pos).normalized().y) < 0.99 else Vector3.RIGHT
		light.look_at(target, up)

func _add_softbox(b: Dictionary) -> void:
	var panel := MeshInstance3D.new()
	panel.name = "Softbox_" + str(b["id"])
	var quad := QuadMesh.new()
	quad.size = Vector2(float(b["width_m"]), float(b["height_m"]))
	panel.mesh = quad
	var colour := _hex(b["colour"])
	var shader := Shader.new()
	shader.code = """
shader_type spatial;
render_mode unshaded, cull_disabled, blend_add, depth_draw_never;
uniform vec3 tint = vec3(1.0);
void fragment() {
	vec2 q = abs(UV - 0.5) * 2.0;
	float m = (1.0 - smoothstep(0.7, 1.0, q.x)) * (1.0 - smoothstep(0.55, 1.0, q.y));
	ALBEDO = tint * m;
}
"""
	var mat := ShaderMaterial.new()
	mat.shader = shader
	var strength := clampf(float(b["energy"]) / 5.0, 0.3, 1.2)
	mat.set_shader_parameter("tint", Vector3(colour.r, colour.g, colour.b) * strength)
	panel.material_override = mat
	panel.layers = LAYER_REAL
	_world_root.add_child(panel)
	var pos: Vector3 = b["position"]
	panel.position = pos
	var target: Vector3 = b["target"]
	var up := Vector3.UP if abs((target - pos).normalized().y) < 0.99 else Vector3.RIGHT
	panel.look_at(target, up)

# Sky shader: dark studio gradient plus the softboxes as bright rectangles, so the
# paint (metallic/clear-coat) and the floor reflect them.
func _make_sky() -> Sky:
	var shader := Shader.new()
	shader.code = """
shader_type sky;
uniform vec3 top_colour = vec3(0.05, 0.058, 0.08);
uniform vec3 horizon_colour = vec3(0.16, 0.17, 0.2);
uniform vec3 bottom_colour = vec3(0.02, 0.02, 0.026);
uniform int box_count = 0;
uniform vec3 box_dir[8];
uniform vec3 box_u[8];
uniform vec3 box_v[8];
uniform vec2 box_half[8];
uniform vec3 box_colour[8];
void sky() {
	vec3 d = normalize(EYEDIR);
	float h = d.y;
	vec3 col = h > 0.0 ? mix(horizon_colour, top_colour, pow(h, 0.45)) : mix(horizon_colour, bottom_colour, pow(-h, 0.45));
	for (int i = 0; i < 8; i++) {
		if (i >= box_count) { break; }
		float t = dot(d, box_dir[i]);
		if (t > 0.05) {
			vec3 p = d / t - box_dir[i];
			float x = dot(p, box_u[i]) / box_half[i].x;
			float y = dot(p, box_v[i]) / box_half[i].y;
			float m = (1.0 - smoothstep(0.86, 1.0, abs(x))) * (1.0 - smoothstep(0.86, 1.0, abs(y)));
			col += box_colour[i] * m;
		}
	}
	COLOR = col;
}
"""
	var material := ShaderMaterial.new()
	material.shader = shader
	var boxes: Array = _set["softboxes"]
	var count := mini(boxes.size(), 8)
	material.set_shader_parameter("box_count", count)
	var dirs := PackedVector3Array()
	var us := PackedVector3Array()
	var vs := PackedVector3Array()
	var halves := PackedVector2Array()
	var cols := PackedVector3Array()
	for i in range(8):
		if i < count:
			var b: Dictionary = boxes[i]
			var pos: Vector3 = b["position"]
			var target: Vector3 = b["target"]
			var dist := pos.length()
			var dir := pos.normalized()
			var facing := (target - pos).normalized()
			var up_ref := Vector3.UP if absf(facing.y) < 0.99 else Vector3.RIGHT
			# the panel's own axes (as MeshInstance3D.look_at builds them)
			var z_axis := -facing
			var x_axis := up_ref.cross(z_axis).normalized()
			var y_axis := z_axis.cross(x_axis).normalized()
			dirs.append(dir)
			us.append(x_axis)
			vs.append(y_axis)
			halves.append(Vector2(float(b["width_m"]) * 0.5, float(b["height_m"]) * 0.5) / maxf(dist, 0.1))
			var c := _hex(b["colour"])
			var e := float(b["energy"])
			cols.append(Vector3(c.r, c.g, c.b) * e * 0.55)
		else:
			dirs.append(Vector3.UP)
			us.append(Vector3.RIGHT)
			vs.append(Vector3.FORWARD)
			halves.append(Vector2(0.1, 0.1))
			cols.append(Vector3.ZERO)
	material.set_shader_parameter("box_dir", dirs)
	material.set_shader_parameter("box_u", us)
	material.set_shader_parameter("box_v", vs)
	material.set_shader_parameter("box_half", halves)
	material.set_shader_parameter("box_colour", cols)
	var sky := Sky.new()
	sky.sky_material = material
	sky.radiance_size = Sky.RADIANCE_SIZE_256
	return sky

# ---- cars --------------------------------------------------------------------

func _load_car(info: Dictionary) -> bool:
	var path := str(info.get("model_path", ""))
	if path == "" or not FileAccess.file_exists(path):
		push_warning("garage_scene.gd: model not found at '%s'" % path)
		return false
	var doc := GLTFDocument.new()
	var state := GLTFState.new()
	var err: int = doc.append_from_file(path, state)
	if err != OK:
		push_warning("garage_scene.gd: GLTFDocument.append_from_file failed (err=%d) for %s" % [err, path])
		return false
	var model: Node = doc.generate_scene(state)
	if model == null:
		push_warning("garage_scene.gd: generate_scene returned null for %s" % path)
		return false
	var holder := Node3D.new()
	holder.name = "Car_" + vehicle_id
	# glTF forward is +Z, ISO forward -> Godot -Z: a half turn about Y (see the header)
	holder.rotation.y = PI
	holder.add_child(model)
	_unique_materials(model)
	var bounds := _model_bounds_iso(model)
	_car_pivot.add_child(holder)
	for mi in _mesh_instances(holder):
		mi.layers = LAYER_REAL
		mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_ON
	var twin := holder.duplicate() as Node3D
	twin.name = "Mirror_" + vehicle_id
	_mirror_pivot.add_child(twin)
	for mi in _mesh_instances(twin):
		mi.layers = LAYER_MIRROR
		mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	_cars[vehicle_id] = {"real": holder, "mirror": twin, "bounds_min": bounds[0], "bounds_max": bounds[1]}
	_models_loaded += 1
	print("RG_GARAGE model loaded id=%s bounds_iso=%s..%s" % [vehicle_id, bounds[0], bounds[1]])
	return true

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

# The model's AABB in the car (ISO) frame: iso = (gltf.z, gltf.x, gltf.y).
func _model_bounds_iso(model: Node) -> Array:
	var lo := Vector3(1e9, 1e9, 1e9)
	var hi := Vector3(-1e9, -1e9, -1e9)
	for mi in _mesh_instances(model):
		var mesh_node := mi as MeshInstance3D
		if mesh_node.mesh == null:
			continue
		var xf := Transform3D.IDENTITY
		var n: Node = mesh_node
		while n != null and n != model:
			if n is Node3D:
				xf = (n as Node3D).transform * xf
			n = n.get_parent()
		if n == model and model is Node3D:
			xf = (model as Node3D).transform * xf
		var box: AABB = xf * mesh_node.mesh.get_aabb()
		lo = Vector3(minf(lo.x, box.position.x), minf(lo.y, box.position.y), minf(lo.z, box.position.z))
		var end := box.position + box.size
		hi = Vector3(maxf(hi.x, end.x), maxf(hi.y, end.y), maxf(hi.z, end.z))
	if lo.x > hi.x:
		return [Vector3(-2.3, -0.95, 0.0), Vector3(2.3, 0.95, 1.3)]
	return [Vector3(lo.z, lo.x, lo.y), Vector3(hi.z, hi.x, hi.y)]

# Every surface gets its own copy of its material, so recolouring one car never
# touches another (and the mirror twin, a duplicate of the nodes, shares the copy).
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

func _material_role(mat: Material) -> String:
	var n := mat.resource_name.to_lower()
	if n == "paint" or n.begins_with("paint"):
		return "paint"
	if n.begins_with("rim"):
		return "rim"
	return ""

func _apply_colours() -> void:
	if not _cars.has(vehicle_id):
		return
	var holder: Node3D = _cars[vehicle_id]["real"]
	for node in _mesh_instances(holder):
		var mi := node as MeshInstance3D
		if mi.mesh == null:
			continue
		for s in range(mi.mesh.get_surface_count()):
			var mat := mi.get_surface_override_material(s)
			if not mat is StandardMaterial3D:
				continue
			var std := mat as StandardMaterial3D
			var role := _material_role(std)
			if role == "paint":
				std.vertex_color_use_as_albedo = false
				std.albedo_texture = null
				std.albedo_color = _paint
				std.metallic = 0.55
				std.roughness = PALETTE_ROUGHNESS
				std.clearcoat_enabled = true
				std.clearcoat = 1.0
				std.clearcoat_roughness = 0.06
			elif role == "rim":
				std.vertex_color_use_as_albedo = false
				std.albedo_texture = null
				std.albedo_color = _rim
				std.metallic = 0.9
				std.roughness = 0.25
