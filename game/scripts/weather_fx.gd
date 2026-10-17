extends Node3D
# Wet road look and rain / snow around the camera (presentation only).
#
# Every frame: RgSimulation.get_wet_look(dt) (core rg/wet_look.h: the physics water film under the car and the rain
# rate -> wetness, puddles, drop / flake density, smoothed) -> the rg_wetness / rg_puddles shader globals
# (surface_materials.gd) and two GPUParticles3D boxes that follow the active camera. Particles live in world
# coordinates (a fast car drives through the rain instead of dragging it along); a floating-origin rebase restarts
# them. Numbers: data/world/wet_look.json ("particles" block). density = the graphics.rain_density setting.
const SurfaceMaterials := preload("res://scripts/surface_materials.gd")
const CONFIG := "world/wet_look.json"

var simulation: Node
var director: Node
var density := 1.0
var wet_look: Dictionary = {}
var _rain: GPUParticles3D
var _snow: GPUParticles3D
var _cfg: Dictionary = {}
var _ok := false
var _last_origin := Vector3.ZERO

func _ready() -> void:
	process_priority = 50
	var path := SurfaceMaterials.data_dir().path_join(CONFIG)
	if simulation == null or not simulation.has_method("load_wet_look"):
		return
	var err: String = simulation.load_wet_look(path)
	if err != "":
		push_warning("weather_fx: " + err)
		return
	var parsed = JSON.parse_string(FileAccess.get_file_as_string(path))
	_cfg = parsed.get("particles", {}) if parsed is Dictionary else {}
	_rain = _make_particles(false)
	_snow = _make_particles(true)
	_ok = true

func _vec(key: String, fallback: Array) -> Array:
	var v: Array = _cfg.get(key, fallback)
	return v if v.size() == fallback.size() else fallback

func _make_particles(snow: bool) -> GPUParticles3D:
	var p := GPUParticles3D.new()
	p.name = "Snow" if snow else "Rain"
	var box: Array = _vec("box_m", [22.0, 12.0, 22.0])
	var speed := float(_cfg.get("flake_speed_mps" if snow else "drop_speed_mps", 1.2 if snow else 9.0))
	p.amount = int(_cfg.get("max_flakes" if snow else "max_drops", 3000 if snow else 6000))
	p.amount_ratio = 0.0
	p.lifetime = 2.0 * float(box[1]) / maxf(speed, 0.1)
	p.preprocess = p.lifetime
	p.local_coords = false
	p.emitting = false
	p.visibility_aabb = AABB(Vector3(-box[0], -box[1] * 3.0, -box[2]), Vector3(box[0], box[1] * 4.0, box[2]) * 2.0)
	p.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	var m := ParticleProcessMaterial.new()
	m.emission_shape = ParticleProcessMaterial.EMISSION_SHAPE_BOX
	m.emission_box_extents = Vector3(box[0], box[1], box[2])
	m.direction = Vector3(0, -1, 0)
	m.spread = 4.0 if not snow else 25.0
	m.initial_velocity_min = speed * 0.9
	m.initial_velocity_max = speed * 1.1
	m.gravity = Vector3.ZERO
	if snow:
		var drift := float(_cfg.get("flake_drift_mps", 0.6))
		m.turbulence_enabled = true
		m.turbulence_noise_strength = drift
		m.turbulence_noise_scale = 4.0
		m.turbulence_influence_min = 0.05
		m.turbulence_influence_max = 0.15
	else:
		m.particle_flag_align_y = true
	p.process_material = m
	var mesh := QuadMesh.new()
	var mat := StandardMaterial3D.new()
	mat.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	mat.transparency = BaseMaterial3D.TRANSPARENCY_ALPHA
	mat.vertex_color_use_as_albedo = true
	mat.cull_mode = BaseMaterial3D.CULL_DISABLED
	var col: Array = _vec("flake_color" if snow else "drop_color", [1.0, 1.0, 1.0, 0.5])
	mat.albedo_color = Color(col[0], col[1], col[2], col[3])
	if snow:
		var s := float(_cfg.get("flake_size_m", 0.035))
		mesh.size = Vector2(s, s)
		mat.billboard_mode = BaseMaterial3D.BILLBOARD_PARTICLES
	else:
		var size: Array = _vec("drop_size_m", [0.012, 0.55])
		mesh.size = Vector2(size[0], size[1])
		# Streak faces the camera about its own (velocity-aligned) axis.
		mat.billboard_mode = BaseMaterial3D.BILLBOARD_FIXED_Y
		mat.billboard_keep_scale = true
	mesh.material = mat
	p.draw_pass_1 = mesh
	add_child(p)
	return p

func _process(delta: float) -> void:
	if not _ok:
		return
	wet_look = simulation.get_wet_look(delta)
	SurfaceMaterials.set_wet(float(wet_look.get("wetness", 0.0)), float(wet_look.get("puddles", 0.0)))
	var cam: Camera3D = get_viewport().get_camera_3d()
	if cam == null:
		return
	var origin: Vector3 = simulation.get_render_origin_session()
	var rebased := origin != _last_origin
	_last_origin = origin
	var h := float(_cfg.get("height_m", 8.0))
	for pair in [[_rain, float(wet_look.get("drops", 0.0))], [_snow, float(wet_look.get("flakes", 0.0))]]:
		var p: GPUParticles3D = pair[0]
		var ratio := clampf(float(pair[1]) * density, 0.0, 1.0)
		p.global_position = cam.global_position + Vector3(0.0, h, 0.0)
		p.amount_ratio = ratio
		var on := ratio > 0.001
		if on != p.emitting:
			p.emitting = on
		if rebased and on:
			p.restart()
