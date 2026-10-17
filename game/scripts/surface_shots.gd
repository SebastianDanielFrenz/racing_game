extends SceneTree
# Review shots of the ground and road materials without map data (real renderer, e.g. under xvfb):
#   godot --path game --resolution 1280x720 --script res://scripts/surface_shots.gd -- --out <dir> [--detail off|low|high]
# A synthetic 240 m hill field (ramp colours like rg::chunk_vertex_colors: green lowland, brown higher, a steep rock
# face, a paved and an unpaved strip) and four 80 m road ribbons in the ribbon vertex format of road_stream.gd
# (asphalt with markings, concrete, paving stones, sett, each with gravel verges). Writes dry_* and wet_* PNGs
# (wet = rg_wetness 1, rg_puddles 1) from three cameras and prints RG_SURFACE_SHOT lines with the frame time.
const SurfaceMaterials := preload("res://scripts/surface_materials.gd")

var _out := ""
var _detail := "high"
var _cams: Array[Camera3D] = []
var _frame := 0
var _step := 0
var _plan: Array = []
var _t_last := 0

func _init() -> void:
	var args := OS.get_cmdline_user_args()
	for i in range(args.size()):
		if args[i] == "--out" and i + 1 < args.size(): _out = args[i + 1]
		if args[i] == "--detail" and i + 1 < args.size(): _detail = args[i + 1]
	if _out == "":
		push_error("surface_shots: --out <dir> required")
		quit(2)
		return
	DirAccess.make_dir_recursive_absolute(_out)
	SurfaceMaterials.set_detail(_detail)
	SurfaceMaterials.set_render_origin(Vector3.ZERO)
	SurfaceMaterials.set_wet(0.0, 0.0)
	var world := Node3D.new()
	root.add_child(world)
	_environment(world)
	_terrain(world)
	var lanes_flags := 1 | 2
	_road(world, Vector3(0, 0, 0), 7.0, 2, 1, lanes_flags)       # asphalt, edge + centre line
	_road(world, Vector3(14, 0, 0), 7.0, 2, 1, lanes_flags | 4096) # concrete
	_road(world, Vector3(28, 0, 0), 6.0, 1, 1, 2048)               # paving stones, no paint
	_road(world, Vector3(42, 0, 0), 6.0, 1, 1, 2048 | 16384)       # sett
	_camera(world, Vector3(1.8, 1.3, 30.0), Vector3(1.5, 0.2, -40.0))   # driver eye on the asphalt road
	_camera(world, Vector3(-12.0, 14.0, 34.0), Vector3(30.0, 0.0, -10.0)) # all four surfaces from above
	_camera(world, Vector3(70.0, 30.0, 80.0), Vector3(-30.0, 0.0, -60.0)) # hills, rock, the strips
	for wet in [false, true]:
		for i in range(_cams.size()):
			_plan.append([wet, i])

func _environment(world: Node3D) -> void:
	var env := Environment.new()
	env.background_mode = Environment.BG_SKY
	var sky := Sky.new()
	sky.sky_material = ProceduralSkyMaterial.new()
	env.sky = sky
	env.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	env.reflected_light_source = Environment.REFLECTION_SOURCE_SKY
	env.tonemap_mode = Environment.TONE_MAPPER_FILMIC
	var we := WorldEnvironment.new()
	we.environment = env
	world.add_child(we)
	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-35.0, -150.0, 0.0)
	sun.shadow_enabled = true
	world.add_child(sun)

func _height(x: float, z: float) -> float:
	var h := 3.0 * sin(x * 0.03) * cos(z * 0.025) + 1.5 * sin(z * 0.07 + 1.0)
	if x < -60.0: # a steep face for the rock layer
		h += (-60.0 - x) * 0.9
	# Flat where the roads lie.
	var road := smoothstep(70.0, 50.0, x) * smoothstep(-30.0, -18.0, x) * smoothstep(-70.0, -45.0, z) * smoothstep(60.0, 45.0, z)
	return lerpf(h, -0.05, road)

func _ramp(x: float, z: float, h: float) -> Color:
	# Lowland green, browner higher up (rg::chunk_vertex_colors shape), alpha = kind (1 natural, 0 road with the
	# asphalt grey or dirt brown paint).
	if absf(x + 12.0) < 3.5 and z > -60.0:
		return Color(0.30, 0.30, 0.32, 0.0) # a paved strip on the terrain itself
	if absf(z - 70.0) < 2.5:
		return Color(0.36, 0.28, 0.16, 0.0) # an unpaved track
	var t := clampf((h + 3.0) / 12.0, 0.0, 1.0)
	return Color(0.30, 0.42, 0.22).lerp(Color(0.48, 0.40, 0.28), t)

func _terrain(world: Node3D) -> void:
	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	var n := 120
	var step := 2.0
	var x0 := -120.0
	for j in range(n):
		for i in range(n):
			var quad := [[i, j], [i + 1, j], [i + 1, j + 1], [i, j], [i + 1, j + 1], [i, j + 1]]
			for v in quad:
				var x: float = x0 + v[0] * step
				var z: float = x0 + v[1] * step
				var h := _height(x, z)
				var nx := _height(x - 0.5, z) - _height(x + 0.5, z)
				var nz := _height(x, z - 0.5) - _height(x, z + 0.5)
				st.set_normal(Vector3(nx, 1.0, nz).normalized())
				st.set_color(_ramp(x, z, h))
				st.add_vertex(Vector3(x, h, z))
	var mi := MeshInstance3D.new()
	mi.mesh = st.commit()
	var mat := ShaderMaterial.new()
	mat.shader = load("res://shaders/terrain.gdshader")
	SurfaceMaterials.shared().apply_terrain(mat)
	mi.material_override = mat
	world.add_child(mi)

func _road(world: Node3D, origin: Vector3, width: float, lanes: int, forward: int, flags: int) -> void:
	var road_mat := ShaderMaterial.new()
	road_mat.shader = load("res://shaders/road.gdshader")
	SurfaceMaterials.shared().apply_road(road_mat)
	var st := SurfaceTool.new()
	st.begin(Mesh.PRIMITIVE_TRIANGLES)
	var length := 80.0
	var hw := width * 0.5
	# Road bed, then two 0.3 m verges (flag 256).
	_strip(st, origin, -hw, hw, length, width, lanes, forward, flags)
	_strip(st, origin, -hw - 0.3, -hw, length, width, lanes, forward, 256)
	_strip(st, origin, hw, hw + 0.3, length, width, lanes, forward, 256)
	var mi := MeshInstance3D.new()
	mi.mesh = st.commit()
	mi.material_override = road_mat
	world.add_child(mi)

func _strip(st: SurfaceTool, o: Vector3, t0: float, t1: float, length: float, width: float, lanes: int, forward: int, flags: int) -> void:
	var col := Color(lanes / 32.0, forward / 32.0, float(flags & 255) / 255.0, float(flags >> 8) / 255.0)
	var rows := int(length / 2.0)
	for r in range(rows):
		var s0 := r * 2.0
		var s1 := s0 + 2.0
		var corners := [[t0, s0], [t1, s0], [t1, s1], [t0, s0], [t1, s1], [t0, s1]]
		for c in corners:
			st.set_normal(Vector3.UP)
			st.set_color(col)
			st.set_uv(Vector2(c[0], c[1]))
			st.set_uv2(Vector2(width, 1.0))
			# Lateral +t to the right (+x), along the road towards -z.
			st.add_vertex(o + Vector3(c[0], 0.02, 40.0 - c[1]))

func _camera(world: Node3D, pos: Vector3, target: Vector3) -> void:
	var cam := Camera3D.new()
	cam.fov = 70.0
	cam.far = 2000.0
	world.add_child(cam)
	cam.look_at_from_position(pos, target, Vector3.UP)
	_cams.append(cam)

func _process(_delta: float) -> bool:
	_frame += 1
	if _step >= _plan.size():
		quit(0)
		return true
	var item: Array = _plan[_step]
	if _frame == 1:
		SurfaceMaterials.set_wet(1.0 if item[0] else 0.0, 1.0 if item[0] else 0.0)
		_cams[item[1]].current = true
	elif _frame == 8:
		_t_last = Time.get_ticks_usec()
	elif _frame == 12:
		var ms := (Time.get_ticks_usec() - _t_last) / 4000.0
		var name := "%s_%d_%s.png" % ["wet" if item[0] else "dry", item[1], _detail]
		root.get_viewport().get_texture().get_image().save_png(_out.path_join(name))
		print("RG_SURFACE_SHOT %s frame_ms=%.1f" % [name, ms])
		_step += 1
		_frame = 0
	return false
