extends SceneTree
# tools/cargen/godot_shots.gd - garage-style review renders of a generated car through Godot's own renderer
# (no GDExtension, no game data): studio sky with softboxes (what the paint reflects), key light with shadows,
# grey floor. Needs a real window (xvfb works):
#   xvfb-run -s "-screen 0 1600x900x24" godot --path game --script <abs>/tools/cargen/godot_shots.gd \
#       -- <model.glb> <out_dir> [prefix]
# Writes <prefix>front_34.png, rear_34.png, side.png, wheel_close.png, front_close.png, rear_close.png,
# interior.png. The .glb uses glTF axes (car faces +Z, +X = left, +Y up).

var shots := [
	# name, camera position, look-at target, fov
	["front_34", Vector3(-3.6, 1.45, 4.6), Vector3(0, 0.55, 0.2), 34.0],
	["rear_34", Vector3(3.4, 1.55, -4.6), Vector3(0, 0.6, -0.2), 34.0],
	["side", Vector3(-6.4, 0.9, 0.0), Vector3(0, 0.6, 0.0), 34.0],
	["wheel_close", Vector3(-2.1, 0.55, 2.35), Vector3(-0.75, 0.35, 1.25), 34.0],
	["front_close", Vector3(-1.4, 0.95, 3.6), Vector3(-0.2, 0.6, 1.9), 34.0],
	["rear_close", Vector3(1.4, 1.0, -3.5), Vector3(0.2, 0.7, -1.9), 34.0],
	["interior", Vector3(-1.9, 1.35, 0.1), Vector3(0.1, 0.8, 0.35), 50.0],
]

func _init() -> void:
	var args := OS.get_cmdline_user_args()
	if args.size() < 2:
		push_error("usage: -- <model.glb> <out_dir> [prefix]")
		quit(2)
		return
	_run.call_deferred(args[0], args[1], args[2] if args.size() > 2 else "")

func _sky() -> Sky:
	var shader := Shader.new()
	shader.code = """
shader_type sky;
void sky() {
	vec3 d = normalize(EYEDIR);
	float h = d.y;
	vec3 col = h > 0.0 ? mix(vec3(0.42, 0.44, 0.48), vec3(0.20, 0.22, 0.26), pow(h, 0.5))
			: mix(vec3(0.42, 0.44, 0.48), vec3(0.10, 0.10, 0.11), pow(-h, 0.5));
	// two long softboxes overhead and one at each side: the highlights that show facets in the paint
	float a = atan(d.x, d.z);
	col += vec3(2.2) * smoothstep(0.80, 0.84, h) * (1.0 - smoothstep(0.93, 0.97, h)) * step(0.0, cos(a * 2.0));
	col += vec3(1.6) * (1.0 - smoothstep(0.08, 0.12, abs(h - 0.25))) * (1.0 - smoothstep(0.25, 0.32, abs(abs(d.x) - 0.92)));
	COLOR = col;
}
"""
	var mat := ShaderMaterial.new()
	mat.shader = shader
	var sky := Sky.new()
	sky.sky_material = mat
	sky.radiance_size = Sky.RADIANCE_SIZE_256
	return sky

func _run(glb: String, out_dir: String, prefix: String) -> void:
	DirAccess.make_dir_recursive_absolute(out_dir)
	var root := Node3D.new()
	get_root().add_child(root)
	var env := Environment.new()
	env.background_mode = Environment.BG_SKY
	env.sky = _sky()
	env.ambient_light_source = Environment.AMBIENT_SOURCE_SKY
	env.ambient_light_energy = 0.7
	env.reflected_light_source = Environment.REFLECTION_SOURCE_SKY
	env.tonemap_mode = Environment.TONE_MAPPER_ACES
	env.tonemap_exposure = 1.0
	var we := WorldEnvironment.new()
	we.environment = env
	root.add_child(we)
	var sun := DirectionalLight3D.new()
	sun.rotation_degrees = Vector3(-55, -35, 0)
	sun.light_energy = 1.2
	sun.shadow_enabled = true
	sun.directional_shadow_max_distance = 20.0
	root.add_child(sun)
	var fill := DirectionalLight3D.new()
	fill.rotation_degrees = Vector3(-25, 150, 0)
	fill.light_energy = 0.5
	root.add_child(fill)
	var floor := MeshInstance3D.new()
	var plane := PlaneMesh.new()
	plane.size = Vector2(40, 40)
	floor.mesh = plane
	var fm := StandardMaterial3D.new()
	fm.albedo_color = Color(0.16, 0.165, 0.17)
	fm.roughness = 0.6
	floor.material_override = fm
	root.add_child(floor)

	var doc := GLTFDocument.new()
	var state := GLTFState.new()
	var err := doc.append_from_file(glb, state)
	if err != OK:
		push_error("cannot load %s (%d)" % [glb, err])
		quit(1)
		return
	var car: Node = doc.generate_scene(state)
	root.add_child(car)
	# hide optional variants (hidden child nodes in the rig: hood variants, tops)
	for name in ["hood_bulge", "hood_scoop", "top"]:
		var n := car.find_child(name, true, false)
		if n is Node3D:
			(n as Node3D).visible = false

	var cam := Camera3D.new()
	root.add_child(cam)
	cam.current = true
	for i in range(8):
		await process_frame
	for s in shots:
		cam.fov = s[3]
		cam.position = s[1]
		cam.look_at(s[2], Vector3.UP)
		for i in range(4):
			await process_frame
		await RenderingServer.frame_post_draw
		var img := get_root().get_texture().get_image()
		var path: String = out_dir.path_join(prefix + s[0] + ".png")
		img.save_png(path)
		print("SHOT ", path)
	quit(0)
