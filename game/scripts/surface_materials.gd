extends RefCounted
# Ground and road textures for terrain.gdshader / road.gdshader (presentation only).
#
# Reads data/textures/surface_textures.json (rg.surface_textures/1), loads every layer's colour and normal map once per
# process into two Texture2DArrays (mipmapped; block-compressed when the running Godot has a compressor, i.e. the
# editor binary the game runs on), and hands them plus the per-layer numbers to a ShaderMaterial. The global shader
# parameters (project.godot [shader_globals]) are set here too: rg_surface_detail (the graphics.surface_detail setting),
# rg_origin_mod (render origin modulo 1024 m, see terrain.gdshader), rg_wetness / rg_puddles (rg/wet_look.h, fed by
# weather_fx.gd).
#
# Use: SurfaceMaterials.shared().apply_terrain(mat) / apply_road(mat) / apply_ground(mat); SurfaceMaterials.set_detail(
# "high"); SurfaceMaterials.set_render_origin(origin_session).

const CONFIG := "textures/surface_textures.json"
const ORIGIN_PERIOD_M := 1024.0
const DETAIL_LEVELS := {"off": 0, "low": 1, "high": 2}

static var _shared: RefCounted = null

var ok := false
var error := ""
var load_ms := 0.0
var vram_bytes := 0
var compressed := false
var _config: Dictionary = {}
var _layer_index: Dictionary = {}
var _albedo: Texture2DArray
var _normal: Texture2DArray
var _tile := PackedFloat32Array()
var _tint := PackedVector3Array()
var _mean := PackedVector3Array()
var _rough := PackedFloat32Array()
var _strength := PackedFloat32Array()

static func data_dir() -> String:
	return ProjectSettings.globalize_path("res://").path_join("..").path_join("data").simplify_path()

static func shared() -> RefCounted:
	if _shared == null:
		_shared = load("res://scripts/surface_materials.gd").new()
		_shared._load()
	return _shared

static func set_detail(choice: String) -> void:
	RenderingServer.global_shader_parameter_set("rg_surface_detail", int(DETAIL_LEVELS.get(choice, 2)))

static func set_render_origin(origin_session: Vector3) -> void:
	# Taken in double precision here (GDScript floats are 64-bit), so the shader only ever sees 0..1024 m.
	RenderingServer.global_shader_parameter_set("rg_origin_mod", Vector2(fposmod(origin_session.x, ORIGIN_PERIOD_M), fposmod(origin_session.y, ORIGIN_PERIOD_M)))

static func set_wet(wetness: float, puddles: float) -> void:
	RenderingServer.global_shader_parameter_set("rg_wetness", clampf(wetness, 0.0, 1.0))
	RenderingServer.global_shader_parameter_set("rg_puddles", clampf(puddles, 0.0, 1.0))

func layer(id: String) -> int:
	return int(_layer_index.get(id, 0))

func _load() -> void:
	var t0 := Time.get_ticks_usec()
	var path := data_dir().path_join(CONFIG)
	var parsed = JSON.parse_string(FileAccess.get_file_as_string(path))
	if not (parsed is Dictionary) or str(parsed.get("format", "")) != "rg.surface_textures/1":
		error = "%s: missing or not rg.surface_textures/1" % path
		push_warning("surface textures: " + error)
		return
	_config = parsed
	var size := int(_config.get("texture_size", 1024))
	var albedo_images: Array[Image] = []
	var normal_images: Array[Image] = []
	var layers: Array = _config.get("layers", [])
	if layers.is_empty() or layers.size() > 8:
		error = "%s: 1..8 layers expected" % path
		push_warning("surface textures: " + error)
		return
	for i in range(layers.size()):
		var l: Dictionary = layers[i]
		_layer_index[str(l["id"])] = i
		var color := _image(str(l["color"]), size, false)
		var normal := _image(str(l["normal"]), size, true)
		if color == null or normal == null:
			return
		_mean.append(_mean_colour(color))
		albedo_images.append(color)
		normal_images.append(normal)
		_tile.append(float(l.get("tile_m", 4.0)))
		var tint: Array = l.get("tint", [1, 1, 1])
		_tint.append(Vector3(float(tint[0]), float(tint[1]), float(tint[2])))
		_rough.append(float(l.get("roughness", 0.9)))
		_strength.append(float(l.get("normal_strength", 1.0)))
	# Pad to 8 so the uniform arrays are always full.
	while _tile.size() < 8:
		_tile.append(4.0); _tint.append(Vector3.ONE); _mean.append(Vector3(0.5, 0.5, 0.5)); _rough.append(0.9); _strength.append(0.0)
	_albedo = _array(albedo_images, Image.COMPRESS_SOURCE_SRGB)
	_normal = _array(normal_images, Image.COMPRESS_SOURCE_NORMAL)
	ok = _albedo != null and _normal != null
	load_ms = (Time.get_ticks_usec() - t0) / 1000.0
	print("RG_SURFACE_TEXTURES layers=%d size=%d compressed=%s vram_mb=%.1f load_ms=%.0f" % [layers.size(), size, str(compressed), vram_bytes / 1048576.0, load_ms])

func _image(rel: String, size: int, is_normal: bool) -> Image:
	var file := data_dir().path_join("textures").path_join(rel)
	var img := Image.load_from_file(file)
	if img == null or img.is_empty():
		error = "cannot load " + file
		push_warning("surface textures: " + error)
		return null
	img.convert(Image.FORMAT_RGBA8 if not is_normal else Image.FORMAT_RGB8)
	if img.get_width() != size or img.get_height() != size:
		img.resize(size, size, Image.INTERPOLATE_LANCZOS)
	img.generate_mipmaps(is_normal)
	return img

func _mean_colour(img: Image) -> Vector3:
	# Box-filtered mean of the whole map, in the map's own (sRGB) encoding: the gl_compatibility renderer hands
	# source_color samples to the shader encoded, so the far-distance mean colour must match that.
	var small := img.duplicate() as Image
	small.clear_mipmaps()
	while small.get_width() > 1:
		small.shrink_x2()
	var c := small.get_pixel(0, 0)
	return Vector3(c.r, c.g, c.b)

func _array(images: Array[Image], source: int) -> Texture2DArray:
	var use_compression := true
	for img in images:
		if img.compress(Image.COMPRESS_S3TC, source) != OK:
			use_compression = false
			break
	if not use_compression:
		for img in images:
			if img.is_compressed():
				img.decompress()
	compressed = use_compression
	var arr := Texture2DArray.new()
	if arr.create_from_images(images) != OK:
		error = "Texture2DArray.create_from_images failed"
		push_warning("surface textures: " + error)
		return null
	for img in images:
		vram_bytes += img.get_data_size()
	return arr

func _set_common(mat: ShaderMaterial) -> void:
	if not ok:
		return
	mat.set_shader_parameter("surface_albedo", _albedo)
	mat.set_shader_parameter("surface_normal", _normal)
	mat.set_shader_parameter("layer_tile_m", _tile)
	mat.set_shader_parameter("layer_tint", _tint)
	mat.set_shader_parameter("layer_mean", _mean)
	mat.set_shader_parameter("layer_roughness", _rough)
	mat.set_shader_parameter("layer_normal_strength", _strength)
	var wet: Dictionary = _config.get("wet", {})
	for key in ["darken", "wet_roughness", "wet_specular", "puddle_roughness", "puddle_darken", "puddle_tile_m", "puddle_cover", "puddle_spread", "ground_wet_roughness", "ground_wet_specular"]:
		if wet.has(key):
			mat.set_shader_parameter("wet_darken" if key == "darken" else key, float(wet[key]))

func apply_terrain(mat: ShaderMaterial) -> void:
	_set_common(mat)
	var t: Dictionary = _config.get("terrain", {})
	for key in ["grass", "dirt", "rock", "paved", "unpaved"]:
		mat.set_shader_parameter("layer_" + key, layer(str(t.get(key, ""))))
	mat.set_shader_parameter("layer_macro", layer(str(t.get("macro_layer", "dirt"))))
	var g: Array = t.get("greenness", [-0.02, 0.12])
	mat.set_shader_parameter("greenness", Vector2(float(g[0]), float(g[1])))
	var r: Array = t.get("rock_slope", [0.18, 0.4])
	mat.set_shader_parameter("rock_slope", Vector2(float(r[0]), float(r[1])))
	mat.set_shader_parameter("macro_tile_m", float(t.get("macro_tile_m", 128.0)))
	mat.set_shader_parameter("macro_strength", float(t.get("macro_strength", 0.22)))
	mat.set_shader_parameter("detail_far_m", float(t.get("detail_far_m", 600.0)))

func apply_road(mat: ShaderMaterial) -> void:
	_set_common(mat)
	var r: Dictionary = _config.get("road", {})
	for key in ["asphalt", "concrete", "sett", "unpaved", "verge"]:
		mat.set_shader_parameter("layer_" + key, layer(str(r.get(key, ""))))
	mat.set_shader_parameter("layer_paving", layer(str(r.get("paving_stones", ""))))
	mat.set_shader_parameter("layer_macro", layer(str(_config.get("terrain", {}).get("macro_layer", "dirt"))))
	mat.set_shader_parameter("detail_far_m", float(r.get("detail_far_m", 400.0)))

# The flat test world's ground box: one layer, placed by the box's own local x/y, with a painted grid.
func apply_ground(mat: ShaderMaterial) -> void:
	apply_terrain(mat)
	var g: Dictionary = _config.get("flat_ground", {})
	mat.set_shader_parameter("force_layer", layer(str(g.get("layer", "asphalt"))))
	mat.set_shader_parameter("local_uv", true)
	mat.set_shader_parameter("grid_spacing_m", float(g.get("grid_spacing_m", 50.0)))
	mat.set_shader_parameter("grid_line_width_m", float(g.get("grid_line_width_m", 0.15)))
	mat.set_shader_parameter("macro_strength", 0.0)
