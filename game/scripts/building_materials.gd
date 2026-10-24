extends RefCounted
# Thin PBR bindings for engine-neutral material IDs. Assets and physical repeat
# scales live in data/textures/buildings; no proprietary BeamNG content/shader.
static var _shared: Array[StandardMaterial3D] = []
static func shared() -> Array[StandardMaterial3D]:
	if not _shared.is_empty():
		return _shared
	var root := ProjectSettings.globalize_path("res://../data/textures/buildings").simplify_path()
	var path := root.path_join("materials.json")
	var config: Dictionary = JSON.parse_string(FileAccess.get_file_as_string(path))
	assert(str(config.get("format","")) == "rg.building_materials/1")
	var bytes := 0
	var count := 0
	for entry: Dictionary in config.materials:
		var material := StandardMaterial3D.new()
		material.vertex_color_use_as_albedo = true
		material.cull_mode = BaseMaterial3D.CULL_DISABLED
		material.roughness = float(entry.get("roughness",.9))
		if str(entry.name) == "glass":
			material.metallic = .25
		if str(entry.name) in ["metal","cladding"]:
			material.metallic = .7
		var maps: Dictionary = entry.get("maps",{})
		for channel: String in maps:
			var file := root.path_join(str(maps[channel]))
			if not FileAccess.file_exists(file):
				push_warning("Building textures missing; run python tools/fetch_building_assets.py: " + file)
				continue
			var image := Image.load_from_file(file)
			if image == null or image.is_empty():
				push_warning("Cannot load building texture: " + file)
				continue
			image.generate_mipmaps(channel == "nor_gl")
			# Reuse the ground-texture compression policy; exported runtimes without
			# a compressor retain mipmapped pixels, never lose the texture.
			image.compress(Image.COMPRESS_S3TC,Image.COMPRESS_SOURCE_NORMAL if channel == "nor_gl" else Image.COMPRESS_SOURCE_GENERIC)
			bytes += image.get_data_size()
			var texture := ImageTexture.create_from_image(image)
			count += 1
			if channel == "Diffuse":
				material.albedo_texture = texture
			elif channel == "nor_gl":
				material.normal_enabled = true
				material.normal_texture = texture
				material.normal_scale = float(entry.get("normal_strength",.6))
			elif channel == "Metal":
				material.metallic = 1.0
				material.metallic_texture = texture
				material.metallic_texture_channel = BaseMaterial3D.TEXTURE_CHANNEL_RED
			elif channel == "Rough":
				material.roughness_texture = texture
				material.roughness_texture_channel = BaseMaterial3D.TEXTURE_CHANNEL_RED
		if entry.has("tile_m"):
			material.uv1_scale = Vector3(1.0/float(entry.tile_m[0]),1.0/float(entry.tile_m[1]),1.0)
		_shared.append(material)
	print("RG_BUILDING_MATERIALS materials=%d maps=%d vram_mb=%.2f" % [_shared.size(),count,bytes/1048576.0])
	return _shared
