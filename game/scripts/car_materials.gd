extends RefCounted
# Shared look of recoloured car materials (paint clear coat, rims, glass) for the garage and
# the drive. Numbers live in data/garage/car_materials.json; this file only applies them.
const PATH := "res://../data/garage/car_materials.json"
static var _look: Dictionary = {}

static func look() -> Dictionary:
	if _look.is_empty():
		var parsed = JSON.parse_string(FileAccess.get_file_as_string(ProjectSettings.globalize_path(PATH)))
		if parsed is Dictionary and parsed.get("format", "") == "rg.car_materials/1":
			_look = parsed
		else:
			push_warning("car_materials.gd: cannot read " + PATH)
			_look = {"paint": {}, "rim": {}, "glass": {"match": []}}
	return _look

# "paint", "rim", "glass" or "" for a material.
static func role_of(mat: Material) -> String:
	var n := mat.resource_name.to_lower()
	if n.begins_with("paint"):
		return "paint"
	if n.begins_with("rim"):
		return "rim"
	var g: Dictionary = look().get("glass", {})
	if mat is BaseMaterial3D and (mat as BaseMaterial3D).transparency != BaseMaterial3D.TRANSPARENCY_DISABLED:
		for word in g.get("exclude", []):
			if n.contains(str(word)):
				return ""
		for word in g.get("match", []):
			if n.contains(str(word)):
				return "glass"
	return ""

# Styles a material copy of the given role. colour.a == 0 keeps the model's own colour.
static func style(std: StandardMaterial3D, role: String, colour: Color) -> void:
	var spec: Dictionary = look().get(role, {})
	if role == "paint" or role == "rim":
		if colour.a > 0.0:
			std.vertex_color_use_as_albedo = false
			if not std.resource_name.to_lower().begins_with("paint_tinted"):
				std.albedo_texture = null
			std.albedo_color = colour
	if spec.has("metallic"):
		std.metallic = float(spec["metallic"])
	if spec.has("roughness"):
		std.roughness = float(spec["roughness"])
	if spec.has("metallic_specular"):
		std.metallic_specular = float(spec["metallic_specular"])
	if spec.has("clearcoat"):
		std.clearcoat_enabled = true
		std.clearcoat = float(spec["clearcoat"])
		std.clearcoat_roughness = float(spec.get("clearcoat_roughness", 0.1))
