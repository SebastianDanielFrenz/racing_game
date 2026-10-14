extends RefCounted
# Optional model parts (rig.json nodes carrying an "option"): the Runabout's folding top "roof" (a setup option) and
# the Ventura GT's raised hoods "hood_bulge" / "hood_scoop" (picked by the installed engine's height). A part is
# shown when its option is in `on`, hidden otherwise. Used by vehicle_visual.gd, garage_scene.gd and car_thumbnails.gd.

static func _rig(rig_path: String) -> Dictionary:
	if rig_path == "" or not FileAccess.file_exists(rig_path):
		return {}
	var rig: Variant = JSON.parse_string(FileAccess.get_file_as_string(rig_path))
	return rig if rig is Dictionary else {}

# `root` is the loaded model, `rig_path` its rig.json, `on` the option names switched on (Array / PackedStringArray).
static func apply(root: Node, rig_path: String, on: Variant) -> void:
	if root == null:
		return
	var wanted := Array(on)
	for n in _rig(rig_path).get("nodes", []):
		var option := str(n.get("option", ""))
		if option == "":
			continue
		var node: Node3D = root.find_child(str(n.get("name", "")), true, false)
		if node != null:
			node.visible = wanted.has(option)

# The raised hood a tall engine needs: the lowest of the rig's engine_bay.hood_variants whose clearance takes
# `engine_tall_mm`; "" for the flat hood (or a car without variants).
static func hood_variant(rig_path: String, engine_tall_mm: float) -> String:
	var variants: Dictionary = _rig(rig_path).get("engine_bay", {}).get("hood_variants", {})
	var pick := ""
	var best := INF
	for v in variants:
		var c := float(variants[v].get("clearance_mm", 0.0))
		if c >= engine_tall_mm and c < best:
			best = c
			pick = str(v)
	return "" if pick == "flat" else pick
