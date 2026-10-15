extends RefCounted
# Car level of detail by camera distance (perf, 2026-10-15). A rig.json "lods" list names LOD1 / LOD2 .glb files with
# the same nodes, pivots and material names as LOD0 (tools/cargen); "quality_tiers.high.lod_switch_m" gives the camera
# distances at which LOD1, then LOD2 take over (default 15 m and 60 m). attach() puts, under every LOD0 mesh node, a
# child MeshInstance3D per coarser LOD with that node's mesh from the LOD file, and sets the visibility ranges (LOD0
# [0, d1), LOD1 [d1, d2), LOD2 [d2, inf)). Children follow every joint, wheel and option node of LOD0 as they are, and
# the colour / lamp scripts find their surfaces like any other (role and lamp materials are matched by name). The
# chase, bumper, cockpit and garage cameras stay inside d1 at "high", so what they show is unchanged; orbit, drone, cinematic
# and free cameras far from the car draw fewer triangles. Draw calls do not change (one LOD of each part is drawn).
# Hysteresis: LOD_MARGIN_M around each switch distance (no fade).

const LOD_MARGIN_M := 1.0
# Setting graphics.car_lod (main.gd): "full" = LOD0 only (the default, the full look), else the rig's quality tier.
static var tier := "full"

static func attach(model: Node3D, model_path: String, use_tier: String = "") -> int:
	var chosen := use_tier if use_tier != "" else tier
	if chosen == "full":
		return 0
	var rig_path := model_path.get_basename() + ".rig.json"
	if not FileAccess.file_exists(rig_path):
		return 0
	var rig: Variant = JSON.parse_string(FileAccess.get_file_as_string(rig_path))
	if not rig is Dictionary or not rig.has("lods"):
		return 0
	var switch_m: Array = rig.get("quality_tiers", {}).get(chosen, {}).get("lod_switch_m", [15.0, 60.0])
	var levels := {}
	for entry in rig["lods"]:
		var level := int(entry.get("level", 0))
		if level >= 1 and level <= switch_m.size():
			levels[level] = model_path.get_base_dir().path_join(str(entry.get("glb", "")))
	if levels.is_empty():
		return 0
	var bounds := [0.0]
	for d in switch_m:
		bounds.append(float(d))
	bounds.append(0.0) # 0 = no end
	var maps := {} # level -> {node name: MeshInstance3D of that LOD}
	var roots: Array = []
	for level in levels:
		var path: String = levels[level]
		if not FileAccess.file_exists(path):
			push_warning("car_lods.gd: LOD%d file missing: %s" % [level, path])
			continue
		var doc := GLTFDocument.new()
		var state := GLTFState.new()
		if doc.append_from_file(path, state) != OK:
			push_warning("car_lods.gd: cannot load %s" % path)
			continue
		var lod_root := doc.generate_scene(state)
		if lod_root == null:
			continue
		roots.append(lod_root)
		var by_name := {}
		for n in lod_root.find_children("*", "MeshInstance3D", true, false):
			if (n as MeshInstance3D).mesh != null:
				by_name[str(n.name)] = n
		maps[level] = by_name
	var added := 0
	# Each part switches only between the LODs that have it: a part a coarser file leaves out keeps the previous LOD
	# up to the next one that has it (or for good), so nothing disappears.
	for node in model.find_children("*", "MeshInstance3D", true, false):
		var mi := node as MeshInstance3D
		if mi.mesh == null:
			continue
		var present: Array = [0]
		for level in range(1, bounds.size() - 1):
			if maps.has(level) and maps[level].has(str(mi.name)):
				present.append(level)
		if present.size() == 1:
			continue
		for i in present.size():
			var level: int = present[i]
			var end: float = bounds[present[i + 1]] if i + 1 < present.size() else 0.0
			var target := mi
			if level > 0:
				var source: MeshInstance3D = maps[level][str(mi.name)]
				target = MeshInstance3D.new()
				target.name = "%s_lod%d" % [mi.name, level]
				target.mesh = source.mesh
				target.cast_shadow = mi.cast_shadow
				target.layers = mi.layers
				target.visibility_range_begin = bounds[level]
				target.visibility_range_begin_margin = LOD_MARGIN_M
				mi.add_child(target)
				added += 1
			target.visibility_range_end = end
			target.visibility_range_end_margin = LOD_MARGIN_M if end > 0.0 else 0.0
	for r in roots:
		r.free()
	return added
