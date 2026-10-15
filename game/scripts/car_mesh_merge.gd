extends RefCounted
# Draw-call reduction for a loaded car model (perf, 2026-10-15). Each mesh surface is one draw call in the colour pass
# and one more per sun-shadow cascade it reaches; the shipped car_hyper is 160 surfaces once the cockpit parts are
# mounted. Two steps:
#  1. merge(): the model is split into rigid groups - the model root plus every node that moves or is switched on its
#     own (rig nodes with a joint, except doors / lids, which only the garage opens; rig "option" parts; wheels; the
#     jointed controls of the mounted cockpit parts; every node a .cockpit.json names). Inside each group, surfaces
#     that share a material are baked, in the group node's frame, into one surface of a "MergedStatic" MeshInstance3D
#     under that node, so they still move with it. Picture-identical: same vertices, same material objects. Kept as
#     they are: a group node's own mesh (scripts read it), materials used once in a group (nothing to gain), lamp
#     materials (their emission is switched), transparent surfaces (depth sorting is per instance), anything hidden,
#     skinned or non-uniformly scaled, and the CockpitControls / LiveInstruments subtrees.
#  2. small cockpit parts (largest dimension under SMALL_PART_M: switches, knobs, buttons, stalks) stop casting sun
#     shadows. This one is a look change, a very small one: those parts sit under the roof, and their own shadow on
#     the dash in direct sun is a few millimetres.
# Used by vehicle_visual.gd after the model, its colours, lamps and cockpit are set up, and by garage_scene.gd (with
# all_joints: the hood opens there) before the mirror twin is copied. Colours keep working: the merged surfaces carry
# the (override) material copies the colour code restyles. RG_CAR_MERGE=0 turns both steps off (A/B).

const STATIC_JOINT_PREFIXES := ["door_", "frunk", "engine_cover", "hood", "bonnet", "trunk", "boot", "tailgate"]
const KEEP_SUBTREES := ["CockpitControls", "LiveInstruments"]
const SMALL_PART_M := 0.15
const PARTS_RIG := "res://../data/models/cockpit_parts/cockpit_parts.rig.json"

static func _json(path: String) -> Dictionary:
	if path == "" or not FileAccess.file_exists(path):
		return {}
	var v: Variant = JSON.parse_string(FileAccess.get_file_as_string(path))
	return v if v is Dictionary else {}

# Names of the nodes a .cockpit.json animates (every string under a "node" key, at any depth).
static func _cockpit_nodes(v: Variant, out: Dictionary) -> void:
	if v is Dictionary:
		for k in v:
			if k == "node" and v[k] is String:
				out[v[k]] = true
			else:
				_cockpit_nodes(v[k], out)
	elif v is Array:
		for e in v:
			_cockpit_nodes(e, out)

# Transform of `node` in `ancestor`'s frame (no scene tree needed).
static func _in(node: Node3D, ancestor: Node) -> Transform3D:
	var t := Transform3D.IDENTITY
	var n: Node = node
	while n != null and n != ancestor:
		if n is Node3D:
			t = (n as Node3D).transform * t
		n = n.get_parent()
	return t

static func _uniform(b: Basis) -> bool:
	var s := b.get_scale()
	return absf(s.x - s.y) < 1e-4 * absf(s.x) and absf(s.x - s.z) < 1e-4 * absf(s.x)

# Every visible MeshInstance3D under `root` with its rigid group: [mi, group node (null = keep as is), mi is the group
# node itself, inside a cockpit part]. Hidden subtrees are left out (a hidden option part stays as it is).
static func _walk(root: Node3D, moving: Dictionary, cockpit_nodes: Dictionary) -> Array:
	var out: Array = []
	var stack: Array = [[root, root, false]]
	while not stack.is_empty():
		var item: Array = stack.pop_back()
		var node: Node = item[0]
		var name := str(node.name)
		var group: Node3D = item[1]
		var own_group := false
		if KEEP_SUBTREES.has(name):
			group = null
		elif group != null and node != root and moving.has(name) and node is Node3D:
			group = node as Node3D
			own_group = true
		var cockpit_part: bool = item[2] or name.begins_with("cockpit_") or cockpit_nodes.has(name) or name == "CockpitControls"
		if node is Node3D and not (node as Node3D).visible:
			continue
		var children := node.get_children()
		for i in range(children.size() - 1, -1, -1):
			stack.append([children[i], group, cockpit_part])
		if node is MeshInstance3D and (node as MeshInstance3D).mesh != null:
			out.append([node, group, own_group, cockpit_part])
	return out

# Shadow proxies: in the shadow pass a surface's material only matters through its cull mode (opaque and depth
# pre-pass surfaces cast, alpha-blended ones do not), so per rigid group the opaque surfaces of every shadow-casting instance go into one
# SHADOWS_ONLY instance per cull mode and those instances stop casting themselves. Same triangles, same normals (the
# normal bias), same cull mode: the same shadow, from 1-2 draws per group and cascade instead of one per surface.
# Instances with anything the shadow pass would read beyond that (alpha scissor / hash, grow, billboard, depth draw
# always, vertex colours as alpha, non-standard materials) keep casting their own shadow.
static func _shadow_proxies(root: Node3D, moving: Dictionary, cockpit_nodes: Dictionary, report: Dictionary) -> void:
	var proxies := {} # "group id|cull|layers" -> {group, cull, layers, tool, surfaces}
	var proxy_order: Array = []
	var casters: Array = []
	for entry in _walk(root, moving, cockpit_nodes):
		var mi: MeshInstance3D = entry[0]
		var group: Node3D = entry[1]
		if group == null or mi.cast_shadow != GeometryInstance3D.SHADOW_CASTING_SETTING_ON or mi.skin != null \
				or mi.material_override != null or not mi.mesh is ArrayMesh or mi.mesh.get_blend_shape_count() > 0:
			continue
		var xform := _in(mi, group)
		if not _uniform(xform.basis):
			continue
		var parts: Array = []
		var ok := true
		for s in range(mi.mesh.get_surface_count()):
			var std := mi.get_active_material(s) as BaseMaterial3D
			if std == null or mi.mesh.surface_get_primitive_type(s) != Mesh.PRIMITIVE_TRIANGLES or std.grow \
					or std.billboard_mode != BaseMaterial3D.BILLBOARD_DISABLED or std.depth_draw_mode == BaseMaterial3D.DEPTH_DRAW_ALWAYS \
					or std.no_depth_test or std.use_point_size or std.shadow_to_opacity:
				ok = false
				break
			if std.transparency == BaseMaterial3D.TRANSPARENCY_ALPHA:
				continue # blended: never in the shadow pass
			# Depth pre-pass glass casts a full shadow in the Compatibility renderer (no alpha test in its shadow pass;
			# checked against screenshots: leaving it out lightened the dash under the windscreen), so it joins the proxy.
			# The other renderers were not checked: there such an instance keeps its own shadow.
			var prepass_ok := RenderingServer.get_current_rendering_method() == "gl_compatibility"
			if std.transparency != BaseMaterial3D.TRANSPARENCY_DISABLED and not (prepass_ok and std.transparency == BaseMaterial3D.TRANSPARENCY_ALPHA_DEPTH_PRE_PASS):
				ok = false # alpha scissor / hash / depth pre-pass read the texture alpha in the shadow pass
				break
			parts.append([s, std.cull_mode])
		if not ok:
			continue
		casters.append(mi)
		for p in parts:
			var key := "%d|%d|%d" % [group.get_instance_id(), p[1], mi.layers]
			if not proxies.has(key):
				proxies[key] = {"group": group, "cull": p[1], "layers": mi.layers, "tool": SurfaceTool.new(), "surfaces": 0}
				proxy_order.append(key)
			proxies[key]["tool"].append_from(mi.mesh, p[0], xform)
			proxies[key]["surfaces"] += 1
	for mi in casters:
		mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	for key in proxy_order:
		var p: Dictionary = proxies[key]
		var mesh := ArrayMesh.new()
		mesh.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, p["tool"].commit_to_arrays())
		var mat := StandardMaterial3D.new()
		mat.cull_mode = p["cull"]
		mesh.surface_set_material(0, mat)
		var proxy := MeshInstance3D.new()
		proxy.name = "ShadowProxy"
		proxy.mesh = mesh
		proxy.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_SHADOWS_ONLY
		proxy.layers = p["layers"]
		(p["group"] as Node3D).add_child(proxy)
		report["shadow_surfaces_proxied"] += p["surfaces"]
		report["shadow_proxies"] += 1

# Returns {merged_surfaces, merged_into, kept_surfaces, small_parts_shadow_off, shadow_proxies, shadow_surfaces_proxied}.
# shadow_proxy: step 3 (off in the garage, which sets every instance's shadow itself).
# all_joints: keep every jointed node separate, doors and lids too (the garage opens the hood in the engine bay view).
static func merge(root: Node3D, rig_path: String, cockpit_path: String = "", all_joints: bool = false, shadow_proxy: bool = true) -> Dictionary:
	var report := {"merged_surfaces": 0, "merged_into": 0, "kept_surfaces": 0, "small_parts_shadow_off": 0,
		"shadow_proxies": 0, "shadow_surfaces_proxied": 0}
	if root == null or root.has_meta("static_merged"):
		return report
	root.set_meta("static_merged", true)
	var moving := {}
	var lamp_materials := {}
	for rig in [_json(rig_path), _json(ProjectSettings.globalize_path(PARTS_RIG))]:
		for n in rig.get("nodes", []):
			var name := str(n.get("name", ""))
			if n.has("option"):
				moving[name] = true
			elif n.has("joint") or n.has("wheel"):
				var lid := false
				for p in ([] if all_joints else STATIC_JOINT_PREFIXES):
					lid = lid or name.begins_with(p)
				if not lid:
					moving[name] = true
		for spec in rig.get("lights", {}).values():
			for m in spec.get("materials", []):
				lamp_materials[str(m)] = true
	var cockpit_nodes := {}
	_cockpit_nodes(_json(cockpit_path), cockpit_nodes)
	for k in cockpit_nodes:
		moving[k] = true

	var groups := {} # key -> {material, group (Node3D), cast_shadow, layers, items: [[mesh, surface, transform, mi]]}
	var order: Array = [] # group keys in first-seen order (deterministic surface order)
	for entry in _walk(root, moving, cockpit_nodes):
		var mi: MeshInstance3D = entry[0]
		var group: Node3D = entry[1]
		var own_group: bool = entry[2]
		if entry[3] and mi.cast_shadow != GeometryInstance3D.SHADOW_CASTING_SETTING_OFF:
			var size := mi.get_aabb().size * _in(mi, root).basis.get_scale()
			if maxf(size.x, maxf(size.y, size.z)) < SMALL_PART_M:
				mi.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
				report["small_parts_shadow_off"] += 1
		var mesh := mi.mesh as ArrayMesh
		if group == null or own_group or mesh == null or mi.skin != null or mesh.get_blend_shape_count() > 0 \
				or mi.material_override != null:
			report["kept_surfaces"] += mi.mesh.get_surface_count()
			continue
		var xform := _in(mi, group)
		if not _uniform(xform.basis):
			report["kept_surfaces"] += mesh.get_surface_count()
			continue
		for s in range(mesh.get_surface_count()):
			var mat := mi.get_active_material(s)
			var std := mat as BaseMaterial3D
			if std == null or mesh.surface_get_primitive_type(s) != Mesh.PRIMITIVE_TRIANGLES \
					or lamp_materials.has(std.resource_name) or std.transparency != BaseMaterial3D.TRANSPARENCY_DISABLED:
				report["kept_surfaces"] += 1
				continue
			var source := mesh.surface_get_material(s)
			var key := "%d|%d|%s|%d|%d|%d" % [group.get_instance_id(), source.get_instance_id() if source != null else 0,
				std.resource_name, mesh.surface_get_format(s), mi.cast_shadow, mi.layers]
			if not groups.has(key):
				groups[key] = {"material": mat, "group": group, "cast_shadow": mi.cast_shadow, "layers": mi.layers, "items": []}
				order.append(key)
			groups[key]["items"].append([mesh, s, xform, mi])

	# A material used once in a group gains nothing from a merge: that surface stays where it is.
	var touched := {} # MeshInstance3D -> {surface index: true} merged away
	var outputs := {} # "group id|cast_shadow|layers" -> {group, cast_shadow, layers, mesh}
	var output_order: Array = []
	for key in order:
		var g: Dictionary = groups[key]
		var items: Array = g["items"]
		if items.size() < 2:
			report["kept_surfaces"] += 1
			continue
		var out_key := "%d|%d|%d" % [g["group"].get_instance_id(), g["cast_shadow"], g["layers"]]
		if not outputs.has(out_key):
			outputs[out_key] = {"group": g["group"], "cast_shadow": g["cast_shadow"], "layers": g["layers"], "mesh": ArrayMesh.new()}
			output_order.append(out_key)
		var st := SurfaceTool.new()
		for it in items:
			st.append_from(it[0], it[1], it[2])
			if not touched.has(it[3]):
				touched[it[3]] = {}
			touched[it[3]][it[1]] = true
		var m: ArrayMesh = outputs[out_key]["mesh"]
		m.add_surface_from_arrays(Mesh.PRIMITIVE_TRIANGLES, st.commit_to_arrays())
		m.surface_set_material(m.get_surface_count() - 1, g["material"])
		report["merged_surfaces"] += items.size()
		report["merged_into"] += 1
	for out_key in output_order:
		var o: Dictionary = outputs[out_key]
		var out := MeshInstance3D.new()
		out.name = "MergedStatic"
		out.mesh = o["mesh"]
		# The material also as the surface override: the colour code (vehicle_visual.gd / garage_scene.gd apply_colours)
		# styles the override copies.
		for i in range(out.mesh.get_surface_count()):
			out.set_surface_override_material(i, out.mesh.surface_get_material(i))
		out.cast_shadow = o["cast_shadow"]
		out.layers = o["layers"]
		(o["group"] as Node3D).add_child(out)

	# What is left of each touched mesh: its unmerged surfaces, with the materials they showed (lamp copies included,
	# so the lamp scripts keep driving the same material objects).
	for mi in touched:
		var old := (mi as MeshInstance3D).mesh as ArrayMesh
		var rest := ArrayMesh.new()
		for s in range(old.get_surface_count()):
			if touched[mi].has(s):
				continue
			var mat: Material = mi.get_active_material(s)
			rest.add_surface_from_arrays(old.surface_get_primitive_type(s), old.surface_get_arrays(s))
			rest.surface_set_material(rest.get_surface_count() - 1, mat)
		mi.mesh = rest if rest.get_surface_count() > 0 else null
		for s in range(rest.get_surface_count()):
			mi.set_surface_override_material(s, rest.surface_get_material(s))
	if shadow_proxy:
		_shadow_proxies(root, moving, cockpit_nodes, report)
	return report
