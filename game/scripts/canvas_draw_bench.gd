extends SceneTree
# Canvas draw-call bench (perf, 2026-10-15): builds one 2D overlay widget in an empty window and prints the root
# viewport's canvas draw calls, so a widget's per-frame draw-call cost can be measured without a world or a map store.
#   godot --path game --script res://scripts/canvas_draw_bench.gd -- --widget=minimap [--lines=N]
# Widgets: minimap (N synthetic map polylines, 10 points each, ranks 0..9, plus a route).
# Prints: RG_CANVAS_BENCH widget=<name> canvas_dc=<n> canvas_prims=<n> total_dc=<n>

func _arg(name: String, fallback: String) -> String:
	for a in OS.get_cmdline_user_args():
		if a.begins_with("--%s=" % name):
			return a.substr(name.length() + 3)
	return fallback

func _initialize() -> void:
	var widget := _arg("widget", "minimap")
	var node: Control
	if widget == "minimap":
		node = Control.new()
		node.set_script(load("res://scripts/minimap.gd"))
		node.position = Vector2(16, 16)
		node.size = Vector2(220, 220)
		root.add_child(node)
		var rng := RandomNumberGenerator.new()
		rng.seed = 1
		var lines: Array = []
		for i in int(_arg("lines", "1500")):
			var p := Vector2(rng.randf_range(-900, 900), rng.randf_range(-900, 900))
			var pts := PackedVector2Array([p])
			for k in 9:
				p += Vector2(rng.randf_range(-60, 60), rng.randf_range(-60, 60))
				pts.append(p)
			lines.append({"points": pts, "rank": i % 10})
		node.set_map({"lines": lines})
		node.set_route(PackedVector2Array([Vector2(-500, -500), Vector2(0, 0), Vector2(400, 200)]))
		node.set_car(Vector2.ZERO, 30.0)
	for i in 5:
		await process_frame
	var vp := root
	print("RG_CANVAS_BENCH widget=%s canvas_dc=%d canvas_prims=%d total_dc=%d" % [widget,
		vp.get_render_info(Viewport.RENDER_INFO_TYPE_CANVAS, Viewport.RENDER_INFO_DRAW_CALLS_IN_FRAME),
		vp.get_render_info(Viewport.RENDER_INFO_TYPE_CANVAS, Viewport.RENDER_INFO_PRIMITIVES_IN_FRAME),
		int(Performance.get_monitor(Performance.RENDER_TOTAL_DRAW_CALLS_IN_FRAME))])
	var img := root.get_texture().get_image()
	var out := _arg("png", "")
	if out != "":
		img.save_png(out)
	quit()
