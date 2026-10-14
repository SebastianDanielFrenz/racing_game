extends SceneTree
# Rendered regression: run without --headless (Compatibility or Mobile).
class Fixture extends Node:
	func get_set() -> Dictionary:
		var path := ProjectSettings.globalize_path("res://../data/garage/garage_set.json")
		var data: Dictionary = JSON.parse_string(FileAccess.get_file_as_string(path))
		for list in [data.lights,data.softboxes]:
			for item in list:
				for key in ["position","target"]:
					var p: Array = item[key]
					item[key] = Vector3(-p[1],p[2],-p[0])
		return data
	func setup_camera(_id,_lo,_hi,_on): pass
var failures := 0
func _initialize(): run.call_deferred()
func check(ok: bool, message: String):
	if not ok:
		failures += 1
		push_error(message)
func capture(scene) -> Image:
	for i in range(5): await process_frame
	await RenderingServer.frame_post_draw
	return scene.capture_image()
func dark_pixels(reference: Image, actual: Image) -> int:
	var count := 0
	# Opaque red bodywork above the floor; glass/interior and reflections excluded.
	for y in range(200,600):
		for x in range(200,1050):
			var r := reference.get_pixel(x,y)
			if r.r>0.5 and r.g<r.r*0.7 and r.b<r.r*0.7:
				if r.r-actual.get_pixel(x,y).r>0.25: count += 1
	return count
func run():
	root.size = Vector2i(1200,800)
	var fixture := Fixture.new()
	root.add_child(fixture)
	var scene = load("res://scripts/garage_scene.gd").new()
	scene.garage = fixture
	root.add_child(scene)
	scene.set_process(false)
	check(scene.show_vehicle({"id":"shadow_fixture","model_path":ProjectSettings.globalize_path("res://../data/models/pico/pico.glb"),"paint":"#ef412a"}),"Pico loads")
	scene._camera.position = Vector3(4,2.4,-4)
	scene._camera.fov = 38
	scene._camera.look_at(Vector3(0,0.8,0))
	var light = scene._world_root.get_node("Light_key")
	check(light.shadow_enabled and light.shadow_reverse_cull_face,"production spotlight retains shadows and back-face casting")
	var renderer := RenderingServer.get_current_rendering_method()
	var old_total := 0
	var fixed_total := 0
	for degrees in range(0,360,30):
		scene._car_pivot.rotation.y = deg_to_rad(degrees)
		scene._mirror_pivot.rotation.y = deg_to_rad(degrees)
		light.shadow_enabled = false
		var reference: Image = await capture(scene)
		light.shadow_enabled = true
		light.shadow_reverse_cull_face = false
		var old: Image = await capture(scene)
		light.shadow_reverse_cull_face = true
		var fixed: Image = await capture(scene)
		var a := dark_pixels(reference,old)
		var b := dark_pixels(reference,fixed)
		old_total += a
		fixed_total += b
		print("RG_GARAGE_SHADOW yaw=%d old_dark=%d fixed_dark=%d" % [degrees,a,b])
		if degrees in [120,300]:
			old.save_png(ProjectSettings.globalize_path("res://../out/garage_shadow_%s_old_%d.png" % [renderer,degrees]))
			fixed.save_png(ProjectSettings.globalize_path("res://../out/garage_shadow_%s_fixed_%d.png" % [renderer,degrees]))
	if renderer == "gl_compatibility":
		check(old_total>1000,"fixture reproduces old dark bands")
		check(fixed_total<old_total*0.35,"shadow bands reduced across a full rotation: %d -> %d" % [old_total,fixed_total])
	else:
		# Mobile does not reproduce this Compatibility artifact.
		check(fixed_total<=old_total+100,"other renderer does not gain dark bands")
	print("RG_GARAGE_SHADOW failures=%d old=%d fixed=%d" % [failures,old_total,fixed_total])
	fixture.queue_free()
	scene.queue_free()
	await process_frame
	quit(1 if failures else 0)

