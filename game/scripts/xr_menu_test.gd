extends SceneTree
var failures := 0
var clicks := 0
class FakeDirector:
	extends RefCounted
	func active_camera() -> Camera3D: return null
class FakeMain:
	extends Node
	var _director := FakeDirector.new()
	var backs := 0
	func _on_escape() -> void: backs += 1
func check(condition: bool, message: String) -> void:
	if not condition:
		failures += 1
		push_error(message)
func _initialize() -> void:
	call_deferred("run")
func run() -> void:
	# Explicitly compile integration scripts too; editor scan alone is insufficient.
	check(load("res://scripts/main.gd") != null,"main compiles")
	check(load("res://scripts/xr_rig.gd") != null,"XR rig compiles")
	var host := FakeMain.new()
	root.add_child(host)
	var menu = load("res://scripts/xr_menu.gd").new()
	menu.main = host
	host.add_child(menu)
	var layer := CanvasLayer.new()
	host.add_child(layer)
	var button := Button.new()
	button.position = Vector2(860,490)
	button.size = Vector2(200,100)
	button.text = "Choose spawn"
	button.pressed.connect(func(): clicks += 1)
	layer.add_child(button)
	menu.attach(layer)
	menu.show_screen("spawn_picker")
	await process_frame
	check(layer.get_viewport()==menu.viewport,"shell canvas uses VR viewport")
	check(menu.active,"spawn picker is visible")
	check(menu.origin.current,"menu owns the active XR origin as well as camera")
	var center: Vector2 = menu.ray_pixel(Vector3.ZERO,Vector3.FORWARD)
	check(center.distance_to(Vector2(960,540))<0.01,"center ray maps to center pixel")
	check(menu.ray_pixel(Vector3.ZERO,Vector3.BACK).x<0,"ray pointing away rejected")
	check(menu.ray_pixel(Vector3.ZERO,Vector3(10,0,-1).normalized()).x<0,"outside panel rejected")
	menu.pointer=center
	menu._click(true)
	menu._click(false)
	await process_frame
	check(clicks==1,"gaze/trigger mouse forwarding selects actual GUI button")
	menu._button("by_button",true,1)
	check(host.backs==1,"controller B goes back once")
	menu.show_screen("configurator")
	check(menu.active,"garage remains headset visible")
	menu.show_screen("loading")
	check(menu.active,"loading remains headset visible")
	menu.show_screen("drive")
	check(not menu.active,"menu hidden when driving")
	menu._button("menu_button",true,1)
	check(host.backs==2,"controller menu can request pause from driving")
	check(menu.viewport.render_target_update_mode==SubViewport.UPDATE_DISABLED,"driving disables menu rendering")
	menu.show_screen("spawn_picker")
	menu.camera.position=Vector3(1,1.7,2)
	menu.camera.rotation=Vector3(0.35,0.6,0.45)
	menu._place_panel()
	check(absf(menu.panel.global_position.y-menu.camera.global_position.y)<0.0001,"panel uses tracked eye height")
	check(menu.panel.global_basis.y.distance_to(Vector3.UP)<0.0001,"panel upright despite head pitch and roll")
	var toward: Vector3=(menu.panel.global_position-menu.camera.global_position).normalized()
	check(menu.ray_pixel(menu.camera.global_position,toward).distance_to(Vector2(960,540))<0.01,"tracked eye ray reaches panel center")
	host.queue_free()
	await process_frame
	print("RG_XR_MENU_TEST failures=%d" % failures)
	quit(1 if failures else 0)
