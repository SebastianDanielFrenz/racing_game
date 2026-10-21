extends SceneTree
# Pure-GDScript checks of the visual-world adapters (no GDExtension needed):
#   godot --headless --path game --script res://scripts/visual_world_test.gd
# weather_icon.gd draws every icon name, weather_widget.gd builds its menu from
# a fake RgSky (disabled items stay disabled, a choice is forwarded),
# headlights.gd finds the lamp clusters on a synthetic model and aims the spots
# forward/down, sky_controller.gd keeps the fixed sun without RgSky.

var failures := 0

func check(ok: bool, what: String) -> void:
	if ok:
		print("ok   ", what)
	else:
		failures += 1
		print("FAIL ", what)

class FakeSky:
	extends RefCounted
	var chosen: Array = []
	var time_h := 13.0
	func get_state() -> Dictionary:
		return {"icon": "moon_cloud", "clock_text": "13:00", "date_text": "2026-06-21", "local_time_h": time_h, "weather_label": "Partly cloudy"}
	func get_menu() -> Array:
		return [
			{"id": "time:noon", "group": "time", "label": "Noon", "enabled": true, "selected": false, "reason": ""},
			{"id": "rate:1", "group": "rate", "label": "x1", "enabled": true, "selected": true, "reason": ""},
			{"id": "weather:clear", "group": "weather", "label": "Clear", "enabled": true, "selected": true, "reason": ""},
			{"id": "weather:rain", "group": "weather", "label": "Rain", "enabled": false, "selected": false, "reason": "not wired yet"},
		]
	func choose(id: String) -> bool:
		chosen.append(id)
		return true
	func set_local_time_h(h: float) -> void:
		time_h = h
	func format_temperature_c(k: float) -> String:
		return "%d °C" % roundi(k - 273.15)

class FakeSkyController:
	extends Node
	var sky = null
	var applied := 0
	func apply_now() -> void:
		applied += 1

class FakeSim:
	extends Node
	var lights := 0.0
	func get_control(channel: String) -> float:
		return lights if channel == "cabin.lights" else 0.0
	func get_aero_state() -> Dictionary:
		return {"temperature_k": 288.15}

func _initialize() -> void:
	_run.call_deferred()

func _run() -> void:
	# --- icons ---
	var icon_script = load("res://scripts/weather_icon.gd")
	for name in icon_script.ICONS:
		var icon: Control = icon_script.new()
		icon.size = Vector2(44, 44)
		icon.icon = name
		root.add_child(icon)
	await process_frame
	check(icon_script.ICONS.size() == 9, "nine icon names (rg::weather_icon_name)")

	# --- widget ---
	var controller := FakeSkyController.new()
	controller.sky = FakeSky.new()
	root.add_child(controller)
	var widget = load("res://scripts/weather_widget.gd").new()
	widget.sky_controller = controller
	widget.simulation = FakeSim.new()
	root.add_child(widget.simulation)
	root.add_child(widget)
	await process_frame
	await process_frame
	check(widget._time_label.text == "13:00", "widget shows the clock")
	check(widget._info_label.text.begins_with("15 °C"), "widget shows the temperature: " + widget._info_label.text)
	check(widget._icon.icon == "moon_cloud", "widget shows the icon from the view-model")
	widget.toggle_menu()
	await process_frame
	var buttons: Array = widget._menu_box.find_children("*", "Button", true, false)
	var rain: Button = null
	var noon: Button = null
	for b in buttons:
		if b.text == "Rain": rain = b
		if b.text == "Noon": noon = b
	check(rain != null and rain.disabled and rain.tooltip_text == "not wired yet", "unwired weather is greyed out with its reason")
	noon.pressed.emit()
	check(controller.sky.chosen == ["time:noon"] and controller.applied == 1, "a menu choice is forwarded and applied")
	widget._slider.value = 6.0
	check(is_equal_approx(controller.sky.time_h, 6.0), "the time slider sets the time of day")

	# --- headlights ---
	var model := Node3D.new()
	root.add_child(model)
	for x in [0.6, -0.6]:
		var mi := MeshInstance3D.new()
		var box := BoxMesh.new()
		box.size = Vector3(0.2, 0.1, 0.1)
		var mat := StandardMaterial3D.new()
		mat.resource_name = "light_head"
		box.material = mat
		mi.mesh = box
		mi.position = Vector3(x, 0.7, 2.1)
		model.add_child(mi)
	var clusters: Array = load("res://scripts/headlights.gd").lamp_clusters(model, ["light_head"])
	check(clusters.size() == 2, "two headlamp clusters")
	check(clusters.size() == 2 and is_equal_approx(clusters[0].x, 0.6) and is_equal_approx(clusters[0].z, 2.17), "left cluster at the lens front: %s" % [clusters])
	var sim := FakeSim.new()
	root.add_child(sim)
	var lights = load("res://scripts/headlights.gd").attach_to(model, sim, "test", {"lights": {"head": {"materials": ["light_head"]}}})
	await process_frame
	check(lights.spots.size() == 2 and not lights.spots[0].visible, "two spots, off while cabin.lights is 0")
	sim.lights = 1.0
	await process_frame
	var forward: Vector3 = -lights.spots[0].basis.z
	check(lights.spots[0].visible and forward.z > 0.99 and forward.y < 0.0, "dipped beam on, aimed forward and down: %s" % forward)
	sim.lights = 2.0
	await process_frame
	check(lights.spots[0].spot_range > 100.0, "main beam reaches further")

	# --- lamp photometry x scene exposure (light_exposure.gd; numbers from headlights.json) ---
	var LE = load("res://scripts/light_exposure.gd")
	var cfg: Dictionary = lights.config
	var night: float = float(cfg["fallback_exposure"])
	LE.exposure = -1.0
	check(is_equal_approx(LE.current(cfg), night), "no sky: the fallback (night) exposure")
	var main_night: float = lights.spots[0].light_energy
	var mb: Dictionary = cfg["main_beam"]
	var ref: float = float(mb["reference_m"])
	# At the reference distance the spot gives intensity / d^2 lux, i.e. energy / d^attenuation = cd / d^2 / pi x exposure.
	check(is_equal_approx(main_night / pow(ref, float(mb["attenuation"])), float(mb["intensity_cd"]) / (ref * ref) / PI * night), "main beam: cd / d^2 at reference_m, x exposure: %s" % main_night)
	var noon_e := 1.5 * PI / 100000.0 # sky.json key * pi / ~1e5 lux (high sun)
	LE.exposure = noon_e
	await process_frame
	check(is_equal_approx(lights.spots[0].light_energy, main_night * noon_e / night), "beam energy follows the exposure (noon: %s of night)" % str(noon_e / night))
	LE.exposure = -1.0
	await process_frame
	check(is_equal_approx(lights.spots[0].light_energy, main_night), "and back")
	var red := Color(1.0, 0.02, 0.01)
	check(is_equal_approx(LE.lens_energy(8000.0, night, red), 1.0), "brake lens at night: clipped to the colour's peak (stays red)")
	check(LE.lens_energy(8000.0, noon_e, red) < 0.5, "brake lens at noon: below a sunlit surface")
	check(is_equal_approx(LE.lens_energy(100.0, 0.001, Color(0.5, 0.25, 0.0)), 0.1), "lens energy = nits x exposure below the clip")
	check(is_equal_approx(LE.lens_energy(1e6, 1.0, Color(0.5, 0.25, 0.0)), 2.0), "clip = 1 / peak channel")

	# --- sky controller without RgSky ---
	var sky_node = load("res://scripts/sky_controller.gd").new()
	root.add_child(sky_node)
	sky_node.setup("", "")
	check(sky_node.sun != null and sky_node.sun.visible and (ClassDB.class_exists("RgSky") or sky_node.sky == null), "fixed sun without RgSky")

	print("visual world test: %d failure(s)" % failures)
	quit(1 if failures > 0 else 0)
