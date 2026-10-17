extends Node
# Sun, moon, sky and ambient light from rg::TimeWeather (RgSky, data/world/sky.json).
# Owns the scene's DirectionalLight3D "Sun", a shadowless "Moon" and the
# WorldEnvironment. All astronomy, light levels and colours come from rg_core;
# this node only applies them, at UPDATE_HZ and only when something changed
# (a sky colour change makes Godot re-render the sky radiance).
# Without the GDExtension class (pure-GDScript runs) it keeps the old fixed sun.

# Exposure (rg/exposure.h): RgSky reports the sky's illuminance and the time-adapted camera
# exposure; they go to light_exposure.gd for the lamps and the automatic headlamps. The
# Compatibility renderer clips scene values well below daylight luminances, so the sky keeps
# its tuned look at environment exposure 1 and only the lamps are multiplied by the exposure
# (identical on screen for the linear tonemap). See docs/visual_world_plan.md "Exposure".
const LightExposure := preload("res://scripts/light_exposure.gd")
const UPDATE_HZ := 10.0
var sky = null # RgSky
var simulation: Node = null # RgSimulation: fed the clock and weather (physics environment, V8) at UPDATE_HZ
var sun: DirectionalLight3D
var moon: DirectionalLight3D
var environment: Environment
var _sky_material: ProceduralSkyMaterial
var _accum := 1.0
var _last_colors := []

func setup(sky_json_path: String, world_config_path: String) -> void:
	sun = DirectionalLight3D.new()
	sun.name = "Sun"
	sun.rotation_degrees = Vector3(-55.0, -35.0, 0.0)
	sun.light_energy = 1.1
	sun.shadow_enabled = true
	sun.directional_shadow_max_distance = 300.0
	add_child(sun)
	moon = DirectionalLight3D.new()
	moon.name = "Moon"
	moon.shadow_enabled = false
	moon.visible = false
	add_child(moon)
	var env_node := WorldEnvironment.new()
	environment = Environment.new()
	environment.background_mode = Environment.BG_SKY
	environment.sky = Sky.new()
	_sky_material = ProceduralSkyMaterial.new()
	environment.sky.sky_material = _sky_material
	env_node.environment = environment
	add_child(env_node)
	if not ClassDB.class_exists("RgSky"):
		return
	sky = ClassDB.instantiate("RgSky")
	if not sky.initialize(sky_json_path, world_config_path):
		push_warning("sky: " + sky.get_error() + " (fixed sun kept)")
		sky = null
		return
	if sky.get_climate_error() != "":
		push_warning("sky: no physics weather: " + sky.get_climate_error())
	environment.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	apply_now()

# Called by the weather widget after a menu choice, so the change shows at once.
func apply_now() -> void:
	_accum = 1.0 / UPDATE_HZ
	_process(0.0)

func _process(delta: float) -> void:
	if sky == null:
		return
	sky.advance(delta)
	_accum += delta
	if _accum < 1.0 / UPDATE_HZ:
		return
	_accum = 0.0
	if simulation != null and simulation.has_method("sync_weather"):
		simulation.sync_weather() # clock jumps, day change, weather choice, temperature drift -> the Session
	var s: Dictionary = sky.get_state()
	LightExposure.exposure = float(s.get("exposure", -1.0))
	LightExposure.ambient_lux = float(s.get("illuminance_lux", NAN))
	_aim(sun, s["sun_direction"])
	sun.light_energy = s["sun_energy"]
	sun.light_color = s["sun_color"]
	sun.visible = s["sun_energy"] > 0.0 # below the horizon: no light pass, no shadow pass
	_aim(moon, s["moon_direction"])
	moon.light_energy = s["moon_energy"]
	moon.light_color = s["moon_color"]
	moon.visible = s["moon_energy"] > 0.0
	var top: Color = s["sky_top_color"]
	var horizon: Color = s["sky_horizon_color"]
	environment.ambient_light_color = top.lerp(horizon, 0.5)
	environment.ambient_light_energy = s["ambient_energy"]
	var colors := [top, horizon]
	if _changed(colors):
		_last_colors = colors
		_sky_material.sky_top_color = top
		_sky_material.sky_horizon_color = horizon
		_sky_material.ground_horizon_color = horizon.darkened(0.2)
		_sky_material.ground_bottom_color = top.darkened(0.6)

func _changed(colors: Array) -> bool:
	if _last_colors.size() != colors.size():
		return true
	for i in colors.size():
		var a: Color = colors[i]
		var b: Color = _last_colors[i]
		if absf(a.r - b.r) + absf(a.g - b.g) + absf(a.b - b.b) > 0.004:
			return true
	return false

# A DirectionalLight3D shines along its -Z: point -Z away from the body.
func _aim(light: DirectionalLight3D, towards_body: Vector3) -> void:
	if towards_body.length_squared() < 1e-6:
		return
	var forward := -towards_body.normalized()
	var up := Vector3.UP if absf(forward.y) < 0.99 else Vector3.FORWARD
	light.basis = Basis.looking_at(forward, up)
