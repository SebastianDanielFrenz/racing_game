extends RefCounted
# Input translation only: never generates steering without driver input.
var settings: Dictionary = {}
var wheelbase_m: float = 2.7
var max_angle_rad: float = deg_to_rad(32.0)
var _keyboard: float = 0.0
var _recovery: float = 0.0
var _relocations: int = -1

func _init() -> void:
	var parsed = JSON.parse_string(FileAccess.get_file_as_string(ProjectSettings.globalize_path("res://../data/controls/steering.json")))
	if parsed is Dictionary:
		settings = parsed

func reset() -> void:
	_keyboard = 0.0
	_recovery = 0.0
	_relocations = -1

func configure_vehicle(path: String) -> void:
	var vehicle = JSON.parse_string(FileAccess.get_file_as_string(path))
	if not vehicle is Dictionary:
		return
	max_angle_rad = maxf(0.01, float(vehicle.get("steering", {}).get("max_wheel_angle", max_angle_rad)))
	var front := -INF
	var rear := INF
	for wheel in vehicle.get("wheels", []):
		var x: float = float(wheel.get("attachment_local", [0,0,0])[0])
		front = maxf(front, x)
		rear = minf(rear, x)
	if is_finite(front) and is_finite(rear):
		wheelbase_m = maxf(0.1, front-rear)

func translate(raw: float, keyboard: bool, wheel: bool, motion: Dictionary, delta: float) -> float:
	raw = clampf(raw, -1.0, 1.0)
	var profile: String = str(settings.get("input_profile", "auto"))
	if not bool(settings.get("enabled", true)) or (not keyboard and (profile == "wheel" or (profile == "auto" and wheel))):
		reset()
		return raw
	var relocations: int = int(motion.get("relocations", 0))
	if _relocations != relocations:
		reset()
		_relocations = relocations
	var forward: float = float(motion.get("forward_mps", 0.0))
	var lateral: float = float(motion.get("left_mps", 0.0))
	var slip: float = atan2(lateral, maxf(absf(forward), 1.0))
	var yaw: float = float(motion.get("yaw_rad_s", 0.0))
	var correction: float = slip-yaw*clampf(float(settings.get("yaw_anticipation_s", 0.15)),0.0,0.5)
	var countersteering: bool = raw*correction > 0.0 and forward > 5.0 and float(motion.get("upright", 1.0)) > 0.5
	var start: float = maxf(0.1, float(settings.get("slip_start_deg", 3.0)))
	var full: float = maxf(start+0.1, float(settings.get("slip_full_deg", 20.0)))
	var demand: float = smoothstep(deg_to_rad(start), deg_to_rad(full), absf(slip)) if countersteering else 0.0
	var dt: float = clampf(delta,0.0,0.05)
	var recovery_rate: float = maxf(0.1,float(settings.get("recovery_rise_per_s",12.0) if demand > _recovery else settings.get("recovery_fall_per_s",4.0)))
	_recovery = move_toward(_recovery,demand,recovery_rate*dt)
	# Never retain increased authority for input that steers into the slide.
	var authority: float = _recovery if countersteering else 0.0
	var input_value: float = raw
	if keyboard:
		var rise: float = maxf(0.1,float(settings.get("keyboard_rise_per_s",4.0)))
		var fast: float = maxf(rise,float(settings.get("recovery_keyboard_per_s",18.0)))
		var rate: float = lerpf(rise,fast,authority)
		if is_zero_approx(raw):
			rate = maxf(0.1,float(settings.get("keyboard_return_per_s",8.0)))
		_keyboard = move_toward(_keyboard,raw,rate*dt)
		input_value = _keyboard
	else:
		_keyboard = 0.0
	if input_value*correction <= 0.0:
		authority = 0.0
	var accel: float = maxf(0.1,float(settings.get("normal_lateral_accel_mps2",18.0)))
	var floor_fraction: float = clampf(float(settings.get("normal_min_fraction",0.14)),0.01,0.95)
	var normal_limit: float = clampf(atan(wheelbase_m*accel/maxf(forward*forward,1.0))/max_angle_rad,floor_fraction,1.0)
	var exponent: float = lerpf(clampf(float(settings.get("input_exponent",1.8)),1.0,4.0),1.0,authority)
	var shaped: float = signf(input_value)*pow(absf(input_value),exponent)
	return clampf(shaped*lerpf(normal_limit,1.0,authority),-1.0,1.0)
