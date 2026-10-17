extends RefCounted
# Thin caller of rg::SteeringAssist (core/include/rg/steering_assist.h, through RgSteeringAssist): the adaptive
# steering input translation lives in rg_core so every engine front end shapes the steer input the same way. Input
# translation only: it never generates steering without driver input. Tuning: data/controls/steering.json.
var _native := RgSteeringAssist.new()

func _init() -> void:
	var message: String = _native.load_config(ProjectSettings.globalize_path("res://../data/controls/steering.json"))
	if message != "":
		push_warning("Adaptive steering disabled: " + message)

func reset() -> void:
	_native.reset()

func configure_vehicle(path: String) -> void:
	_native.configure_vehicle(path)

func translate(raw: float, keyboard: bool, wheel: bool, motion: Dictionary, delta: float) -> float:
	return _native.translate(raw, keyboard, wheel, motion, delta)
