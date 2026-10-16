extends RefCounted
# Scene light level shared by the lamp scripts (exterior_lamps.gd, headlights.gd) and the
# automatic headlamps (input_map.gd -> RgDriverActions). Written by sky_controller.gd from
# RgSky.get_state() (rg::TimeWeather::sample(), rg/exposure.h) at its update rate.
# Static vars: preload this script and read/write through the constant, no instance.
#
# exposure: display value per cd/m2 (linear). < 0 = no sky drives it (garage, pure-GDScript
#   runs): the lamps use headlights.json "fallback_exposure" (= the night exposure).
# ambient_lux: horizontal illuminance (light sensor); NAN = no reading (auto lamps hold).
static var exposure := -1.0
static var ambient_lux := NAN

# Godot unit conversion of the lamp photometry (Compatibility renderer: the sky stays at its
# tuned exposure-1 look, so lamps are multiplied by the exposure instead; exact for the
# linear tonemap). Measured (Compatibility, 2026-10-15): a SpotLight3D lights an albedo-a
# surface by a * energy / d^attenuation (spot_attenuation; 2.0 = inverse square), on the same
# scale as a DirectionalLight3D's energy (a * energy), whose energy is therefore lux / pi.
# A beam with a flatter falloff (attenuation < 2: Godot spots have no beam pattern, a real
# headlamp aims more light far than near) is anchored at reference_m: there it gives the
# illuminance intensity_cd / reference_m^2 exactly.
static func spot_energy(intensity_cd: float, exp_value: float, attenuation := 2.0, reference_m := 1.0) -> float:
	return maxf(intensity_cd, 0.0) / PI * exp_value * pow(maxf(reference_m, 0.01), attenuation - 2.0)

# Emission energy of a lamp lens of `nits` cd/m2 with colour `rgb`. Clipped so no channel
# exceeds 1 (a brighter red lens would otherwise saturate green/blue and turn white, as the
# Compatibility renderer clips each channel; the hue is kept).
static func lens_energy(nits: float, exp_value: float, rgb: Color) -> float:
	var peak := maxf(maxf(rgb.r, rgb.g), maxf(rgb.b, 1e-6))
	return clampf(nits * exp_value, 0.0, 1.0 / peak)

static func current(config: Dictionary) -> float:
	return exposure if exposure > 0.0 else float(config.get("fallback_exposure", 0.0942))
