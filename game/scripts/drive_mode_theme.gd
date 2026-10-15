extends RefCounted
# game/scripts/drive_mode_theme.gd - the one place that knows the drive-mode theme colours (owner 2026-10-13):
# ECO = blue, NORMAL = gray, SPORT = orange, RACING = red, following the mode that RUNS (RgSimulation.get_vehicle_powertrain's
# drive_mode_effective, physics_sim's PowertrainSnapshot::drive_mode_effective), not the one requested. Drawing only:
# which modes a car offers, the names and the cycling live in rg_core (rg/assist_info.h, RgAssists).
# A car without a drive_modes part (drive_mode_fitted false) keeps its own look: every caller passes its own colour as
# the fallback and gets it back unchanged.

const ECO := Color(0.25, 0.55, 1.0)
const NORMAL := Color(0.62, 0.64, 0.68)
const SPORT := Color(1.0, 0.55, 0.12)
const RACING := Color(0.92, 0.16, 0.18)

const MODE_ECO := 0
const MODE_NORMAL := 1
const MODE_SPORT := 2
const MODE_RACING := 3

# The theme colour of a drive-mode code (0 eco, 1 normal, 2 sport, 3 racing).
static func color_for_mode(code: int) -> Color:
	if code == MODE_ECO:
		return ECO
	if code == MODE_SPORT:
		return SPORT
	if code == MODE_RACING:
		return RACING
	return NORMAL

# True when the car has a drive_modes part (RgSimulation.get_vehicle_powertrain's snapshot dictionary).
static func is_themed(pt: Dictionary) -> bool:
	return bool(pt.get("drive_mode_fitted", false))

# The accent colour for the running mode, or `fallback` for a car without drive modes.
static func accent(pt: Dictionary, fallback: Color) -> Color:
	if not is_themed(pt):
		return fallback
	return color_for_mode(int(pt.get("drive_mode_effective", MODE_NORMAL)))

# "ECO" / "NORMAL" / "SPORT" / "RACING" of the running mode, "" for a car without drive modes.
static func label(pt: Dictionary) -> String:
	if not is_themed(pt):
		return ""
	return str(pt.get("drive_mode_name", ""))

# Display palette only; logical fitting/mode comes from the shared ControlState.
static func accent_state(control: Dictionary, fallback: Color) -> Color:
	if not bool(control.get("fitted",false)): return fallback
	var palette := {"eco":ECO,"normal":NORMAL,"sport":SPORT,"racing":RACING}
	return palette.get(str(control.get("colour_role","normal")),NORMAL)
