extends RefCounted
# One powertrain read for every dashboard (HUD gauge, digital cluster, cockpit instruments): RgSimulation.get_vehicle_powertrain
# answers an EMPTY dictionary when the car cannot be found (no session yet, a name that does not match the loaded car, a
# session being rebuilt). A reader that took that at face value drew "N, 0 km/h, 0 rpm" (owner 2026-10-18). The feed holds the
# last valid reading while the car is running and logs the first such miss once per drive; a drive (re)start calls reset().
# Adapter-side presentation only: it never changes what the simulation does.

var last: Dictionary = {}
var misses := 0          # frames that came back empty since the last reset
var logged := false      # the one log line per drive was written
var _label := ""

func _init(label := "dashboard") -> void:
	_label = label

func reset() -> void:
	last = {}
	misses = 0
	logged = false

static func is_valid(pt: Dictionary) -> bool:
	return not pt.is_empty()

# fresh: what get_vehicle_powertrain returned this frame. Returns the reading to draw.
func filter(fresh: Dictionary, vehicle_name := "") -> Dictionary:
	if is_valid(fresh):
		last = fresh
		return fresh
	misses += 1
	if not logged:
		logged = true
		print("RG_DASHBOARD %s: empty powertrain reading for '%s' (holding the last valid value: %s)" % [_label, vehicle_name, "yes" if not last.is_empty() else "none yet"])
	return last

func read(simulation: Node, vehicle_name: String) -> Dictionary:
	return filter(simulation.get_vehicle_powertrain(vehicle_name), vehicle_name)
