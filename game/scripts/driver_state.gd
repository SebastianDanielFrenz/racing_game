extends RefCounted
# Adapter-only conversion: every display consumes rg::DriverActions::state.
const CHANNELS := ["assist.auto_shift","assist.traction_control","assist.aero_control","assist.launch_control","assist.drive_mode","nitrous_arm","park_lock","gear_request","transmission.selector","epb","assist.abs","assist.esc","ignition","starter","brake","cabin.blink_lit","cabin.hazard","cabin.indicator","cabin.indicator_left","cabin.indicator_right","cabin.brake_lamp","cabin.horn","cabin.lights","cabin.light_mode"]
const ACTIONS := {"tc":"traction_control","lc":"toggle_launch_control","dm":"drive_mode","aero":"toggle_aero","auto_shift":"auto_shift","nitrous":"toggle_nitrous","park":"park","epb":"epb","abs":"abs","esc":"esc","hazard":"hazard","left":"indicator_left","right":"indicator_right","horn":"horn","start":"start_stop","lights":"lights","light_mode":"light_mode"}

static func channels(simulation: Node) -> Dictionary:
	var values: Dictionary = {}
	for channel in CHANNELS: values[channel] = simulation.get_control(channel)
	return values

static func from_snapshot(snapshot: Dictionary, values: Dictionary) -> Dictionary:
	var states: Dictionary = {}
	for alias in ACTIONS: states[alias] = RgDriverActions.state(ACTIONS[alias],snapshot,values)
	return states

static func read(simulation: Node, snapshot: Dictionary) -> Dictionary:
	return from_snapshot(snapshot,channels(simulation))
