extends RefCounted
# Adapter-only conversion: every display consumes rg::DriverActions::state.
const CHANNELS := ["assist.auto_shift","assist.traction_control","assist.aero_control","assist.launch_control","assist.drive_mode","nitrous_arm"]
const ACTIONS := {"tc":"traction_control","lc":"toggle_launch_control","dm":"drive_mode","aero":"toggle_aero","auto_shift":"auto_shift","nitrous":"toggle_nitrous"}

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
