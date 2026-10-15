extends RefCounted
# game/scripts/renderer_choice.gd - the graphics.renderer setting (rg::Settings) on the Godot side.
#
# Godot picks its renderer once, before any script runs, from project.godot (rendering_method = "mobile") and the
# project-settings override file project.godot names (application/config/project_settings_override =
# user://renderer_override.cfg). This file only translates the setting's engine-neutral choice into that override
# file: "compatibility" / "forward_plus" write rendering_method = gl_compatibility / forward_plus there, "mobile" (the default) removes the file. A
# --rendering-method on the command line (tools/run.ps1 -Renderer) wins over both. No decision is made here.

const OVERRIDE_PATH := "user://renderer_override.cfg"
const METHODS := {"mobile": "mobile", "compatibility": "gl_compatibility", "forward_plus": "forward_plus"}

# Writes or removes the override for `choice`. Returns "" or an error text.
# `path` is for the test (renderer_choice_test.gd) only.
static func store(choice: String, path: String = OVERRIDE_PATH) -> String:
	if not METHODS.has(choice):
		return "unknown renderer choice '%s'" % choice
	var method: String = METHODS[choice]
	if method == "mobile":
		if FileAccess.file_exists(path):
			var err := DirAccess.remove_absolute(ProjectSettings.globalize_path(path))
			if err != OK:
				return "cannot remove %s (error %d)" % [path, err]
		return ""
	var cfg := ConfigFile.new()
	cfg.set_value("rendering", "renderer/rendering_method", method)
	var err := cfg.save(path)
	return "" if err == OK else "cannot write %s (error %d)" % [path, err]

# The renderer that actually runs, as a choice id ("mobile", "compatibility", or the engine's own name).
static func running_choice() -> String:
	var method := RenderingServer.get_current_rendering_method()
	for choice in METHODS:
		if METHODS[choice] == method:
			return choice
	return method

# The rendering method the project settings asked for at start-up (project.godot plus the override file, as Godot read
# them). A --rendering-method on the command line does not change this value, so it tells the two apart.
static func started_method() -> String:
	return str(ProjectSettings.get_setting("rendering/renderer/rendering_method", "mobile"))

# One console line: what was chosen, what runs, and why they differ. Godot falls back to Compatibility on its own
# when Vulkan cannot start (rendering_device/fallback_to_opengl3).
static func report(choice: String) -> String:
	var running := running_choice()
	var method := RenderingServer.get_current_rendering_method()
	var why := ""
	if running != choice:
		if method != started_method():
			if method == "gl_compatibility":
				why = " (Vulkan did not start: engine fallback, or --rendering-method on the command line)"
			else:
				why = " (--rendering-method on the command line)"
		else:
			why = " (applies at the next start)"
	return "RG_RENDERER setting=%s running=%s driver=%s adapter=%s%s" % [choice, running,
		RenderingServer.get_current_rendering_driver_name(), RenderingServer.get_video_adapter_name(), why]
