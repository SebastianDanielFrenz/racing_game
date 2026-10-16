extends SceneTree
# Pure-GDScript check of the renderer setting's Godot side (renderer_choice.gd, project.godot):
#   godot --headless --path game --script res://scripts/renderer_choice_test.gd
# Writes only into a temporary user:// file, never the player's renderer_override.cfg.
const RendererChoice := preload("res://scripts/renderer_choice.gd")
var _failures := 0

func _check(ok: bool, what: String) -> void:
	if not ok:
		_failures += 1
		push_error("renderer_choice_test: " + what)

func _init() -> void:
	# project.godot: Mobile is the default and the override file is the one renderer_choice.gd writes.
	_check(RendererChoice.started_method() == "mobile" or FileAccess.file_exists(RendererChoice.OVERRIDE_PATH),
		"project default renderer is mobile")
	var project := ConfigFile.new()
	_check(project.load("res://project.godot") == OK, "project.godot parses")
	_check(str(project.get_value("rendering", "renderer/rendering_method", "")) == "mobile", "project.godot names mobile")
	_check(str(project.get_value("application", "config/project_settings_override", "")) == RendererChoice.OVERRIDE_PATH,
		"project.godot names the override file")
	_check(not project.has_section_key("rendering", "renderer/rendering_method.mobile"), "no stale .mobile override")
	var path := "user://renderer_choice_test.cfg"
	_check(RendererChoice.store("compatibility", path) == "", "store compatibility")
	var cfg := ConfigFile.new()
	_check(cfg.load(path) == OK, "override file parses")
	_check(str(cfg.get_value("rendering", "renderer/rendering_method", "")) == "gl_compatibility", "override names gl_compatibility")
	_check(RendererChoice.store("forward_plus", path) == "", "store forward_plus")
	cfg = ConfigFile.new()
	_check(cfg.load(path) == OK, "forward_plus override file parses")
	_check(str(cfg.get_value("rendering", "renderer/rendering_method", "")) == "forward_plus", "override names forward_plus")
	_check(RendererChoice.store("mobile", path) == "", "store mobile")
	_check(not FileAccess.file_exists(path), "mobile removes the override file")
	_check(RendererChoice.store("mobile", path) == "", "mobile without a file is fine")
	_check(RendererChoice.store("vulkan", path) != "", "unknown choice refused")
	_check(not FileAccess.file_exists(path), "refused choice writes nothing")
	_check(RendererChoice.METHODS.keys() == ["mobile", "compatibility", "forward_plus"], "choices match rg::Settings")
	_check(RendererChoice.report("mobile").begins_with("RG_RENDERER setting=mobile running="), "report line")
	print("renderer_choice_test failures=%d" % _failures)
	quit(1 if _failures > 0 else 0)
