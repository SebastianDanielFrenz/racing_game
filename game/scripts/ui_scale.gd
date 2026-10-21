extends RefCounted
# game/scripts/ui_scale.gd — scales every 2D/Control element (menus, garage,
# HUD, loading overlay) by the OS display scaling factor (owner 2026-10-06:
# "increase UI size. Grab the windows scaling factor").
#
# Windows reports its scaling (100/125/150/... %) only through the monitor DPI
# (DisplayServer.screen_get_scale() is 1.0 there; it is real on macOS and
# Wayland), so the factor is max(screen_get_scale(), dpi / 96). The stretch
# mode stays "disabled", so the root's content_scale_factor applies as-is to
# the canvas only - 3D renders at full window resolution.
#
# Override: user arg --ui-scale=F (after `--`) or env RG_UI_SCALE=F.

const MIN_SCALE := 0.5
const MAX_SCALE := 4.0

static func detect() -> float:
	for a in OS.get_cmdline_user_args():
		if a.begins_with("--ui-scale="):
			return clampf(a.get_slice("=", 1).to_float(), MIN_SCALE, MAX_SCALE)
	var env := OS.get_environment("RG_UI_SCALE")
	if env != "":
		return clampf(env.to_float(), MIN_SCALE, MAX_SCALE)
	var screen := DisplayServer.window_get_current_screen()
	var s := DisplayServer.screen_get_scale(screen)
	var dpi := DisplayServer.screen_get_dpi(screen)
	if dpi > 0:
		s = maxf(s, float(dpi) / 96.0)
	return clampf(s, 1.0, MAX_SCALE)

# A mouse position (InputEvent.position, Viewport.get_mouse_position) is in canvas units, i.e. divided by the root's
# content_scale_factor, while Camera3D.project_ray_* / unproject_position work in the viewport's physical pixels (3D
# renders at full window resolution, see above). Picking a 3D object with the mouse must convert first, or every ray
# misses by the UI scale (the hyper cockpit buttons ignored every click at 150 % Windows scaling, 2026-10-21).
static func canvas_to_viewport_pixels(viewport: Viewport, at: Vector2) -> Vector2:
	var window := viewport as Window
	if window != null and window.content_scale_mode == Window.CONTENT_SCALE_MODE_DISABLED:
		return at * window.content_scale_factor
	return at

static func apply(tree: SceneTree) -> void:
	var f := detect()
	tree.root.content_scale_factor = f
	print("RG_UI_SCALE factor=%.3f dpi=%d" % [f, DisplayServer.screen_get_dpi(DisplayServer.window_get_current_screen())])
