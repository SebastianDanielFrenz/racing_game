// game/shaders/terrain.gdshader — PLAN.md R2.1's terrain preview shader:
// per-vertex COLOR (rg::chunk_vertex_colors' hypsometric ramp, uploaded as
// the mesh's ARRAY_COLOR by RgTerrainView) times a simple Lambert diffuse
// term against the scene's own DirectionalLight - no LandClass texturing
// (G2.5), no procedural detail, nothing beyond what's needed to READ the
// terrain shape and rough ground type by eye while flying around.
//
// No world-space coordinates: every calculation here uses VERTEX/NORMAL
// (object space) or values Godot's built-ins already provide per-fragment
// (screen UV, view-space depth) - never a raw world-space position/normal
// varying, which breaks under the floating-origin rebase this project uses
// everywhere else (a shader that baked in world-space coordinates would
// visibly swim the instant RgTerrainView.set_render_origin moves a chunk's
// instance transform).
shader_type spatial;
render_mode cull_back, diffuse_lambert, specular_disabled;

// A small ambient floor so the far (unlit, night) side of a slope doesn't
// go fully black - terrain has no bounce-light/GI in this preview.
uniform float ambient_floor : hint_range(0.0, 1.0) = 0.12;

void fragment() {
	ALBEDO = COLOR.rgb;
	// COLOR.a is always 1.0 (rg::chunk_vertex_colors' pack_rgba call always
	// passes a=1.0) - not used as opacity here, left at the material's own
	// default (opaque).
	SPECULAR = 0.0;
	ROUGHNESS = 1.0;
}

void light() {
	float ndotl = max(dot(NORMAL, LIGHT), 0.0);
	DIFFUSE_LIGHT += ALBEDO * LIGHT_COLOR * (ambient_floor + (1.0 - ambient_floor) * ndotl) * ATTENUATION;
}
