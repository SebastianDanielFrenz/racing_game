// game/shaders/terrain.gdshader - terrain chunks (RgTerrainView) and the flat test ground.
//
// Per vertex COLOR = rg::chunk_vertex_colors: RGB the hypsometric height/slope ramp (or the road paint colour), A the
// vertex kind (1 natural ground, 0 road). With surface detail on, the colours only choose the layer (green -> grass,
// else dirt, steep -> rock; road: the dirt-brown paint -> unpaved, else paved) and the layer's CC0 texture
// (data/textures, game/scripts/surface_materials.gd) draws it. rg_surface_detail 0 = the plain colours.
//
// Floating origin: textures are placed by SESSION position = render-frame position + the render origin. The render
// frame (Godot world) is the chunk instance transform (chunk origin - render origin, set by RgTerrainView), so a
// fragment's world position is small and exact near the camera; the render origin is added as rg_origin_mod, the
// origin modulo 1024 m (camera_director.gd / surface_materials.gd), so every texture repeat (a power of two up to 1024
// m) lines up across a rebase and no large session coordinate ever reaches a float in the shader.
shader_type spatial;
// cull_disabled: chunk skirts (terrain_chunk.h write_skirt) can face either way; see the old note in git history.
render_mode cull_disabled, diffuse_lambert, specular_schlick_ggx;

global uniform int rg_surface_detail;   // 0 off (plain colours), 1 colour textures, 2 colour + relief + puddles
global uniform vec2 rg_origin_mod;      // render origin (session east, north) modulo 1024 m
global uniform float rg_wetness;        // 0..1 (rg/wet_look.h)
global uniform float rg_puddles;        // 0..1

// A small ambient floor so the unlit side of a slope does not go black (no GI here).
uniform float ambient_floor : hint_range(0.0, 1.0) = 0.12;

uniform sampler2DArray surface_albedo : source_color, filter_linear_mipmap_anisotropic, repeat_enable;
uniform sampler2DArray surface_normal : hint_normal, filter_linear_mipmap_anisotropic, repeat_enable;
// Per layer (data/textures/surface_textures.json): metres per repeat, colour tint, mean colour (far), roughness,
// relief strength.
uniform float layer_tile_m[8];
uniform vec3 layer_tint[8];
uniform vec3 layer_mean[8];
uniform float layer_roughness[8];
uniform float layer_normal_strength[8];
uniform int layer_grass = 5;
uniform int layer_dirt = 6;
uniform int layer_rock = 7;
uniform int layer_paved = 0;
uniform int layer_unpaved = 4;
uniform int layer_macro = 6;
uniform vec2 greenness = vec2(-0.02, 0.12);
uniform vec2 rock_slope = vec2(0.18, 0.4);
// A road vertex whose paint is redder than this (red minus blue) is unpaved (the dirt brown), else paved.
uniform float unpaved_red_minus_blue = 0.12;
uniform float macro_tile_m = 128.0;
uniform float macro_strength = 0.22;
uniform float detail_far_m = 600.0;
// Flat test ground: no vertex colours; force one layer (-1 = read COLOR) and place the texture by the object's own
// local x/y (the ground box never moves relative to the sim origin).
uniform int force_layer = -1;
uniform bool local_uv = false;
uniform float grid_spacing_m = 0.0;
uniform float grid_line_width_m = 0.15;
// Wet look (surface_textures.json "wet").
uniform float wet_darken = 0.62;
uniform float wet_roughness = 0.18;
uniform float wet_specular = 0.5;
uniform float ground_wet_roughness = 0.6;
uniform float ground_wet_specular = 0.3;
uniform float puddle_roughness = 0.04;
uniform float puddle_darken = 0.45;
uniform float puddle_tile_m = 16.0;
uniform float puddle_cover = 0.35;
uniform float puddle_spread = 0.1;
// Sun glints only below this roughness (from x = none to y = full): wet and puddle fragments.
uniform vec2 gloss_roughness = vec2(0.6, 0.15);

varying vec2 v_local_xy;

void vertex() {
	v_local_xy = VERTEX.xy;
}

// Session (east, north) of a render-frame position: Godot = (-north, up, -east) relative to the render origin.
vec2 session_xy(vec3 world) {
	return vec2(-world.z, -world.x) + rg_origin_mod;
}

vec3 layer_albedo(int layer, vec2 p, float lod_far) {
	if (lod_far >= 1.0) { return layer_mean[layer] * layer_tint[layer]; }
	vec3 c = texture(surface_albedo, vec3(p / layer_tile_m[layer], float(layer))).rgb * layer_tint[layer];
	return mix(c, layer_mean[layer] * layer_tint[layer], lod_far);
}

// Tangent-space relief (x along east, y along north), unit-less, scaled by the layer strength.
vec2 layer_relief(int layer, vec2 p) {
	vec2 n = texture(surface_normal, vec3(p / layer_tile_m[layer], float(layer))).rg * 2.0 - 1.0;
	return n * layer_normal_strength[layer];
}

void fragment() {
	vec3 world = (INV_VIEW_MATRIX * vec4(VERTEX, 1.0)).xyz;
	vec3 world_n = normalize((INV_VIEW_MATRIX * vec4(NORMAL, 0.0)).xyz);
	if (!FRONT_FACING) { world_n = -world_n; }
	float kind = COLOR.a;
	float dist = length(VERTEX);
	ALBEDO = COLOR.rgb;
	ROUGHNESS = 1.0;
	SPECULAR = 0.0;
	if (rg_surface_detail > 0) {
		vec2 p = local_uv ? v_local_xy : session_xy(world);
		float lod_far = smoothstep(detail_far_m * 0.6, detail_far_m, dist);
		// Layer weights: natural ground blends grass/dirt by the ramp colour and rock by slope; road (alpha 0) takes
		// the paved or unpaved layer, with a narrow blend across the interpolated edge.
		float wr = force_layer >= 0 ? 1.0 : smoothstep(0.6, 0.4, kind);
		int lr = force_layer >= 0 ? force_layer : (COLOR.r - COLOR.b > unpaved_red_minus_blue ? layer_unpaved : layer_paved);
		bool near = rg_surface_detail > 1 && lod_far < 1.0;
		vec3 albedo = vec3(0.0);
		float rough = 0.0;
		vec2 relief = vec2(0.0);
		float wc = 0.0;
		if (wr < 0.999) {
			float wb = smoothstep(greenness.x, greenness.y, COLOR.g - COLOR.r);
			wc = smoothstep(rock_slope.x, rock_slope.y, 1.0 - world_n.y);
			albedo = layer_albedo(layer_dirt, p, lod_far);
			rough = layer_roughness[layer_dirt];
			if (near) { relief = layer_relief(layer_dirt, p); }
			if (wb > 0.001) {
				albedo = mix(albedo, layer_albedo(layer_grass, p, lod_far), wb);
				rough = mix(rough, layer_roughness[layer_grass], wb);
				if (near) { relief = mix(relief, layer_relief(layer_grass, p), wb); }
			}
			if (wc > 0.001) {
				// Rock on steep ground: projected on the vertical plane the slope faces (session x = -Godot z runs
				// along a face whose normal is mostly Godot x), so it is not stretched down the face.
				vec2 q = abs(world_n.x) > abs(world_n.z) ? vec2(p.x, world.y) : vec2(p.y, world.y);
				albedo = mix(albedo, layer_albedo(layer_rock, q, lod_far), wc);
				rough = mix(rough, layer_roughness[layer_rock], wc);
				if (near) { relief = mix(relief, layer_relief(layer_rock, q), wc); }
			}
		}
		if (wr > 0.001) {
			albedo = mix(albedo, layer_albedo(lr, p, lod_far), wr);
			rough = mix(rough, layer_roughness[lr], wr);
			if (near) { relief = mix(relief, layer_relief(lr, p), wr); }
		}
		// Large-scale brightness variation so the repeats do not show from afar.
		if (macro_strength > 0.0) {
			float m = dot(texture(surface_albedo, vec3(p / macro_tile_m, float(layer_macro)), 2.0).rgb, vec3(0.333));
			float mean = dot(layer_mean[layer_macro], vec3(0.333));
			albedo *= clamp(1.0 + macro_strength * (m / max(mean, 0.01) - 1.0) * 2.0, 0.6, 1.4);
		}
		// Wet: darker, glossier; puddles on flat paved/unpaved road and flat ground where the noise is low.
		float wet = rg_wetness * (1.0 - 0.6 * wc);
		float puddle = 0.0;
		if (rg_puddles > 0.0 && rg_surface_detail > 1) {
			// The low spots of a blurred noise (the macro layer at a coarse mip) fill first; the threshold rises with
			// rg_puddles so at full puddles about puddle_cover of the flat area is under water.
			float noise = texture(surface_albedo, vec3(p / puddle_tile_m + vec2(0.37, 0.71), float(layer_macro)), 3.0).r;
			float flat_ground = smoothstep(0.985, 0.998, world_n.y);
			float t = layer_mean[layer_macro].r + puddle_spread * (2.0 * puddle_cover * rg_puddles - 1.0);
			puddle = smoothstep(t + 0.015, t - 0.015, noise) * flat_ground * smoothstep(0.0, 0.2, rg_puddles);
			puddle *= mix(0.5, 1.0, wr);
		}
		albedo *= mix(1.0, wet_darken, wet);
		rough = mix(rough, mix(ground_wet_roughness, wet_roughness, wr), wet);
		albedo *= mix(1.0, puddle_darken / max(wet_darken, 0.01), puddle * wet);
		rough = mix(rough, puddle_roughness, puddle);
		relief *= 1.0 - puddle;
		ALBEDO = albedo;
		ROUGHNESS = rough;
		SPECULAR = mix(0.0, mix(ground_wet_specular, wet_specular, max(wr, puddle)), max(wet, puddle));
		if (local_uv && grid_spacing_m > 0.0) {
			vec2 g = abs(mod(p + grid_spacing_m * 0.5, grid_spacing_m) - grid_spacing_m * 0.5);
			float aa = max(max(fwidth(p.x), fwidth(p.y)), 0.01);
			// Lines thinner than a pixel fade by their coverage instead of aliasing into a bright band.
			float line = (1.0 - smoothstep(grid_line_width_m * 0.5 - aa, grid_line_width_m * 0.5 + aa, min(g.x, g.y)))
				* min(1.0, grid_line_width_m / (2.0 * aa));
			ALBEDO = mix(ALBEDO, vec3(0.82, 0.82, 0.78) * mix(1.0, wet_darken, wet * 0.5), line * (1.0 - lod_far));
		}
		if (near) {
			// Relief in the plane of the ground: east = -Z, north = -X in the render frame (local x/y for the flat
			// ground, whose transform maps local x/y onto the same horizontal plane).
			vec3 t_e = local_uv ? normalize((MODEL_MATRIX * vec4(1.0, 0.0, 0.0, 0.0)).xyz) : vec3(0.0, 0.0, -1.0);
			vec3 t_n = local_uv ? normalize((MODEL_MATRIX * vec4(0.0, 1.0, 0.0, 0.0)).xyz) : vec3(-1.0, 0.0, 0.0);
			// OpenGL normal maps: green points up the image, i.e. towards -v (Godot samples v downwards).
			vec3 n = normalize(world_n + (t_e * relief.x - t_n * relief.y) * (1.0 - lod_far));
			world_n = n;
		}
	}
	// Back faces (skirts seen from behind) shade like the ground above: always hand the up-facing normal on.
	NORMAL = normalize((VIEW_MATRIX * vec4(world_n, 0.0)).xyz);
	if (!FRONT_FACING) { NORMAL = -NORMAL; }
}

// gl_compatibility: ALBEDO and source_color samples are sRGB-encoded values here (measured), LIGHT_COLOR carries the
// light energy times PI. Diffuse: Lambert with an ambient floor. With surface detail off the pre-texture formula is
// kept (ALBEDO once more in DIFFUSE_LIGHT; the renderer multiplies by ALBEDO again), so the plain ramp looks as before.
// Specular: only glossy (wet) fragments, recognised by their roughness (SPECULAR_AMOUNT is not proportional to SPECULAR
// in this renderer); normalised Blinn-Phong with Schlick's Fresnel. Sky reflections come from the renderer's ambient
// specular (SPECULAR / ROUGHNESS written in fragment()).
void light() {
	float ndotl = max(dot(NORMAL, LIGHT), 0.0);
	float lambert = (ambient_floor + (1.0 - ambient_floor) * ndotl) * ATTENUATION;
	if (rg_surface_detail > 0) {
		DIFFUSE_LIGHT += LIGHT_COLOR / PI * lambert;
		float gloss = smoothstep(gloss_roughness.x, gloss_roughness.y, ROUGHNESS);
		if (gloss > 0.0 && ndotl > 0.0) {
			vec3 h = normalize(LIGHT + VIEW);
			float shininess = 2.0 / max(ROUGHNESS * ROUGHNESS, 0.002) - 2.0;
			float fresnel = 0.02 + 0.98 * pow(1.0 - max(dot(VIEW, h), 0.0), 5.0);
			float spec = pow(max(dot(NORMAL, h), 0.0), shininess) * (shininess + 8.0) / (8.0 * PI);
			SPECULAR_LIGHT += LIGHT_COLOR / PI * spec * fresnel * gloss * ndotl * ATTENUATION;
		}
	} else {
		DIFFUSE_LIGHT += ALBEDO * LIGHT_COLOR * lambert;
	}
}
