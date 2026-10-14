shader_type spatial;
render_mode cull_disabled, diffuse_lambert, specular_disabled;
uniform float line_width_m = 0.15;
// Distance-growing depth pull toward the camera along the view ray (the
// screen position is unchanged, only the depth moves): the terrain far away is
// a coarse chunk (L2+: 4 m grid and up) whose triangles cut the road by metres,
// and a fixed 1.8 cm lift cannot win the depth test there. pull = max(near_pull,
// depth_pull_per_m * distance), capped, so the road does not show through hills
// that really stand in front of it (the cap keeps it below a typical rise).
uniform float depth_pull_per_m = 0.002;
uniform float depth_pull_near_m = 0.0;
uniform float depth_pull_max_m = 12.0;
void vertex() {
	// VERTEX is MODEL space here (no skip_vertex_transform): measure the distance
	// in view space, pull along the view ray there and bring the offset back to
	// model space (a pull taken in model space would move the ribbon toward the
	// tile origin - metres sideways and into the ground).
	vec3 view_pos = (MODELVIEW_MATRIX * vec4(VERTEX, 1.0)).xyz;
	float d = length(view_pos);
	float pull = min(max(depth_pull_near_m, depth_pull_per_m * d), depth_pull_max_m);
	vec3 pull_view = -(view_pos / max(d, 0.001)) * pull;
	VERTEX += inverse(mat3(MODELVIEW_MATRIX)) * pull_view;
}
float stripe(float distance_m, float width_m) {
	float aa = max(fwidth(distance_m),0.008);
	return 1.0-smoothstep(width_m*0.5-aa,width_m*0.5+aa,abs(distance_m));
}
void fragment() {
	float width = UV2.x;
	float half_width = width*0.5;
	float lateral = UV.x;
	float lanes = round(COLOR.r*32.0);
	float forward_lanes = round(COLOR.g*32.0);
	// Bits as road_visual.h documents them: low byte in COLOR.b, high byte in COLOR.a (256 = verge strip).
	int flags = int(round(COLOR.b*255.0)) + 256*int(round(COLOR.a*255.0));
	float grain = sin(UV.x*29.1+UV.y*18.7)*sin(UV.x*9.2-UV.y*26.3);
	float detail = 1.0-smoothstep(0.08,0.3,max(fwidth(UV.x),fwidth(UV.y)));
	vec3 asphalt = vec3(0.105,0.113,0.122)+vec3(grain*0.008*detail*UV2.y);
	if ((flags & 2048)!=0) {
		// Paving stones (also sett, cobblestone): 0.20 x 0.10 m blocks in a running bond across the road, each a
		// little lighter or darker, with dark joints that fade out with distance like the asphalt grain.
		vec2 cell = vec2(UV.y/0.20, UV.x/0.10);
		cell.x += 0.5*mod(floor(cell.y),2.0);
		vec2 id = floor(cell);
		vec2 f = fract(cell);
		float shade = fract(sin(dot(id,vec2(12.9898,78.233)))*43758.5453);
		vec3 stone = vec3(0.30,0.285,0.26)*(0.88+0.24*shade);
		vec2 aa = max(fwidth(cell),vec2(0.02));
		float joint = 1.0-(smoothstep(0.0,aa.x*1.5+0.04,f.x)*smoothstep(0.0,aa.x*1.5+0.04,1.0-f.x)*smoothstep(0.0,aa.y*1.5+0.08,f.y)*smoothstep(0.0,aa.y*1.5+0.08,1.0-f.y));
		float sharp = 1.0-smoothstep(0.25,0.6,max(aa.x,aa.y));
		asphalt = mix(vec3(0.29,0.275,0.25), mix(stone,vec3(0.16,0.15,0.14),joint*0.8), sharp);
	} else if ((flags & 4096)!=0) {
		// Concrete: a lighter grey slab with a transverse joint every 5 m.
		float slab = mod(UV.y,5.0);
		float aa = max(fwidth(UV.y),0.01);
		float joint = 1.0-smoothstep(0.0,0.02+aa,min(slab,5.0-slab));
		asphalt = vec3(0.36,0.36,0.345)+vec3(grain*0.01*detail*UV2.y)-vec3(0.12*joint);
	}
	// The gravel verge strip beside the road bed (no paint there).
	float edge = (flags & 256)!=0 ? 1.0 : 0.0;
	ALBEDO = mix(asphalt,vec3(0.27,0.25,0.22),edge);
	float paint = 0.0;
	if ((flags & 1)!=0) {
		if ((flags & 2)!=0) {
			// Edge line; across a side road's mouth (flags 64 left / 128 right) it is broken: a 6 m / 6 m broad
			// dash on a motorway or trunk (an exit or a merge), else 1.5 m / 1.5 m.
			bool fast = (flags & 8)!=0;
			bool broken = (lateral>0.0 && (flags & 64)!=0) || (lateral<0.0 && (flags & 128)!=0);
			float edge_width = broken && fast ? 0.30 : line_width_m;
			float edge_paint = stripe(abs(lateral)-(half_width-0.25),edge_width);
			if (broken) {
				float dash = fast ? 6.0 : 1.5;
				float phase = mod(UV.y,dash*2.0);
				float aa = max(fwidth(UV.y),0.015);
				edge_paint *= (1.0-smoothstep(dash-aa,dash+aa,phase))*smoothstep(0.0,aa,phase);
			}
			bool hidden = (lateral>0.0 && (flags & 512)!=0) || (lateral<0.0 && (flags & 1024)!=0);
			paint = hidden ? 0.0 : edge_paint;
		}
		if (lanes>1.0) {
			float lane_width = width/lanes;
			float boundary = round((lateral+half_width)/lane_width);
			if (boundary>0.0 && boundary<lanes) {
				float distance_m = lateral+half_width-boundary*lane_width;
				bool separator = forward_lanes>0.0 && forward_lanes<lanes && abs(boundary-forward_lanes)<0.25;
				bool solid = separator && ((flags & 4)!=0);
				float dash = ((flags & 8)!=0)?6.0:3.0;
				float period = dash*3.0;
				float phase = mod(UV.y,period);
				float aa = max(fwidth(UV.y),0.015);
				float dashed = (1.0-smoothstep(dash-aa,dash+aa,phase))*smoothstep(0.0,aa,phase);
				paint = max(paint,stripe(distance_m,line_width_m)*(solid?1.0:dashed));
			}
		}
	}
	paint *= UV2.y*(1.0-edge);
	if ((flags & 16)!=0) { paint = 1.0; }
	// Junction overlay: only the paint, the junction patch below shows everywhere else.
	if ((flags & 32)!=0 && paint < 0.5) { discard; }
	ALBEDO = mix(ALBEDO,vec3(0.85,0.86,0.83),paint);
	ROUGHNESS = 0.94;
	SPECULAR = 0.0;
	if (!FRONT_FACING) { NORMAL = -NORMAL; }
}
