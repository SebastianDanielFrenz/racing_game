shader_type spatial;
render_mode cull_disabled, diffuse_lambert, specular_disabled;
uniform float line_width_m = 0.15;
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
	int flags = int(round(COLOR.b*16.0));
	float grain = sin(UV.x*29.1+UV.y*18.7)*sin(UV.x*9.2-UV.y*26.3);
	float detail = 1.0-smoothstep(0.08,0.3,max(fwidth(UV.x),fwidth(UV.y)));
	vec3 asphalt = vec3(0.105,0.113,0.122)+vec3(grain*0.008*detail*UV2.y);
	float edge = smoothstep(half_width-0.03,half_width+0.05,abs(lateral))*UV2.y;
	ALBEDO = mix(asphalt,vec3(0.27,0.25,0.22),edge);
	float paint = 0.0;
	if ((flags & 1)!=0) {
		if ((flags & 2)!=0) {
			paint = stripe(abs(lateral)-(half_width-0.25),line_width_m);
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
	paint *= UV2.y;
	if ((flags & 16)!=0) { paint = 1.0; }
	ALBEDO = mix(ALBEDO,vec3(0.85,0.86,0.83),paint);
	ROUGHNESS = 0.94;
	SPECULAR = 0.0;
	if (!FRONT_FACING) { NORMAL = -NORMAL; }
}
