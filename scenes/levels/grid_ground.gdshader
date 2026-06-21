shader_type spatial;
// World-space grid for the ground/platforms. Lines are derived from world
// position (not UVs) so they stay continuous and evenly spaced across the
// separate ground boxes, which gives a readable sense of scale and horizontal
// speed as the arm guy slides past. fwidth() keeps the lines ~1px regardless of
// distance, so they never shimmer or thicken up close.

uniform vec3 base_color : source_color = vec3(0.3, 0.35, 0.3);
uniform vec3 line_color : source_color = vec3(0.19, 0.23, 0.2);
uniform float grid_size = 1.0;
uniform float roughness_val : hint_range(0.0, 1.0) = 0.95;

varying vec3 world_pos;

void vertex() {
	world_pos = (MODEL_MATRIX * vec4(VERTEX, 1.0)).xyz;
}

void fragment() {
	// Grid is drawn on the X-Y plane (the faces the side-on camera sees).
	vec2 coord = world_pos.xy / grid_size;
	vec2 g = abs(fract(coord - 0.5) - 0.5) / fwidth(coord);
	float line = 1.0 - min(min(g.x, g.y), 1.0);
	ALBEDO = mix(base_color, line_color, line);
	ROUGHNESS = roughness_val;
	METALLIC = 0.0;
}
