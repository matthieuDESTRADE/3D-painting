#version 330 core

// Vertex shader - this code is executed for every vertex of the shape

// Inputs coming from VBOs
layout (location = 0) in vec3 vertex_position; // vertex position in local space (x,y,z)
layout (location = 1) in vec3 vertex_normal;   // vertex normal in local space   (nx,ny,nz)
layout (location = 2) in vec3 vertex_color;    // vertex color      (r,g,b)
layout (location = 3) in vec2 vertex_uv;       // vertex uv-texture (u,v)
layout (location = 4) in vec3 instance_position;  // instance position  (x,y,z)
layout (location = 5) in vec3 instance_rotation_x;  // instance position  (x,y,z)
layout (location = 6) in vec3 instance_rotation_y;  // instance position  (x,y,z)
layout (location = 7) in vec3 instance_rotation_z;  // instance position  (x,y,z)
layout (location = 8) in vec3 instance_color;  // instance position  (x,y,z)
layout (location = 9) in vec2 instance_text;  // instance position  (x,y,z)


//layout (location = 5) in mat3 instance_rotation;  // instance position  (x,y,z)


// Output variables sent to the fragment shader
out struct fragment_data
{
    vec3 position; // vertex position in world space
    vec3 normal;   // normal position in world space
    vec3 color;    // vertex color
    vec2 uv;       // vertex uv
} fragment;

// Uniform variables expected to receive from the C++ program
uniform mat4 model; // Model affine transform matrix associated to the current shape
uniform mat4 view;  // View matrix (rigid transform) of the camera
uniform mat4 projection; // Projection (perspective or orthogonal) matrix of the camera

uniform mat4 modelNormal; // Model without scaling used for the normal. modelNormal = transpose(inverse(model))

uniform float time;
float rand(vec2 n) { 
	return fract(sin(dot(n, vec2(12.9898, 4.1414))) * 43758.5453);
}

float noise(vec2 p){
	vec2 ip = floor(p);
	vec2 u = fract(p);
	u = u*u*(3.0-2.0*u);
	
	float res = mix(
		mix(rand(ip),rand(ip+vec2(1.0,0.0)),u.x),
		mix(rand(ip+vec2(0.0,1.0)),rand(ip+vec2(1.0,1.0)),u.x),u.y);
	return res*res;
}
void main()
{
	mat3 rot = mat3(instance_rotation_x,instance_rotation_y,instance_rotation_z);


	rot = transpose(rot);
	// The position of the vertex in the world space - Add the offset related to the current instance
	vec4 position = model * vec4(rot*vertex_position + instance_position, 1.0);

	// The normal of the vertex in the world space
	vec4 normal = modelNormal * vec4(rot*vertex_normal, 0.0);

	// The projected position of the vertex in the normalized device coordinates:
	vec4 position_projected = projection * view * position;

	// Fill the parameters sent to the fragment shader
	fragment.position = position.xyz;
	fragment.normal   = normal.xyz;
	fragment.color = vertex_color*instance_color;
	fragment.uv = vec2(vertex_uv.x/4,vertex_uv.y) + instance_text/4;

	// gl_Position is a built-in variable which is the expected output of the vertex shader
	gl_Position = position_projected; // gl_Position is the projected vertex position (in normalized device coordinates)
}
