#version 400

//in vec3 EyeNormal;       // Normal in eye coordinates
//in vec4 EyePosition;     // Position in eye coordinates

//in float depth;

layout( location = 0 ) out vec4 color;

in vec3 vcolour;

void main() {
     vec3 c = vec3(0.5, 0.7, 0.4);
     color = vec4(vcolour, 1.0);

}
