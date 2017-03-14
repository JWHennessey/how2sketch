#version 400

layout (location = 0) in vec4 vertex;
layout (location = 1) in vec3 normal;


out vec3 EyeNormal;       // Normal in eye coordinates
out vec4 EyePosition;
out vec3 normal_test;

uniform mat4 projMatrix;
uniform mat4 mvMatrix;
uniform mat3 normalMatrix;
uniform mat4 modelMatrix;

void main() {
   normal_test = normal;
   EyeNormal = normalize(normalMatrix * normal);
   EyePosition = mvMatrix * vertex;
   gl_Position = projMatrix*mvMatrix*vertex;
}


