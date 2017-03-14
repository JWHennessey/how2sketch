#version 400

layout (location = 0) in vec4 vertex;

uniform mat4 mvMatrix;
uniform mat4 projMatrix;
uniform mat3 ellipse_rotation;
uniform vec3 translation;
uniform float radius;

void main() {


   vec4 newVertex = vertex;
   newVertex[0] *= radius;
   newVertex[2] *= radius;

   vec3 rotated = translation + ellipse_rotation*newVertex.xyz;

   gl_Position = projMatrix*mvMatrix*vec4(rotated.xyz, newVertex[3]);

}
