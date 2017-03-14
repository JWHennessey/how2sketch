#version 400

layout (location = 0) in vec4 vertex;

uniform mat4 mvMatrix;
uniform mat4 projMatrix;
uniform bool hasGeomShader;

out vec2 ab_line;
out vec2 a_point;
/*
;
uniform mat3 normalMatrix;
*/

void main() {

    //Only setting these for when no geometry shadder is used
   ab_line = vec2(0,0);
   a_point = vec2(0,0);

   if(!hasGeomShader)
        gl_Position = projMatrix*mvMatrix*vertex;
   else
       gl_Position = mvMatrix*vertex;
}
