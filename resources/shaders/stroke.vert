#version 400

layout (location = 0) in vec4 vertex;
layout (location = 1) in vec4 new_vertex;

uniform mat4 projMatrix;
uniform mat4 viewMatrix;
uniform bool isCurrent;

void main(void)
{
//    if(isCurrent)
//        gl_Position = projMatrix * new_vertex;
//    else
        gl_Position = projMatrix * vertex;
}

