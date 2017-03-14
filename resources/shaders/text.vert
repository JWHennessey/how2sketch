#version 400

layout (location = 0) in vec4 vertex;
layout (location = 1) in vec2 texCoord;

out vec2 texc;   // Normal in eye coordinates
uniform mat4 matrix;

void main()
{
    gl_Position = matrix*vertex;
    texc = texCoord;
}
