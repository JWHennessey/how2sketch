#version 400

layout (location = 0) in vec4 vertex;
layout (location = 7) in vec3 colour;

//layout (location = 1) in vec3 normal;

uniform mat4 mvMatrix;
uniform mat4 modelMatrix;
//uniform mat3 normalMatrix;
uniform mat4 projMatrix;
out vec3 vcolour;
//out float depth;

//out vec3 EyeNormal;       // Normal in eye coordinates
//out vec4 EyePosition;     // Position in eye coordinates

void main()
{
    //EyeNormal = normalize(normalMatrix * normal);
    //EyePosition = mvMatrix * vertex;

    gl_Position = projMatrix * mvMatrix * vertex;
    vcolour = colour;
    //scolour = seg_colour;
    //depth = gl_Position.z;
}

