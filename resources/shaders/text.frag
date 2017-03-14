#version 400


uniform sampler2D textTexture;
in vec2 texc;

layout( location = 0 ) out vec4 color;

void main()
{
   vec3 rgb = texture(textTexture, texc).rgb;
   color = vec4(rgb, 1);
}

