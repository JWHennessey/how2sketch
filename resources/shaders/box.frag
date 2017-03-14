#version 400

layout( location = 0 ) out vec4 color;

uniform vec3 box_colour;
uniform bool hasGeomShader;
uniform bool dashedLines;

in vec2 ab_line;
in vec2 a_point;

void main()
{

    float alpha = hasGeomShader ? 0.5 : 1.0;
    if(dashedLines)
    {
        vec2 ap_line = a_point - gl_FragCoord.xy;
        vec2 line_proj = a_point + dot(ap_line, ab_line) / dot(ab_line, ab_line) * ab_line;

        if (cos(0.4*abs(distance(a_point, line_proj))) + 0.1 > 0.0)
        {
            color = vec4(255,0,0,0);
        }
        else
        {
            color = vec4(box_colour[0], box_colour[1], box_colour[2], 0.2);
        }
    }
    else
    {
        color = vec4(box_colour[0], box_colour[1], box_colour[2], alpha);
    }
}

