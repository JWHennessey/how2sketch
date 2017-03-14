#version 400



layout( location = 0 ) out vec4 color;

uniform bool isCurrent;
in vec3 vcolour;

void main() {

     if(isCurrent)
        color = vec4(0.5);
     else
        color =  vec4(0.0, 0.0, 0.0, 0.5);

}
