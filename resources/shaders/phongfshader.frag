#version 400

in vec3 EyeNormal;       // Normal in eye coordinates
in vec4 EyePosition;
in vec3 normal_test;

layout( location = 0 ) out vec4 color;

struct MaterialInfo {
    vec3 Kd;
    vec3 Ks;
    vec3 Ka;
    float Shininess;
};
uniform MaterialInfo Material;

struct LightInfo {
    vec3 Intensity;
    vec3 Position;
};

uniform LightInfo Light;

vec3 phongModel( vec3 pos, vec3 norm ) {
    vec3 s = normalize(vec3(Light.Position) - pos);
    vec3 v = normalize(-pos.xyz);
    vec3 r = reflect( -s, norm );
    vec3 ambient = Light.Intensity * Material.Ka;
    float sDotN = max( dot(s,norm), 0.0 );
    vec3 diffuse = Light.Intensity * Material.Kd * sDotN;
    vec3 spec = vec3(0.0);
    if( sDotN > 0.0 )
        spec = Light.Intensity * Material.Ks *
               pow( max( dot(r,v), 0.0 ), Material.Shininess );

    return ambient + diffuse + spec;
}

void main()
{
   vec3 c = phongModel(vec3(EyePosition), EyeNormal);
   color = vec4(c, 1.0);
}
