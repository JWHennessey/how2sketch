#version 400

layout(triangles) in;
layout(triangle_strip, max_vertices=48) out;

#define SMALL_FLOAT 0.1e-5
#define THICK_RADIUS 0.003
#define THIN_RADIUS 0.002

out vec2 ab_line;
out vec2 a_point;

uniform bool showMidLines;
uniform bool showDiagLines;
uniform mat4 projMatrix;
uniform mat4 mvMatrix;
uniform mat3 normalMatrix;

vec4 makeVec4(vec3 inVec)
{
  return vec4(inVec.x, inVec.y, inVec.z, 1);
}

vec2 screen_space(vec4 vertex)
{
    return vec2( vertex.xy / vertex.w );
}

void renderFaceA(vec3 v1, vec3 v2, vec3 n, float r)
{

   vec2 ab = v1.xy - v2.xy;
   vec2 a = v1.xy;

   gl_Position = makeVec4(v1 + n * r);
   ab_line = ab;
   a_point = a;
   EmitVertex();
   gl_Position = makeVec4(v1 - n * r);
   ab_line = ab;
   a_point = a;
   EmitVertex();
   gl_Position = makeVec4(v2 - n * r);
   ab_line = ab;
   a_point = a;
   EmitVertex();

   EndPrimitive();
}

void renderFaceB(vec3 v1, vec3 v2, vec3 n, float r)
{
   vec2 ab = v2.xy - v1.xy;
   vec2 a = v2.xy;

   gl_Position = makeVec4(v1 - n * r);
   ab_line = ab;
   a_point = a;
   EmitVertex();
   gl_Position = makeVec4(v1 + n * r);
   ab_line = ab;
   a_point = a;
   EmitVertex();
   gl_Position = makeVec4(v2 + n * r);
   ab_line = ab;
   a_point = a;
   EmitVertex();

   EndPrimitive();
}

vec4 renderLineFaces(vec4 a, vec4 b, float r)
{
    vec4 line = (a - b);
    b = a + line * 1000;
    vec3 l = line.xyz / line.w;
    if(abs(dot(l, vec3(0,1,0))) <= abs(dot(l, vec3(1,0,0))))
    {
        vec3 n = normalize(vec3(-l.y, l.x, 0));
        renderFaceA(a.xyz / a.w, b.xyz / b.w, n, r);
        renderFaceB(b.xyz / b.w, a.xyz / a.w, n, r);
        return b;
    }
    return vec4(-1,-1,-1,-1);
}

vec4 renderLine(int a, int b)
{
    vec4 x = projMatrix*gl_in[a].gl_Position;
    vec4 y = projMatrix*gl_in[b].gl_Position;
    return renderLineFaces(x,y, THICK_RADIUS);
}

void drawHorizonLine(vec4 p)
{
    //p[2] = 0;
    vec4 d = vec4(10000,0,0,0);
    vec4 a = p - d;
    vec4 b = p + d;

    vec3 aprime = a.xyz / a.w;
    vec3 bprime = b.xyz / b.w;

    aprime[2] = 0;
    bprime[2] = 0;

    vec3 n = vec3(0,1,0);
    renderFaceA(aprime, bprime, n, THICK_RADIUS);
    renderFaceB(bprime, aprime, n, THICK_RADIUS);
}

void main()
{
  vec4 edge1 = (gl_in[0].gl_Position - gl_in[1].gl_Position);
  vec4 edge2 = (gl_in[1].gl_Position - gl_in[2].gl_Position);
  vec4 edge3 = (gl_in[2].gl_Position - gl_in[0].gl_Position);

  float d1 = dot(edge1, edge2);
  float d2 = dot(edge2, edge3);
  float d3 = dot(edge3, edge1);

  vec4 vp1 = vec4(-1,-1,-1,-1);
  vec4 vp2 = vec4(-1,-1,-1,-1);
  if(abs(d1) < SMALL_FLOAT)
  {
        vp1 = renderLine(0,1);
        vp2 = renderLine(1,2);
  }
  else if(abs(d2) < SMALL_FLOAT)
  {
        vp1 = renderLine(1,2);
        vp2 = renderLine(2,0);
  }
  else if(abs(d3) < SMALL_FLOAT)
  {
        vp1 = renderLine(2,0);
        vp2 = renderLine(1,0);
  }

//  if(length(vp1) != 4.0)
//        drawHorizonLine(vp1);
//  if(length(vp2) != 4.0)
//        drawHorizonLine(vp2);
}

