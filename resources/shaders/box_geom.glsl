#version 400

layout(triangles) in;
layout(triangle_strip, max_vertices=84) out;

#define SMALL_FLOAT 0.1e-5
#define THICK_RADIUS 0.003
#define THIN_RADIUS 0.0025

out vec2 ab_line;
out vec2 a_point;

uniform bool showMidLines;
uniform bool showDiagLines;
uniform bool showThirdsDiagLine;
uniform bool showTwoThirdsDiagLine;
uniform bool showThirdsLine;
uniform bool showTwoThirdsLine;
uniform mat4 projMatrix;
uniform mat4 mvMatrix;
uniform mat3 normalMatrix;
uniform bool render_mid_plane;

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

void renderLineFaces(vec4 a, vec4 b, float r)
{
   vec4 line = (a - b);

   vec3 l = line.xyz / line.w;

   if(l.x < 0.0001 && render_mid_plane && length(l) > 0.0)
       return;

   vec3 n = normalize(vec3(-l.y, l.x, 0));
   renderFaceA(a.xyz / a.w, b.xyz / b.w, n, r);
   renderFaceB(b.xyz / b.w, a.xyz / a.w, n, r);
}

void renderLine(int a, int b)
{
    vec4 x = projMatrix*gl_in[a].gl_Position;
    vec4 y = projMatrix*gl_in[b].gl_Position;
    renderLineFaces(x,y, THICK_RADIUS);
}

void renderHalfMidLine(int a, int b, int other)
{
   vec4 x = projMatrix*gl_in[a].gl_Position;
   vec4 y = projMatrix*gl_in[b].gl_Position;
   vec4 o = projMatrix*gl_in[other].gl_Position;
   vec4 xyAvg =  (x + y) / 2.0;
   vec4 allAvg = (x + o) / 2.0;




   float diff = abs(x[0] - y[0]);
   if(diff > SMALL_FLOAT)
   {
      renderLineFaces(xyAvg, allAvg, THIN_RADIUS);
   }
}

void renderHalfThirdsLine(int a, int b, int other)
{
   vec4 x = projMatrix*gl_in[a].gl_Position;
   vec4 y = projMatrix*gl_in[b].gl_Position;
   vec4 o = projMatrix*gl_in[other].gl_Position;
   vec4 xyAvg =  y + (x - y) * 0.333;
   vec4 allAvg = o + (x - o) * 0.333;
   renderLineFaces(xyAvg, allAvg, THIN_RADIUS);
}

void renderHalfTwoThirdsLine(int a, int b, int other)
{
   vec4 x = projMatrix*gl_in[a].gl_Position;
   vec4 y = projMatrix*gl_in[b].gl_Position;
   vec4 o = projMatrix*gl_in[other].gl_Position;
   vec4 xyAvg =  x + (y - x) * 0.333;
   vec4 allAvg = x + (o - x) * 0.333;

   vec4 mid = (xyAvg + allAvg) * 0.5;

   allAvg = mid + (allAvg - mid) * 2.0;

   renderLineFaces(xyAvg, allAvg, THIN_RADIUS);
}

void renderHalfDiagLine(int a, int b, int other)
{
   vec4 x = projMatrix*gl_in[a].gl_Position;
   vec4 y = projMatrix*gl_in[b].gl_Position;
   vec4 o = projMatrix*gl_in[other].gl_Position;
   vec4 allAvg = (x + o) / 2.0;
   renderLineFaces(o, allAvg, THIN_RADIUS);
   renderLineFaces(y, allAvg, THIN_RADIUS);

}

void renderThirdDiagLine(int a, int b, int other, bool isThird)
{
   vec4 x = projMatrix*gl_in[a].gl_Position;
   vec4 y = projMatrix*gl_in[b].gl_Position;
   vec4 o = projMatrix*gl_in[other].gl_Position;


   vec4 allAvg = (y + x) / 2.0;
   if(allAvg.x < o.x || isThird)
        renderLineFaces(o, allAvg, THIN_RADIUS);
   //renderLineFaces(y, allAvg, THIN_RADIUS);

}

void main()
{
  vec4 edge1 = (gl_in[0].gl_Position - gl_in[1].gl_Position);
  vec4 edge2 = (gl_in[1].gl_Position - gl_in[2].gl_Position);
  vec4 edge3 = (gl_in[2].gl_Position - gl_in[0].gl_Position);

  float d1 = dot(edge1, edge2);
  float d2 = dot(edge2, edge3);
  float d3 = dot(edge3, edge1);

  float min = abs(d1);
  if(min > abs(d2))
  {
      min = abs(d2);
  }
  if(min > abs(d3))
  {
      min = abs(d3);
  }

  //if(abs(d1) < SMALL_FLOAT)
  if(abs(d1) == min)
  {

        if(!render_mid_plane)
        {
            renderLine(0,1);
            renderLine(1,2);
        }
//
        if(showMidLines)
        {
                renderHalfMidLine(0,1,2);
                renderHalfMidLine(2,1,0);
        }


        if(showDiagLines)
        {
                renderHalfDiagLine(0,1,2);
                renderHalfDiagLine(2,1,0);
        }
/*
        if(showTwoThirdsDiagLine )
        {
            renderThirdDiagLine(0,1,2, false);
        }
*/
  }
  //else if(abs(d2) < SMALL_FLOAT)
  else if(abs(d2) == min)
  {

      if(!render_mid_plane)
      {
        renderLine(1,2);
        renderLine(2,0);
      }

        if(showMidLines)
        {
                renderHalfMidLine(1,2,0);
                renderHalfMidLine(0,2,1);
        }


        if(showDiagLines)
        {
                renderHalfDiagLine(1,2,0);
                renderHalfDiagLine(0,2,1);
        }

       /* if(showThirdsDiagLine)
        {
            renderThirdDiagLine(1,2,0, true);
            renderThirdDiagLine(0,2,0, true);
        }


        if(showThirdsLine)
        {
            renderHalfThirdsLine(1,2,0);
        }

        if(showTwoThirdsLine)
        {
            renderHalfTwoThirdsLine(1,2,0);
        } */


  }
  //else if(abs(d3) < SMALL_FLOAT)
  else if(abs(d3) == min)
  {

      if(!render_mid_plane)
      {
        renderLine(2,0);
        renderLine(1,0);
      }

        if(showMidLines)
        {
                renderHalfMidLine(2,0,1);
                renderHalfMidLine(1,0,2);
        }

        if(showDiagLines)
        {
                renderHalfDiagLine(2,0,1);
                renderHalfDiagLine(1,0,2);
        }

/*
        if(showThirdsLine)
        {
            renderHalfThirdsLine(1,0,2);
        }

        if(showTwoThirdsLine)
        {
            renderHalfTwoThirdsLine(1,0,2);
        }
*/

  }


}

