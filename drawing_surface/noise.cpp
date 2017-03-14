/* coherent noise function over 1, 2 or 3 dimensions */
/* (copyright Ken Perlin) */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "noise.h"

#define B 0x100
#define BM 0xff

#define N 0x1000
#define NP 12   /* 2^N */
#define NM 0xfff

static int p[B + B + 2];
static float g3[B + B + 2][3];
static float g2[B + B + 2][2];
static float g1[B + B + 2];
static bool start = true;

static void init(void);

inline float s_curve(float t) 
{
  return t * t * (3. - 2. * t);
}

inline float lerp(float t,float a,float  b)
{
  return  a + t * (b - a); 
}

/*
#define setup(i,b0,b1,r0,r1)\
        t = vec[i] + N;\
        b0 = ((int)t) & BM;\
        b1 = (b0+1) & BM;\
        r0 = t - (int)t;\
        r1 = r0 - 1.;
*/

inline void setup(int i,float & t,int & b0,int & b1,float & r0, float & r1,
		  float * vec)
{
  t = vec[i] + N;
  b0 = ((int)t) & BM;
  b1 = (b0+1) & BM;
  r0 = t - (int)t;
  r1 = r0 - 1.;
} 

double noise1(double arg)
{
  int bx0, bx1;
  float rx0, rx1, sx, t, u, v, vec[1];

  vec[0] = arg;
  if (start) {
    start = false;
    init();
  }

  setup(0,t, bx0,bx1, rx0,rx1,vec);

  sx = s_curve(rx0);

  u = rx0 * g1[ p[ bx0 ] ];
  v = rx1 * g1[ p[ bx1 ] ];

  return lerp(sx, u, v);
}

inline float at2(float rx,float ry,float *q)
{
  return rx*q[0] + ry*q[1];
}

inline float at3(float rx,float ry,float rz,float *q)
{
  return rx * q[0] + ry * q[1] + rz * q[2];
}



float noise2(float vec[2])
{
  int bx0, bx1, by0, by1, b00, b10, b01, b11;
  float rx0, rx1, ry0, ry1, *q, sx, sy, a, b, t, u, v;
  int i, j;

  if (start) {
    start = false;
    init();
  }

  setup(0,t,bx0,bx1, rx0,rx1,vec);
  setup(1,t,by0,by1, ry0,ry1,vec);

  i = p[ bx0 ];
  j = p[ bx1 ];

  b00 = p[ i + by0 ];
  b10 = p[ j + by0 ];
  b01 = p[ i + by1 ];
  b11 = p[ j + by1 ];

  sx = s_curve(rx0);
  sy = s_curve(ry0);

  //#define at2(rx,ry) ( rx * q[0] + ry * q[1] )

  q = g2[ b00 ] ; u = at2(rx0,ry0,q);
  q = g2[ b10 ] ; v = at2(rx1,ry0,q);
  a = lerp(sx, u, v);

  q = g2[ b01 ] ; u = at2(rx0,ry1,q);
  q = g2[ b11 ] ; v = at2(rx1,ry1,q);
  b = lerp(sx, u, v);

  return lerp(sy, a, b);
}

float noise3(float vec[3])
{
  int bx0, bx1, by0, by1, bz0, bz1, b00, b10, b01, b11;
  float rx0, rx1, ry0, ry1, rz0, rz1, *q, sy, sz, a, b, c, d, t, u, v;
  int i, j;

  if (start) {
    start = false;
    init();
  }

  setup(0,t,bx0,bx1, rx0,rx1,vec);
  setup(1,t,by0,by1, ry0,ry1,vec);
  setup(2,t,bz0,bz1, rz0,rz1,vec);

  i = p[ bx0 ];
  j = p[ bx1 ];

  b00 = p[ i + by0 ];
  b10 = p[ j + by0 ];
  b01 = p[ i + by1 ];
  b11 = p[ j + by1 ];

  t  = s_curve(rx0);
  sy = s_curve(ry0);
  sz = s_curve(rz0);

  //#define at3(rx,ry,rz) ( rx * q[0] + ry * q[1] + rz * q[2] )

  q = g3[ b00 + bz0 ] ; u = at3(rx0,ry0,rz0,q);
  q = g3[ b10 + bz0 ] ; v = at3(rx1,ry0,rz0,q);
  a = lerp(t, u, v);

  q = g3[ b01 + bz0 ] ; u = at3(rx0,ry1,rz0,q);
  q = g3[ b11 + bz0 ] ; v = at3(rx1,ry1,rz0,q);
  b = lerp(t, u, v);

  c = lerp(sy, a, b);

  q = g3[ b00 + bz1 ] ; u = at3(rx0,ry0,rz1,q);
  q = g3[ b10 + bz1 ] ; v = at3(rx1,ry0,rz1,q);
  a = lerp(t, u, v);

  q = g3[ b01 + bz1 ] ; u = at3(rx0,ry1,rz1,q);
  q = g3[ b11 + bz1 ] ; v = at3(rx1,ry1,rz1,q);
  b = lerp(t, u, v);

  d = lerp(sy, a, b);

  return lerp(sz, c, d);
}

static void normalize2(float v[2])
{
  float s;

  s = sqrt(v[0] * v[0] + v[1] * v[1]);
  v[0] = v[0] / s;
  v[1] = v[1] / s;
}

static void normalize3(float v[3])
{
  float s;

  s = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  v[0] = v[0] / s;
  v[1] = v[1] / s;
  v[2] = v[2] / s;
}

static void init(void)
{
  int i, j, k;

  for (i = 0 ; i < B ; i++) {
    p[i] = i;

//    g1[i] = (float)((random() % (B + B)) - B) / B;

//    for (j = 0 ; j < 2 ; j++)
//      g2[i][j] = (float)((random() % (B + B)) - B) / B;
//    normalize2(g2[i]);

//    for (j = 0 ; j < 3 ; j++)
//      g3[i][j] = (float)((random() % (B + B)) - B) / B;
//    normalize3(g3[i]);
  }

  while (--i) {
    k = p[i];
//    p[i] = p[j = random() % B];
    p[j] = k;
  }

  for (i = 0 ; i < B + 2 ; i++) {
    p[B + i] = p[i];
    g1[B + i] = g1[i];
    for (j = 0 ; j < 2 ; j++)
      g2[B + i][j] = g2[i][j];
    for (j = 0 ; j < 3 ; j++)
      g3[B + i][j] = g3[i][j];
  }
}
