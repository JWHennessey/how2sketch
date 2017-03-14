#include <stdio.h>
#include <float.h>
#include "Point.h"

void ScanY(Point v[], int nv, int bot,void (*drawPoint)(int x,int y));

//void drawTriangle(Point v1,Point v2,Point v3);
void drawQuad(Point v1,Point v2,Point v3,Point v4);
void drawPoly(Point v[]);

inline float det2(Point v1,Point v2)
{
  //    printf("v1' = (%f,%f)\n",v1.x,v1.y);
  //    printf("v2' = (%f,%f)\n",v2.x,v2.y);

  return v1.x*v2.y - v1.y*v2.x;
}

inline bool ccw(Point v1,Point v2,Point v3)
{
  /*  printf("v1 = (%f,%f)\n",v1.x,v1.y);
  printf("v2 = (%f,%f)\n",v2.x,v2.y);
  printf("v3 = (%f,%f)\n",v3.x,v3.y);
  printf("%f\n",det2(v1-v2,v2-v3));
  */
  return det2(v2-v1,v3-v1) >= 0;
}

