#include <math.h>
#include "GL/glut.h"
#include <assert.h>
#include "polyfill.h"
#include "Point.h"

void ScanX(Point&l,Point & r, float y,void (*drawPoint)(GLint x,GLint y))
{
  int lx = ceil(l.x), rx = ceil(r.x);
  for(int x= lx;x < rx; x++)
    drawPoint(x,y);
}

void DiffY(Point&b, Point&t,Point&m,Point&dy,int y)
{
  dy = (t-b)/(t.y-b.y);
  m = b+dy*(y-b.y);
}

int mod(int x,int y)
{
  if (x < 0)
    return mod(x+y,y);
  else
    return x%y;

  // or:  MOD(x,y) is x - y.*floor(x./y) if y ~= 0.  By convention, MOD(x,0) is x.

}


void ScanY(Point v[], int nv, int bot,void (*drawPoint)(GLint x,GLint y))
{
  // v: array of vertecies in counterclockwise order
  // nv: number of vertecies
  // bot: number of the Point with the min y coordinate

  int li = bot, ri = li;
  int y = ceil(v[bot].y), ly = y, ry = y;
  
  Point l, dl, r, dr;

  for(int rem = nv; rem > 0; )
    {
      // find left boundary edge of tile
      while (ly <= y && rem --  > 0)
	{
	  int i = li; 
	  li = mod(li-1,nv);  
	  assert(li >= 0);
	  ly = ceil(v[li].y);
	  if (ly > y) 
	    DiffY(v[i],v[li],l,dl,y);
	}

      // find right boundary edge of  atile

      while (ry <= y && rem -- > 0)
	{
	  int i= ri; 
	  ri = mod(ri+1,nv); 
	  ry = ceil(v[ri].y);
	  if (ry> y)   
	    DiffY(v[i],v[ri],r,dr,y);
	}
      
      for(; y < ly && y < ry; y++)
	{	
	  ScanX(l,r,y,drawPoint); 
	  l+=dl; 
	  r+= dr;
	}
    }
}

void drawPoly(Point v[],int nv)
{
  assert(ccw(v[0],v[1],v[2]));

  int bot = 0;
  float miny = v[bot].y;

  for(int i=1;i<nv;i++)
    if (v[i].y < miny)
      {
	miny = v[i].y;
	bot = i;
      }

  //  printf("POLYGON\n");
  //  printf("v[%d] = (%f,%f)\n",bot,v[bot].x,v[bot].y);

  //  for(i=(bot+1)%nv;i!=bot;i=(i+1)%nv)
  //    printf("v[%d] = (%f,%f)\n",i,v[i].x,v[i].y);

    //  printf("det = %f\n",det2(v[1]-v[0],v[2]-v[0]));

  glPointSize(1);
  glBegin(GL_POINTS);

  ScanY(v,nv,bot,glVertex2i);

  glEnd();
}
	
//void drawTriangle(Point v1,Point v2,Point v3)
//{
//  if(ccw(v1,v2,v3))
//    {
//      //      assert(ccw(v1,v2,v3));
//      //      printf("noflip\n");
//      Point v[] = {v1,v2,v3};
//      drawPoly(v,3);
//    }
//  else
//    {
//      //      assert(ccw(v1,v3,v2));
//      //      printf("flip\n");
//      Point v[] = {v1,v3,v2};
//      drawPoly(v,3);
//    }
//}

void drawQuad(Point v1,Point v2,Point v3,Point v4)
{
  if (ccw(v1,v2,v3))
    {
      Point v[] = {v1,v2,v3,v4};
      drawPoly(v,4);
    }
  else
    {
      Point v[] = {v4,v3,v2,v1};
      drawPoly(v,4);
    }
}
