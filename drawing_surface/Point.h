#ifndef __POINT_HH__
#define __POINT_HH__

struct Point
{
  float x,y;

  Point() { };
  Point(float x,float y) { this->x = x; this->y = y; }
  Point operator+(const Point & v) const { return Point(x+v.x,y+v.y); }
  Point operator-(const Point & v) const { return Point(x-v.x,y-v.y); }
  Point operator*(float v) const { return Point(x*v,y*v); }
  Point operator/(float v) const { return Point(x/v,y/v); }

  Point & operator=(const Point & v) { x = v.x;y=v.y; return *this; }
  Point & operator+=(const Point & v) { x += v.x;y+=v.y; return *this; }
};

#endif
