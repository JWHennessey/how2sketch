double noise1(double arg);
float noise2(float vec[2]);
float noise3(float vec[3]);
inline float noise2(float x,float y)
{ float vec[2] = {x,y}; return noise2(vec); }
