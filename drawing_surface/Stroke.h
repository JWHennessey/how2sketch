//#include "GL/glut.h"
#include <math.h>
#include <vector>
#include "Point.h"
#include <QOpenGLFunctions>
#include "Eigen/Dense"
#include "Eigen/Sparse"

using namespace std;


#define SCAN_CONVERT
#define NUM_SLICES 20

typedef enum { CUBIC_BSPLINE, FOUR_POINT } CurveType;

using MatrixXf = Eigen::Matrix< float,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>;
using MatrixXus = Eigen::Matrix<unsigned short,Eigen::Dynamic, Eigen::Dynamic,Eigen::RowMajor>;


class Stroke : protected QOpenGLFunctions
{
private:
  vector<Point> control;
  vector<Point> * limit;
  vector<Point> * temp;
  vector<Point> * vertices;
  //vector<int> * faces;

  bool computed;

  int numLevels;

  //static GLUquadricObj * qobj;
public:

  float z;

  float radius;
  float taperStepSize;

  bool useTexture;
  float ufreq;
  float vfreq;
  float ustart;
  float vstart;

  CurveType curveType;

  Stroke();
  ~Stroke();
  void add(float x, float y);
  void clear();
  void clearVertices();
  static void drawLines(vector<Point> * curve);
  void forceRecompute();
  void discPoint(float x,float y,float brushRadius);
  void drawCap(const Point & p0, float dx, float dy,float texU,float texV);
  void drawThickCurve(vector<Point> * curve, float radius,bool cap=true);
  void drawControl();
  void drawLineCurve();
  void render();
  void subdivideCubicBSpline(vector<Point> * inputCurve, 
			     vector<Point> * outputCurve);
  void subdivideFourPoint(vector<Point> * inputCurve, 
			  vector<Point> * outputCurve);
  void subdivide(vector<Point> * inputCurve, 
		 vector<Point> * outputCurve);
  void computeLimitCurve();
  void computeMesh(MatrixXf& v);
  void createMesh(vector<Point> * curve, float radius,bool cap=true);
  void drawTriangle(Point v1,Point v2,Point v3);




};

