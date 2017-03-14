#ifndef MESHSEGMENT_H
#define MESHSEGMENT_H

#include <QJsonObject>
#include "trimesh/Vec.h"
#include "trimesh/TriMesh.h"
#include "Eigen/Core"
#include <Eigen/Geometry>
#include <utility>
#include <igl/writePLY.h>
#include <igl/writeOBJ.h>
#include <QMatrix3x3>
#include <QVector3D>
#include <map>

class Ellipse : public QObject
{
   Q_OBJECT
public:
   Ellipse()
   {
        bbRotation = Eigen::Matrix3f::Identity();
   }
   auto getRotation() -> QMatrix3x3 { return rotation;}
   auto getTranslation() -> QVector3D { return translation;}
   auto getRadius() -> float { return radius;}
   auto calculateRotation() -> void
   {
       Eigen::Vector3f new_vec(normal[0], normal[1], normal[2]);
       new_vec = bbRotation * new_vec;
       Eigen::Vector3f original(0.0f,1.0f,0.0f);
       original = bbRotation * original;
       Eigen::Quaternion<float> rot =
               Eigen::Quaternion<float>::FromTwoVectors(original, new_vec);

       Eigen::Matrix3f rotMat = rot.toRotationMatrix();
       std::cout << rotMat << std::endl;

       //rotMat = bbRotation * rotMat;
       QMatrix3x3 ellipseRotation(rotMat.data());
       rotation = ellipseRotation;
   }
   auto setTranslation(QVector3D t) -> void { translation = t; }
   auto setNormal(QVector3D new_vec_qt) -> void { normal = new_vec_qt; }
   auto getNormal() -> QVector3D { return normal; }
   auto setBBRotation(Eigen::Matrix3f r) -> void { bbRotation = r;}

public slots:
   void setRadius(double r) { radius = (float) r; }
   void setTranslationX(double x) { translation[0] = (float) x; }
   void setTranslationY(double y) { translation[1] = (float) y; }
   void setTranslationZ(double z) { translation[2] = (float) z; }
   void setNormalX(double x) { normal[0] = (float) x; calculateRotation(); }
   void setNormalY(double y) { normal[1] = (float) y; calculateRotation(); }
   void setNormalZ(double z) { normal[2] = (float) z; calculateRotation(); }

private:
   QMatrix3x3 rotation;
   QVector3D translation;
   QVector3D normal;
   float radius;
   Eigen::Matrix3f bbRotation;

};

class MeshSegment;

class PlanarRelation
{
public:
    PlanarRelation(MeshSegment* ms1, int aId1, int fId1, MeshSegment* ms2, int aId2, int fId2)
        : fromSegment(ms1)
        , fromFaceId(fId1)
        , fromAxisId(aId1)
        , toSegment(ms2)
        , toFaceId(fId2)
        , toAxisId(aId2)
    {

    }
    auto getFromSegment() -> MeshSegment* { return fromSegment; }
    auto getFromFaceId() -> int { return fromFaceId; }
    auto getFromAxisId() -> int { return fromAxisId; }
    auto getToSegment() -> MeshSegment* { return toSegment; }
    auto getToFaceId() -> int { return toFaceId; }
    auto getToAxisId() -> int { return toAxisId; }
    auto isMidPlane() -> bool { return (fromFaceId == 3 || toFaceId == 3); }
private:
    MeshSegment* fromSegment;
    int fromFaceId;
    int fromAxisId;
    MeshSegment* toSegment;
    int toFaceId;
    int toAxisId;
};


class MeshSegment
{
public:
    MeshSegment();
    MeshSegment(trimesh::Color c);
    ~MeshSegment();
    auto size() -> int;
    auto addPoint(trimesh::point p) -> void;
    auto axisAlignedMinMax(Eigen::Matrix3f eigTran, Eigen::Matrix3f eigTransInv, Eigen::Vector3f& min, Eigen::Vector3f& max) -> void;
    auto getColour() -> trimesh::Color;
    auto getCuboidArea() -> float;
    auto computeBoundingBox() -> void;
    auto computeBoundingBox(Eigen::Matrix3f eigTran, Eigen::Matrix3f eigTransInv) -> void;
    auto bBoxData() -> float*;
    auto getBBox() -> std::vector<trimesh::point>;
    auto setBoundingBox(std::vector<trimesh::point> newBBox, bool translateOnly, int id = -1) -> void;
    auto writeDataAndGetJSON(int i) -> QJsonObject;
    auto writeBox(int i) -> QString;
    auto writeMesh(QString fileName, std::vector<trimesh::point> new_bbox) -> void;
    auto addFaces(std::vector<trimesh::TriMesh::Face> f, int id = -1) -> void;
    auto setName(QString name) -> void;
    auto getName() -> QString;
    auto setRed() -> void;
    auto clearColour() -> void;
    auto select() -> void;
    auto deselect() -> void;
    auto isSelected() -> bool;
    auto setAxis(int i) -> void;
    auto getAxis() -> int;
    auto setFace(int i) -> void;
    auto getFace() -> int;
    auto setPlaneWidth(double width) -> void;
    auto setPlaneLength(double width) -> void;
    auto getPlaneWidthAndLength() -> std::pair<double, double>;
    auto updatePositions(int i, int j, double dist) -> void;
    auto makeFacePlane() -> void;
    auto setToOriginalBBox() -> void;
    auto getCentroid() -> trimesh::point;
    auto translateCentroid(trimesh::point new_centroid) -> void;
    auto getEllipses() -> std::vector<Ellipse*>;
    auto addEllipse(Ellipse* ellipse) -> void;
    auto getJsonBBox() -> QJsonArray;
    auto getJsonEllipses() -> QJsonArray;
    auto getIsPlane() -> bool;
    auto setIsPlane(bool val) -> bool;
    auto setIsTruncatedPyramid(bool val) -> void;
    auto getIsTruncatedPyramid() -> bool { return isTruncatedPyramid;}
    auto getPlaneFace() -> int { return planeFace; }
    auto getPlaneAxis() -> int { return planeAxis; }
    auto setPlaneFace(int val) -> void { planeFace = val; }
    auto setPlaneAxis(int val) -> void { planeAxis = val; }
    auto readJsonBBox(QJsonArray jsonBBox) -> void;
    auto readJsonEllipses(QJsonArray jsonEllipses) -> void;
    auto getInvRotation() -> Eigen::Matrix3f { return bbRotationInv; }
    auto getRotation() -> Eigen::Matrix3f { return bbRotation; }
    auto setInvRotation(Eigen::Matrix3f r) -> void { bbRotationInv = r; }
    auto setRotation(Eigen::Matrix3f r) -> void { bbRotation = r; }
    auto getFace(int axis_id, int face_id) -> Eigen::Matrix<float, 4, 3>;
    auto addPlanarRelation(int aId, int fId, PlanarRelation* relation) -> void;
    auto findCoPlanarRelations(MeshSegment* segment) -> std::vector<PlanarRelation*>;
    auto findCoPlanarRelationsNotIncluding(MeshSegment* segment) -> std::vector<PlanarRelation*>;
    auto getPrimitiveType() -> QString;
    auto hasNoRelations() -> bool { return (planarRelations.size() == 0); }

private:
    std::vector<trimesh::point> points;
    std::vector<trimesh::point> bBox;
    std::vector<trimesh::point> originalBBox;
    std::vector<trimesh::TriMesh::Face> faces;
    std::vector<Ellipse*> ellipses;
    std::map<int, std::vector<PlanarRelation*>> planarRelations;
    Eigen::MatrixXf iglVertices;
    trimesh::Color colour;
    float min_y;
    float max_y;
    Eigen::Vector3f minCoefs;
    Eigen::Vector3f maxCoefs;
    //Rotations for finding the originted bounding box
    Eigen::Matrix3f bbRotation;
    Eigen::Matrix3f bbRotationInv;
    QString segmentName;
    bool selected;
    int axis;
    int face;
    bool isPlane;
    bool isTruncatedPyramid;
    int planeAxis;
    int planeFace;
    auto initIglVertices() -> void;
    auto createFace(int a, int b, int c, int d) ->Eigen::Matrix<float, 4, 3>;


};

#endif // MESHSEGMENT_H
