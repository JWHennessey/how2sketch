#ifndef PRIMITIVEOPTIMISATION_H
#define PRIMITIVEOPTIMISATION_H

#include <QObject>
#include "mesh/mesh.h"
#include <QString>
#include "optimisation/utility_functions.h"


class Guide
{
public:
    Guide(Eigen::Vector3f t, Eigen::Vector3f f)
        : to(t)
        , from(f)
    {
        userDifficulty = 2;
    }
    auto getPoints() -> std::vector<trimesh::point>
    {
       std::vector<trimesh::point> points = {trimesh::point(to(0), to(1), to(2)), trimesh::point(from(0), from(1), from(2))};
       return points;
    }
    auto getUserDifficulty() -> int { return userDifficulty; }
    auto setUserDifficulty(int val) -> void { userDifficulty = val; }
    auto getToPoint() -> Eigen::Vector3f { return to;}
    auto getFromPoint() -> Eigen::Vector3f { return from;}
private:
    Eigen::Vector3f to;
    Eigen::Vector3f from;
    int userDifficulty;
};

class Step
{
public:
    Step(int t)
        : type(t)
    {

    }
    auto addGuide(Guide* g) -> void
    {
        guides.push_back(g);
    }
    auto getGuides() -> std::vector<Guide*>
    {
        return guides;
    }
    auto getPoints() -> std::vector<trimesh::point>
    {
        std::vector<trimesh::point> points;
        for(auto g : guides)
        {
            auto pts = g->getPoints();
            points.insert(std::begin(points), std::begin(pts), std::end(pts));
        }
        return points;
    }
    auto size() -> int
    {
        return guides.size();
    }
private:
    std::vector<Guide*> guides;
    int type;

};

class AxisPartialPrimitive
{
public:
    AxisPartialPrimitive(int a, float m1, float m2, int g1, int g2, Step* mins, Step* maxs, QString pp, int ppA, int ppF, float psa, QString cp, int cpA, int cpF, float me, float we, bool ug = false)
        : axis(a)
        , min(m1)
        , max(m2)
        , minGuide(g1)
        , maxGuide(g2)
        , minStep(mins)
        , maxStep(maxs)
        , parentPrimitive(pp)
        , parentAxisId(ppA)
        , parentFaceId(ppF)
        , projectedSurfaceArea(psa)
        , candidateType(cp)
        , guideAxisId(cpA)
        , guideFaceId(cpF)
        , midError(me)
        , widthError(we)
        , unguidedAxis(ug)
    {

    }
    AxisPartialPrimitive(int a, float m1, float m2, QString cp)
        : AxisPartialPrimitive(a, m1, m2, -1, -1, nullptr, nullptr, QString("Null"), -1, -1, 1.0, cp, -1, -1, 1.0, 1.0, true)
    {
        //
    }
    AxisPartialPrimitive(int a, float m1, float m2, QString cp, float error)
        : AxisPartialPrimitive(a, m1, m2, -1, -1, nullptr, nullptr, QString("Null"), -1, -1, 1.0, cp, -1, -1, error, error, true)
    {
        //
    }
    auto getAxis() -> int { return axis; }
    auto getMin() -> float { return min; }
    auto getMax() -> float { return max; }
    auto getMinGuide() -> int { return minGuide; }
    auto getMaxGuide() -> int { return maxGuide; }
    auto getParentPrimitiveName() -> QString { return parentPrimitive; }
    auto getParentPrimitiveFaceId() -> int { return parentFaceId; }
    auto getParentPrimitiveAxisId() -> int { return parentAxisId; }
    auto getCandidateTypeName() -> QString { return candidateType; }
    auto getGuidesFaceId() -> int { return guideFaceId;}
    auto getGuidesAxisId() -> int { return guideAxisId;}
    auto getWidthError() -> float { return widthError;}
    auto getMidError() -> float { return midError;}
    auto getGeometryError() -> float { return midError + widthError; }
    auto getGuideError() -> float {

        if(minStep == nullptr || maxStep == nullptr)
            return 0.0f;

        return (minStep->size() + maxStep->size()) / projectedSurfaceArea;

    }
    auto getTotalError() -> float { return getGeometryError() + getGuideError(); }
    auto isUnguided() -> bool { return unguidedAxis; }
    auto getGuidePoints() -> std::vector<trimesh::point>
    {

        if(minStep == nullptr || maxStep == nullptr)
        {
            auto ret = std::vector<trimesh::point>();
            return ret;
        }

        auto pts1 = minStep->getPoints();
        auto pts2 = maxStep->getPoints();
        pts1.insert(std::begin(pts1), std::begin(pts2), std::end(pts2));
        return pts1;
    }
    auto getPoints(Step* step) -> std::vector<trimesh::point>
    {

        if(step == nullptr)
        {
            auto ret = std::vector<trimesh::point>();
            return ret;
        }
        return step->getPoints();
    }
    auto getMinGuidePoints() -> std::vector<trimesh::point> { return getPoints(minStep); }
    auto getMaxGuidePoints() -> std::vector<trimesh::point> { return getPoints(maxStep); }
    auto getMinGuides() -> std::vector<Guide*>{ return minStep->getGuides(); }
    auto getMaxGuides() -> std::vector<Guide*>{ return maxStep->getGuides(); }
    auto merge(AxisPartialPrimitive* toMerge) -> void
    {


    }

private:
    int axis;
    float min;
    float max;
    int minGuide;
    int maxGuide;
    Step* minStep;
    Step* maxStep;
    QString parentPrimitive;
    int parentFaceId;
    int parentAxisId;
    float projectedSurfaceArea;
    QString candidateType;
    int guideFaceId;
    int guideAxisId;
    float midError;
    float widthError;
    bool unguidedAxis;


};

class PlanePartialPrimitive
{
 public:
    PlanePartialPrimitive(AxisPartialPrimitive* app1, AxisPartialPrimitive* app2, int pi, float pv, bool mid = false)
        : axisParialPrimitiveA(app1)
        , axisParialPrimitiveB(app2)
        , planeIndex(pi)
        , planeValue(pv)
        , midPlane(mid)
    {
        qDebug() << "planeIndex " << planeIndex << " value " << planeValue;
//        qDebug() << axisParialPrimitiveA->getCandidateTypeName() << " " << axisParialPrimitiveA->getMin() << " " << axisParialPrimitiveA->getMax();
//        qDebug() << axisParialPrimitiveB->getCandidateTypeName() << " " << axisParialPrimitiveB->getMin() << " " << axisParialPrimitiveB->getMax();
    }

    auto getPoints() -> std::vector<trimesh::point>
    {
        std::vector<trimesh::point> points;

        auto aIndex = axisParialPrimitiveA->getAxis();
        auto bIndex = axisParialPrimitiveB->getAxis();
        auto pt1 = trimesh::point(0,0,0);
        pt1[planeIndex] = planeValue;
        pt1[aIndex] = axisParialPrimitiveA->getMax();
        pt1[bIndex] = axisParialPrimitiveB->getMin();
        points.push_back(pt1);

        auto pt2 = trimesh::point(0,0,0);
        pt2[planeIndex] = planeValue;
        pt2[aIndex] = axisParialPrimitiveA->getMax();
        pt2[bIndex] = axisParialPrimitiveB->getMax();
        points.push_back(pt2);

        auto pt3 = trimesh::point(0,0,0);
        pt3[planeIndex] = planeValue;
        pt3[aIndex] = axisParialPrimitiveA->getMin();
        pt3[bIndex] = axisParialPrimitiveB->getMax();
        points.push_back(pt3);

        auto pt4 = trimesh::point(0,0,0);
        pt4[planeIndex] = planeValue;
        pt4[aIndex] = axisParialPrimitiveA->getMin();
        pt4[bIndex] = axisParialPrimitiveB->getMin();
        points.push_back(pt4);

        return points;
    }
    auto getAxisPrimitiveA() -> AxisPartialPrimitive* { return axisParialPrimitiveA; }
    auto getAxisPrimitiveB() -> AxisPartialPrimitive* { return axisParialPrimitiveB; }
    auto getPlaneIndex() -> int { return planeIndex; }
    auto getPlaneValue() -> float { return planeValue; }
    auto isMidPlane() -> bool { return midPlane; }
    auto getAxisValues(int axis) -> std::vector<float>
    {
        std::vector<float> axisValues;
        if(axisParialPrimitiveA->getAxis() == axis)
        {
            axisValues.push_back(axisParialPrimitiveA->getMin());
            axisValues.push_back(axisParialPrimitiveA->getMax());
        }
        else if(axisParialPrimitiveB->getAxis() == axis)
        {
            axisValues.push_back(axisParialPrimitiveB->getMin());
            axisValues.push_back(axisParialPrimitiveB->getMax());
        }
        return axisValues;
    }


private:
    AxisPartialPrimitive* axisParialPrimitiveA;
    AxisPartialPrimitive* axisParialPrimitiveB;
    int planeIndex;
    float planeValue;
    bool midPlane;
    std::vector<trimesh::point> customPoints;
};

class CandidatePrimitive
{
public:
    CandidatePrimitive(MeshSegment* ms, AxisPartialPrimitive* x, AxisPartialPrimitive* y, AxisPartialPrimitive* z)
        : meshSegment(ms)
        , axisParialPrimitiveX(x)
        , axisParialPrimitiveY(y)
        , axisParialPrimitiveZ(z)
    {
        //
    }
    auto setName(QString name) -> void
    {
        candidateName = name;
    }
    auto getName() -> QString
    {
        return candidateName;
    }
    auto getPrimitiveName() -> QString
    {
        return axisParialPrimitiveX->getCandidateTypeName();
    }
    auto getGeometryError() -> float
    {
        return axisParialPrimitiveX->getGeometryError() + axisParialPrimitiveY->getGeometryError() + axisParialPrimitiveZ->getGeometryError();
    }
    auto getGuideError() -> float
    {
        return axisParialPrimitiveX->getGuideError() + axisParialPrimitiveY->getGuideError() + axisParialPrimitiveZ->getGuideError();
    }
    auto getError() -> float
    {
        return axisParialPrimitiveX->getTotalError() + axisParialPrimitiveY->getTotalError() + axisParialPrimitiveZ->getTotalError();
    }
    auto isInputPrimitive() -> bool
    {
        return (6.0f == (axisParialPrimitiveX->getTotalError() + axisParialPrimitiveY->getTotalError() + axisParialPrimitiveZ->getTotalError()));
    }
    auto getParentPrimitives() -> std::set<QString>
    {
        std::set<QString> parentPrimitives;
        if(!axisParialPrimitiveX->isUnguided())
            parentPrimitives.insert(axisParialPrimitiveX->getParentPrimitiveName());

        if(!axisParialPrimitiveY->isUnguided())
            parentPrimitives.insert(axisParialPrimitiveY->getParentPrimitiveName());

        if(!axisParialPrimitiveZ->isUnguided())
            parentPrimitives.insert(axisParialPrimitiveZ->getParentPrimitiveName());

        return parentPrimitives;
    }
    auto getGuidePoints() -> std::vector<trimesh::point>
    {
        auto pts1 = axisParialPrimitiveX->getGuidePoints();
        auto pts2 = axisParialPrimitiveY->getGuidePoints();
        auto pts3 = axisParialPrimitiveZ->getGuidePoints();
        pts1.insert(std::begin(pts1), std::begin(pts2), std::end(pts2));
        pts1.insert(std::begin(pts1), std::begin(pts3), std::end(pts3));
        return pts1;
    }
    auto setPoints(std::vector<trimesh::point> points) -> void
    {
        customPoints = points;
    }
    auto getPoints() -> std::vector<trimesh::point>
    {
        if(customPoints.size() > 0)
            return customPoints;

        std::vector<trimesh::point> points;

        auto pt1 = trimesh::point(0,0,0);
        pt1[0] = axisParialPrimitiveX->getMin();
        pt1[1] = axisParialPrimitiveY->getMin();
        pt1[2] = axisParialPrimitiveZ->getMin();
        points.push_back(pt1);

        auto pt2 = trimesh::point(0,0,0);
        pt2[0] = axisParialPrimitiveX->getMin();
        pt2[1] = axisParialPrimitiveY->getMax();
        pt2[2] = axisParialPrimitiveZ->getMin();
        points.push_back(pt2);

        auto pt3 = trimesh::point(0,0,0);
        pt3[0] = axisParialPrimitiveX->getMin();
        pt3[1] = axisParialPrimitiveY->getMin();
        pt3[2] = axisParialPrimitiveZ->getMax();
        points.push_back(pt3);

        auto pt4 = trimesh::point(0,0,0);
        pt4[0] = axisParialPrimitiveX->getMin();
        pt4[1] = axisParialPrimitiveY->getMax();
        pt4[2] = axisParialPrimitiveZ->getMax();
        points.push_back(pt4);

        auto pt5 = trimesh::point(0,0,0);
        pt5[0] = axisParialPrimitiveX->getMax();
        pt5[1] = axisParialPrimitiveY->getMin();
        pt5[2] = axisParialPrimitiveZ->getMax();
        points.push_back(pt5);

        auto pt6 = trimesh::point(0,0,0);
        pt6[0] = axisParialPrimitiveX->getMax();
        pt6[1] = axisParialPrimitiveY->getMax();
        pt6[2] = axisParialPrimitiveZ->getMax();
        points.push_back(pt6);

        auto pt7 = trimesh::point(0,0,0);
        pt7[0] = axisParialPrimitiveX->getMax();
        pt7[1] = axisParialPrimitiveY->getMin();
        pt7[2] = axisParialPrimitiveZ->getMin();
        points.push_back(pt7);

        auto pt8 = trimesh::point(0,0,0);
        pt8[0] = axisParialPrimitiveX->getMax();
        pt8[1] = axisParialPrimitiveY->getMax();
        pt8[2] = axisParialPrimitiveZ->getMin();
        points.push_back(pt8);

        return points;
    }
    auto createFace( std::vector<trimesh::point> bBox, int a, int b, int c, int d) ->Eigen::Matrix<float, 4, 3>
    {
        Eigen::Matrix<float, 4, 3> face;
        auto pt = bBox[a];
        for(auto i = 0; i < 3; i++)
            face(0,i) = pt[i];

        pt = bBox[b];
        for(auto i = 0; i < 3; i++)
            face(1,i) = pt[i];

        pt = bBox[c];
        for(auto i = 0; i < 3; i++)
            face(2,i) = pt[i];

        pt = bBox[d];
        for(auto i = 0; i < 3; i++)
            face(3,i) = pt[i];


        return face;

    }
    auto getFace(int axis_id, int face_id) -> Eigen::Matrix<float, 4, 3>
    {
        auto points = getPoints();

        Eigen::Matrix<float, 4, 3> face;

        if(axis_id == 1) //X
        {
            if(face_id == 1)
            {
                //Back
                face = createFace(points,0,7,1,6);
            }
            if(face_id == 2)
            {
                //Front
                face = createFace(points, 2,3,4,5);
            }
            if(face_id == 3)
            {
                auto faceA = createFace(points, 0,7,1,6);
                auto faceB = createFace(points, 2,3,4,5);
                face = utility::findMidPlane(axis_id, faceA, faceB);
            }
        }
        else if(axis_id == 2) //Y
        {
            if(face_id == 1)
            {
                //Left Side
                face = createFace(points, 0,1,2,3);
            }
            if(face_id == 2)
            {
                //Right
                face = createFace(points, 4,5,6,7);
            }
            if(face_id == 3)
            {
                auto faceA = createFace(points, 0,1,2,3);
                auto faceB = createFace(points, 4,5,6,7);
                face = utility::findMidPlane(axis_id, faceA, faceB);
            }
        }
        else if(axis_id == 3) //Z
        {
            if(face_id == 1)
            {
                //Top
                face = createFace(points, 1,3,5,7);
            }
            if(face_id == 2)
            {
                //Bottom
                face = createFace(points, 0,2,4,6);
            }
            if(face_id == 3)
            {
                auto faceA = createFace(points, 1,3,5,7);
                auto faceB = createFace(points, 0,2,4,6);
                face = utility::findMidPlane(axis_id, faceA, faceB);
            }
        }
        return face;
    }
    auto getXAxis() -> AxisPartialPrimitive* { return axisParialPrimitiveX;}
    auto getYAxis() -> AxisPartialPrimitive* { return axisParialPrimitiveY;}
    auto getZAxis() -> AxisPartialPrimitive* { return axisParialPrimitiveZ;}
    auto writeBox(QString name) -> void
    {
        auto bb = getPoints();
        if(meshSegment->getIsPlane())
        {
            Eigen::MatrixXf iglAllPoints(8, 3);
            Eigen::MatrixXi iglFaces(2, 3);
            iglFaces << 0,1,3,
                        0,3,2;

            for(auto j = 0; j < 8; j++)
            {
                auto p = bb[j];
                iglAllPoints.row(j) = Eigen::Vector3f(p[1], p[2], p[0]);
                //iglAllPoints.row(j) = Eigen::Vector3f(p[2], p[0], p[1]);
            }

            Eigen::Vector3f maxCoefs = iglAllPoints.colwise().maxCoeff();
            Eigen::Vector3f minCoefs = iglAllPoints.colwise().minCoeff();
            Eigen::MatrixXf iglVert(4, 3);
            iglVert.row(0) = maxCoefs;
            iglVert.row(2) = minCoefs;

            auto planeIndex = 0;
            for(auto i = 0; i < 3; i++)
            {
                if(maxCoefs(i) == minCoefs(i))
                {
                    planeIndex = i;
                    break;
                }
            }
            Eigen::Vector3f point2(maxCoefs);
            Eigen::Vector3f point3(maxCoefs);
            for(auto i = 0; i < 3; i++)
            {
                if(i != planeIndex)
                {
                    point2(i) = minCoefs(i);
                    break;
                }
            }
            for(auto i = 2; i >= 0; i--)
            {
                if(i != planeIndex)
                {
                    point3(i) = minCoefs(i);
                    break;
                }
            }

            iglVert.row(1) = point2;
            iglVert.row(3) = point3;



            igl::writePLY(name.toUtf8().constData(), iglVert, iglFaces);

        }
        else
        {

            Eigen::MatrixXf iglVert(8, 3);
            Eigen::MatrixXi iglFaces(12, 3);
            iglFaces << 0,7,1,
                    0,6,7,
                    0,2,1,
                    2,1,3,
                    0,6,2,
                    6,4,2,
                    2,4,3,
                    4,5,3,
                    7,5,3,
                    7,3,1,
                    6,7,5,
                    4,6,5;

            for(auto j = 0; j < 8; j++)
            {
                auto p = bb[j];
                iglVert.row(j) = Eigen::Vector3f(p[1], p[2], p[0]);
                //iglVert.row(j) = Eigen::Vector3f(p[2], p[0], p[1]);
            }

            igl::writePLY(name.toUtf8().constData(), iglVert, iglFaces);
        }
    }
    auto writeStartPlane(QString name, bool isMidPlane = false, int planeIndex = 0) -> void
    {
        auto bb = getPoints();
//        if(meshSegment->getIsPlane())
//        {
            Eigen::MatrixXf iglAllPoints(8, 3);
            Eigen::MatrixXi iglFaces(2, 3);
            iglFaces << 0,1,3,
                        0,3,2;

            for(auto j = 0; j < 8; j++)
            {
                auto p = bb[j];
                iglAllPoints.row(j) = Eigen::Vector3f(p[1], p[2], p[0]);
                //iglAllPoints.row(j) = Eigen::Vector3f(p[0], p[1], p[2]);
            }


            Eigen::Vector3f maxCoefs = iglAllPoints.colwise().maxCoeff();
            Eigen::Vector3f minCoefs = iglAllPoints.colwise().minCoeff();


            //Train plane index = 0
            //auto planeIndex = 0;

            if(isMidPlane)
            {
                auto planeValue =  (maxCoefs(planeIndex) + minCoefs(planeIndex)) * 0.5;
                maxCoefs(planeIndex) = planeValue;
                minCoefs(planeIndex) = planeValue;
            }


            maxCoefs(planeIndex) = minCoefs(planeIndex);

            Eigen::MatrixXf iglVert(4, 3);
            iglVert.row(0) = maxCoefs;
            iglVert.row(3) = minCoefs;

            Eigen::Vector3f point2(maxCoefs);
            Eigen::Vector3f point3(maxCoefs);
            for(auto i = 0; i < 3; i++)
            {
                if(i != planeIndex)
                {
//                    if(isMidPlane)
//                    {
//                        point2(i) = (minCoefs(i) + maxCoefs(i)) * 0.5f;
//                    }
//                    else
//                    {
                        point2(i) = minCoefs(i);
                   // }
                    break;
                }
            }
            for(auto i = 2; i >= 0; i--)
            {
                if(i != planeIndex)
                {
//                    if(isMidPlane)
//                    {
//                        point3(i) = (minCoefs(i) + maxCoefs(i)) * 0.5f;
//                    }
//                    else
//                    {
                        point3(i) = minCoefs(i);
                   // }
                    break;
                }
            }

            iglVert.row(1) = point2;
            iglVert.row(2) = point3;
            Eigen::Vector3f temp = iglVert.row(2);
            iglVert.row(2)  = iglVert.row(3);
            iglVert.row(3) = temp;

            auto plyName = QString(name).append(QString(".ply"));
            auto objName = QString(name).append(QString(".obj"));
            igl::writePLY(plyName.toUtf8().constData(), iglVert, iglFaces);
            igl::writeOBJ(objName.toUtf8().constData(), iglVert, iglFaces);

        //}
   }
    auto makeArrow(float x1, float y1, float z1, float x2, float y2, float z2) -> QJsonObject
    {
        QJsonObject from1;
        QJsonObject to1;
        from1["x"] = x1;
        from1["y"] = y1;
        from1["z"] = z1;

        to1["x"] = x2;
        to1["y"] = y2;
        to1["z"] = z2;

        QJsonObject arrow1;
        arrow1["from"] = from1;
        arrow1["to"] = to1;

        return arrow1;
    }

    auto getStartArrows(bool isMidPlane = false, int planeIndex = 0) -> QJsonArray
    {
        auto bb = getPoints();

        Eigen::MatrixXf iglAllPoints(8, 3);

        for(auto j = 0; j < 8; j++)
        {
            auto p = bb[j];
            iglAllPoints.row(j) = Eigen::Vector3f(p[1], p[2], p[0]);
        }

        Eigen::Vector3f maxCoefs = iglAllPoints.colwise().maxCoeff();
        Eigen::Vector3f minCoefs = iglAllPoints.colwise().minCoeff();

        if(isMidPlane)
        {
            //auto planeIndex = 0;
            auto planeValue =  (maxCoefs(planeIndex) + minCoefs(planeIndex)) * 0.5;
            minCoefs(planeIndex) = planeValue;
        }

        QJsonArray arrows;

        if(planeIndex == 0)
        {

            auto arrow1 = makeArrow(minCoefs(0), minCoefs(1), minCoefs(2), maxCoefs(0), minCoefs(1), minCoefs(2));
            arrows.push_back(arrow1);
            auto arrow2 = makeArrow(minCoefs(0), minCoefs(1), maxCoefs(2), maxCoefs(0), minCoefs(1), maxCoefs(2));
            arrows.push_back(arrow2);
            auto arrow3 = makeArrow(minCoefs(0), maxCoefs(1), maxCoefs(2), maxCoefs(0), maxCoefs(1), maxCoefs(2));
            arrows.push_back(arrow3);
            auto arrow4 = makeArrow(minCoefs(0), maxCoefs(1), minCoefs(2), maxCoefs(0), maxCoefs(1), minCoefs(2));
            arrows.push_back(arrow4);
        }
        else if(planeIndex == 2)
        {
            auto arrow1 = makeArrow(minCoefs(0), minCoefs(1), minCoefs(2), minCoefs(0), minCoefs(1), maxCoefs(2));
            arrows.push_back(arrow1);
            auto arrow2 = makeArrow(maxCoefs(0), minCoefs(1), minCoefs(2), maxCoefs(0), minCoefs(1), maxCoefs(2));
            arrows.push_back(arrow2);
            auto arrow3 = makeArrow(maxCoefs(0), maxCoefs(1), minCoefs(2), maxCoefs(0), maxCoefs(1), maxCoefs(2));
            arrows.push_back(arrow3);
            auto arrow4 = makeArrow(minCoefs(0), maxCoefs(1), minCoefs(2), minCoefs(0), maxCoefs(1), maxCoefs(2));
            arrows.push_back(arrow4);
        }


        if(isMidPlane)
        {
            minCoefs = iglAllPoints.colwise().maxCoeff();
            maxCoefs = iglAllPoints.colwise().minCoeff();

            //auto planeIndex = 0;
            auto planeValue =  (maxCoefs(planeIndex) + minCoefs(planeIndex)) * 0.5;
            minCoefs(planeIndex) = planeValue;

            auto arrow1 = makeArrow(minCoefs(0), minCoefs(1), minCoefs(2), maxCoefs(0), minCoefs(1), minCoefs(2));
            arrows.push_back(arrow1);
            auto arrow2 = makeArrow(minCoefs(0), minCoefs(1), maxCoefs(2), maxCoefs(0), minCoefs(1), maxCoefs(2));
            arrows.push_back(arrow2);
            auto arrow3 = makeArrow(minCoefs(0), maxCoefs(1), maxCoefs(2), maxCoefs(0), maxCoefs(1), maxCoefs(2));
            arrows.push_back(arrow3);
            auto arrow4 = makeArrow(minCoefs(0), maxCoefs(1), minCoefs(2), maxCoefs(0), maxCoefs(1), minCoefs(2));
            arrows.push_back(arrow4);
        }

        return arrows;

    }
    auto getArrows(int axis, int face1, int face2) -> QJsonArray
    {

        auto from_face = getFace(axis, face1);
        auto to_face_wrong_order = getFace(axis, face2);
        auto minCoeff = from_face.colwise().minCoeff();
        auto maxCoeff = from_face.colwise().maxCoeff();
        auto planeIndex = -1;
        for(auto i = 0; i < 3; i++)
        {
            if(minCoeff(i) == maxCoeff(i))
            {
                planeIndex = i;
                break;
            }
        }

        auto correct_height = to_face_wrong_order(0, planeIndex);
        auto to_face = from_face;
        to_face.col(planeIndex) = Eigen::Vector4f(correct_height,correct_height,correct_height,correct_height);

        std::cout << "minCoeff  " << minCoeff << std::endl;
        std::cout << "maxCoeff  " << maxCoeff << std::endl;
        std::cout << "from_face  " << from_face << std::endl;
        std::cout << "to_face_wrong_order  " << to_face_wrong_order << std::endl;
        std::cout << "correct_height  " << correct_height << std::endl;
        std::cout << "to_face  " << to_face << std::endl;

        QJsonArray arrows;
        for(auto i = 0; i < 4; i ++)
        {
            auto arrow = makeArrow(from_face(i,1), from_face(i,2), from_face(i,0), to_face(i,1), to_face(i,2), to_face(i,0));
            arrows.push_back(arrow);
        }

        return arrows;

    }
    auto getMeshSegment() -> MeshSegment*
    {
        return meshSegment;
    }
    auto findCoPlanarRelations(MeshSegment* segment) -> std::vector<PlanarRelation*>
    {
        std::vector<PlanarRelation*> newRelations;
        auto relations = meshSegment->findCoPlanarRelations(segment);

        for(auto relation : relations)
        {
            auto newRelation = new PlanarRelation(newMeshSegment, relation->getFromAxisId(), relation->getFromFaceId(), relation->getToSegment(), relation->getToAxisId(), relation->getToFaceId());
            newRelations.push_back(newRelation);
        }
        return newRelations;
    }
    auto createNewMeshSegment() -> void
    {
        newMeshSegment = new MeshSegment();
        auto bbox = getPoints();
        newMeshSegment->setBoundingBox(bbox, false);
        newMeshSegment->setName(candidateName);
    }
    auto getNewMeshSegment() -> MeshSegment*
    {
        return newMeshSegment;
    }
private:
    MeshSegment* meshSegment;
    MeshSegment* newMeshSegment;
    AxisPartialPrimitive* axisParialPrimitiveX;
    AxisPartialPrimitive* axisParialPrimitiveY;
    AxisPartialPrimitive* axisParialPrimitiveZ;
    QString candidateName;
    std::vector<trimesh::point> customPoints;
};


class PrimitiveOptimisation : public QObject
{
    Q_OBJECT
public:
    PrimitiveOptimisation(Mesh* m);
    auto generateCandidates(QMatrix4x4 viewport, QMatrix4x4 mvp) -> void;
    auto runOptimisation() -> void;
    auto getPartialPrimitives()   -> std::map<QString, std::vector<PlanePartialPrimitive*> > { return partialPrimitivesMap; }
    auto getCandidatePrimitives() -> std::map<QString, std::vector<CandidatePrimitive*> > { return candidatePrimitivesMap; }
    auto getFinalPrimitives() -> std::map<QString, CandidatePrimitive* > { return finalPrimitivesMap; }

private:
    Mesh* mesh;
    std::map<QString, std::vector<PlanePartialPrimitive*>> partialPrimitivesMap;
    std::map<QString, std::vector<CandidatePrimitive*>> candidatePrimitivesMap;
    std::map<QString, CandidatePrimitive*> finalPrimitivesMap;
    std::map<QString, std::vector<AxisPartialPrimitive*>> unguidedAxisMap;
    auto findOrdering() -> void;
};

#endif // PRIMITIVEOPTIMISATION_H
