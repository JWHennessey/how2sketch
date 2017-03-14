#include "meshsegment.h"
#include "Eigen/Core"
#include <Eigen/LU>
#include <QDebug>
#include <QJsonArray>
#include <QJsonObject>
#include "optimisation/utility_functions.h"

MeshSegment::MeshSegment()
    : MeshSegment(trimesh::Color(255,255,255))
{
    bbRotation = Eigen::Matrix3f::Identity();
    bbRotationInv = bbRotation.inverse();
}

MeshSegment::MeshSegment(trimesh::Color c)
    : colour(c)
    , selected(false)
    , axis(-1)
    , face(-1)
    , isPlane(false)
    , isTruncatedPyramid(false)
    , planeAxis(-1)
    , planeFace(-1)
{
    bbRotation = Eigen::Matrix3f::Identity();
    bbRotationInv = bbRotation.inverse();
}

MeshSegment::~MeshSegment()
{

}

auto MeshSegment::setName(QString name) -> void
{
    segmentName = name;
}

auto MeshSegment::getName() -> QString
{
    return segmentName;
}

auto MeshSegment::select() -> void
{
    selected = true;
}

auto MeshSegment::deselect() -> void
{
    selected = false;
}

auto MeshSegment::isSelected() -> bool
{
    return selected;
}

auto MeshSegment::setAxis(int i) -> void
{
    axis = i;
}
auto MeshSegment::getAxis() -> int
{
    return axis;
}

auto MeshSegment::setFace(int i) -> void
{
    face = i;
}

auto MeshSegment::getFace() -> int
{
    return face;
}

auto MeshSegment::getJsonBBox() -> QJsonArray
{
    QJsonArray jsonPoints;
    for(auto point : bBox)
    {
        QJsonArray jsonPoint;
        for(auto i = 0; i < 3; i ++)
            jsonPoint.push_back(point[i]);

        jsonPoints.push_back(jsonPoint);
    }
    return jsonPoints;
}

auto MeshSegment::getJsonEllipses() -> QJsonArray
{
    QJsonArray jsonEllipses;
    for(auto ellipse : ellipses)
    {
        QJsonObject jsonEllipse;
        QJsonArray jsonTranslation;
        auto translation = ellipse->getTranslation();
        for(auto i = 0; i < 3; i++)
            jsonTranslation.push_back(translation[i]);
        jsonEllipse["translation"] = jsonTranslation;

        QJsonArray jsonNormal;
        auto normal = ellipse->getNormal();
        for(auto i = 0; i < 3; i++)
            jsonNormal.push_back(normal[i]);
        jsonEllipse["normal"] = jsonNormal;

        jsonEllipse["diameter"] = ellipse->getRadius();

        jsonEllipses.push_back(jsonEllipse);
    }

    return jsonEllipses;
}


auto MeshSegment::readJsonBBox(QJsonArray jsonBBox) -> void
{
    for(auto i = 0; i < 8; i++)
    {
        auto pointArray = jsonBBox[i].toArray();
//        bBox[i][0] = pointArray[2].toDouble();
//        bBox[i][1] = pointArray[0].toDouble();
//        bBox[i][2] = pointArray[1].toDouble();

        for(auto j = 0; j < 3; j++)
            bBox[i][j] = pointArray[j].toDouble();
    }
}

auto MeshSegment::readJsonEllipses(QJsonArray jsonEllipses) -> void
{
    for(auto jsonEllipseVal : jsonEllipses)
    {
        auto jsonEllipse = jsonEllipseVal.toObject();

        auto ellipse = new Ellipse();

        auto it = jsonEllipse.find(QString("diameter"));
        if(it != jsonEllipse.end())
            ellipse->setRadius(jsonEllipse["diameter"].toDouble());
        else
            ellipse->setRadius(jsonEllipse["radius"].toDouble());

        auto jsonTranslation = jsonEllipse["translation"].toArray();
        ellipse->setTranslation(QVector3D(jsonTranslation[0].toDouble(), jsonTranslation[1].toDouble(), jsonTranslation[2].toDouble()));

        auto jsonNormal = jsonEllipse["normal"].toArray();
        ellipse->setNormal(QVector3D(jsonNormal[0].toDouble(), jsonNormal[1].toDouble(), jsonNormal[2].toDouble()));

        //std::cout << "bbRotation " << bbRotation << std::endl;

        ellipse->setBBRotation(bbRotationInv);

        ellipse->calculateRotation();

        addEllipse(ellipse);
    }
}

auto MeshSegment::setIsPlane(bool val) -> bool
{
    isPlane = val;
    if(isPlane)
    {
        planeAxis = axis;
        planeFace = face;
    }
}

auto MeshSegment::getIsPlane() -> bool
{
    return isPlane;
}


auto axisFaceToIndex(int aId, int fId) -> int
{
    return ((aId - 1) * 2) + (fId - 1);
}

auto MeshSegment::addPlanarRelation(int aId, int fId, PlanarRelation* relation) -> void
{
    auto index = axisFaceToIndex(aId, fId);
    auto findIt = planarRelations.find(index);
    if(findIt != planarRelations.end()) //Already exisits
    {
        findIt->second.push_back(relation);
    }
    else //Create new vector
    {
        auto relationVector = std::vector<PlanarRelation*>();
        relationVector.push_back(relation);
        planarRelations.insert(std::make_pair(index, relationVector));
    }

}

auto MeshSegment::findCoPlanarRelations(MeshSegment* segment) -> std::vector<PlanarRelation*>
{
    //qDebug() << segmentName << " " << planarRelations.size();
    auto relations = std::vector<PlanarRelation*>();
    for(auto it : planarRelations)
    {
        for(auto r : it.second)
        {
            auto s = r->getToSegment();
            //qDebug() << s->getName() << " " << segment->getName();
            if(s->getName() == segment->getName())
            {
                relations.push_back(r);
            }
        }
    }
    return relations;
}

auto MeshSegment::findCoPlanarRelationsNotIncluding(MeshSegment* segment) -> std::vector<PlanarRelation*>
{
    auto relations = std::vector<PlanarRelation*>();
    for(auto it : planarRelations)
    {
        for(auto r : it.second)
        {
            auto s = r->getToSegment();
            //qDebug() << s->getName() << " " << segment->getName();
            if(s->getName() != segment->getName())
            {
                relations.push_back(r);
            }
        }
    }
    return relations;
}

auto MeshSegment::updatePositions(int i, int j, double dist) -> void
{
//    qDebug() << "i " << bBox[i][0] << " " << bBox[i][1] << " " << bBox[i][2];
//    qDebug() << "j " << bBox[j][0] << " " << bBox[j][1] << " " << bBox[j][2];
    auto vec = bBox[i] - bBox[j];
    auto len = (float) trimesh::len(vec);
    auto mid = (bBox[i] + bBox[j]) * 0.5f;
    auto ratio = (float) (dist / len);
//    qDebug() << "len " << len << " dist " << dist << " ratio " << ratio;
    //normalize(vec);
    bBox[i] = mid + (vec * ratio * 0.5f);
    bBox[j] = mid - (vec * ratio * 0.5f);
//    qDebug() << "i " << bBox[i][0] << " " << bBox[i][1] << " " << bBox[i][2];
//    qDebug() << "j " << bBox[j][0] << " " << bBox[j][1] << " " << bBox[j][2];
}

auto MeshSegment::setPlaneWidth(double width) -> void
{
    qDebug() << " MeshSegment::setPlaneWidth(double " << width << ")";
    if(getAxis() > -1)
    {
        if(getAxis() == 1) //X
        {
            if(getFace() == -1 || getFace() == 1)
            {
                //Back
                updatePositions(1,7,width);
                updatePositions(0,6,width);
            }
            if(getFace() == -1 || getFace() == 2)
            {
                //Front
                //auto custom_start_index = (start_index + (6 * 3));
                updatePositions(2,4,width);
                updatePositions(3,5,width);
            }
        }
        else if(getAxis() == 2) //Y
        {
            if(getFace() == -1 || getFace() == 1)
            {
                //Left Side
                //auto custom_start_index = (start_index + (2 * 3));
                updatePositions(0, 2, width);
                updatePositions(1, 3, width);
            }
            if(getFace() == -1 || getFace() == 2)
            {
                //Right
                //auto custom_start_index = (start_index + (10 * 3));
                updatePositions(7, 5, width);
                updatePositions(6, 4, width);
            }
        }
        else if(getAxis() == 3) //Z
        {

            if(getFace() == -1 || getFace() == 1)
            {
                //Top
                //auto custom_start_index = (start_index + (8 * 3));
                updatePositions(5, 3, width);
                updatePositions(7, 1, width);
            }
            if(getFace() == -1 || getFace() == 2)
            {
                //Bottom
                //auto custom_start_index = (start_index + (4 * 3));
                updatePositions(0, 6, width);
                updatePositions(4, 2, width);
            }
        }
    }

}

auto MeshSegment::setPlaneLength(double length) -> void
{
    qDebug() << " MeshSegment::setPlaneLength(double " << length << ")";
    if(getAxis() > -1)
    {
        if(getAxis() == 1) //X
        {
            if(getFace() == -1 || getFace() == 1)
            {
                //Back
                updatePositions(1,0,length);
                updatePositions(7,6,length);
            }
            if(getFace() == -1 || getFace() == 2)
            {
                //Front
                //auto custom_start_index = (start_index + (6 * 3));
                updatePositions(2,3,length);
                updatePositions(4,5,length);
            }
        }
        else if(getAxis() == 2) //Y
        {
            if(getFace() == -1 || getFace() == 1)
            {
                //Left Side
                //auto custom_start_index = (start_index + (2 * 3));
                updatePositions(0, 1, length);
                updatePositions(2, 3, length);
            }
            if(getFace() == -1 || getFace() == 2)
            {
                //Right
                //auto custom_start_index = (start_index + (10 * 3));
                updatePositions(7, 6, length);
                updatePositions(5, 4, length);
            }
        }
        else if(getAxis() == 3) //Z
        {

            if(getFace() == -1 || getFace() == 1)
            {
                //Top
                //auto custom_start_index = (start_index + (8 * 3));
                updatePositions(5, 7, length);
                updatePositions(3, 1, length);
            }
            if(getFace() == -1 || getFace() == 2)
            {
                //Bottom
                //auto custom_start_index = (start_index + (4 * 3));
                updatePositions(0, 4, length);
                updatePositions(6, 2, length);
            }
        }
    }
}


auto MeshSegment::createFace(int a, int b, int c, int d) ->Eigen::Matrix<float, 4, 3>
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

//unsigned short bbIndicesTemplate[] = {0,7,1,
//                                      0,6,7,
//                                      0,2,1,
//                                      2,1,3,
//                                      0,6,2,
//                                      6,4,2,
//                                      2,4,3,
//                                      4,5,3,
//                                      7,5,3,
//                                      7,3,1,
//                                      6,7,5,
//                                      4,6,5};

auto MeshSegment::getFace(int axis_id, int face_id) -> Eigen::Matrix<float, 4, 3>
{
    //qDebug() << "MeshSegment::getFace";
    Eigen::Matrix<float, 4, 3> face;

    if(isPlane)
    {
        axis_id = planeAxis;
        face_id = planeFace;
    }

    if(axis_id == 1) //X
    {
        if(face_id == 1)
        {
            //Back
            face = createFace(0,7,1,6);
        }
        if(face_id == 2)
        {
            //Front
            face = createFace(2,3,4,5);
        }
        if(face_id == 3)
        {
            auto faceA = createFace(0,7,1,6);
            auto faceB = createFace(2,3,4,5);
            face = utility::findMidPlane(axis_id, faceA, faceB);
        }
        //
    }
    else if(axis_id == 2) //Y
    {
        if(face_id == 1)
        {
            //Left Side
            face = createFace(0,1,2,3);
        }
        if(face_id == 2)
        {
            //Right
            face = createFace(4,5,6,7);
        }
        if(face_id == 3)
        {
            auto faceA = createFace(0,1,2,3);
            auto faceB = createFace(4,5,6,7);
            face = utility::findMidPlane(axis_id, faceA, faceB);
        }
    }
    else if(axis_id == 3) //Z
    {

        if(face_id == 1)
        {
            //Top
            face = createFace(1,3,5,7);
        }
        if(face_id == 2)
        {
            //Bottom
            face = createFace(0,2,4,6);
        }
        if(face_id == 3)
        {
            auto faceA = createFace(1,3,5,7);
            auto faceB = createFace(0,2,4,6);
            face = utility::findMidPlane(axis_id, faceA, faceB);
        }
    }
    return face;
}


auto MeshSegment::getPlaneWidthAndLength() -> std::pair<double, double>
{
    if(getAxis() > -1)
    {
        if(getAxis() == 1) //X
        {
            if(getFace() == -1 || getFace() == 1)
            {
                //Back
                auto vec = bBox[0] - bBox[1];
                auto length = trimesh::len(vec);
                vec = bBox[1] - bBox[7];
                auto width = trimesh::len(vec);
                return std::make_pair((double) width, (double) length);
            }
            if(getFace() == -1 || getFace() == 2)
            {
                //Front
                //auto custom_start_index = (start_index + (6 * 3));
                auto vec = bBox[2] - bBox[3];
                auto length = trimesh::len(vec);
                vec = bBox[2] - bBox[4];
                auto width = trimesh::len(vec);
                return std::make_pair((double) width, (double) length);
            }
        }
        else if(getAxis() == 2) //Y
        {

            if(getFace() == -1 || getFace() == 1)
            {
                //Left Side
                //auto custom_start_index = (start_index + (2 * 3));
                auto vec = bBox[0] - bBox[1];
                auto length = trimesh::len(vec);
                vec = bBox[0] - bBox[2];
                auto width = trimesh::len(vec);
                return std::make_pair((double) width, (double) length);
            }
            if(getFace() == -1 || getFace() == 2)
            {
                //Right
                //auto custom_start_index = (start_index + (10 * 3));
                auto vec = bBox[7] - bBox[6];
                auto length = trimesh::len(vec);
                vec = bBox[7] - bBox[5];
                auto width = trimesh::len(vec);
                return std::make_pair((double) width, (double) length);
            }
        }
        else if(getAxis() == 3) //Z
        {

            if(getFace() == -1 || getFace() == 1)
            {
                //Top
                //auto custom_start_index = (start_index + (8 * 3));
                auto vec = bBox[5] - bBox[7];
                auto length = trimesh::len(vec);
                vec = bBox[5] - bBox[3];
                auto width = trimesh::len(vec);
                return std::make_pair((double) width, (double) length);
            }
            if(getFace() == -1 || getFace() == 2)
            {
                //Bottom
                //auto custom_start_index = (start_index + (4 * 3));
                auto vec = bBox[0] - bBox[2];
                auto length = trimesh::len(vec);
                vec = bBox[0] - bBox[6];
                auto width = trimesh::len(vec);
                return std::make_pair((double) width, (double) length);
            }
        }
    }
    else
    {
        return std::make_pair(-1.0, -1.0);
    }
}


unsigned short bbIndicesTemplate[] = {0,7,1,
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
                                      4,6,5};

auto MeshSegment::makeFacePlane() -> void
{
    qDebug() << " MeshSegment::makeFacePlane() " << getAxis() << " " << getFace();
    if(getAxis() > -1 && getFace() > -1 )
    {
        isPlane = true;
        planeAxis = getAxis();
        planeFace = getFace();
        if(getAxis() == 1) //X
        {
            if(getFace() == 1)
            {
                //Back
//                updatePositions(1,0,length);
//                updatePositions(7,6,length);
                bBox[2] = bBox[0];
                bBox[3] = bBox[1];
                bBox[4] = bBox[6];
                bBox[5] = bBox[7];
            }
            if(getFace() == 2)
            {
                //Front
                //auto custom_start_index = (start_index + (6 * 3));
//                updatePositions(2,3,length);
//                updatePositions(4,5,length);
                bBox[0] = bBox[2];
                bBox[1] = bBox[3];
                bBox[6] = bBox[4];
                bBox[7] = bBox[5];
            }
        }
        else if(getAxis() == 2) //Y
        {
            if(getFace() == 1)
            {
                //Left Side
                //auto custom_start_index = (start_index + (2 * 3));
//                updatePositions(0, 1, length);
//                updatePositions(2, 3, length);
                bBox[7] = bBox[1];
                bBox[6] = bBox[0];
                bBox[5] = bBox[3];
                bBox[4] = bBox[2];

            }
            if(getFace() == 2)
            {
                //Right
                //auto custom_start_index = (start_index + (10 * 3));
//                updatePositions(7, 6, length);
//                updatePositions(5, 4, length);
                bBox[1] = bBox[7];
                bBox[0] = bBox[6];
                bBox[3] = bBox[5];
                bBox[2] = bBox[4];
            }
        }
        else if(getAxis() == 3) //Z
        {
            if(getFace() == 1)
            {
                //Top
                //auto custom_start_index = (start_index + (8 * 3));
//                updatePositions(5, 7, length);
//                updatePositions(3, 1, length);
                bBox[0] = bBox[1];
                bBox[4] = bBox[5];
                bBox[6] = bBox[7];
                bBox[2] = bBox[3];
            }
            if(getFace() == 2)
            {
                //Bottom
                //auto custom_start_index = (start_index + (4 * 3));
//                updatePositions(0, 4, length);
//                updatePositions(6, 2, length);
                bBox[1] = bBox[0];
                bBox[5] = bBox[4];
                bBox[7] = bBox[6];
                bBox[3] = bBox[2];
            }
        }
    }
}


auto MeshSegment::getEllipses() -> std::vector<Ellipse*>
{
    return ellipses;
}

auto MeshSegment::addEllipse(Ellipse* ellipse) -> void
{
    ellipses.push_back(ellipse);
}

auto MeshSegment::getCentroid() -> trimesh::point
{
    auto centroid = trimesh::point(0,0,0);
    for(auto p : bBox)
    {
        centroid += p;
    }
    centroid = centroid / 8.0f;
    Eigen::Vector3f eigenCentroid(centroid[0], centroid[1], centroid[2]);
    eigenCentroid = bbRotationInv * eigenCentroid;
//    qDebug() << "Before " << centroid[0] << " " << centroid[1] << " " << centroid[2];
    for(auto i = 0; i < 3; i++)
        centroid[i] = eigenCentroid[i];

//    qDebug() << "After " << centroid[0] << " " << centroid[1] << " " << centroid[2];
    return centroid;
}

auto MeshSegment::translateCentroid(trimesh::point new_centroid) -> void
{
//    qDebug() << "translateCentroid " << new_centroid[0] << " " << new_centroid[1] << " " << new_centroid[2];
    auto centroid = getCentroid();
    Eigen::Vector3f eigenCentroid(centroid[0], centroid[1], centroid[2]);
    eigenCentroid = bbRotation * eigenCentroid;

    Eigen::Vector3f eigenNewCentroid(new_centroid[0], new_centroid[1], new_centroid[2]);
    eigenNewCentroid = bbRotation * eigenNewCentroid;

    for(auto i = 0; i < 3; i++)
    {
        new_centroid[i] = eigenNewCentroid[i];
        centroid[i] = eigenCentroid[i];
    }

    auto delta = new_centroid - centroid;
    for(auto& p : bBox)
    {
        p += delta;
    }

//    auto newCentroid = getCentroid();
//    qDebug() << "newCentroid " << newCentroid[0] << " " << newCentroid[1] << " " << newCentroid[2];

//    if(newCentroid != new_centroid)
//        qDebug() << "Translate error!!!";
}

auto MeshSegment::setToOriginalBBox() -> void
{
    isPlane = false;
    planeAxis = -1;
    planeFace = -1;
    bBox = originalBBox;
}


auto MeshSegment::setRed() -> void
{
    colour = trimesh::Color(255,0,0);
}

auto MeshSegment::clearColour() -> void
{
    colour = trimesh::Color(255,255,255);
}

auto MeshSegment::size() -> int
{
    return points.size();
}

auto MeshSegment::addPoint(trimesh::point p) -> void
{
    points.push_back(p);
    min_y = std::min(min_y, p[1]);
    max_y = std::max(max_y, p[1]);
}

auto MeshSegment::initIglVertices() -> void
{
    iglVertices.resize(points.size(), 3);
    for(auto i = size_t(0); i < points.size(); i++)
    {
        auto p = points[i];
        iglVertices.row(i) = Eigen::Vector3f(p[1], p[2], p[0]);
    }
}

//This is really hacky but I am in a rush. No need to rotations and look at the axisAlignedMatrix
//function is it often calls this one.
auto MeshSegment::computeBoundingBox() -> void
{
    bBox.clear();

    if(iglVertices.rows() <  points.size())
        initIglVertices();

    Eigen::Vector3f max;
    Eigen::Vector3f min;

    Eigen::Matrix<float,3,Eigen::Dynamic> newVertices = bbRotationInv * iglVertices.transpose();

    //Find max and min for all of the new axis
    max = Eigen::Vector3f(newVertices.rowwise().maxCoeff());
    min = Eigen::Vector3f(newVertices.rowwise().minCoeff());

    Eigen::Vector3f pt_a = bbRotation * Eigen::Vector3f(min.x(), min.y(), min.z());
    //std::cout << "pt_a " << pt_a << std::endl;
    bBox.push_back(trimesh::point(pt_a[0], pt_a[1], pt_a[2]));
    Eigen::Vector3f pt_b = bbRotation * Eigen::Vector3f(min.x(), max.y(), min.z());
    bBox.push_back(trimesh::point(pt_b[0], pt_b[1], pt_b[2]));

    Eigen::Vector3f pt_c = bbRotation * Eigen::Vector3f(min.x(), min.y(), max.z());
    bBox.push_back(trimesh::point(pt_c[0], pt_c[1], pt_c[2]));
    Eigen::Vector3f pt_d = bbRotation * Eigen::Vector3f(min.x(), max.y(), max.z());
    bBox.push_back(trimesh::point(pt_d[0], pt_d[1], pt_d[2]));

    Eigen::Vector3f pt_e = bbRotation * Eigen::Vector3f(max.x(), min.y(), max.z());
    bBox.push_back(trimesh::point(pt_e[0], pt_e[1], pt_e[2]));
    Eigen::Vector3f pt_f = bbRotation * Eigen::Vector3f(max.x(), max.y(), max.z());
    bBox.push_back(trimesh::point(pt_f[0], pt_f[1], pt_f[2]));

    Eigen::Vector3f pt_g = bbRotation * Eigen::Vector3f(max.x(), min.y(), min.z());
    bBox.push_back(trimesh::point(pt_g[0], pt_g[1], pt_g[2]));
    Eigen::Vector3f pt_h = bbRotation * Eigen::Vector3f(max.x(), max.y(), min.z());
    bBox.push_back(trimesh::point(pt_h[0], pt_h[1], pt_h[2]));

    originalBBox = bBox;
}

auto MeshSegment::computeBoundingBox(Eigen::Matrix3f eigTrans, Eigen::Matrix3f eigTransInv) -> void
{
    bbRotation = eigTrans;
    bbRotationInv = eigTransInv;
    computeBoundingBox();
}


auto MeshSegment::bBoxData() -> float*
{
    auto data = bBox.data();
    return data[0];
}

auto  MeshSegment::getBBox() -> std::vector<trimesh::point>
{
    return bBox;
}

auto MeshSegment::axisAlignedMinMax(Eigen::Matrix3f eigTrans, Eigen::Matrix3f eigTransInv, Eigen::Vector3f& min, Eigen::Vector3f& max) -> void
{
    if(iglVertices.rows() <  points.size())
        initIglVertices();

    Eigen::Matrix<float,3,Eigen::Dynamic> newVertices = eigTransInv * iglVertices.transpose();

    max = Eigen::Vector3f(newVertices.rowwise().maxCoeff());
    min = Eigen::Vector3f(newVertices.rowwise().minCoeff());
    minCoefs = min;
    maxCoefs = max;

    bbRotation = eigTrans;
    bbRotationInv = eigTransInv;

    computeBoundingBox();
}

auto MeshSegment::getColour() -> trimesh::Color
{
    return colour;
}

auto MeshSegment::getCuboidArea() -> float
{
   auto l = 1.0 + std::fabs(minCoefs[0] - maxCoefs[0]);
   auto w = 1.0 + std::fabs(minCoefs[1] - maxCoefs[2]);
   auto h = 1.0 + std::fabs(minCoefs[1] - maxCoefs[2]);
   return 2.0f * (l*w + w*h + l*h);
}

auto MeshSegment::setIsTruncatedPyramid(bool val) -> void
{
    isTruncatedPyramid = val;
}

auto MeshSegment::getPrimitiveType() -> QString
{
    if(isPlane)
    {
        return QString("Plane");
    }
    else if(isTruncatedPyramid)
    {
        return QString("TruncatedPyramid");
    }
    else
    {
        return QString("Cuboid");
    }
}

auto computeBBoxMean(std::vector<trimesh::point> bBox) -> Eigen::Vector3f
{

    Eigen::Vector3f mean;
    for(auto i = 0; i < bBox.size(); i++)
    {
        mean += Eigen::Vector3f(bBox[i][0], bBox[i][1], bBox[i][2]);
    }

    mean /= bBox.size();
    return mean;
}

auto computeEigenBBox(std::vector<trimesh::point> bBox) -> Eigen::MatrixXf
{

    Eigen::MatrixXf eigenBBox(8,3);
    for(auto i = 0; i < bBox.size(); i++)
    {
        eigenBBox.row(i) = Eigen::Vector3f(bBox[i][0], bBox[i][1], bBox[i][2]);
        //qDebug() << "row " << bBox[i][0] << " " << bBox[i][1] << " " << bBox[i][2];
    }
    return eigenBBox;
}

auto MeshSegment::setBoundingBox(std::vector<trimesh::point> newBBox,  bool translateOnly, int id) -> void
{
    qDebug() << "MeshSegment::setBoundingBox " << id;
    bBox = newBBox;
//    if(id != -1)
//    {
//        if(iglVertices.rows() <  points.size())
//            initIglVertices();

//        Eigen::MatrixXf oldBBoxEigen = computeEigenBBox(bBox);
//        Eigen::MatrixXf newBBoxEigen = computeEigenBBox(newBBox);
//        Eigen::Vector3f translate = oldBBoxEigen.colwise().mean() - newBBoxEigen.colwise().mean();

//        float oldXRange = oldBBoxEigen.col(0).maxCoeff() - oldBBoxEigen.col(0).minCoeff();
//        float oldYRange = oldBBoxEigen.col(1).maxCoeff() - oldBBoxEigen.col(1).minCoeff();
//        float oldZRange = oldBBoxEigen.col(2).maxCoeff() - oldBBoxEigen.col(2).minCoeff();

//        float newXRange = newBBoxEigen.col(0).maxCoeff() - newBBoxEigen.col(0).minCoeff();
//        float newYRange = newBBoxEigen.col(1).maxCoeff() - newBBoxEigen.col(1).minCoeff();
//        float newZRange = newBBoxEigen.col(2).maxCoeff() - newBBoxEigen.col(2).minCoeff();

//        auto xScale = newXRange / oldXRange;
//        auto yScale = newYRange / oldYRange;
//        auto zScale = newZRange / oldZRange;

//        //initIglVertices = iglVertices.colwise() + translate;




//        qDebug() << "xScale " << xScale;
//        qDebug() << "yScale " << yScale;
//        qDebug() << "zScale " << zScale;

////        iglVertices.col(0) *= xScale;
////        iglVertices.col(1) *= yScale;
////        iglVertices.col(2) *= zScale;

////        if(!translateOnly)
////        {
//            for(auto i = 0; i < points.size(); i++)
//            {
//                //qDebug() << "before " << iglVertices(i,0) << " " << iglVertices(i,1) << " " << iglVertices(i,2);
//                iglVertices(i,0) = iglVertices(i,0) * xScale;
//                iglVertices(i,1) = iglVertices(i,1) * yScale;
//                iglVertices(i,2) = iglVertices(i,2) * zScale;


//            }
//      //  }

//        for(auto i = 0; i < iglVertices.rows(); i++)
//            iglVertices.row(i) -= translate;


//        for(auto i = 0; i < points.size(); i++)
//        {
//            //qDebug() << "after " << iglVertices(i,0) << " " << iglVertices(i,1) << " " << iglVertices(i,2);
//            for(auto j =0; j < 3; j++)
//            {
//                points[i][j] = iglVertices(i,j);
//            }
//        }


//        QString localFolderSeg = "/Users/JamesHennessey/Dropbox/engd/multimodel_drawing_assistance/output_data/";
//        QString filenameSeg = "segment";
//        filenameSeg.append(QString::number(id));
//        filenameSeg.append(QString(".obj"));
//        localFolderSeg.append(filenameSeg);
//        Eigen::MatrixXi meshFaces(faces.size(), 3);
//        for(auto i = 0; i < (int)faces.size(); i++)
//        {
//            auto f = faces[i];
//            meshFaces.row(i) = Eigen::Vector3i(f[0], f[1], f[2]);
//        }

//        qDebug() << filenameSeg;
//        igl::writeOBJ(localFolderSeg.toUtf8().constData(), iglVertices, meshFaces);

//    }

   //initIglVertices();
}


auto MeshSegment::writeMesh(QString fileName, std::vector<trimesh::point> new_bbox) -> void
{
   // qDebug() << "MeshSegment::setBoundingBox " << i;


        if(!isPlane)
        {

        //        if(iglVertices.rows() <  points.size())
        //            initIglVertices();

                //qDebug() << "originalBBox";
                Eigen::MatrixXf oldBBoxEigen = computeEigenBBox(bBox);
                Eigen::Vector3f oldBoxCentroid = (oldBBoxEigen.colwise().maxCoeff() + oldBBoxEigen.colwise().minCoeff()) * 0.5;
                //qDebug() << "newBBoxEigen";

                Eigen::Vector3f centroid = iglVertices.colwise().mean();

                for(auto i = 0; i < iglVertices.rows(); i++)
                        iglVertices.row(i) -= centroid;

                Eigen::MatrixXf newBBoxEigen = computeEigenBBox(new_bbox);
                Eigen::Vector3f newBoxCentroid = (newBBoxEigen.colwise().maxCoeff() + newBBoxEigen.colwise().minCoeff()) * 0.5;
                Eigen::Vector3f translate = oldBoxCentroid - newBoxCentroid;

                float oldXRange = oldBBoxEigen.col(0).maxCoeff() - oldBBoxEigen.col(0).minCoeff();
                float oldYRange = oldBBoxEigen.col(1).maxCoeff() - oldBBoxEigen.col(1).minCoeff();
                float oldZRange = oldBBoxEigen.col(2).maxCoeff() - oldBBoxEigen.col(2).minCoeff();

                float newXRange = newBBoxEigen.col(0).maxCoeff() - newBBoxEigen.col(0).minCoeff();
                float newYRange = newBBoxEigen.col(1).maxCoeff() - newBBoxEigen.col(1).minCoeff();
                float newZRange = newBBoxEigen.col(2).maxCoeff() - newBBoxEigen.col(2).minCoeff();

                //Hack to translate the ellipse to the correct position
                auto qtTranslate = QVector3D();
                for(auto i = 0; i < 3; i++)
                    qtTranslate[i] =  translate(i);

                for(auto ellipse : ellipses)
                {
                    auto currentTranslation = ellipse->getTranslation();
                    auto newTranslation = currentTranslation - qtTranslate;
                    ellipse->setTranslation(newTranslation);
                }

        //        qDebug() << "newBBoxEigen.col(0).minCoeff() " << newBBoxEigen.col(0).minCoeff();
        //        qDebug() << "newBBoxEigen.col(1).minCoeff() " << newBBoxEigen.col(1).minCoeff();
        //        qDebug() << "newBBoxEigen.col(2).minCoeff() " << newBBoxEigen.col(2).minCoeff();

        //        for(auto i = 0; i < points.size(); i++)
        //        {
        //            //qDebug() << "before " << iglVertices(i,0) << " " << iglVertices(i,1) << " " << iglVertices(i,2);
        //            iglVertices(i,0) = (iglVertices(i,0) - newBBoxEigen.col(0).minCoeff()) / newXRange;
        //            iglVertices(i,1) = (iglVertices(i,1) - newBBoxEigen.col(1).minCoeff()) / newYRange;
        //            iglVertices(i,2) = (iglVertices(i,2) - newBBoxEigen.col(2).minCoeff()) / newZRange;
        //        }


                auto xScale = newXRange / oldXRange;
                auto yScale = newYRange / oldYRange;
                auto zScale = newZRange / oldZRange;

        //        auto xScale = oldXRange / newXRange;
        //        auto yScale =  oldYRange / newYRange;
        //        auto zScale = oldZRange / newZRange;

                //qDebug() << "cluster id " << id;
        //        qDebug() << "translate " << translate[0] << " " << translate[1] << " "  << translate[2];
        //        qDebug() << "xScale " << xScale;
        //        qDebug() << "yScale " << yScale;
        //        qDebug() << "zScale " << zScale;

        //        iglVertices.col(0) *= xScale;
        //        iglVertices.col(1) *= yScale;
        //        iglVertices.col(2) *= zScale;


                    for(auto i = 0; i < points.size(); i++)
                    {
                        //qDebug() << "before " << iglVertices(i,0) << " " << iglVertices(i,1) << " " << iglVertices(i,2);

                        iglVertices(i,0) = iglVertices(i,0) * xScale;
                        iglVertices(i,1) = iglVertices(i,1) * yScale;
                        iglVertices(i,2) = iglVertices(i,2) * zScale;
                    }


                for(auto i = 0; i < iglVertices.rows(); i++)
                {
                    iglVertices.row(i) -= translate;
                    iglVertices.row(i) += centroid;
                }

                //iglVertices.colwise() += centroid;

        //        for(auto i = 0; i < points.size(); i++)
        //        {
        //            //qDebug() << "after " << iglVertices(i,0) << " " << iglVertices(i,1) << " " << iglVertices(i,2);
        //            for(auto j =0; j < 3; j++)
        //            {
        //                points[i][j] = iglVertices(i,j);

        //        }

        //        QString localFolderSeg = "/Users/JamesHennessey/Dropbox/engd/multimodel_drawing_assistance/output_data/";
        //        QString filenameSeg = "segment";
        //        filenameSeg.append(QString::number(id));
        //        filenameSeg.append(QString(".obj"));
        //        localFolderSeg.append(filenameSeg);
        }

        Eigen::MatrixXi meshFaces(faces.size(), 3);
        for(auto i = 0; i < (int)faces.size(); i++)
        {
            auto f = faces[i];
            meshFaces.row(i) = Eigen::Vector3i(f[0], f[1], f[2]);
        }


        for(auto i = 0; i < points.size(); i++)
        {
            //qDebug() << "before " << iglVertices(i,0) << " " << iglVertices(i,1) << " " << iglVertices(i,2);

            Eigen::Vector3f temp = iglVertices.row(i);

            iglVertices(i,0) = temp(1);
            iglVertices(i,1) = temp(2);
            iglVertices(i,2) = temp(0);

            //qDebug() << "after " << iglVertices(i,0) << " " << iglVertices(i,1) << " " << iglVertices(i,2);
        }

        //qDebug() << filenameSeg;
        igl::writeOBJ(fileName.toUtf8().constData(), iglVertices, meshFaces);
}

auto MeshSegment::writeDataAndGetJSON(int i) -> QJsonObject
{

    qDebug() << "auto MeshSegment::writeDataAndGetJSON(int i) " << i;
    QString localFolderSeg = "/Users/JamesHennessey/Dropbox/engd/multimodel_drawing_assistance/output_data/";
    QString filenameSeg = "segment";
    filenameSeg.append(QString::number(i));
    filenameSeg.append(QString(".ply"));
    localFolderSeg.append(filenameSeg);
    Eigen::MatrixXi meshFaces(faces.size(), 3);
    for(auto i = 0; i < (int)faces.size(); i++)
    {
        auto f = faces[i];
        meshFaces.row(i) = Eigen::Vector3i(f[0], f[1], f[2]);
    }

    qDebug() << filenameSeg;
    igl::writePLY(localFolderSeg.toUtf8().constData(), iglVertices, meshFaces);

    QString localFolderBox = "/Users/JamesHennessey/Dropbox/engd/multimodel_drawing_assistance/output_data/";

    QString filenameBox = "box";
    filenameBox.append(QString::number(i));
    filenameBox.append(QString(".ply"));
    localFolderBox.append(filenameBox);

    if(isPlane)
    {
            qDebug() << "write " << i;
            auto bb = getBBox();
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
            Eigen::Vector3f minCoefs = iglAllPoints.colwise().maxCoeff();
            Eigen::MatrixXf iglVert(4, 3);
            iglVert.row(0) = maxCoefs;
            iglVert.row(3) = minCoefs;

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
            iglVert.row(2) = point3;

            qDebug() << filenameBox;
            igl::writePLY(localFolderBox.toUtf8().constData(), iglVert, iglFaces);
    }
    else
    {
        qDebug() << "write " << i;
        auto bb = getBBox();
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
            //iglVert.row(j) = Eigen::Vector3f(p[0], p[1], p[2]);
        }


        qDebug() << filenameBox;
        igl::writePLY(localFolderBox.toUtf8().constData(), iglVert, iglFaces);
    }

    QJsonObject object;

    object["box"] = filenameBox;
    object["segment"] = filenameSeg;

    return object;
}

auto MeshSegment::writeBox(int i) -> QString
{

//    QString localFolderSeg = "/Users/JamesHennessey/Dropbox/engd/multimodel_drawing_assistance/output_data/";
//    QString filenameSeg = "segment";
//    filenameSeg.append(QString::number(i));
//    filenameSeg.append(QString(".ply"));
//    localFolderSeg.append(filenameSeg);
//    Eigen::MatrixXi meshFaces(faces.size(), 3);
//    for(auto i = 0; i < (int)faces.size(); i++)
//    {
//        auto f = faces[i];
//        meshFaces.row(i) = Eigen::Vector3i(f[0], f[1], f[2]);
//    }

//    qDebug() << filenameSeg;
//    igl::writePLY(localFolderSeg.toUtf8().constData(), iglVertices, meshFaces);

    qDebug() << "auto MeshSegment::writeBox(int i)  write " << i;
    auto bb = getBBox();
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

    }

    QString localFolderBox = "/Users/JamesHennessey/Dropbox/engd/multimodel_drawing_assistance/output_data/";

    QString filenameBox = "box";
    filenameBox.append(QString::number(i));
    filenameBox.append(QString(".ply"));
    localFolderBox.append(filenameBox);
    qDebug() << filenameBox;
    igl::writePLY(localFolderBox.toUtf8().constData(), iglVert, iglFaces);

    return filenameBox;
}

auto MeshSegment::addFaces(std::vector<trimesh::TriMesh::Face> f, int id) -> void
{
    faces = f;



}
