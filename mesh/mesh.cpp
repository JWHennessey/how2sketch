#ifndef MESH_CPP
#define MESH_CPP

#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QFileDialog>
#include <limits>
#include "mesh/mesh.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/Dense"
#include <Eigen/LU>
#include <Eigen/Eigenvalues>
//#include <array>
#include <iostream>
#include <QFile>
#include "trimesh/Vec.h"
#include "trimesh/lineqn.h"
#include <QMatrix4x4>
#include <QVector2D>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Simple_cartesian.h>
#include <CGAL/squared_distance_3.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/min_quadrilateral_2.h>
#include <CGAL/convex_hull_2.h>
#include <igl/read_triangle_mesh.h>

#include <igl/writeOBJ.h>
//#include <igl/cotmatrix.h>
//#include <igl/per_vertex_normals.h>
//#include <igl/principal_curvature.h>

#include <QTextStream>
#include <unistd.h>

#include "optimisation/utility_functions.h"

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using Polygon_2 = CGAL::Polygon_2<K>;
using Point = CGAL::Point_2<K>;
using namespace trimesh;
using namespace utility;

// i+1 and i-1 modulo 3
// This way of computing it tends to be faster than using %
#define NEXT(i) ((i)<2 ? (i)+1 : (i)-2)
#define PREV(i) ((i)>0 ? (i)-1 : (i)+2)

Mesh::Mesh()
    : bbXRot(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX()))
    , bbYRot(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY()))
    , bbZRot(Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ()))
    , drawnBoxes(Eigen::Matrix<float, 200, 4>::Zero())
    , drawnBoxesCount(0)
{
    //fillTestData();
}
Mesh::Mesh(const QString filename)
    : Mesh()
{
    openMesh(filename);
}

Mesh::~Mesh()
{
    //qDebug() << "Mesh::~Mesh()";
}


auto Mesh::openMeshJSON(const QString filename) -> QJsonDocument
{
    QString jsonString;
    QFile file;
    file.setFileName(filename);
    file.open(QIODevice::ReadOnly | QIODevice::Text);
    jsonString = file.readAll();
    file.close();

    QJsonDocument jsonDocument = QJsonDocument::fromJson(jsonString.toUtf8());
    return jsonDocument;
//    if(!jsonDocument.isEmpty() && !jsonDocument.isNull())
//    {
//        auto jsonObject = jsonDocument.object();
//        auto segmentArray = jsonObject["array"].toArray();
//        meshSegmentCluster.create(segmentArray);



//    }
//    qWarning() << sd.isNull(); // <- print false :)
//    QJsonObject sett2 = sd.object();
//    qWarning() << sett2.value(QString("title"));  // <- print my title
}


void Mesh::openMesh(const QString filename)
{
    qDebug() << filename;
    QString completeObjectFilename;
    QJsonObject jsonObject;
    if(filename.contains(".json"))
    {
        qDebug() << "Has JSON";
        QJsonDocument jsonDocument = openMeshJSON(filename);
        jsonObject = jsonDocument.object();
        auto file = jsonObject["mesh"].toString();
        auto dir = jsonObject["directory"].toString();
        directory = dir;
        meshFilename = file;



        completeObjectFilename = dir.append(file);

    }
    else
    {
        completeObjectFilename = filename;
    }


    trimesh = trimesh::TriMesh::read(completeObjectFilename.toUtf8().constData());
    trimesh->need_normals();
    trimesh->need_curvatures();
    trimesh->need_dcurv();
    trimesh->need_faces();

    //Normalize mesh so is centeres around 0,0,0 and ranges between 0 and 1
    auto mean = trimesh::point (0,0,0);
    auto min_x = trimesh->vertices[0][0];
    auto min_y = trimesh->vertices[0][1];
    auto min_z = trimesh->vertices[0][2];
    auto max_x = min_x;
    auto max_y = min_y;
    auto max_z = min_z;

    for(auto i = 0; i < (int)trimesh->vertices.size(); i++)
    {
        auto pt = trimesh->vertices[i];
        mean += pt;
        if(pt[0] > max_x)
            max_x = pt[0];
        else if(pt[0] < min_x)
            min_x = pt[0];

        if(pt[1] > max_y)
            max_y = pt[1];
        else if(pt[1] < min_y)
            min_y = pt[1];

        if(pt[2] > max_z)
            max_z = pt[2];
        else if(pt[2] < min_z)
            min_z = pt[2];

    }

    mean /= trimesh->vertices.size();
    auto max = trimesh::point(max_x, max_y, max_z);
    auto min = trimesh::point(min_x, min_y, min_z);
    max -= mean;
    min -= mean;
    auto diff = max - min;
    auto max_coeff = diff.max();

    iglVertices.resize(trimesh->vertices.size(), 3);

//    mean[0] = 2845.71;
//    mean[1] = 1581.59;
//    mean[2] = 60.5878;
//    max_coeff = 270.08;

    //qDebug() << "mean " << mean[0] << " " << mean[1] << " " << mean[2];
    //qDebug() << "max_coeff " << max_coeff;

    for(auto i = 0; i < (int)trimesh->vertices.size(); i++)
    {
        trimesh->vertices[i] -= mean;
        trimesh->vertices[i] /= max_coeff;
        auto p = trimesh->vertices[i];
        trimesh->vertices[i][0] = p[1];
        trimesh->vertices[i][1] = p[2];
        trimesh->vertices[i][2] = p[0];
        iglVertices.row(i) = Eigen::Vector3f(p[1], p[2], p[0]);
    }

    iglFaces.resize(trimesh->faces.size(), 3);
    for(auto i = size_t(0); i < size_t(trimesh->faces.size()); i++)
    {
        auto f = trimesh->faces[i];
        iglFaces.row(i) = Eigen::Vector3i(f[0], f[1], f[2]);
    }

    if(filename.contains(QString("teapot"), Qt::CaseInsensitive ))
    {
        Eigen::AngleAxisf XRot = Eigen::AngleAxisf(0.03, Eigen::Vector3f::UnitX());
        Eigen::AngleAxisf YRot = Eigen::AngleAxisf(-0.34, Eigen::Vector3f::UnitY());
        Eigen::AngleAxisf ZRot = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());

        Eigen::Quaternion<float> rot = XRot * YRot * ZRot;

        Eigen::Matrix3f rotation = rot * Eigen::Matrix3f::Identity();
        Eigen::Matrix3f rotationInv = rotation.inverse();

        for(auto i = 0; i < (int)trimesh->vertices.size(); i++)
        {
            auto p = trimesh->vertices[i];
            Eigen::Vector3f aap = rotationInv * Eigen::Vector3f(p[0], p[1], p[2]);
            trimesh->vertices[i] = trimesh::point(aap[0], aap[1], aap[2]);
            iglVertices.row(i) = aap;
        }
    }


    if(filename.contains(".json"))
    {

        auto segmentArray = jsonObject["segments"].toArray();
        auto directory = jsonObject["directory"].toString();

        segmentsGraph.loadSegments(directory, segmentArray, mean, max_coeff);

        segmentsGraph.getSegments(segments);
        orientedBoundingBox();

        if(jsonObject.find("primitives") != jsonObject.end())
        {

            auto rotationArray = jsonObject["bbox_rotation"].toArray();

            bbXRot = Eigen::AngleAxisf(rotationArray[0].toDouble(), Eigen::Vector3f::UnitX());
            bbYRot = Eigen::AngleAxisf(rotationArray[1].toDouble(), Eigen::Vector3f::UnitY());
            bbZRot = Eigen::AngleAxisf(rotationArray[2].toDouble(), Eigen::Vector3f::UnitZ());

            Eigen::Matrix3f eigTransposed;
            Eigen::Matrix3f eigTransposedInv;

            Eigen::Quaternion<float> rot = bbXRot * bbYRot * bbZRot;

            eigTransposed = rot * Eigen::Matrix3f::Identity();
            eigTransposedInv = eigTransposed.inverse();

            std::cout << "eigTransposed " << eigTransposed << std::endl;

            auto primitivesArray = jsonObject["primitives"].toArray();
            segmentsGraph.loadPrimitives(primitivesArray, eigTransposed, eigTransposedInv);
            updateBoundingBoxes();

        }
//        auto idCounterRef = 0;
//        meshSegmentCluster.create(directory, segmentArray, mean, max_coeff, idCounterRef);

//        qDebug() << "segments.size() " << segments.size();
//        //Get all mesh segments
//        meshSegmentCluster.getAllSegments(segments);

//        qDebug() << "segments.size() " << segments.size();
//        for(auto& s : segments)
//        {
//            qDebug() << "s.size() " << s->size();
//        }
        setDefaultColour();
    }
    else
    {
        readColour(completeObjectFilename);
        orientedBoundingBox();
    }





    boundingBoxesConnectivity();



}


auto Mesh::writePrimitivesAndRelations() -> void
{
    auto filename = QFileDialog::getSaveFileName(0, tr("Save File"), "/Users/JamesHennessey/Dropbox/engd/multimodel_drawing_assistance/primitives_relations.json");

    QJsonObject outputObject;

    outputObject["directory"] = directory;
    outputObject["mesh"] = meshFilename;


     QJsonArray rotationJson;
     rotationJson.push_back(bbXRot.angle());
     rotationJson.push_back(bbYRot.angle());
     rotationJson.push_back(bbZRot.angle());

    outputObject["bbox_rotation"] = rotationJson;

    QJsonArray segmentsArray;
    for(auto s: segments)
    {
        segmentsArray.push_back(QJsonValue(s->getName()));
    }
    outputObject["segments"] = segmentsArray;

    QJsonArray primitivesArray;
    for(auto s: segments)
    {
        QJsonObject primitiveObject;
        primitiveObject["bbox"] = s->getJsonBBox();
        primitiveObject["ellipses"] = s->getJsonEllipses();
        primitiveObject["isPlane"] = s->getIsPlane();
        primitiveObject["planeAxis"] = s->getPlaneAxis();
        primitiveObject["planeFace"] = s->getPlaneFace();

        primitivesArray.push_back(primitiveObject);
    }
    outputObject["primitives"] = primitivesArray;

    QFile saveFile(filename);
    if (!saveFile.open(QIODevice::WriteOnly)) {
            qWarning("Couldn't open save file.");
    }
    QJsonDocument saveDoc(outputObject);
    saveFile.write(saveDoc.toJson());
}

auto Mesh::getMeshSegments() -> std::vector<MeshSegment*>
{
    return segments;
}

const static int no_face_indexes = 3;
auto Mesh::findRelations(int& id) -> void
{

    qDebug() << "Find Relations";
    int count = 0;
    auto seg_count = (int) segments.size();
    for(auto seg_1 = 0; seg_1 < seg_count; seg_1++)
    {
        auto segment1 = segments[seg_1];
        segment1->select();
        for(auto a_1 = 1; a_1 <= 3; a_1++)
        {

            segment1->setAxis(a_1);

            if(segment1->getIsPlane() && segment1->getPlaneAxis() != a_1)
            {
                  continue;
//                auto a = segment1->getPlaneAxis();
//                segment1->setAxis(a);
//                if(a_1 > 1)
//                    continue;
            }

            for(auto f_1 = 1; f_1 <= no_face_indexes; f_1++)
            {
                auto face1 = segment1->getFace(a_1, f_1);
                segment1->setFace(f_1);

                if(segment1->getIsPlane() && segment1->getPlaneFace() != f_1)
                {
                      continue;
//                    auto f = segment1->getPlaneFace();
//                    segment1->setFace(f);
//                    if(f_1 > 1)
//                        continue;
                }

                for(auto seg_2 = seg_1 + 1; seg_2 < seg_count; seg_2++)
                {
                    auto segment2 = segments[seg_2];
                    segment2->select();
                    for(auto a_2 = 1; a_2 <= 3; a_2++)
                    {

                        segment2->setAxis(a_2);
                        if(segment2->getIsPlane() && segment2->getPlaneAxis() != a_2)
                        {
                            continue;
//                            auto a = segment2->getPlaneAxis();
//                            segment2->setAxis(a);
//                            if(a_2 > 1)
//                                continue;
                        }

                        for(auto f_2 = 1; f_2 <= no_face_indexes; f_2++)
                        {
                            segment2->setFace(f_2);

                            if(segment2->getIsPlane() && segment2->getPlaneFace() != f_2)
                            {
                                    continue;
                            }


                            auto face2 = segment2->getFace(a_2, f_2);
                            for(auto r_type = 0; r_type < 3; r_type++)
                            {
                                 if(count == id)
                                 {
//                                     qDebug() << segment1->getName() << " axis " << a_1 << " face " << f_1;
//                                     qDebug() << segment2->getName() << " axis " << a_2 << " face " << f_2;
                                     if(r_type == 0)
                                     {
                                         auto isCoPlanr = hasCoPlanarRelation(face1, face2);


                                         if(isCoPlanr)
                                         {
//                                              if((f_1 == 3 && f_2 != 3) || (f_2 == 3 && f_1 != 3) || a_1 != a_2)
//                                                 continue;

//                                             if(isCoPlanr && (f_1 == 3 || f_2 == 3) )
//                                             {
//                                                 //continue;

//                                                 Eigen::Vector3f normal1 = completeNormal(face1);
//                                                 Eigen::Vector3f normal2 = completeNormal(face2);
//                                                 Eigen::Vector3f centroid1 = completeCentroid(face1) * 1000.0f;
//                                                 Eigen::Vector3f centroid2 = completeCentroid(face2) * 1000.0f;

//                                           //    centroid1.normalize();
//                                           //    centroid2.normalize();

//                                                 Eigen::Vector3f delta = centroid2 - centroid1;
//                                                 auto dist = std::fabs(normal1.dot(delta));
//                                                 std::cout << "dist " << dist << std::endl;
//                                                 std::cout << "normal 1 " << normal1 << std::endl;
//                                                 std::cout << "normal 2 " << normal2 << std::endl;
//                                                 std::cout << "f_1 " << f_1 << " f_2 " << f_2 << std::endl;
//                                                 std::cout << "face1 " << face1 << std::endl;
//                                                 std::cout << "face2 " << face2 << std::endl;
//                                             }

//                                            qDebug() << "------------------------------------------";
//                                            qDebug() << "Co-Planar";
//                                             if( segment1->getName() == QString("stand.ply") || segment2->getName() == QString("stand.ply") )
//                                                 continue;



                                            qDebug() << segment1->getName() << " axis " << a_1 << " face " << f_1;
                                            qDebug() << segment2->getName() << " axis " << a_2 << " face " << f_2;


                                            auto relation1 = new PlanarRelation(segment1, a_1, f_1, segment2, a_2, f_2);
                                            segment1->addPlanarRelation(a_1, f_1, relation1);
                                            auto relation2 = new PlanarRelation(segment2, a_2, f_2, segment1, a_1, f_1);
                                            segment2->addPlanarRelation(a_2, f_2, relation2);
                                            return;
                                         }
                                     }
                                     else if(r_type == 1)
                                     {
                                         auto orthogonalRelation = hasOrthogonalRelation(face1, face2);
                                         if(orthogonalRelation == 1)
                                         {
                                             //qDebug() << "Parallel";
                                             //return;
                                         }
                                         else if(orthogonalRelation == 2)
                                         {
                                             //qDebug() << "Orthogonal";
                                             //return;
                                         }
                                     }
                                     else if(r_type == 2)
                                     {
//                                         auto isCoAxis = hasCoAxialRelation(a_1, face1, a_2, face2);
//                                         if(isCoAxis)
//                                         {
//                                             qDebug() << "Co-Axial";
//                                             return;
//                                         }
                                     }
                                 }
                                 count++;
                                 if(count > id)
                                     id = count;

                           }
                           segment2->setFace(-1);
                        }
                        segment2->setAxis(-1);
                    }
                    segment2->deselect();
                }
            }
            segment1->setAxis(-1);
        }
        segment1->deselect();
    }
}



auto Mesh::fSize() -> int
{
    return trimesh->faces.size();
}

auto Mesh::fData() -> unsigned short*
{
    auto data = trimesh->faces.data();
    auto ui_data = (unsigned short*) std::malloc(trimesh->faces.size() * 3 * sizeof(unsigned short));
    auto ui_count = 0;
    for(auto i = 0; i < trimesh->faces.size(); i++)
    {
        auto face_vec = data[i];
        ui_data[ui_count] = (unsigned short) face_vec[0];
        ui_data[ui_count+1] = (unsigned short) face_vec[1];
        ui_data[ui_count+2] = (unsigned short) face_vec[2];
        ui_count+=3;
    }
    return &ui_data[0];
}


auto Mesh::nSize() -> int
{
    // return normals.rows();
    return trimesh->normals.size();
}

auto Mesh::nData() -> float*
{
    auto data = trimesh->normals.data();
    return data[0];
}


auto Mesh::bBoxSize() -> int
{
    qDebug() << "bBoxSize() " << bBox.size();
    return bBox.size();
}

auto Mesh::bBoxData() -> float*
{
    auto data = bBox.data();
    return data[0];
}

auto Mesh::bBoxFaceData() -> unsigned short*
{
    return boundingBoxFaces;
}


auto Mesh::cSize() -> int
{
    return trimesh->colors.size();
}

auto Mesh::cData() -> float*
{
    auto data = trimesh->colors.data();
    return data[0];
    //   auto float_data = (float*) std::malloc(trimesh->vertices.size() * 3 * sizeof(float));
    //   auto float_count = 0;
    //   for(auto i = size_t(0); i < vSize(); i++)
    //   {
    //       float_data[float_count]   = trimesh->colors[i][0];
    //       float_data[float_count+1] = trimesh->colors[i][1];
    //       float_data[float_count+2] = trimesh->colors[i][2];
    //       float_count+=3;
    //   }
    //   //qDebug() << typeid(data).name();
    //   //qDebug() << data[0];// << " " << data[1] << " " << data[2] << " " << data[3] << " " << data[4];
    //   return &float_data[0];
}

auto Mesh::vSize() -> int
{
    return trimesh->vertices.size();
}

auto Mesh::vData() -> float*
{
    auto data = trimesh->vertices.data();
    return data[0];
}

auto Mesh::maxDirSize() -> int
{
    return trimesh->pdir1.size();
}

auto Mesh::maxDirData() -> float*
{
    auto data = trimesh->pdir1.data();

    return data[0];
}

auto Mesh::minDirSize() -> int
{
    return trimesh->pdir2.size();
}

auto Mesh::minDirData() -> float*
{
    auto data = trimesh->pdir2.data();
    return data[0];
}

auto Mesh::maxCurveSize() -> int
{
    return trimesh->curv1.size();
}

auto Mesh::maxCurveData() -> float*
{
    auto data = trimesh->curv1.data();
    return &data[0];
}

auto Mesh::minCurveSize() -> int
{
    return trimesh->curv1.size();
}

auto Mesh::minCurveData() -> float*
{
    auto data = trimesh->curv2.data();
    return &data[0];
}

auto Mesh::dCurveSize() -> int
{
    return trimesh->dcurv.size();
}

auto Mesh::dCurveData() -> float*
{
    auto data = trimesh->dcurv.data();
    return data[0];
}

auto Mesh::featureSize() -> float
{
    return trimesh->feature_size();
}

//auto Mesh::saveObject(QString filename) -> void
//{
//    auto CommaInitFmt = Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", " << ", ";");

//    auto filenameOBJ = QString(filename);
//    filenameOBJ.append(".obj");
//    //igl::writeOBJ(filenameOBJ.toUtf8().constData(), vertices, faces);


//    auto filenameTXT = QString(filename);
//    filenameTXT.append(".txt");
//    QFile file(filenameTXT);
//    if(file.open(QIODevice::ReadWrite))
//    {
//        QTextStream stream(&file);
//        stream << "Rotations:";
//        stream << data.xRot << " ";
//        stream << data.yRot << " ";
//        stream << data.zRot << " " << endl;
//        stream << "Position:";
//        stream << data.xPos << " ";
//        stream << data.yPos << " ";
//        stream << data.zPos << " " << endl;
//        stream << "Scale:" << data.scale << endl;
//        stream << "Colour:";
//        for(auto i = 0; i < colours.rows(); i++)
//          for(auto j = 0; j < colours.cols(); j++)
//          {
//              stream << colours(i,j);
//              if(i == (colours.rows() - 1) && j == (colours.cols() - 1))
//                stream << endl;
//              else
//                stream << ",";
//          }
//    }
//}

auto Mesh::readColour(QString filename) -> void
{
    //Auto fill color to ensure color vector is right size
    setDefaultColour();

    filename = filename.replace(QString(".obj"), QString(".ply"));
    QFile file(filename);
    if(file.open(QIODevice::ReadOnly))
    {
        bool endOfHeader = false;
        bool hasColourInfo = false;
        int propertyCount = 0;
        int r = -1;
        int g = -1;
        int b = -1;
        int rowCount = 0;
        QTextStream in(&file);

        while (!in.atEnd())
        {
            auto line = in.readLine();
            QStringList lstLine = line.split(" ");
            if(!endOfHeader)
            {
                if(lstLine[0] == QString("property"))
                {
                    propertyCount++;
                    if(lstLine[2] == QString("red"))
                        r = propertyCount;
                    if(lstLine[2] == QString("green"))
                        g = propertyCount;
                    if(lstLine[2] == QString("blue"))
                        b = propertyCount;
                }
                else if(lstLine[0] == QString("end_header"))
                {
                    endOfHeader = true;
                    if(r >= 0 && b >= 0 && g >= 0 )
                    {
                        hasColourInfo = true;

                        //qDebug() << vertices.rows() << ", " << vertices.cols();
                        line = in.readLine();
                        lstLine = line.split(" ");
                    }
                }
            }
            if(endOfHeader && hasColourInfo && rowCount < trimesh->vertices.size())
            {
                //qDebug() << "add colour row " << r << rowCount << lstLine.size() << lstLine[0];
                //colours.row(rowCount) = Eigen::Vector3f(0.5, 0.5, 0.5);
                auto r1 = clampColour(lstLine[r - 1].toInt());
                auto g1 = clampColour(lstLine[g - 1].toInt());
                auto b1 = clampColour(lstLine[b - 1].toInt());
                //                qDebug() << "in " << lstLine[r - 1].toInt() << " " << lstLine[g - 1].toInt() << " " << lstLine[b - 1].toInt();
                //                qDebug() << "out " << r1 << " " << g1 << " " << b1;
                auto c = trimesh::Color(r1, g1, b1);
                //Hack because some vertices as missed in segmentation, they are white by default
                if(c != trimesh::Color(255,255,255));
                {

                    trimesh->colors[rowCount] = c;

                    //Update map and add point to mesh segment
                    float cId = c[0] + c[1] * 17.4f + c[2] * 39.2f;
                    auto search = colourSegmentMap.find(cId);
                    if(search != colourSegmentMap.end())
                    {
                        segments[search->second]->addPoint(trimesh->vertices[rowCount]);
                    }
                    else
                    {
                        auto id = (int) colourSegmentMap.size();
                        //qDebug() << cId << " " << id;
                        colourSegmentMap.insert(std::pair<float, int>(cId, id));
                        segments.push_back(new MeshSegment(c));
                        segments[id]->addPoint(trimesh->vertices[rowCount]);
                    }
                }
                //                qDebug() << rowCount;
                //                qDebug() << line;
                //                qDebug() << trimesh->vertices[rowCount][0] << " " << trimesh->vertices[rowCount][1] << " " << trimesh->vertices[rowCount][2];
                //                qDebug() << trimesh->colors[rowCount][0] << " " << trimesh->colors[rowCount][1] << " " << trimesh->colors[rowCount][2];
                rowCount++;
            }
        }

        file.close();
    }


    for(auto it = segments.begin(); it != segments.end();)
    {


        //Hack for the noisy segmentation
        if((*it)->size() < std::max(vSize() / 100, 110) || (*it)->size() == 378)
        {
            segments.erase(it);
        }
        else if(((*it)->getColour() == trimesh::Color(255,255,255)))
        {
            segments.erase(it);
        }
        else
        {
            //            qDebug() << "(*it)->getCuboidArea() " << (*it)->getCuboidArea();
            //            qDebug() << "(*it)->size() " << (*it)->size();
            it++;
        }
    }

    qDebug() << "Reading " << segmentSize() << " segments";
    //qDebug() << "Close File";


}

auto Mesh::clampColour(int i) -> int
{
    if(i < 85)
        return 0;
    else if(i < 170)
        return 128;
    else
        return 255;
}

auto Mesh::setDefaultColour() -> void
{
    trimesh->colors.assign(trimesh->vertices.size(), trimesh::Color(255, 255, 0));
}

auto Mesh::segmentSize() -> int
{
    //Start at 1 to include the segment of the whole object. Mainly used for rendering using one VBO for all boxes
    //    auto x = 1;
    //    for(auto& i : segments)
    //    {
    //        //qDebug() << i->size();
    //        if(i->size() > 100)
    //            x++;
    //    }
    return segments.size() + 1;

}

auto Mesh::segmentColour(int i) -> trimesh::Color
{
    //qDebug() << "segmentColour " << i;
    if(i < segmentSize() - 1)
    {
        return segments[i]->getColour();
    }
    return trimesh::Color(255,255,255);
}

auto Mesh::segmentIsSelected(int i) -> bool
{
    if(i < segmentSize() - 1)
    {
        return segments[i]->isSelected();
    }
    return false;
}

auto Mesh::getSelectedSegment(int i) -> MeshSegment*
{
    return segments[i];
}

auto Mesh::addPointsToBoundingBox(Eigen::Matrix3f eigTransposed, Eigen::Vector3f min, Eigen::Vector3f max ) -> void
{
    //Build box for rendering
    //Ordering specific to the faces I have manually generated

    Eigen::Vector3f pt_a = eigTransposed * Eigen::Vector3f(min.x(), min.y(), min.z());
    //std::cout << "pt_a " << pt_a << std::endl;
    bBox.push_back(trimesh::point(pt_a[0], pt_a[1], pt_a[2]));
    Eigen::Vector3f pt_b = eigTransposed * Eigen::Vector3f(min.x(), max.y(), min.z());
    bBox.push_back(trimesh::point(pt_b[0], pt_b[1], pt_b[2]));

    Eigen::Vector3f pt_c = eigTransposed * Eigen::Vector3f(min.x(), min.y(), max.z());
    bBox.push_back(trimesh::point(pt_c[0], pt_c[1], pt_c[2]));
    Eigen::Vector3f pt_d = eigTransposed * Eigen::Vector3f(min.x(), max.y(), max.z());
    bBox.push_back(trimesh::point(pt_d[0], pt_d[1], pt_d[2]));

    Eigen::Vector3f pt_e = eigTransposed * Eigen::Vector3f(max.x(), min.y(), max.z());
    bBox.push_back(trimesh::point(pt_e[0], pt_e[1], pt_e[2]));
    Eigen::Vector3f pt_f = eigTransposed * Eigen::Vector3f(max.x(), max.y(), max.z());
    bBox.push_back(trimesh::point(pt_f[0], pt_f[1], pt_f[2]));

    Eigen::Vector3f pt_g = eigTransposed * Eigen::Vector3f(max.x(), min.y(), min.z());
    bBox.push_back(trimesh::point(pt_g[0], pt_g[1], pt_g[2]));
    Eigen::Vector3f pt_h = eigTransposed * Eigen::Vector3f(max.x(), max.y(), min.z());
    bBox.push_back(trimesh::point(pt_h[0], pt_h[1], pt_h[2]));
}

auto vec3ToEigen(QVector4D in) -> Eigen::Vector3d
{
    return Eigen::Vector3d(in[0], in[1], in[2]);
}

//Closest point on the edge closes to the ray
//Implemented by using a plane defined by the ray and its normal
//Then finding the intersection between the plane and the edge
auto closestPoint(CGAL::Line_3<K> ray, CGAL::Line_3<K> edge) -> CGAL::Point_3<K>
{
    // Plane for ray
    Eigen::Vector3f forward(0,0,-1);
    Eigen::Vector3f up(0,1,0);
    auto rayVec = ray.to_vector();
    Eigen::Vector3f rayEigen(rayVec.x(), rayVec.y(), rayVec.z());
    Eigen::Quaternion<float> rot = Eigen::Quaternion<float>::FromTwoVectors(forward, rayEigen);
    Eigen::Vector3f normal = rot * up;
    auto plane = CGAL::Plane_3<K>(ray.point(), CGAL::Vector_3<K>(normal[0], normal[1], normal[2]));

    auto result = CGAL::intersection(plane, edge);
    CGAL::Point_3<K> intersection;
    CGAL::Line_3<K> intersectionLine;
    if(!CGAL::assign(intersection, result))
        qDebug() << "No intersection between ray and edge";

    return intersection;
}

auto Mesh::screenSpaceVector(QMatrix4x4 viewport, QMatrix4x4 mvp, int boxId, int edgeId) -> QVector2D
{
    int fixedEdgeIndex = boxId * bb_faces_count + edgeId;
    auto fixedEdgeVertexIDs = edgeToVertices[fixedEdgeIndex];
    auto fixedA = bBox[fixedEdgeVertexIDs.first];
    auto fixedB = bBox[fixedEdgeVertexIDs.second];
    auto fixedV1 = QVector4D(fixedA[0], fixedA[1], fixedA[2], 1);
    auto fixedV2 = QVector4D(fixedB[0], fixedB[1], fixedB[2], 1);
    auto fixedSS1 = mvp * fixedV1;
    fixedSS1 /= fixedSS1[3];
    fixedSS1 = viewport * fixedSS1;
    auto fixedSS2 = mvp * fixedV2;
    fixedSS2 /= fixedSS2[3];
    fixedSS2 = viewport * fixedSS2;

    auto fixedSSVector = QVector2D(fixedSS1[0], fixedSS1[1]) - QVector2D(fixedSS2[0], fixedSS2[1]);

    return fixedSSVector;
}

auto Mesh::findMovablePoint(int& movablePointId, int& fixedPointId, QVector4D& movablePoint, QVector4D& fixedPoint, int boxId, int unitEdgeId, int selectedEdgeId, bool selectedEdgeAlreadyMapped) -> bool
{
    int unitEdgeIndex = boxId * bb_faces_count + unitEdgeId;
    auto unitEdgeIDs = edgeToVertices[unitEdgeIndex];
    auto x = bBox[unitEdgeIDs.first];
    auto y = bBox[unitEdgeIDs.second];
    auto ue1 = QVector4D(x[0], x[1], x[2],1);
    auto ue2 = QVector4D(y[0], y[1], y[2],1);
    auto unitEdgeVec = ue1 - ue2;

    int selectedEdgeIndex = selectedEdgeAlreadyMapped ? selectedEdgeId : ( boxId * bb_faces_count + selectedEdgeId);

    auto selectedEdgeIDs = edgeToVertices[selectedEdgeIndex];
    auto a = bBox[selectedEdgeIDs.first];
    auto b = bBox[selectedEdgeIDs.second];
    auto v1 = QVector4D(a[0], a[1], a[2],1);
    auto v2 = QVector4D(b[0], b[1], b[2],1);
    auto selectedEdgeVec = v1 - v2;

    //Cross product of two vectors is zero then they are parallel
    auto cp = QVector3D::crossProduct(unitEdgeVec.toVector3D(), selectedEdgeVec.toVector3D());
    qDebug() << "crossProduct " << cp.length();
    if(cp.length() < 0.1e-5)
    {
        return false;
    }

    //If the first point is connected to the fixed edge
    if(selectedEdgeIDs.first == unitEdgeIDs.first || selectedEdgeIDs.first == unitEdgeIDs.second)
    {
        movablePoint = v2;
        fixedPoint = v1;
        movablePointId = selectedEdgeIDs.second;
        fixedPointId = selectedEdgeIDs.first;
        return true;
    }
    //Is the second point is connected to the fixed edge
    else if(selectedEdgeIDs.second == unitEdgeIDs.first || selectedEdgeIDs.second == unitEdgeIDs.second)
    {
        movablePoint = v1;
        fixedPoint = v2;
        movablePointId = selectedEdgeIDs.first;
        fixedPointId = selectedEdgeIDs.second;
        return true;
    }
    else // The edge does not share a point but should be moveable
    {
        //Iterate over all faces to see if either of the selected edge
        //vertices are connected by one edge
        auto box_start = boxId * bb_faces_count * 3;
        for(auto i = box_start; i < box_start + 12 * 3; i+=3)
        {
            bool hasSelectedEdgePoint = false;
            bool hasUnitEdgePoint = false;
            for(auto j = 0; j < 3; j++)
            {
                int pointID = (int)boundingBoxFaces[i+j];
                if(pointID == unitEdgeIDs.first || pointID == unitEdgeIDs.second)
                    hasUnitEdgePoint = true;
                if(pointID == selectedEdgeIDs.first)
                {
                    hasSelectedEdgePoint = true;
                    movablePoint = v2;
                    fixedPoint = v1;
                    movablePointId = selectedEdgeIDs.second;
                    fixedPointId = selectedEdgeIDs.first;
                }
                if(pointID == selectedEdgeIDs.second)
                {
                    hasSelectedEdgePoint = true;
                    movablePoint = v1;
                    fixedPoint = v2;
                    movablePointId = selectedEdgeIDs.first;
                    fixedPointId = selectedEdgeIDs.second;
                }
            }

            if(hasSelectedEdgePoint && hasUnitEdgePoint)
                return true;
        }
    }

    qDebug() << "Something is wrong as end of findMovablePoint function";
    //function shouldn't get to this point as all conditions are tested above
    return false;
}


auto Mesh::propagateBoundingBoxEdit(int selectedBoxId, int movedVertexIndex, int fixedVertexIndex, trimesh::point newPoint) -> void
{
    auto toMovePoint = bBox[movedVertexIndex];
    auto fixedPoint = bBox[fixedVertexIndex];

    auto edgeDirection = toMovePoint - fixedPoint;
    Eigen::Vector3f normal(edgeDirection[0], edgeDirection[1], edgeDirection[2]);
    normal.normalize();
    Eigen::Hyperplane<float, 3> fixedPlane(normal, Eigen::Vector3f(fixedPoint[0], fixedPoint[1], fixedPoint[2]));
    Eigen::Hyperplane<float, 3> movedPlane(normal, Eigen::Vector3f(newPoint[0], newPoint[1], newPoint[2]));

    //qDebug() << "new point " << newPoint[0] << " " << newPoint[1] << " " << newPoint[2];
    auto start_index = selectedBoxId * 8;
    for(auto i = start_index; i < start_index+8; i++)
    {
        auto point = Eigen::Vector3f(bBox[i][0], bBox[i][1], bBox[i][2]);
        //If the point isn't on the fixed plane
        if(std::fabs(fixedPlane.signedDistance(point)) > 0.1e-5)
        {
            Eigen::Vector3f p = movedPlane.projection(point);
            bBox[i] = trimesh::point(p[0], p[1], p[2]);
            //qDebug() << "on plane " << p[0] << " " << p[1] << " " << p[2];
        }
    }
}

auto Mesh::objectSpacePointFromSSLength(QMatrix4x4 viewport, QMatrix4x4 proj, QMatrix4x4 view, QMatrix4x4 model, QVector4D selectedEdgeFixedPoint, QVector4D selectedEdgeMovablePoint, QVector2D fixedEdgeSSVector, float newRatioLength) -> QVector4D
{
    auto mvp = proj * view * model;
    //qDebug() << viewport;
    auto ss1 = mvp * selectedEdgeFixedPoint;//selectedEdgeMovablePoint;
    ss1 /= ss1[3];

    auto ss2 = mvp * selectedEdgeMovablePoint;
    ss2 /= ss2[3];

    auto ttemp = QVector2D(ss2[0], ss2[1]) - QVector2D(ss1[0], ss1[1]);
    ttemp.normalize();
    //    qDebug() << "normalised edge to move " << ttemp;

    ss1 = viewport * ss1;
    ss2 = viewport * ss2;

    //auto mvpInv = mvp.inverted();
    auto projInv = proj.inverted();
    auto viewInv = view.inverted();
    auto modelInv = model.inverted();

    auto edgeToMoveScreen = QVector2D(ss2[0], ss2[1]) - QVector2D(ss1[0], ss1[1]) ;
    edgeToMoveScreen.normalize();
    qDebug() << "edgeToMoveScreen " << edgeToMoveScreen;
    qDebug() << "fixedEdgeSSVector.length()  " << fixedEdgeSSVector.length();
    qDebug() << "newRatioLength  " << newRatioLength;
    qDebug() << "fixedEdgeSSVector.length() * newRatioLength" << fixedEdgeSSVector.length() * newRatioLength;
    qDebug() << "edgeToMoveScreen.normalize() " << edgeToMoveScreen.length();
    auto ray_clip2D = QVector2D(ss1[0], ss1[1]) +  (edgeToMoveScreen * fixedEdgeSSVector.length() * newRatioLength);


    auto new_vector = QVector2D(ss1[0], ss1[1]) - ray_clip2D;

    qDebug() << "new edge length " << new_vector.length();

    auto viewportInv = viewport.inverted();
    auto ray_clipViewport = viewportInv *  QVector4D(ray_clip2D[0], ray_clip2D[1], 0, 1);
    auto ray_clip = QVector4D(ray_clipViewport[0], ray_clipViewport[1], -1.0, 1.0);
    auto ray_eye = projInv * ray_clip;

    auto ray_world = QVector3D(ray_eye[0], ray_eye[1], -1.0);
    ray_world.normalize();

    auto p1 = CGAL::Point_3<K>(0, 0, 0);
    auto p2 = CGAL::Point_3<K>(ray_world[0], ray_world[1], ray_world[2]);

    CGAL::Line_3<K> ray(p1, p2);

    auto v1World = (view * model) * selectedEdgeFixedPoint;
    auto v2World = (view * model) * selectedEdgeMovablePoint;

    auto p3 = CGAL::Point_3<K>(v1World[0], v1World[1], v1World[2]);
    auto p4 = CGAL::Point_3<K>(v2World[0], v2World[1], v2World[2]);
    CGAL::Line_3<K> boxEdge(p3, p4);

    auto distance = CGAL::squared_distance(ray, boxEdge);

    if(std::sqrt(distance) < 0.1e-5)
    {
        auto point = closestPoint(ray, boxEdge);
        auto pointObject = (modelInv * viewInv) * QVector4D(point.x(), point.y(), point.z(), 1);
        return pointObject;
    }
    else
    {
        qDebug() << "Error can't find point";
        return QVector4D(-1,-1,-1,-1);
    }
}




auto Mesh::manualAdjustBoundingBox(QMatrix4x4 viewport, QMatrix4x4 proj, QMatrix4x4 view, QMatrix4x4 model, int boxId, int unitEdgeId, int selectedEdgeId, float newRatioLength) -> void
{
    //qDebug() << "newRatio " << newRatioLength;
    if(unitEdgeId == selectedEdgeId)
    {
        qDebug() << "Unit edge and selected edge are the same, cannot adjust length";
        return;
    }

    auto mvp = proj * view * model;

    //Screen space vector for the fixed edge
    QVector2D fixedEdgeSSVector = screenSpaceVector(viewport, mvp, boxId, unitEdgeId);
    //From the selected edge find, if possible, the point that should be moved
    QVector4D selectedEdgeFixedPoint;
    QVector4D selectedEdgeMovablePoint;
    int movablePointId;
    int fixedPointId;
    if(!findMovablePoint(movablePointId, fixedPointId, selectedEdgeMovablePoint, selectedEdgeFixedPoint, boxId, unitEdgeId, selectedEdgeId))
    {
        qDebug() << "Cannot adjust edge as is parallel to unit edge";
        return;
    }

    auto pointObject = objectSpacePointFromSSLength(viewport, proj, view, model, selectedEdgeFixedPoint, selectedEdgeMovablePoint, fixedEdgeSSVector, newRatioLength);
    //pointObject / pointObject[3];
    auto newPoint = trimesh::point(pointObject.x(), pointObject.y(), pointObject.z());
    //        auto newSS = mvp * QVector4D(pointObject.x(), pointObject.y(), pointObject.z(), 1);
    //        newSS /= newSS[3];
    //        auto new_vector2 = QVector2D(ss1[0], ss1[1]) - QVector2D(newSS[0], newSS[1]);
    //qDebug() << "new_vector2 length " << new_vector2.length();

    propagateBoundingBoxEdit(boxId, movablePointId, fixedPointId, newPoint);


}


auto makeAbsolute(QVector4D& vec) -> void
{
    for(auto i = 0; i < 4; i++)
    {
        vec[i] = std::fabs(vec[i]);
        if(vec[i] < 1.0e-5)
        {
            vec[i] = 0;
        }

    }
    // vec = QVector4D(), std::fabs(vec[1]), std::fabs(vec[2]),  std::fabs(vec[3]));
}


auto Mesh::leastSquaresFixedEdge(QMatrix4x4 viewport, QMatrix4x4 proj, QMatrix4x4 view, QMatrix4x4 model, int boxId, int unitEdgeId, std::vector<std::pair<int, float>>& desiredEdgeRatios) -> void
{
    qDebug() << "boxId " << boxId;
    qDebug() << "unitEdgeId " << unitEdgeId;
    auto mvp = proj * view * model;
    QVector2D fixedEdgeSSVector = screenSpaceVector(viewport, mvp, boxId, unitEdgeId);

    qDebug() << "fixedEdgeSSVector.length() " << fixedEdgeSSVector.length();

    Eigen::Quaternion<float> q_rot = bbXRot * bbYRot * bbZRot;
    Eigen::Matrix3f rotation = q_rot * Eigen::Matrix3f::Identity();
    Eigen::Matrix3f rotationInv = rotation.inverse();

    int  index = boxId * bb_faces_count + unitEdgeId;
    auto fixedEdgeVertexIDs = edgeToVertices[index];
    auto fixedA = bBox[fixedEdgeVertexIDs.first];
    auto fixedB = bBox[fixedEdgeVertexIDs.second];

    //    auto fixedV1Original = QVector4D(fixedA[0], fixedA[1], fixedA[2], 1);

    //    //This is needed if the bounding box isn't axis aligned
    //    Eigen::Vector3f a1 = rotationInv * Eigen::Vector3f(fixedA[0], fixedA[1], fixedA[2]);
    //    Eigen::Vector3f b1 = rotationInv * Eigen::Vector3f(fixedB[0], fixedB[1], fixedB[2]);

    //    auto fixedV1 = QVector4D(a1[0], a1[1], a1[2], 1);
    //    auto fixedV2 = QVector4D(b1[0], b1[1], b1[2], 1);

    auto fixedV1 = QVector4D(fixedA[0], fixedA[1], fixedA[2], 1);
    auto fixedV2 = QVector4D(fixedB[0], fixedB[1], fixedB[2], 1);

    auto knownDimension = (fixedV1 - fixedV2);//.normalized();
    //auto knownDimensionLength = knownDimension.length();
    //qDebug() << "knownDimension " << knownDimension;
    //qDebug() << "knownDimensionLength " << knownDimensionLength;
    knownDimension.normalize();
    makeAbsolute(knownDimension);

    //These don't actually correspond to W H L but for the
    //formulation and intuition this makes the most sense
    int unknownWIndex = -1;
    int unknownHIndex = -1;
    int knownLIndex = -1;
    for(auto i = 0; i < 3; i++)
    {
        if(knownDimension[i] == 1)
            knownLIndex = i;
        else if(unknownWIndex == -1 && knownDimension[i] == 0)
            unknownWIndex = i;
        else if(unknownHIndex == -1 && knownDimension[i] == 0)
            unknownHIndex = i;
    }

    Eigen::Matrix2f A = Eigen::Matrix2f::Zero();
    Eigen::Vector2f b = Eigen::Vector2f::Zero();

    for(auto edgeRatio : desiredEdgeRatios)
    {
        //qDebug() << "edgeRatio.first " << edgeRatio.first;
        //        auto edgeVertices = edgeToVertices[edgeRatio.first];
        //        auto vertexA = QVector4D(bBox[edgeVertices.first][0], bBox[edgeVertices.first][1], bBox[edgeVertices.first][2], 1);
        //        auto vertexB = QVector4D(bBox[edgeVertices.second][0], bBox[edgeVertices.second][1], bBox[edgeVertices.second][2], 1);

        QVector4D selectedEdgeFixedPoint;
        QVector4D selectedEdgeMovablePoint;
        int movablePointId;
        int fixedPointId;
        if(!findMovablePoint(movablePointId, fixedPointId, selectedEdgeMovablePoint, selectedEdgeFixedPoint, boxId, unitEdgeId, edgeRatio.first, true))
        {
            qDebug() << "Cannot adjust edge as is parallel to unit edge";
            return;
        }

        auto direction = (fixedV1 - selectedEdgeMovablePoint);
        direction.normalize();
        //qDebug() << "direction " << direction;

        auto p = objectSpacePointFromSSLength(viewport, proj, view, model, selectedEdgeFixedPoint, selectedEdgeMovablePoint, fixedEdgeSSVector, edgeRatio.second);

        //        Eigen::Vector3f p2 = rotationInv * Eigen::Vector3f(p1[0], p1[1], p1[2]);

        //        auto p = QVector3D(p2[0], p2[1], p2[2]);

        //qDebug() << "Target point " << p;

        if(direction[unknownWIndex] != 0)
        {
            A(0,0)++;
            b(0) += p[unknownWIndex] - fixedV1[unknownWIndex];
            //qDebug() << "p[unknownWIndex] - fixedV1[unknownWIndex] " << p[unknownWIndex] << " - " << fixedV1[unknownWIndex] << " = " << p[unknownWIndex] - fixedV1[unknownWIndex];

        }
        if(direction[unknownHIndex] != 0)
        {
            A(1,1)++;
            b(1) += p[unknownHIndex] - fixedV1[unknownHIndex];
            //qDebug() << "p[unknownHIndex] - fixedV1[unknownHIndex] " << p[unknownHIndex] << " - " << fixedV1[unknownHIndex] << " = "<< p[unknownHIndex] - fixedV1[unknownHIndex];

        }

    }

    //    qDebug() << "A(0,0) " << A(0,0);
    //    qDebug() << "A(1,1) " << A(1,1);
    //    qDebug() << "b " << b(0) << " " << b(1);

    Eigen::Vector2f x = A.colPivHouseholderQr().solve(b);

    //qDebug() << "x " << x(0) << " " << x(1);
    bool aMoved = false;
    bool bMoved = false;

    for(auto edgeRatio : desiredEdgeRatios)
    {
        QVector4D selectedEdgeFixedPoint;
        QVector4D selectedEdgeMovablePoint;
        int movablePointId;
        int fixedPointId;
        if(!findMovablePoint(movablePointId, fixedPointId, selectedEdgeMovablePoint, selectedEdgeFixedPoint, boxId, unitEdgeId, edgeRatio.first, true))
        {
            qDebug() << "Cannot adjust edge as is parallel to unit edge";
            return;
        }

        auto direction = (fixedV1 - selectedEdgeMovablePoint);
        direction.normalize();
        //qDebug() << "direction " << direction;

        auto newPoint = trimesh::point(0,0,0);
        newPoint[knownLIndex] = fixedV1[knownLIndex];
        newPoint[unknownWIndex] = fixedV1[unknownWIndex];
        newPoint[unknownHIndex] = fixedV1[unknownHIndex];

        if(direction[unknownWIndex] != 0)
        {
            aMoved = true;
            newPoint[unknownWIndex] += x(0);
        }
        if(direction[unknownHIndex] != 0)
        {
            aMoved = true;
            newPoint[unknownHIndex] += x(1);
        }
        //qDebug() << "newPoint " << newPoint[0] << " " << newPoint[1] << " " << newPoint[2];
        propagateBoundingBoxEdit(boxId, movablePointId, fixedPointId, newPoint);

        if(aMoved && bMoved  )
            break;
    }

}

auto Mesh::ssPoint(QMatrix4x4 viewport, QMatrix4x4 proj, QMatrix4x4 view, QMatrix4x4 model, QVector4D& pt) -> QVector2D
{
    auto mvp = proj * view * model;
    auto ssPt = mvp * pt;
    ssPt /= ssPt[3];
    ssPt = viewport * ssPt;
    return QVector2D(ssPt[0], ssPt[1]);
}


auto Mesh::searchToFitScreenSpaceLength(QMatrix4x4 viewport, QMatrix4x4 proj, QMatrix4x4 view, QMatrix4x4 model, QVector4D& ptA, QVector4D& ptB, QVector2D fixedEdgeSSVector, float ssTargetRatio) -> void
{

    qDebug() << "ssTargetRatio " << ssTargetRatio;
    auto ssTargetLength = fixedEdgeSSVector.length() * ssTargetRatio;//roundf( * 100) / 100);
    qDebug() << "rounded ssTargetLength " << ssTargetLength;

    auto ssPtA = ssPoint(viewport, proj, view, model, ptA);
    auto ssPtB = ssPoint(viewport, proj, view, model, ptB);
    auto ssEdge = ssPtA - ssPtB;
    auto osEdge = ptA - ptB;

    auto osEdgeNormalized = osEdge.normalized();
    makeAbsolute(osEdgeNormalized);

    auto unknownIndex = -1;
    for(auto i = 0; i < 3; i++)
        if(osEdgeNormalized[i] == 1)
            unknownIndex = i;

    if(unknownIndex == -1)
    {
        qDebug() << "Couldn't find known index";
        return;
    }

    auto osMid = (ptA + ptB) * 0.5;

    auto ssEdgeLength = ssEdge.length();//std::fabs(roundf(ssEdge.length() * 100) / 100);

    qDebug() << "ssEdgeLength " << ssEdgeLength;
    qDebug() << "ratio " << ssEdgeLength / fixedEdgeSSVector.length();

    auto ptAMove = 0.0f;
    auto ptBMove = 0.0f;

    if(std::fabs(ssEdgeLength - ssTargetLength) < 0.001)
    {
        return;
    }

    else if(ssEdgeLength > ssTargetLength)
    { // Make point closer
        ptAMove = (ptA[unknownIndex] > osMid[unknownIndex]) ? -0.01 : 0.01;
        ptBMove = (ptB[unknownIndex] > osMid[unknownIndex]) ? -0.01 : 0.01;
    }
    else if(ssEdgeLength < ssTargetLength)
    { // Made point further appart
        ptAMove = (ptA[unknownIndex] > osMid[unknownIndex]) ? 0.01 : -0.01;
        ptBMove = (ptB[unknownIndex] > osMid[unknownIndex]) ? 0.01 : -0.01;
    }

    qDebug() << "ptA " << ptA;
    qDebug() << "ptB " << ptB;
    ptA[unknownIndex] += ptAMove;
    ptB[unknownIndex] += ptBMove;
    qDebug() << "after ptA " << ptA;
    qDebug() << "after ptB " << ptB;
    searchToFitScreenSpaceLength(viewport, proj, view, model, ptA, ptB, fixedEdgeSSVector, ssTargetRatio);

}





auto Mesh::leastSquaresAllEdges(QMatrix4x4 viewport, QMatrix4x4 proj, QMatrix4x4 view, QMatrix4x4 model, int boxId, int unitEdgeId, int selectedBoxId, std::vector<std::pair<int, float>>& desiredEdgeRatios) -> void
{
    auto mvp = proj * view * model;
    qDebug() << "boxId " << boxId;
    qDebug() << "unitEdgeId " << unitEdgeId;
    QVector2D fixedEdgeSSVector = screenSpaceVector(viewport, mvp, boxId, unitEdgeId);
    qDebug() << "fixedEdgeSSVector.length() " << fixedEdgeSSVector.length();
    //    auto c = trimesh::point(0,0,0);
    //    auto start = selectedBoxId * 8;
    //    for(auto i = start; i < start+8; i++)
    //    {
    //        c+=bBox[i];
    //    }
    //    auto centroid = QVector3D(c[0]/8.0, c[1]/8.0, c[2]/8.0);

    auto knownDimension = QVector3D(1,1,1);
    auto noUnknowns = (int)0;
    for(auto edgeRatio : desiredEdgeRatios)
    {
        Eigen::Quaternion<float> q_rot = bbXRot * bbYRot * bbZRot;
        Eigen::Matrix3f rotation = q_rot * Eigen::Matrix3f::Identity();
        Eigen::Matrix3f rotationInv = rotation.inverse();

        //int  index = selectedBoxId * bb_faces_count + edgeRatio.first;
        auto fixedEdgeVertexIDs = edgeToVertices[edgeRatio.first];
        auto fixedA = bBox[fixedEdgeVertexIDs.first];
        auto fixedB = bBox[fixedEdgeVertexIDs.second];

        //This is needed if the bounding box isn't axis aligned
        Eigen::Vector3f a1 = rotationInv * Eigen::Vector3f(fixedA[0], fixedA[1], fixedA[2]);
        Eigen::Vector3f b1 = rotationInv * Eigen::Vector3f(fixedB[0], fixedB[1], fixedB[2]);

        auto fixedV1 = QVector4D(a1[0], a1[1], a1[2], 1);
        auto fixedV2 = QVector4D(b1[0], b1[1], b1[2], 1);

        auto edge = (fixedV1 - fixedV2);//.normalized();
        //auto knownDimensionLength = knownDimension.length();
        //qDebug() << "knownDimension " << knownDimension;
        //qDebug() << "knownDimensionLength " << knownDimensionLength;

        //qDebug() << "edge " << edge;

        edge.normalize();
        makeAbsolute(edge);

        //qDebug() << "edge " << edge;

        for(auto i = 0; i < 3; i++)
        {
            if(edge[i] == 1 && knownDimension[i] != 0)
            {
                knownDimension[i] = 0;
                noUnknowns++;
            }
        }
    }

    //qDebug() << "noUnknowns " << noUnknowns;
    if(noUnknowns == 0)
    {
        qDebug() << "No Unknowns";
        return;

    }


    Eigen::Matrix3f A = Eigen::Matrix3f::Zero();
    Eigen::Vector3f b = Eigen::Vector3f::Zero();

    int unknownIndexes[noUnknowns];
    int count = 0;
    for(auto i = 0; i < 3; i++)
    {
        if(knownDimension[i] == 0)
        {
            //qDebug() << "unknownIndexes[ " << count << " ] = " << i;
            unknownIndexes[count] = i;
            count++;
        }
    }

    //Setup A and b
    for(auto edgeRatio : desiredEdgeRatios)
    {

        //qDebug() << "edgeRatio.first " << edgeRatio.first;
        auto vertexIDs = edgeToVertices[edgeRatio.first];
        auto pt1 = bBox[vertexIDs.first];
        auto pt2 = bBox[vertexIDs.second];
        auto ptA = QVector4D(pt1[0], pt1[1], pt1[2], 1);
        auto ptB = QVector4D(pt2[0], pt2[1], pt2[2], 1);


        auto direction = (ptA - ptB);
        direction.normalize();
        makeAbsolute(direction);

        //        qDebug() << "direction " << direction;
        //        qDebug() << "ptB " << ptB;
        //        qDebug() << "ptA " << ptA;

        auto mid = (ptA + ptB) * 0.5;

        //qDebug() << "mid " << mid;

        // qDebug() << "edgeRatio.second " << edgeRatio.second;
        //        auto Mesh::objectSpacePointFromSSLength(QMatrix4x4 viewport, QMatrix4x4 proj, QMatrix4x4 view, QMatrix4x4 model, QVector4D selectedEdgeFixedPoint, QVector4D selectedEdgeMovablePoint, QVector2D fixedEdgeSSVector, float newRatioLength) -> QVector4D


        //       auto naiveNewPtA = objectSpacePointFromSSLength(viewport, proj, view, model, mid, ptA, fixedEdgeSSVector, edgeRatio.second * 0.5);
        //       auto naiveNewPtB = objectSpacePointFromSSLength(viewport, proj, view, model, mid, ptB, fixedEdgeSSVector, edgeRatio.second * 0.5);

        //        qDebug() << "naiveNewPtA " << naiveNewPtA;
        //        qDebug() << "naiveNewPtB " << naiveNewPtB;

        //        auto ssNaiveNewPtA = ssPoint(viewport, proj, view, model, naiveNewPtA);
        //        auto ssNaiveNewPtB = ssPoint(viewport, proj, view, model, naiveNewPtB);
        //        qDebug() << "ssNaiveNewPtA " << ssNaiveNewPtA;
        //        qDebug() << "ssNaiveNewPtB " << ssNaiveNewPtB;
        //        auto e = (ssNaiveNewPtA - ssNaiveNewPtB);
        //        qDebug() << "naive ss edge " << e;
        //        qDebug() << "naive New Edge Length " << e.length();



        // THIS IS A CRUDE SEARCH TO FIND THE NEW POINTS
        auto newPtA = mid;
        auto newPtB = mid;

        auto targetSSLength = fixedEdgeSSVector.length() * edgeRatio.second;
        auto currentSSLength = 0.0f;
        //        qDebug() << "fixedEdgeSSVector.length() " << fixedEdgeSSVector.length();
        //        qDebug() << "edgeRatio.second " << edgeRatio.second;
        //        qDebug() << "targetSSLength " << targetSSLength;
        while(std::fabs(targetSSLength - currentSSLength) > 0.1f)
        {
            for(auto i = 0; i < noUnknowns; i++)
            {
                auto index = unknownIndexes[i];
                if(direction[index] != 0)
                {
                    if(ptA[index] > mid[index])
                    {
                        newPtA[index] += 0.0001;
                        newPtB[index] -= 0.0001;
                    }
                    else
                    {
                        newPtA[index] -= 0.0001;
                        newPtB[index] += 0.0001;
                    }

                }
            }
            auto ssNewPtA = ssPoint(viewport, proj, view, model, newPtA);
            auto ssNewPtB = ssPoint(viewport, proj, view, model, newPtB);
            auto edge = ssNewPtA - ssNewPtB;
            currentSSLength = edge.length();
        }


        //        qDebug() << "newPtA " << newPtA;
        //        qDebug() << "newPtB " << newPtB;
        //        qDebug() << "Target Ratio " << edgeRatio.second;
        //        qDebug() << "Target Length " <<  fixedEdgeSSVector.length() * edgeRatio.second;
        auto ssNewPtA = ssPoint(viewport, proj, view, model, newPtA);
        auto ssNewPtB = ssPoint(viewport, proj, view, model, newPtB);

        auto edge = (ssNewPtA - ssNewPtB);
        //        qDebug() << "new ss edge " << edge;
        //        qDebug() << "New Edge Length " << edge.length();


        //searchToFitScreenSpaceLength(viewport, proj, view, model, newPtA, newPtB, fixedEdgeSSVector, edgeRatio.second);


        //        qDebug() << "ptA " << ptA;
        //        qDebug() << "newPtA " << newPtA;
        //        qDebug() << "newPtB " << newPtB;
        for(auto i = 0; i < noUnknowns; i++)
        {
            auto index = unknownIndexes[i];

            //qDebug() << "index " << index;
            if(direction[index] != 0)
            {
                A(index,index)++;
                b(index) += newPtA[index] - newPtB[index];
                //               qDebug() << "b(index) " << b(index);
            }
        }

    }

    //    qDebug() << "A(0,0) " << A(0,0);
    //    qDebug() << "A(1,1) " << A(1,1);
    //    qDebug() << "A(2,2) " << A(2,2);
    //    qDebug() << "b " << b(0) << " " << b(1) << " " << b(2);

    Eigen::Vector3f x = A.colPivHouseholderQr().solve(b);

    //qDebug() << "x " << x(0) << " " << x(1) << " " << x(2);
    int verticesMoved = 0;

    //Move points
    for(auto edgeRatio : desiredEdgeRatios)
    {
        auto vertexIDs = edgeToVertices[edgeRatio.first];
        auto pt1 = bBox[vertexIDs.first];
        auto pt2 = bBox[vertexIDs.second];

        auto ptA = QVector4D(pt1[0], pt1[1], pt1[2], 1);
        auto ptB = QVector4D(pt2[0], pt2[1], pt2[2], 1);

        auto direction = (ptA - ptB);
        direction.normalize();
        makeAbsolute(direction);

        //        qDebug() << "ptA " << ptA;
        //        qDebug() << "ptB " << ptB;

        auto mid = (ptA + ptB) * 0.5;

        auto newPointA = trimesh::point(ptA[0], ptA[1], ptA[2]);
        auto newPointB = trimesh::point(ptB[0], ptB[1], ptB[2]);


        for(auto i = 0; i < noUnknowns; i++)
        {
            auto index = unknownIndexes[i];
            if(direction[index] != 0 )
            {
                //               qDebug() << "index " << index;
                //               qDebug() << "newPointA[index] " << newPointA[index];
                //               qDebug() << "newPointB[index] " << newPointB[index];
                auto mid2 = (newPointA[index] + newPointB[index]) * 0.5f;
                //qDebug() << "mid " << mid << " mid2 " << mid2;
                auto delta = x[index] * 0.5f;
                //qDebug() << "delta " << delta;
                newPointA[index] =  mid[index] + delta;
                newPointB[index] =  mid[index] - delta;
                //               qDebug() << "newPointA[index]' " << newPointA[index];
                //               qDebug() << "newPointB[index]' " << newPointB[index];
            }
            ////           //Don't iterate unnecessarily
            ////           if(verticesMoved == noUnknowns)
            ////               break;
        }

        //        qDebug() << "newPointA " << newPointA[0] << " " << newPointA[1] << " " << newPointA[2];
        //        qDebug() << "newPointB " << newPointB[0] << " " << newPointB[1] << " " << newPointB[2];




        propagateBoundingBoxEdit(selectedBoxId, vertexIDs.first, vertexIDs.second, newPointA);
        propagateBoundingBoxEdit(selectedBoxId, vertexIDs.second, vertexIDs.first, newPointB);

    }



}


auto Mesh::computeBoxes(BoxAdjustMethod method, QMatrix4x4 viewport, QMatrix4x4 proj, QMatrix4x4 view, QMatrix4x4 model, int boxId, int unitEdgeId, std::map<int, std::vector<std::pair<int, float>>>& desiredEdgeRatios) -> void
{

    //    qDebug() << "boxId " << boxId;
    //    qDebug() << "unitEdgeId " << unitEdgeId;
    auto mvp = proj * view * model;
    QVector2D fixedEdgeSSVector = screenSpaceVector(viewport, mvp, boxId, unitEdgeId);

    //qDebug() << " Mesh::leastSquaresBox fixedEdgeSSVector.length() " << fixedEdgeSSVector.length();

    //See if unit length edge box needs to be adjusted
    for(auto it = desiredEdgeRatios.begin(); it != desiredEdgeRatios.end();)
    {
        //The unit length edge contrains the box
        if(it->first == boxId)
        {
            //Adjust the box
            switch(method)
            {
            case BoxAdjustMethod::LEAST_SQUARES:
                leastSquaresFixedEdge(viewport, proj, view, model, boxId, unitEdgeId, it->second);
                break;
            }

            desiredEdgeRatios.erase(it);
            break;
        }
        else
        {
            it++;
        }
    }

    //NEED METHOD FOR SELECTING THE NEXT BOX, CURRENTLY JUST GOING TO THE NEXT IN THE MAP!

    for(auto boxIds : desiredEdgeRatios)
    {
        auto minDist = 9999.99f;
        auto edgeId = 0;
        auto targetRatio = 0.0f;
        auto edgeToRemove = boxIds.second.begin();
        for(auto it = boxIds.second.begin(); it != boxIds.second.end(); it++)
        {
            auto vertexIDs = edgeToVertices[it->first];
            auto pt1 = bBox[vertexIDs.first];
            auto pt2 = bBox[vertexIDs.second];
            auto ptA = QVector4D(pt1[0], pt1[1], pt1[2], 1);
            auto ptB = QVector4D(pt2[0], pt2[1], pt2[2], 1);

            auto ssPtA = ssPoint(viewport, proj, view, model, ptA);
            auto ssPtB = ssPoint(viewport, proj, view, model, ptB);

            auto ssEdge = ssPtA - ssPtB;

            auto ssLength = ssEdge.length();
            auto targetLength = fixedEdgeSSVector.length() * it->second;

            auto difference = std::fabs(ssLength - targetLength);
            if(difference < minDist)
            {
                minDist = difference;
                edgeId = it->first;
                targetRatio = it->second;
                edgeToRemove = it;
            }
        }


        qDebug() << "1 boxIds.second.size() " << boxIds.second.size();
        boxIds.second.erase(edgeToRemove);
        qDebug() << "2 boxIds.second.size() " << boxIds.second.size();

        // Manually move both points of the edge to be the desiered length
        auto newRatio = std::make_pair(edgeId, targetRatio);
        auto  newRatios = std::vector<std::pair<int, float>>();
        newRatios.push_back(newRatio);

        switch(method)
        {
        case BoxAdjustMethod::LEAST_SQUARES:
            //This could actually be a much simpler method but as only one ratio LS solution is equivalent
            leastSquaresAllEdges(viewport, proj, view, model, boxId, unitEdgeId, boxIds.first, newRatios);
            break;
        }

        qDebug() << "new fixed target ratio " << targetRatio;

        //Update new edge lengths, relative to the new fixed edge
        for(auto& edgeRatioPair : boxIds.second)
        {
            qDebug() << "old ratio " << edgeRatioPair.second;
            edgeRatioPair.second = edgeRatioPair.second / targetRatio;
            qDebug() << "new ratio " << edgeRatioPair.second;
        }

        for(auto edgeRatioPair : boxIds.second)
        {
            qDebug() << "new ratio " << edgeRatioPair.second;
            //            edgeRatioPair.second = edgeRatioPair.second / targetRatio;
            //            qDebug() << "new ratio " << edgeRatioPair.second;
        }

        int fixedEdgeIndex = newRatio.first - (boxIds.first * bb_faces_count);

        qDebug() << "new fixed box " << boxIds.first;
        qDebug() << "new fixed edge " << fixedEdgeIndex;


        switch(method)
        {
        case BoxAdjustMethod::LEAST_SQUARES:
            //Use thew fixed edge as new unit edge
            leastSquaresFixedEdge(viewport, proj, view, model, boxIds.first, fixedEdgeIndex, boxIds.second);
            break;
        }

    }


}

//This could be used to find what direction it is moving
//auto edgeDimention = (vertexA - vertexB);
//edgeDimention.normalize();
//makeAbsolute(edgeDimention);
////We ignore contraints that conflict with the unit length edge
//if(edgeDimention != knownDimension)
//{
//    qDebug() << "edgeDimention " << edgeDimention;
//    //From the selected edge find, if possible, the point that should be moved

//}

auto Mesh::minWidthBoundingBox(Eigen::Vector3f& maxP, Eigen::Vector3f& minP, Eigen::Matrix3f& eigTransposed, Eigen::Matrix3f& eigTransposedInv) -> void
{
    Polygon_2 p;
    float min_y = trimesh->vertices[0][1], max_y = trimesh->vertices[0][1];
    Point mean = Point(0,0);
    for(auto i = size_t(0); i < size_t(vSize()); i++)
    {
        auto t_pt = trimesh->vertices[i];
        p.push_back(Point(t_pt[0], t_pt[2]));
        min_y = std::min(min_y, t_pt[1]);
        max_y = std::max(max_y, t_pt[1]);
        mean = Point(t_pt[0]+mean[0], t_pt[2]+mean[1]);
    }

    mean = Point(mean[0] / vSize(), mean[1] / vSize());
    //Find convex hull
    Polygon_2 p_ch;
    CGAL::convex_hull_2(p.vertices_begin(), p.vertices_end(), std::back_inserter(p_ch));

    //Find min width strip
    std::vector<CGAL::Line_2< K >> lines;
    CGAL::min_strip_2(p_ch.vertices_begin(), p_ch.vertices_end(), std::back_inserter(lines));


    //auto meanProj = lines[0].proj(mean);

    auto p1 = p_ch.vertex(0), p2 = p_ch.vertex(0);
    auto maxDistPos = 0.0f;
    auto maxDistNeg = 0.0f;
    //    for(auto i = size_t(0); i < size_t(p_ch.size()); i++)
    //    {
    //        auto proj = lines[0].proj(p_ch.vertex(i));

    //    }

}

auto Mesh::pcaBoundingBox(Eigen::MatrixXf& points, Eigen::Vector3f& maxP, Eigen::Vector3f& minP, Eigen::Matrix3f& eigTransposed, Eigen::Matrix3f& eigTransposedInv) -> void
{
    // iglVertices is a X by 3 Eigen::::MatrixXf
    // Covariance matrix and eigen decomposition
    Eigen::MatrixXf centered = points.rowwise() - points.colwise().mean();
    Eigen::MatrixXf cov = centered.adjoint() * centered;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXf> eig(cov);

    eigTransposed = eig.eigenvectors().transpose();
    eigTransposedInv = eigTransposed.inverse();
    Eigen::Matrix<float,3,Eigen::Dynamic> newVertices = eigTransposedInv * iglVertices.transpose();

    //    for(auto i = 0; i < vSize(); i++)
    //    {
    //        Eigen::Vector3f p = newVertices.col(i);
    //        trimesh->vertices[i] = trimesh::point(p[0], p[1], p[2]);
    //    }
    //Find max and min for all of the new axis
    maxP = Eigen::Vector3f(newVertices.rowwise().maxCoeff());
    minP = Eigen::Vector3f(newVertices.rowwise().minCoeff());
}

auto Mesh::orientedBoundingBox() -> void
{
    qDebug() << "Computing oriented bounding box..";

    Eigen::Vector3f maxP;
    Eigen::Vector3f minP;
    Eigen::Matrix3f eigTransposed;
    Eigen::Matrix3f eigTransposedInv;


    eigTransposed = Eigen::Matrix3f::Identity();
    eigTransposedInv = eigTransposed.inverse();

    //    std::cout << "eigTransposed " << eigTransposed << std::endl;
    //    std::cout << "eigTransposedInv " << eigTransposedInv << std::endl;

    //    for(auto i = 0; i < vSize(); i++)
    //    {
    //        Eigen::Vector3f p = newVertices.col(i);
    //        trimesh->vertices[i] = trimesh::point(p[0], p[1], p[2]);
    //    }
    //Find max and min for all of the new axis
    maxP = Eigen::Vector3f(iglVertices.colwise().maxCoeff());
    minP = Eigen::Vector3f(iglVertices.colwise().minCoeff());

    //    std::cout << "max " << maxP << std::endl;
    //    std::cout << "min " << minP << std::endl;

    //pcaBoundingBox(iglVertices, maxP, minP, eigTransposed, eigTransposedInv);
    //minWidthBoundingBox(maxP, minP, eigTransposed, eigTransposedInv);


    bBox.clear();
    addPointsToBoundingBox(eigTransposed, minP, maxP);
    //A must for efficent implementation would be just to iterate over points
    // and find min and max for each segment but this is a ToDo if currne implementation
    // is slow
    for(auto& s : segments)
    {
        qDebug() << "s->size() " << s->size();
//        if(s->size() > 25)
//        {
            qDebug() << "yes";
            s->axisAlignedMinMax(eigTransposed, eigTransposedInv, minP, maxP);
            addPointsToBoundingBox(eigTransposed, minP, maxP);
//        }
    }

    boundingBoxesConnectivity();

    createEdgeMaps();

}



auto Mesh::updateBoundingBoxes() -> void
{
    auto newBBox = std::vector<trimesh::point>();
    for(auto i = 0; i < 8; i++)
    {
        newBBox.push_back(bBox[i]);
    }
    for(auto& s : segments)
    {
        auto box = s->getBBox();
        newBBox.insert(std::end(newBBox), std::begin(box), std::end(box));
    }
    bBox = newBBox;
}

auto Mesh::boundingBoxesConnectivity() -> void
{
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

    auto seg_no = segmentSize();

    boundingBoxFaces = (unsigned short*) std::malloc(seg_no * bb_faces_count * 3 * sizeof(unsigned short));

    for(auto i = 1; i <= seg_no; i++)
    {
        auto step = (i - 1) * 36;
        for(auto j = 0; j < bb_faces_count * 3; j++)
        {
            auto index = (j + step);
            boundingBoxFaces[index] = (unsigned short) bbIndicesTemplate[j] + ((i - 1) * 8);
        }
    }
}

//Switched is only set to two when recursively calling addOrthogonalVertices
auto Mesh::addOrthogonalVertices(int a, int b, bool switched) -> void
{

    auto it = orthogonalVerticesMap.find(a);
    //Not Found so create new empty set
    if (it == orthogonalVerticesMap.end())
    {
        std::set<int> set;
        orthogonalVerticesMap[a] = set;
    }

    //Add the edge
    orthogonalVerticesMap[a].insert(b);
    //If not already added switch them
    if(!switched)
        addOrthogonalVertices(b,a,true);
}

auto Mesh::addToEdgeMaps(int a, int b, int edgeCount, int totalEdgeCount) -> void
{
    auto edgeVertices = std::make_pair(a, b);
    edgeToVertices.emplace(std::make_pair(edgeCount, edgeVertices));
    edgeToEdgeIdMap.emplace(std::make_pair(edgeCount, totalEdgeCount));
    addOrthogonalVertices(a, b);
}

auto Mesh::createEdgeMaps() -> void
{
    edgeToVertices.clear();
    edgeToEdgeIdMap.clear();
    orthogonalVerticesMap.clear();

    auto vertices = bBoxData();

    QMatrix4x4 m_view;
    QMatrix4x4 m_proj;
    QMatrix4x4 m_model;
    m_view.setToIdentity();
    m_view.lookAt(camera_pos, QVector3D(0,0,0), QVector3D(0,1,0));
    m_model.setToIdentity();
    m_proj.perspective(45.0f, 1.0375, 0.01f, 1000.0f);

    QMatrix4x4 mvp = m_proj * m_view * m_model;

    int edgeCount = 0;
    int faceCount = 0;
    int totalEdgeCount = 0;
    for(auto j = 0; j < segmentSize(); j++)
    {
        std::set<std::pair<float, float>> edgeSet;

        for(auto i = 0; i < bb_faces_count; i++)
        {
            auto fIndex = (j * bb_faces_count + i) * 3;
            //qDebug() << "faceIndex " << fIndex;
            auto face = boundingBoxFaces[fIndex] * 3;
            auto v1 = QVector4D(vertices[face], vertices[face+1], vertices[face+2],1);
            face = boundingBoxFaces[fIndex+1]  * 3;
            auto v2 = QVector4D(vertices[face], vertices[face+1], vertices[face+2],1);
            face = boundingBoxFaces[fIndex+2]  * 3;
            auto v3 = QVector4D(vertices[face], vertices[face+1], vertices[face+2],1);
            auto ss1 = mvp * v1;
            ss1 /= ss1[3];
            auto ss2 = mvp * v2;
            ss2 /= ss2[3];
            auto ss3 = mvp * v3;
            ss3 /= ss3[3];

            auto edge1 = QVector2D(ss1[0], ss1[1]) - QVector2D(ss2[0], ss2[1]);
            auto edge2 = QVector2D(ss2[0], ss2[1]) - QVector2D(ss3[0], ss3[1]);
            auto edge3 = QVector2D(ss3[0], ss3[1]) - QVector2D(ss1[0], ss1[1]);

            auto edge1_4d = v1 - v2;
            auto edge1_3d = QVector3D(edge1_4d[0], edge1_4d[1], edge1_4d[2]);
            auto edge2_4d = v2 - v3;
            auto edge2_3d = QVector3D(edge2_4d[0], edge2_4d[1], edge2_4d[2]);
            auto edge3_4d = v3 - v1;
            auto edge3_3d = QVector3D(edge3_4d[0], edge3_4d[1], edge3_4d[2]);

            //qDebug() << fabs(QVector3D::dotProduct(edge1_3d, edge2_3d));
            if(fabs(QVector3D::dotProduct(edge1_3d, edge2_3d)) < 1.0e-7)
            {
                auto it = edgeSet.find(std::pair<float, float>(fabs(edge1[0]), fabs(edge1[1])));
                if(it == edgeSet.end())
                {
                    edgeSet.insert(std::pair<float, float>(fabs(edge1[0]), fabs(edge1[1])));
                    auto a = (int)boundingBoxFaces[faceCount];
                    auto b = (int)boundingBoxFaces[faceCount + 1];
                    addToEdgeMaps(a, b, edgeCount, totalEdgeCount);
                    edgeCount++;
                }
                totalEdgeCount++;
                it = edgeSet.find(std::pair<float, float>(fabs(edge2[0]), fabs(edge2[1])));
                if(it == edgeSet.end())
                {
                    edgeSet.insert(std::pair<float, float>(fabs(edge2[0]), fabs(edge2[1])));

                    auto a = (int)boundingBoxFaces[faceCount + 1];
                    auto b = (int)boundingBoxFaces[faceCount + 2];
                    addToEdgeMaps(a, b, edgeCount, totalEdgeCount);
                    edgeCount++;

                }
                totalEdgeCount++;
            }
            //qDebug() << fabs(QVector3D::dotProduct(edge2_3d, edge3_3d));
            if(fabs(QVector3D::dotProduct(edge2_3d, edge3_3d)) < 1.0e-7)
            {
                auto it = edgeSet.find(std::pair<float, float>(fabs(edge2[0]), fabs(edge2[1])));
                if(it == edgeSet.end())
                {
                    edgeSet.insert(std::pair<float, float>(fabs(edge2[0]), fabs(edge2[1])));

                    auto a = (int)boundingBoxFaces[faceCount + 1];
                    auto b = (int)boundingBoxFaces[faceCount + 2];
                    addToEdgeMaps(a, b, edgeCount, totalEdgeCount);
                    edgeCount++;
                }
                totalEdgeCount++;
                it = edgeSet.find(std::pair<float, float>(fabs(edge3[0]), fabs(edge3[1])));
                if(it == edgeSet.end())
                {
                    edgeSet.insert(std::pair<float, float>(fabs(edge3[0]), fabs(edge3[1])));

                    auto a = (int)boundingBoxFaces[faceCount + 2];
                    auto b = (int)boundingBoxFaces[faceCount];
                    addToEdgeMaps(a, b, edgeCount, totalEdgeCount);
                    edgeCount++;
                }
                totalEdgeCount++;
            }
            //qDebug() << fabs(QVector3D::dotProduct(edge3_3d, edge1_3d));
            if(fabs(QVector3D::dotProduct(edge3_3d, edge1_3d)) < 1.0e-7)
            {

                auto it = edgeSet.find(std::pair<float, float>(fabs(edge3[0]), fabs(edge3[1])));
                if(it == edgeSet.end())
                {
                    edgeSet.insert(std::pair<float, float>(fabs(edge3[0]), fabs(edge3[1])));
                    auto a = (int)boundingBoxFaces[faceCount + 2];
                    auto b = (int)boundingBoxFaces[faceCount];
                    addToEdgeMaps(a, b, edgeCount, totalEdgeCount);
                    edgeCount++;
                }
                totalEdgeCount++;
                it = edgeSet.find(std::pair<float, float>(fabs(edge1[0]), fabs(edge1[1])));
                if(it == edgeSet.end())
                {
                    edgeSet.insert(std::pair<float, float>(fabs(edge1[0]), fabs(edge1[1])));
                    auto a = (int)boundingBoxFaces[faceCount];
                    auto b = (int)boundingBoxFaces[faceCount + 1];
                    addToEdgeMaps(a, b, edgeCount, totalEdgeCount);
                    edgeCount++;
                }
                totalEdgeCount++;
            }
            faceCount+=3;

        }
        //qDebug() << "segment " << j << " edge map size " <<  edgeToEdgeIdMap.size();
    }
}

auto Mesh::edgeToEdgeId(int boxId, int edgeId) -> int
{
    int index = boxId * bb_faces_count + edgeId;
    auto vertexMapped = edgeToEdgeIdMap[index];
    return vertexMapped;
}

auto Mesh::screenSpaceEdgeLength(QMatrix4x4 viewport, QMatrix4x4 mvp, int boxId, int edgeId) -> float
{
    int index = boxId * bb_faces_count + edgeId;
    auto vertexIDs = edgeToVertices[index];
    auto a = bBox[vertexIDs.first];
    auto b = bBox[vertexIDs.second];
    auto v1 = QVector4D(a[0], a[1], a[2],1);
    auto v2 = QVector4D(b[0], b[1], b[2],1);
    auto ss1 = mvp * v1;
    ss1 /= ss1[3];
    ss1 = viewport * ss1;
    auto ss2 = mvp * v2;
    ss2 /= ss2[3];
    ss2 = viewport * ss2;
    auto edge1 = QVector2D(ss1[0], ss1[1]) - QVector2D(ss2[0], ss2[1]);
    return edge1.length();
}

auto Mesh::rotateBoundingBoxXAxis(double val) -> void
{;
    bbXRot = Eigen::AngleAxisf(val, Eigen::Vector3f::UnitX());
    rotateBoundingBox();
}

auto Mesh::rotateBoundingBoxYAxis(double val) -> void
{
    bbYRot = Eigen::AngleAxisf(val, Eigen::Vector3f::UnitY());
    rotateBoundingBox();
}

auto Mesh::rotateBoundingBoxZAxis(double val) -> void
{
    bbZRot = Eigen::AngleAxisf(val, Eigen::Vector3f::UnitZ());
    rotateBoundingBox();
}

//This is very hacky, only this was because the feature is a hack until we
//come up with the best bounding box approach
auto Mesh::rotateBoundingBox() -> void
{

    Eigen::Vector3f maxP;
    Eigen::Vector3f minP;
    Eigen::Matrix3f eigTransposed;
    Eigen::Matrix3f eigTransposedInv;


    Eigen::Quaternion<float> rot = bbXRot * bbYRot * bbZRot;

    eigTransposed = rot * Eigen::Matrix3f::Identity();
    eigTransposedInv = eigTransposed.inverse();

    Eigen::Matrix<float,3,Eigen::Dynamic> newVertices = eigTransposedInv * iglVertices.transpose();


    maxP = Eigen::Vector3f(newVertices.rowwise().maxCoeff());
    minP = Eigen::Vector3f(newVertices.rowwise().minCoeff());



    bBox.clear();
    addPointsToBoundingBox(eigTransposed, minP, maxP);
    //A must for efficent implementation would be just to iterate over points
    // and find min and max for each segment but this is a ToDo if currne implementation
    // is slow
    for(auto& s : segments)
    {
        //qDebug() << "s->size() " << s->size();
//        if(s->size() > 25)
//        {
            //qDebug() << "yes";
            s->axisAlignedMinMax(eigTransposed, eigTransposedInv, minP, maxP);
            addPointsToBoundingBox(eigTransposed, minP, maxP);
//        }
    }

    //meshSegmentCluster.setBBoxRotations(eigTransposed, eigTransposedInv);

    int segCount = 0;
    for(auto& s : segments)
    {
        auto bb = s->getBBox();
        segCount++;
        qDebug() << "Segment " << segCount;
        for(auto& point : bb)
        {
            qDebug() << point[0] << ", " << point[1] << ", " << point[2];
        }
    }

//        meshSegmentCluster.updateBoundingBoxes();

    createEdgeMaps();
}



//auto Mesh::setGLMatrices(QMatrix4x4* proj, QMatrix4x4* view, QMatrix4x4* model) -> void
//{
//    m_proj = proj;
//    m_view = view;
//    m_model = model;
//}

#endif // MESH_CPP
