#ifndef SEGMENTSGRAPH_CPP
#define SEGMENTSGRAPH_CPP

#include "segmentsgraph.h"
#include <qdebug>

SegmentsGraph::SegmentsGraph(QObject *parent) : QObject(parent)
{

}


auto SegmentsGraph::loadSegments(QString dir, QJsonArray jsonArray, trimesh::point mean, float max_coeff) -> void
{
    for(auto it = jsonArray.begin(); it != jsonArray.end(); it++)
    {

//        qDebug() << dir;
//        qDebug() << it->toString();

        auto meshFilename = dir;

        meshFilename = meshFilename.append(it->toString());
        MeshSegment* meshSegment = new MeshSegment();

        meshSegment->setName(it->toString());
        //Load Mesh Segment
         auto trimesh = trimesh::TriMesh::read(meshFilename.toUtf8().constData());
         //Resize points and add points to mesh segment
         for(auto i = 0; i < (int)trimesh->vertices.size(); i++)
         {
             trimesh->vertices[i] -= mean;
             trimesh->vertices[i] /= max_coeff;
             auto p = trimesh->vertices[i];

             p[0] = trimesh->vertices[i][0];
             p[1] = trimesh->vertices[i][1];
             p[2] = trimesh->vertices[i][2];
             meshSegment->addPoint(p);
         }

         meshSegment->addFaces(trimesh->faces);
         meshSegment->computeBoundingBox();

         segments.push_back(meshSegment);
    }
}


auto SegmentsGraph::loadPrimitives(QJsonArray jsonPrimitives, Eigen::Matrix3f rot, Eigen::Matrix3f invRot) -> void
{
    assert(jsonPrimitives.size() == segments.size());

    for(auto i = 0; i < segments.size(); i++)
    {
        segments[i]->setRotation(rot);
        segments[i]->setInvRotation(invRot);
        auto jsonObject = jsonPrimitives[i].toObject();
        segments[i]->readJsonBBox(jsonObject["bbox"].toArray());
        segments[i]->readJsonEllipses(jsonObject["ellipses"].toArray());
        auto isPlane = jsonObject["isPlane"].toBool();
        segments[i]->setIsPlane(isPlane);

        auto it = jsonObject.find(QString("isTruncatedPyramid"));
        if(it != jsonObject.end())
        {
            auto isTruncatedPyramid = jsonObject["isTruncatedPyramid"].toBool();
            segments[i]->setIsTruncatedPyramid(isTruncatedPyramid);
        }

        if(isPlane)
        {
            auto val = jsonObject["planeAxis"].toInt();
            segments[i]->setPlaneAxis(val);
            val = jsonObject["planeFace"].toInt();
            segments[i]->setPlaneFace(val);
        }
    }
}


auto SegmentsGraph::getSegments(std::vector<MeshSegment*>& segs) -> void
{
    segs.insert(std::end(segs), std::begin(segments), std::end(segments));
}

#endif
