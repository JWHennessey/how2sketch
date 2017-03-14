#ifndef MESHSEGMENTCLUSTER_H
#define MESHSEGMENTCLUSTER_H

#include "mesh/meshsegment.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "Eigen/Dense"
#include <igl/read_triangle_mesh.h>
#include <igl/writePLY.h>
#include <QJsonObject>
#include <QJsonArray>
#include <QDebug>
#include <memory>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Iso_rectangle_2.h>

class MeshSegmentCluster
{
public:
    MeshSegmentCluster();
    MeshSegmentCluster(MeshSegmentCluster* parent);
    auto create(QString directory, QJsonArray jsonArray, trimesh::point mean,
                float max_coeff, int& counterRef) -> void;
    auto create(QString directory, QString filename, trimesh::point mean,
                float max_coeff, int& counterRef) -> void;
    auto addClusters(QString directory, QJsonArray jsonArray, trimesh::point mean,
                float max_coeff, int& counterRef) -> void;
    auto getAllSegments(std::vector<MeshSegment*>& segments) -> void;
    auto getCluster(const int id, int& counterRef) -> MeshSegmentCluster*;
    auto clusterId() -> int;
    auto clusterSize() -> int;

    auto boundingBoxesSize() -> int; //Number of vertices for the bounding boxes
    auto boundingBoxes() -> std::vector<trimesh::point>; //pointer to the vertices of all the bounding boxes
    auto getBBox() -> std::vector<trimesh::point>; //
    auto boundingBoxesFaceData() -> unsigned short*; //The faces for the bounding boxes
    auto setBBoxRotations(Eigen::Matrix3f eigTrans, Eigen::Matrix3f eigTransInv) -> void;

    auto hasParentCluster() -> bool;
    auto getParentCluster() ->  MeshSegmentCluster*;

    auto runOptimisation(Eigen::Matrix<float, 200, 4>& drawnBoxes,
                         int& drawnBoxCount, MeshSegmentCluster* cluster39) -> void;

    auto getLeftGuides(int i) -> std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > >;
    auto getRightGuides(int i) -> std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > >;
    auto getTopGuides(int i) -> std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > >;
    auto getBottomGuides(int i) -> std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > >;
    auto getBoxFilename(int i) -> QString;

    auto writeAllMeshes() -> void;
    auto writeBox() -> QString;

    auto noBoxesToDraw() -> int;
    auto getFaceToHighlight(int i) -> std::vector<Eigen::Vector3f>;
    auto getNoChildClusters() -> int;

private:
    int _clusterId;
    MeshSegmentCluster* parentCluster;
    bool hasParent;
    int numberOfChildClusters;
    std::vector<MeshSegmentCluster*> clusters;
    MeshSegment meshSegment;
   // std::vector<MeshSegmentCluster*> children;
    //Bounding box rotations
    Eigen::Matrix3f bbRotation;
    Eigen::Matrix3f bbRotationInv;
    std::vector<trimesh::point> adjustedBBox;

};

#endif // MESHSEGMENTCLUSTER_H
