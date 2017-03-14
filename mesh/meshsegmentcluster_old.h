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

class MeshSegmentCluster //: std::enable_shared_from_this<MeshSegmentCluster>
{
public:
    MeshSegmentCluster();
    MeshSegmentCluster(bool selfParent, MeshSegmentCluster* grandParent);
    MeshSegmentCluster(MeshSegmentCluster* parent, MeshSegmentCluster* grandParent);
    //When cluster is a group of segments, mean and max_coeff is so they are the right size
    auto create(QJsonArray jsonArray, trimesh::point mean, float max_coeff, int& counterRef) -> void;
    //When cluster is just a single segment
    auto create(QString meshFilename, trimesh::point mean, float max_coeff, int& counterRef) -> void;
    //auto createChildren(QJsonArray jsonArray, trimesh::point mean, float max_coeff, int& counterRef) -> void;
    auto getAllSegments(std::vector<MeshSegment*>& segments) -> void;
    auto getCluster(const int id, int& counterRef) -> MeshSegmentCluster*;
    auto clusterId() -> int;
    auto clusterSize() -> int;
    auto boundingBoxesSize() -> int; //Number of vertices for the bounding boxes
    auto boundingBoxes() -> std::vector<trimesh::point>; //pointer to the vertices of all the bounding boxes
    auto boundingBoxesFaceData() -> unsigned short*; //The faces for the bounding boxes
    auto getBBox() -> std::vector<trimesh::point>;
    auto getNoChildClusters() -> int;
    auto setBBoxRotations(Eigen::Matrix3f eigTrans, Eigen::Matrix3f eigTransInv) -> void;
    auto hasParentCluster() -> bool;
    auto getParentCluster() ->  MeshSegmentCluster*;
    auto runOptimisation(Eigen::Matrix<float, 200, 4>& drawnBoxes, int& drawnBoxCount, MeshSegmentCluster* cluster39) -> void;
    auto setBoundingBox(std::vector<trimesh::point> bBox, std::function<bool(int)> functor, std::pair<int, int> planeIndexes, int functorIndex, bool translateOnly) -> void;
    auto setBoundingBox(std::vector<trimesh::point> bBox) -> void;
    auto adjustBBox(std::vector<trimesh::point> currentBBox, std::vector<trimesh::point> newBBox, std::vector<trimesh::point> inner, std::function<bool(int)> functor, std::pair<int, int> planeIndexe, int functorIndex) -> std::vector<trimesh::point>;
    auto getGrandParentBBox() -> std::vector<trimesh::point>;
    auto writeDataAndGetJSON(int i) -> QJsonObject;
    auto calculateOptimisationBoxFace() -> void;
    auto setMaxAreaFaceId(int i) -> void;
    auto getMaxAreaFaceId() -> int;
    auto getLeftGuides(int i) -> std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > >;
    auto getRightGuides(int i) -> std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > >;
    auto getTopGuides(int i) -> std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > >;
    auto getBottomGuides(int i) -> std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > >;
    auto getBoxFilename(int i) -> QString;
    auto getChildClusterId(int i) -> int;
    auto noBoxesToDraw() -> int { return clusterOrderingX.size(); }
    auto writeBox() -> QString;
    auto getFaceToDraw() -> std::vector<Eigen::Vector3f>  { return faceToDraw; }
    auto getFaceToHighlight(int i) -> std::vector<Eigen::Vector3f>;
    auto getChildFaceToDraw(int i) -> std::vector<Eigen::Vector3f>;
    auto getChildFaceToHighlight(int i) -> std::vector<Eigen::Vector3f>;
    auto writeAllMeshes() -> void;


private:
    std::vector<MeshSegmentCluster*> clusters;
    std::vector<int> clusterOrderingX;
    std::vector<int> clusterOrderingY;
    std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > > guidesX;
    std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > > guidesY;
    std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > > guidesLeft;
    std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > > guidesRight;
    std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > > guidesTop;
    std::vector<std::pair<Eigen::Vector3f, Eigen::Vector3f > > guidesBottom;
    std::vector<int> guideCountX;
    std::vector<int> guideCountY;
    std::vector<int> guideCountLeft;
    std::vector<int> guideCountRight;
    std::vector<int> guideCountTop;
    std::vector<int> guideCountBottom;
    std::vector<Eigen::Vector3f> faceToDraw;
    std::vector<Eigen::Vector3f> faceToHighlight;

    //std::vector<MeshSegmentCluster*> childClusters;
    MeshSegment meshSegment;
    MeshSegmentCluster* parentCluster;
    MeshSegmentCluster* grandParentCluster;
    bool hasParent;
    bool hasGrandParent;
    bool firstClusterIsParent;
    int firstClusterIsParentId;
    int id;
    int numberOfChildClusters;
    int maxAreaFaceId;

    //Bounding box rotations
    Eigen::Matrix3f bbRotation;
    Eigen::Matrix3f bbRotationInv;
    std::vector<trimesh::point> adjustedBBox;
};

#endif // MESHSEGMENTCLUSTER_H
