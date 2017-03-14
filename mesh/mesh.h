#ifndef MESH_H
#define MESH_H

//#include <array>

#include <vector>
#include <memory>
#include <algorithm>
#include <map>
#include <set>
#include <QObject>
#include <QVector3D>
#include "Eigen/Dense"
#include "Eigen/Sparse"

#include "mesh/meshdata.h"
//#include "mesh/meshsegmentcluster.h"
#include "mesh/segmentsgraph.h"
#include "trimesh/Vec.h"
#include "trimesh/TriMesh.h"
#include "mesh/meshsegment.h"


using MatrixXf = Eigen::Matrix< float,Eigen::Dynamic,Eigen::Dynamic,Eigen::RowMajor>;
using EigenMatrixXus = Eigen::Matrix<unsigned short,Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;
using MatrixXus = Eigen::Matrix<unsigned short,Eigen::Dynamic, Eigen::Dynamic,Eigen::RowMajor>;
const static auto camera_pos = QVector3D(-2, 1, 0);
const static int bb_faces_count = 12;

enum class BoxAdjustMethod {LEAST_SQUARES};


class Mesh : public QObject
{
    Q_OBJECT
public:
    Mesh();
    Mesh(const QString filename);
    ~Mesh();
    auto vSize() -> int;
    auto vData() -> float*;
    auto nSize() -> int;
    auto nData() -> float*;
    auto cSize() -> int;
    auto cData() -> float*;
    auto fSize() -> int;
    auto fData() -> unsigned short*;
    auto bBoxSize() -> int;
    auto bBoxData() -> float*;
    auto bBoxFaceData() -> unsigned short*;

//    auto totalNoClusters() -> int;
//    auto noClusters(int i) -> int;
//    auto clusterBoundingBoxPointsSize(int i) -> int;
//    auto clusterBoundingBoxPoints(int i) -> std::vector<trimesh::point>;
//    auto clusterFaceData(int i) -> unsigned short*;
    //auto clusterHasParent(int i) -> bool;
    //auto runOptimisationOnCluster(int i) -> void;
    auto maxDirSize() -> int;
    auto maxDirData() -> float*;
    auto minDirSize() -> int;
    auto minDirData() -> float*;

    auto maxCurveSize() -> int;
    auto maxCurveData() -> float*;
    auto minCurveSize() -> int;
    auto minCurveData() -> float*;

    auto dCurveSize() -> int;
    auto dCurveData() -> float*;

    auto featureSize() -> float;

    auto segmentSize() -> int;
    auto segmentColour(int i) -> trimesh::Color;


    auto manualAdjustBoundingBox(QMatrix4x4 viewport, QMatrix4x4 proj, QMatrix4x4 view, QMatrix4x4 model, int boxId, int unitEdgeId, int selectedEdgeId, float newRatioLength) -> void;

    auto edgeToEdgeId(int boxId, int edgeId) -> int;
    auto screenSpaceEdgeLength(QMatrix4x4 viewport, QMatrix4x4 mvp,  int boxId, int edgeId) -> float;
    auto rotateBoundingBoxXAxis(double val) -> void;
    auto rotateBoundingBoxYAxis(double val) -> void;
    auto rotateBoundingBoxZAxis(double val) -> void;
    auto computeBoxes(BoxAdjustMethod method, QMatrix4x4 viewport, QMatrix4x4 proj, QMatrix4x4 view, QMatrix4x4 model, int boxId, int unitEdgeId, std::map<int, std::vector<std::pair<int, float>>>& desiredEdgeRatios) -> void;
    auto outputData(QMatrix4x4 model_view) -> void;
    auto getMeshSegments() -> std::vector<MeshSegment*>;
    auto segmentIsSelected(int i) -> bool;
    auto getSelectedSegment(int i) -> MeshSegment*;
    auto updateBoundingBoxes() -> void;
    auto writePrimitivesAndRelations() -> void;
    auto findRelations(int& id) -> void;


public slots:
    void openMesh(const QString filename);

signals:
    void updateViewer();

private:
    trimesh::TriMesh* trimesh;
    QMatrix4x4* m_proj;
    QMatrix4x4* m_view;
    QMatrix4x4* m_model;
    Eigen::MatrixXf iglVertices;
    Eigen::MatrixXi iglFaces;
//  MeshSegmentCluster meshSegmentCluster;
//  //Cluster that is to used for the showing cluster related methods
//  MeshSegmentCluster* selectedCluster;
    SegmentsGraph segmentsGraph;
    std::vector<MeshSegment*> segments;
    std::vector<trimesh::point> bBox;
    std::map<float, int> colourSegmentMap;
    unsigned short* boundingBoxFaces;
    std::map<int, std::pair<int, int>> edgeToVertices;
    std::map<int, int> edgeToEdgeIdMap;
    std::map<int, std::set<int>> orthogonalVerticesMap; //Vertices that are connected by a right angle
    Eigen::AngleAxisf bbXRot;
    Eigen::AngleAxisf bbYRot;
    Eigen::AngleAxisf bbZRot;
    Eigen::Matrix<float, 200, 4> drawnBoxes;
    QString directory;
    QString meshFilename;
    int drawnBoxesCount;
    auto openMeshJSON(const QString filename) -> QJsonDocument;
    auto readColour(QString filename) -> void;
    auto setDefaultColour() -> void;
    auto clampColour(int i) -> int;
    auto orientedBoundingBox() -> void;
    auto boundingBoxesConnectivity() -> void;
    auto createEdgeMaps() -> void;
    auto addPointsToBoundingBox(Eigen::Matrix3f eigTransposed, Eigen::Vector3f min, Eigen::Vector3f max ) -> void;
    auto minWidthBoundingBox(Eigen::Vector3f& maxP, Eigen::Vector3f& minP, Eigen::Matrix3f& eigTransposed, Eigen::Matrix3f& eigTransposedInv) -> void;
    auto pcaBoundingBox(Eigen::MatrixXf& points, Eigen::Vector3f& maxP, Eigen::Vector3f& minP, Eigen::Matrix3f& eigTransposed, Eigen::Matrix3f& eigTransposedInv) -> void;
    auto screenSpaceVector(QMatrix4x4 viewport, QMatrix4x4 mvp, int boxId, int edgeId) -> QVector2D;
    auto findMovablePoint(int& movablePointId, int& fixedPointId, QVector4D& movablePoint, QVector4D& fixedPoint, int boxId, int unitEdgeId, int selectedEdgeId, bool selectedEdgeAlreadyMapped = false) -> bool;
    auto propagateBoundingBoxEdit(int selectedBoxId, int movedVertexIndex,  int fixedVertexIndex, trimesh::point newPoint) -> void;
    auto addOrthogonalVertices(int a, int b, bool switched = false) -> void; //Switched is only set to two when recursively calling addOrthogonalVertices
    auto addToEdgeMaps(int a, int b, int edgeCount, int totalEdgeCount) -> void;
    auto rotateBoundingBox() -> void;
    auto objectSpacePointFromSSLength(QMatrix4x4 viewport, QMatrix4x4 proj, QMatrix4x4 view, QMatrix4x4 model, QVector4D selectedEdgeFixedPoint, QVector4D selectedEdgeMovablePoint, QVector2D fixedEdgeSSVector, float newRatioLength) -> QVector4D;
    auto leastSquaresFixedEdge(QMatrix4x4 viewport, QMatrix4x4 proj, QMatrix4x4 view, QMatrix4x4 model, int boxId, int unitEdgeId, std::vector<std::pair<int, float>>& desiredEdgeRatios) -> void;
    auto leastSquaresAllEdges(QMatrix4x4 viewport, QMatrix4x4 proj, QMatrix4x4 view, QMatrix4x4 model, int boxId, int unitEdgeId, int selectedBoxId, std::vector<std::pair<int, float>>& desiredEdgeRatios) -> void;
    auto searchToFitScreenSpaceLength(QMatrix4x4 viewport, QMatrix4x4 proj, QMatrix4x4 view, QMatrix4x4 model, QVector4D& ptA, QVector4D& ptB, QVector2D fixedEdgeSSVector, float ssTargetRatio) -> void;
    auto ssPoint(QMatrix4x4 viewport, QMatrix4x4 proj, QMatrix4x4 view, QMatrix4x4 model, QVector4D& ptA) -> QVector2D;
    //auto getSelectedCluster(const int id) -> void;


};



#endif // MESH_H
