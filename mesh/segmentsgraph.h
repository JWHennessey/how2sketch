#ifndef SEGMENTSGRAPH_H
#define SEGMENTSGRAPH_H

//QT
#include <QObject>
#include <QJsonArray>

//std
#include <vector>

//Custom
#include "mesh/meshsegment.h"

class SegmentsGraph : public QObject
{
    Q_OBJECT
public:
    SegmentsGraph(QObject *parent = 0);
    auto loadSegments(QString dir, QJsonArray jsonArray,
                      trimesh::point mean, float max_coeff) -> void;
    auto loadPrimitives(QJsonArray jsonPrimitives, Eigen::Matrix3f rot, Eigen::Matrix3f invRot) -> void;
    auto getSegments(std::vector<MeshSegment*>& segs) -> void;

signals:

public slots:

private:
    std::vector<MeshSegment*> segments;

};

#endif // SEGMENTSGRAPH_H
