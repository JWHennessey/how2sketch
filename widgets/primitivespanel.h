#ifndef PRIMITIVESPANEL_H
#define PRIMITIVESPANEL_H

#include <QWidget>
#include <QVBoxLayout>
#include <QCheckBox>
#include <QPushButton>
#include <QDoubleSpinBox>
#include "mesh/mesh.h"

class PrimitivesPanel : public QWidget
{
    Q_OBJECT
public:
    explicit PrimitivesPanel(QWidget *parent = 0);
    auto init() -> void;
    auto setMesh(Mesh* m) -> void;

signals:
    void updateViewer();
    void initViewer();

private slots:
    void setSelectedSegment(int i);
    void setSegmentFace(int i);
    void setSegmentAxis(int i);
    void updatePlaneWidth();
    void updatePlaneLength();
    void setPlanePrimitive(bool val);
    void segmentPositionChanged();
    void addEllipse();

private:
    Mesh* mesh;
    MeshSegment* selectedSegment;
    int currentFace;
    int currentAxis;
    QVBoxLayout* items;
    QVBoxLayout* ellipseItems;
    QDoubleSpinBox* planeWidth;
    QDoubleSpinBox* planeLength;
    QDoubleSpinBox* segmentXPosition;
    QDoubleSpinBox* segmentYPosition;
    QDoubleSpinBox* segmentZPosition;
    std::vector<MeshSegment*> segments;
    bool openedMesh;
    auto updateMenu() -> void;
    auto updateEllipses() -> void;
    auto addEllipse(Ellipse* ellipse) -> void;

};

#endif // PRIMITIVESPANEL_H
