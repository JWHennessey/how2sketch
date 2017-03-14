#ifndef PRIMITIVESPANEL_CPP
#define PRIMITIVESPANEL_CPP

#include <QGroupBox>
#include <QFormLayout>
#include <QDoubleSpinBox>
#include <QDebug>
#include <QComboBox>
#include <QCheckBox>
#include "widgets/primitivespanel.h"
#include "mesh/mesh.h"

PrimitivesPanel::PrimitivesPanel(QWidget *parent)
    : QWidget(parent)
    , mesh(nullptr)
    , selectedSegment(nullptr)
    , currentFace(-1)
    , currentAxis(-1)
    , openedMesh(false)
{
    setFocusPolicy(Qt::ClickFocus);
    init();
}

auto PrimitivesPanel::init() -> void
{
    qDebug() << "PrimitivesPanel::init()";
    auto layout = new QVBoxLayout(this);
    auto groupBox = new QGroupBox(tr("Primitives"));
    items = new QVBoxLayout;
    items->setAlignment(Qt::AlignTop);
    groupBox->setLayout(items);
    layout->addWidget(groupBox);

    auto ellipseGroupBox = new QGroupBox(tr("Ellipses"));
    ellipseItems = new QVBoxLayout;
    ellipseItems->setAlignment(Qt::AlignTop);
    ellipseGroupBox->setLayout(ellipseItems);
    layout->addWidget(ellipseGroupBox);
}

auto PrimitivesPanel::setMesh(Mesh *m) -> void
{
    auto wasNull = (mesh == nullptr);
    mesh = m;
    segments.clear();
    segments = mesh->getMeshSegments();

    if(wasNull)
        updateMenu();

}

auto PrimitivesPanel::updateMenu() -> void
{

    qDebug() << "PrimitivesPanel::updateMenu()";
    auto segmentCombo = new QComboBox();
    segmentCombo->addItem(QString("All"));
    for(auto segment : segments)
    {
        segmentCombo->addItem(segment->getName());
    }
    connect(segmentCombo, SIGNAL(activated(int)), this, SLOT(setSelectedSegment(int)));

    auto segmentPosition = new QHBoxLayout;
    segmentXPosition = new QDoubleSpinBox();
    segmentXPosition->setSingleStep(0.00001);
    segmentXPosition->setRange(-10.0, 10.0);
    segmentXPosition->setDecimals(5);
    connect(segmentXPosition, SIGNAL(editingFinished()), this, SLOT(segmentPositionChanged()));

    segmentYPosition = new QDoubleSpinBox();
    segmentYPosition->setSingleStep(0.00001);
    segmentYPosition->setRange(-10.0, 10.0);
    segmentYPosition->setDecimals(5);
    connect(segmentYPosition, SIGNAL(editingFinished()), this, SLOT(segmentPositionChanged()));

    segmentZPosition = new QDoubleSpinBox();
    segmentZPosition->setSingleStep(0.00001);
    segmentZPosition->setRange(-10.0, 10.0);
    segmentZPosition->setDecimals(5);
    connect(segmentZPosition, SIGNAL(editingFinished()), this, SLOT(segmentPositionChanged()));

    segmentPosition->addWidget(segmentXPosition);
    segmentPosition->addWidget(segmentYPosition);
    segmentPosition->addWidget(segmentZPosition);

    auto axisCombo = new QComboBox();
    axisCombo->addItem(QString("All"));
    axisCombo->addItem(QString("X"));
    axisCombo->addItem(QString("Y"));
    axisCombo->addItem(QString("Z"));
    connect(axisCombo, SIGNAL(activated(int)), this, SLOT(setSegmentAxis(int)));

    auto faceCombo = new QComboBox();
    faceCombo->addItem(QString("All"));
    faceCombo->addItem(QString("1"));
    faceCombo->addItem(QString("2"));
    connect(faceCombo, SIGNAL(activated(int)), this, SLOT(setSegmentFace(int)));

    planeWidth = new QDoubleSpinBox();
    connect(planeWidth, SIGNAL(editingFinished()), this, SLOT(updatePlaneWidth()));

    planeLength = new QDoubleSpinBox();
    connect(planeLength, SIGNAL(editingFinished()), this, SLOT(updatePlaneLength()));

    auto makePlaneCheckbox = new QCheckBox();

    connect(makePlaneCheckbox, SIGNAL(toggled(bool)), this, SLOT(setPlanePrimitive(bool)));

    auto addEllipseButton = new QPushButton("Add");
    connect(addEllipseButton, SIGNAL(released()), this, SLOT(addEllipse()));

    auto formLayout = new QFormLayout;
    formLayout->addRow(tr("Segment:"), segmentCombo);
    formLayout->addRow(tr("Position:"), segmentPosition);
    formLayout->addRow(tr("Axis:"), axisCombo);
    formLayout->addRow(tr("Face:"), faceCombo);
    formLayout->addRow(tr("Plane Width:"), planeWidth);
    formLayout->addRow(tr("Plane Length:"), planeLength);
    formLayout->addRow(tr("Make Plane:"), makePlaneCheckbox);
    formLayout->addRow(tr("Add Ellipse:"), addEllipseButton);

    items->addLayout(formLayout);
}


auto PrimitivesPanel::addEllipse() -> void
{
    qDebug() << "PrimitivesPanel::addEllipse()";

    if(selectedSegment != nullptr)
    {
        auto centroid = selectedSegment->getCentroid();

        auto ellipse = new Ellipse();
        ellipse->setRadius(1.0);
        ellipse->setTranslation(QVector3D(centroid[0], centroid[1], centroid[2]));
        ellipse->setNormal(QVector3D(0,1,0));

        ellipse->setBBRotation(selectedSegment->getInvRotation());

        selectedSegment->addEllipse(ellipse);

        addEllipse(ellipse);
    }
}

auto PrimitivesPanel::addEllipse(Ellipse* ellipse) -> void
{

    auto centroid = ellipse->getTranslation();

    auto ellipsePosition = new QHBoxLayout;
    auto ellipseXPosition = new QDoubleSpinBox();
    ellipseXPosition->setSingleStep(0.00001);
    ellipseXPosition->setRange(-10.0, 10.0);
    ellipseXPosition->setDecimals(5);
    ellipseXPosition->setValue(centroid[0]);
    connect(ellipseXPosition, SIGNAL(valueChanged(double)), ellipse, SLOT(setTranslationX(double)));
    connect(ellipseXPosition, SIGNAL(valueChanged(double)), this, SIGNAL(updateViewer()));

    auto ellipseYPosition = new QDoubleSpinBox();
    ellipseYPosition->setSingleStep(0.00001);
    ellipseYPosition->setRange(-10.0, 10.0);
    ellipseYPosition->setDecimals(5);
    ellipseYPosition->setValue(centroid[1]);
    connect(ellipseYPosition, SIGNAL(valueChanged(double)), ellipse, SLOT(setTranslationY(double)));
    connect(ellipseYPosition, SIGNAL(valueChanged(double)), this, SIGNAL(updateViewer()));

    auto ellipseZPosition = new QDoubleSpinBox();
    ellipseZPosition->setSingleStep(0.00001);
    ellipseZPosition->setRange(-10.0, 10.0);
    ellipseZPosition->setDecimals(5);
    ellipseZPosition->setValue(centroid[2]);
    connect(ellipseZPosition, SIGNAL(valueChanged(double)), ellipse, SLOT(setTranslationZ(double)));
    connect(ellipseZPosition, SIGNAL(valueChanged(double)), this, SIGNAL(updateViewer()));

    ellipsePosition->addWidget(ellipseXPosition);
    ellipsePosition->addWidget(ellipseYPosition);
    ellipsePosition->addWidget(ellipseZPosition);

    auto normal = ellipse->getNormal();

    auto normalPosition = new QHBoxLayout;
    auto normalXPosition = new QDoubleSpinBox();
    normalXPosition->setSingleStep(0.00001);
    normalXPosition->setRange(-10.0, 10.0);
    normalXPosition->setDecimals(5);
    normalXPosition->setValue(normal[0]);
    connect(normalXPosition, SIGNAL(valueChanged(double)), ellipse, SLOT(setNormalX(double)));
    connect(normalXPosition, SIGNAL(valueChanged(double)), this, SIGNAL(updateViewer()));

    auto normalYPosition = new QDoubleSpinBox();
    normalYPosition->setSingleStep(0.00001);
    normalYPosition->setRange(-10.0, 10.0);
    normalYPosition->setDecimals(5);
    normalYPosition->setValue(normal[1]);
    connect(normalYPosition, SIGNAL(valueChanged(double)), ellipse, SLOT(setNormalY(double)));
    connect(normalYPosition, SIGNAL(valueChanged(double)), this, SIGNAL(updateViewer()));

    auto normalZPosition = new QDoubleSpinBox();
    normalZPosition->setSingleStep(0.00001);
    normalZPosition->setRange(-10.0, 10.0);
    normalZPosition->setDecimals(5);
    normalZPosition->setValue(normal[2]);
    connect(normalZPosition, SIGNAL(valueChanged(double)), ellipse, SLOT(setNormalZ(double)));
    connect(normalZPosition, SIGNAL(valueChanged(double)), this, SIGNAL(updateViewer()));

    normalPosition->addWidget(normalXPosition);
    normalPosition->addWidget(normalYPosition);
    normalPosition->addWidget(normalZPosition);


    auto radius = ellipse->getRadius();

    auto ellipseScale = new QDoubleSpinBox();
    ellipseScale->setSingleStep(0.00001);
    ellipseScale->setRange(-10.0, 10.0);
    ellipseScale->setDecimals(5);
    ellipseScale->setValue(radius);
    connect(ellipseScale, SIGNAL(valueChanged(double)), ellipse, SLOT(setRadius(double)));
    connect(ellipseScale, SIGNAL(valueChanged(double)), this, SIGNAL(updateViewer()));

    auto formLayout = new QFormLayout;
    formLayout->addRow(tr("Position:"), ellipsePosition);
    formLayout->addRow(tr("Normal:"), normalPosition);
    formLayout->addRow(tr("Diameter:"), ellipseScale);

    auto ellipseGroupBox = new QGroupBox(tr("Ellipse"));
    ellipseGroupBox->setLayout(formLayout);

    ellipseItems->addWidget(ellipseGroupBox);
}

auto PrimitivesPanel::segmentPositionChanged() -> void
{
    if(selectedSegment != nullptr)
    {
        auto x = segmentXPosition->value();
        auto y = segmentYPosition->value();
        auto z = segmentZPosition->value();
        auto moved_centroid = trimesh::point(x,y,z);
        selectedSegment->translateCentroid(moved_centroid);
        emit initViewer();
    }
}

auto PrimitivesPanel::setSelectedSegment(int i) -> void
{
    for(auto seg : segments)
    {
        seg->clearColour();
        seg->deselect();
        seg->setAxis(-1);
        seg->setFace(-1);
    }

    if(i == 0)
    {
        selectedSegment = nullptr;
        return;
    }

    i--;

    selectedSegment = segments[i];
    selectedSegment->setRed();
    selectedSegment->select();
    selectedSegment->setAxis(currentAxis);
    selectedSegment->setFace(currentFace);
    auto widthLength = selectedSegment->getPlaneWidthAndLength();
    planeWidth->setValue(widthLength.first);
    planeLength->setValue(widthLength.second);
    auto centroid = selectedSegment->getCentroid();
    segmentXPosition->setValue(centroid[0]);
    segmentYPosition->setValue(centroid[1]);
    segmentZPosition->setValue(centroid[2]);

    updateEllipses();

    emit updateViewer();
}

auto PrimitivesPanel::updateEllipses() -> void
{
    QLayoutItem *child;
    while ((child = ellipseItems->takeAt(0)) != 0) {
        delete child->widget();
        delete child;
    }

    if(selectedSegment != nullptr)
    {
         auto ellipses = selectedSegment->getEllipses();
         for(auto e : ellipses)
         {
             addEllipse(e);
         }
    }
}

auto PrimitivesPanel::setSegmentFace(int i) -> void
{



    if(selectedSegment == nullptr || i == 0)
    {
        currentFace = -1;
        if(selectedSegment != nullptr)
            selectedSegment->setFace(currentFace);
        return;
    }

    currentFace = i;

    selectedSegment->setFace(currentFace);
    auto widthLength = selectedSegment->getPlaneWidthAndLength();
    planeWidth->setValue(widthLength.first);
    planeLength->setValue(widthLength.second);
    emit updateViewer();

}

auto PrimitivesPanel::setSegmentAxis(int i) -> void
{

    qDebug() << "PrimitivesPanel::setSegmentAxis(int i " << i << ")";

    if(selectedSegment == nullptr || i == 0)
    {
        currentAxis = -1;
        if(selectedSegment != nullptr)
            selectedSegment->setAxis(currentAxis);
        return;
    }


    currentAxis = i;

    selectedSegment->setAxis(currentAxis);
    auto widthLength = selectedSegment->getPlaneWidthAndLength();
    planeWidth->setValue(widthLength.first);
    planeLength->setValue(widthLength.second);
    emit updateViewer();
}

auto PrimitivesPanel::updatePlaneWidth() -> void
{
    qDebug() << "PrimitivesPanel::updatePlaneWidth(double val)";
    if(selectedSegment != nullptr && currentAxis != -1)
    {
        selectedSegment->setPlaneWidth(planeWidth->value());
        emit initViewer();
    }
}

auto PrimitivesPanel::updatePlaneLength() -> void
{
    qDebug() << "currentAxis "  << currentAxis;
    if(selectedSegment != nullptr && currentAxis != -1)
    {
        selectedSegment->setPlaneLength(planeLength->value());
        emit initViewer();
    }
}


auto PrimitivesPanel::setPlanePrimitive(bool val) -> void
{
    if(val && selectedSegment != nullptr && currentAxis != -1 && currentFace != -1)
    {
       selectedSegment->makeFacePlane();

       emit initViewer();
    }

    if(!val && selectedSegment != nullptr && currentAxis != -1 && currentFace != -1)
    {
       selectedSegment->setToOriginalBBox();
       emit initViewer();
    }
}

#endif
