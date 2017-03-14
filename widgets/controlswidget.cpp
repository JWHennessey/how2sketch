#ifndef CONTROLSWIDGET_CPP
#define CONTROLSWIDGET_CPP

#include <QVBoxLayout>
#include <QGroupBox>
#include <QFormLayout>


#include "widgets/controlswidget.h"

ControlsWidget::ControlsWidget(QWidget *parent)
    : QWidget(parent)
    , activePanel(-1)

{
    unitlengthBoxCombo = new QComboBox();
    showEdgeRatiosCombo = new QComboBox();
    selectedEdgeBoxCombo = new QComboBox();
    init();
}

auto ControlsWidget::init() -> void
{
    setFocusPolicy(Qt::ClickFocus);
    auto layout = new QVBoxLayout(this);
    auto groupBox = new QGroupBox(tr("Panel Controls"));
    items = new QVBoxLayout;
    items->setAlignment(Qt::AlignTop);


    auto showObjectCheckbox = new QCheckBox();
    showObjectCheckbox->setChecked(true);
    connect(showObjectCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(showObjectToggled(bool)));
    connect(showObjectCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(updateViewer()));

    auto renderModeCombo = new QComboBox();
    renderModeCombo->addItem("Phong");
    renderModeCombo->addItem("Segmentation");
    renderModeCombo->addItem("Suggestive Contours");

    connect(renderModeCombo, SIGNAL(activated(int)), this, SIGNAL(setRenderMode(int)));



    auto selectSegmentCombo = new QComboBox();
    selectSegmentCombo->addItem("All");
    selectSegmentCombo->addItem("1");
    selectSegmentCombo->addItem("2 ");
    selectSegmentCombo->addItem("3");
    selectSegmentCombo->addItem("4");
    selectSegmentCombo->addItem("5");
    selectSegmentCombo->addItem("6");
    selectSegmentCombo->addItem("7");
    selectSegmentCombo->addItem("8");
    selectSegmentCombo->addItem("9");
    connect(selectSegmentCombo, SIGNAL(activated(int)), this, SIGNAL(selectSegment(int)));


    //items->addWidget(renderModeCombo);

    auto boundingBoxCheckbox = new QCheckBox();
    boundingBoxCheckbox->setChecked(false);
    connect(boundingBoxCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(boundingBoxToggled(bool)));
    connect(boundingBoxCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(updateViewer()));

    auto boundingBoxOrientation = new QHBoxLayout;
    auto boundingBoxXRotation = new QDoubleSpinBox();
    boundingBoxXRotation->setSingleStep(0.01);
    boundingBoxXRotation->setRange(-10.0, 10.0);
    connect(boundingBoxXRotation, SIGNAL(valueChanged(double)), this, SIGNAL(bbXRotChanged(double)));

    auto boundingBoxYRotation = new QDoubleSpinBox();
    boundingBoxYRotation->setSingleStep(0.01);
    boundingBoxYRotation->setRange(-10.0, 10.0);
    connect(boundingBoxYRotation, SIGNAL(valueChanged(double)), this, SIGNAL(bbYRotChanged(double)));

    auto boundingBoxZRotation = new QDoubleSpinBox();
    boundingBoxZRotation->setSingleStep(0.01);
    boundingBoxZRotation->setRange(-10.0, 10.0);
    connect(boundingBoxZRotation, SIGNAL(valueChanged(double)), this, SIGNAL(bbZRotChanged(double)));

    boundingBoxOrientation->addWidget(boundingBoxXRotation);
    boundingBoxOrientation->addWidget(boundingBoxYRotation);
    boundingBoxOrientation->addWidget(boundingBoxZRotation);

    auto segmentBoxesCheckbox = new QCheckBox();
    segmentBoxesCheckbox->setChecked(false);
    connect(segmentBoxesCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(segmentBoxesToggled(bool)));
    connect(segmentBoxesCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(updateViewer()));

    auto ellipsesCheckbox = new QCheckBox();
    ellipsesCheckbox->setChecked(false);
    connect(ellipsesCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(ellipsesToggled()));
    connect(ellipsesCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(updateViewer()));

    auto planesCheckbox = new QCheckBox();
    planesCheckbox->setChecked(false);
    connect(planesCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(planesToggled()));
    connect(planesCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(updateViewer()));

    auto candidatesCheckbox = new QCheckBox();
    candidatesCheckbox->setChecked(false);
    connect(candidatesCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(candidatesToggled()));
    connect(candidatesCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(updateViewer()));

    auto guidesCheckbox = new QCheckBox();
    guidesCheckbox->setChecked(false);
    connect(guidesCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(guidesToggled()));
    connect(guidesCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(updateViewer()));


//    auto projectLayout = new QHBoxLayout;

//    auto projectAxisCombo = new QComboBox();
//    projectAxisCombo->addItem(QString("X");
//    projectAxisCombo->addItem(QString("Y");
//    projectAxisCombo->addItem(QString("Z");

//    projectLayout



    auto clustersLayout = new QHBoxLayout;

    auto showClustersCheckbox = new QCheckBox();
    showClustersCheckbox->setChecked(false);
    connect(showClustersCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(showClustersToggled(bool)));
    connect(showClustersCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(updateViewer()));

    clustersLayout->addWidget(showClustersCheckbox);

    auto nextClusterBtn = new QPushButton("Next");
    connect(nextClusterBtn, SIGNAL(clicked(bool)), this, SIGNAL(showNextCluster()));
    connect(nextClusterBtn, SIGNAL(clicked(bool)), this, SIGNAL(updateViewer()));

    clustersLayout->addWidget(nextClusterBtn);

    auto runOptimisationOnClusterBtn = new QPushButton("Fudge");
    connect(runOptimisationOnClusterBtn, SIGNAL(clicked(bool)), this, SIGNAL(runOptimisationOnCluster()));
    connect(runOptimisationOnClusterBtn, SIGNAL(clicked(bool)), this, SIGNAL(updateViewer()));

    clustersLayout->addWidget(runOptimisationOnClusterBtn);

    auto outputDataBtn = new QPushButton("Write");
    connect(outputDataBtn, SIGNAL(clicked(bool)), this, SIGNAL(outputData()));
    connect(outputDataBtn, SIGNAL(clicked(bool)), this, SIGNAL(updateViewer()));

    clustersLayout->addWidget(outputDataBtn);

    auto renderBoxFacesCheckbox = new QCheckBox();
    renderBoxFacesCheckbox->setChecked(false);
    connect(renderBoxFacesCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(renderBoxFacesToggled(bool)));
    connect(renderBoxFacesCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(updateViewer()));

    auto renderMidLinesCheckbox = new QCheckBox();
    renderMidLinesCheckbox->setChecked(true);
    connect(renderMidLinesCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(renderMidLinesToggled(bool)));
    connect(renderMidLinesCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(updateViewer()));

    auto renderDiagLinesCheckbox = new QCheckBox();
    renderDiagLinesCheckbox->setChecked(false);
    connect(renderDiagLinesCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(renderDiagLinesToggled(bool)));
    connect(renderDiagLinesCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(updateViewer()));

//    auto renderTwoThirdsDiagLinesCheckbox = new QCheckBox();
//    renderTwoThirdsDiagLinesCheckbox->setChecked(false);
//    connect(renderTwoThirdsDiagLinesCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(renderTwoThirdsDiagLinesToggled(bool)));
//    connect(renderTwoThirdsDiagLinesCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(updateViewer()));

//    auto renderThirdsLineCheckbox = new QCheckBox();
//    renderThirdsLineCheckbox->setChecked(false);
//    connect(renderThirdsLineCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(renderThirdsLineToggled(bool)));
//    connect(renderThirdsLineCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(updateViewer()));


//    auto renderTwoThirdsLineCheckbox = new QCheckBox();
//    renderTwoThirdsLineCheckbox->setChecked(false);
//    connect(renderTwoThirdsLineCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(renderTwoThirdsLineToggled(bool)));
//    connect(renderTwoThirdsLineCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(updateViewer()));



//    auto renderThirdsDiagLinesCheckbox = new QCheckBox();
//    renderThirdsDiagLinesCheckbox->setChecked(false);
//    connect(renderThirdsDiagLinesCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(renderThirdsDiagLinesToggled(bool)));
//    connect(renderThirdsDiagLinesCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(updateViewer()));

    auto renderHiddenLinesCheckbox = new QCheckBox();
    renderHiddenLinesCheckbox->setChecked(false);
    connect(renderHiddenLinesCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(renderHiddenLinesToggled(bool)));
    connect(renderHiddenLinesCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(updateViewer()));

    auto renderPerspectiveLinesCheckbox = new QCheckBox();
    renderPerspectiveLinesCheckbox->setChecked(false);
    connect(renderPerspectiveLinesCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(renderPerspectiveLinesToggled(bool)));
    connect(renderPerspectiveLinesCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(updateViewer()));


    auto edgeRatiosCheckbox = new QCheckBox();
    edgeRatiosCheckbox->setChecked(false);
    connect(edgeRatiosCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(edgeRatiosToggled(bool)));
    connect(edgeRatiosCheckbox, SIGNAL(toggled(bool)), this, SIGNAL(updateViewer()));


    //updateDropdowns(0);

    connect(showEdgeRatiosCombo, SIGNAL(activated(int)), this, SIGNAL(setShowEdgeRatio(int)));
    connect(showEdgeRatiosCombo, SIGNAL(activated(int)), this, SIGNAL(updateViewer()));


    connect(unitlengthBoxCombo, SIGNAL(activated(int)), this, SIGNAL(setUnitLengthBox(int)));
    connect(unitlengthBoxCombo, SIGNAL(activated(int)), this, SIGNAL(updateViewer()));

    connect(selectedEdgeBoxCombo, SIGNAL(activated(int)), this, SIGNAL(setSelectedEdgeBox(int)));
    connect(selectedEdgeBoxCombo, SIGNAL(activated(int)), this, SIGNAL(updateViewer()));

    auto unitlengthEdgeCombo = new QComboBox();
    for(auto i = 1; i < 13; i++)
        unitlengthEdgeCombo->addItem(QString(QString::number(i)));

    connect(unitlengthEdgeCombo, SIGNAL(activated(int)), this, SIGNAL(setUnitLengthEdge(int)));
    connect(unitlengthEdgeCombo, SIGNAL(activated(int)), this, SIGNAL(updateViewer()));


    auto selectEdgeCombo = new QComboBox();
    for(auto i = 1; i < 13; i++)
        selectEdgeCombo->addItem(QString(QString::number(i)));

    connect(selectEdgeCombo, SIGNAL(activated(int)), this, SIGNAL(setSelectedEdge(int)));
    connect(selectEdgeCombo, SIGNAL(activated(int)), this, SIGNAL(updateViewer()));


    auto adjustSelectedEdgeRatioLayout = new QHBoxLayout;

    adjustSelectedEdgeRatioSpin = new QDoubleSpinBox();
    adjustSelectedEdgeRatioSpin->setRange(-5.0, 5.0);
    adjustSelectedEdgeRatioSpin->setDecimals(2);
    adjustSelectedEdgeRatioSpin->setSingleStep(0.01);


    auto setEdgeRatio = new QPushButton("Set");
    connect(setEdgeRatio, SIGNAL(clicked(bool)), this, SLOT(setDesieredEdgeRatio()));

    boxAdjustMethodCombo  = new QComboBox();
    boxAdjustMethodCombo->addItem("Least Squares");

    auto computeNewBox = new QPushButton("Adjust Boxes");
    connect(computeNewBox, SIGNAL(clicked(bool)), this, SLOT(computeBoxes()));

    adjustSelectedEdgeRatioLayout->addWidget(adjustSelectedEdgeRatioSpin);
    adjustSelectedEdgeRatioLayout->addWidget(setEdgeRatio);
    adjustSelectedEdgeRatioLayout->addWidget(computeNewBox);

//    //connect(adjustUnitLengthEdgeSpin, SIGNAL(valueChanged(double)), this, SIGNAL(manualAdjustUnitLengthEdge(double)));
//    connect(adjustSelectedEdgeRatioSpin, SIGNAL(editingFinished()), this, SLOT(manualUpdateSelectedLength()));
//    connect(adjustUnitLengthEdgeSpin, SIGNAL(valueChanged(double)), this, SIGNAL(updateViewer()));
    auto saveJsonBtn = new QPushButton("Save");
    connect(saveJsonBtn, SIGNAL(clicked(bool)), this, SIGNAL(saveJson()));

    auto zoomSpin = new QDoubleSpinBox();
    zoomSpin->setSingleStep(0.01);
    zoomSpin->setRange(-10.0, 10.0);
    zoomSpin->setDecimals(2);
    zoomSpin->setValue(1.0);


    connect(zoomSpin, SIGNAL(valueChanged(double)), this, SIGNAL(zoomChanged(double)));


    auto findRelationsBtn = new QPushButton("Find");
    connect(findRelationsBtn, SIGNAL(clicked(bool)), this, SIGNAL(findRelations()));

//    auto acceptRelationsBtn = new QPushButton("Accept");
//    connect(acceptRelationsBtn, SIGNAL(clicked(bool)), this, SIGNAL(acceptRelations()));

//    auto clearRelationsBtn = new QPushButton("Reset Count");
//    connect(clearRelationsBtn, SIGNAL(clicked(bool)), this, SIGNAL(clearRelations()));


    auto generateCandidatesBtn = new QPushButton("Generate");
    connect(generateCandidatesBtn, SIGNAL(clicked(bool)), this, SIGNAL(generateCandidates()));

    auto runOptimisationBtn = new QPushButton("Run");
    connect(runOptimisationBtn, SIGNAL(clicked(bool)), this, SIGNAL(runOptimisation()));

    auto formLayout = new QFormLayout;
    formLayout->addRow(tr("Show Object:"), showObjectCheckbox);
    formLayout->addRow(tr("Render Mode"), renderModeCombo);
    formLayout->addRow(tr("Bounding Box:"), boundingBoxCheckbox);
    formLayout->addRow(tr("B-Box Orientation"), boundingBoxOrientation);
    formLayout->addRow(tr("Segment Boxes:"), segmentBoxesCheckbox);
    formLayout->addRow(tr("Ellipses:"), ellipsesCheckbox);
    formLayout->addRow(tr("Candidate Planes:"), planesCheckbox);
    formLayout->addRow(tr("Complete Candidate:"), candidatesCheckbox);
    formLayout->addRow(tr("Guides:"),guidesCheckbox);
//    formLayout->addRow(tr("Clusters:"), clustersLayout);
    formLayout->addRow(tr("Render Box Faces:"), renderBoxFacesCheckbox);
    formLayout->addRow(tr("Render Mid Lines:"), renderMidLinesCheckbox);
//    formLayout->addRow(tr("Render 1/3 Line:"), renderThirdsLineCheckbox);
//    formLayout->addRow(tr("Render 2/3 Line:"), renderTwoThirdsLineCheckbox);
    //formLayout->addRow(tr("Render Diag Lines:"), renderDiagLinesCheckbox);
//    formLayout->addRow(tr("Render 1/3 Diag Line:"), renderThirdsDiagLinesCheckbox);
//    formLayout->addRow(tr("Render 2/3 Diag Line:"), renderTwoThirdsDiagLinesCheckbox);
    formLayout->addRow(tr("Render Hidden Lines:"), renderHiddenLinesCheckbox);
    formLayout->addRow(tr("Render Perspective Lines:"), renderPerspectiveLinesCheckbox);
//    formLayout->addRow(tr("Edge Ratios:"), edgeRatiosCheckbox);
//    formLayout->addRow(tr("Show Edge Ratios:"), showEdgeRatiosCombo);
//    formLayout->addRow(tr("Unit Length Box:"), unitlengthBoxCombo);
//    formLayout->addRow(tr("Unit Length Edge (green):"), unitlengthEdgeCombo);
//    formLayout->addRow(tr("Selected Edge Box:"), selectedEdgeBoxCombo);
//    formLayout->addRow(tr("Select Edge (red):"), selectEdgeCombo);
//    formLayout->addRow(tr("Method"), boxAdjustMethodCombo);
//    formLayout->addRow(tr("Adjust Edge Ratio (orange):"), adjustSelectedEdgeRatioLayout);

    formLayout->addRow(tr("Select Mesh:"), selectSegmentCombo);
    formLayout->addRow(tr("Save JSON:"), saveJsonBtn);
    formLayout->addRow(tr("Relations:"), findRelationsBtn);
    formLayout->addRow(tr("Candidates:"), generateCandidatesBtn);
    formLayout->addRow(tr("Optimisation:"), runOptimisationBtn);
//    formLayout->addRow(tr("Relations:"), acceptRelationsBtn);
//    formLayout->addRow(tr("Relations:"), clearRelationsBtn);
   // formLayout->addRow(tr("Zoom:"), zoomSpin);
    items->addLayout(formLayout);

    auto contoursPanelBtn = new QPushButton("Contour Params");
    contoursPanelBtn->setCheckable(true);
    contoursPanelBtn->setChecked(false);
    connect(contoursPanelBtn, SIGNAL(clicked(bool)), this, SLOT(toggleContoursPanel(bool)));
    items->addWidget(contoursPanelBtn);

    auto primitivesPanelBtn = new QPushButton("Primitives");
    primitivesPanelBtn->setCheckable(true);
    primitivesPanelBtn->setChecked(false);
    connect(primitivesPanelBtn, SIGNAL(clicked(bool)), this, SLOT(togglePrimitivesPanel(bool)));
    items->addWidget(primitivesPanelBtn);


    groupBox->setLayout(items);
    // Set group box style and add widgets
    layout->addWidget(groupBox);

    setupSignals();
}

auto ControlsWidget::setMesh(Mesh *mesh) -> void
{
    primitivesPanel.setMesh(mesh);
}

auto ControlsWidget::updateDropdowns(int i) -> void
{

    while(unitlengthBoxCombo->count() != 0)
        unitlengthBoxCombo->removeItem(0);

    while(showEdgeRatiosCombo->count() != 0)
        showEdgeRatiosCombo->removeItem(0);

    while(selectedEdgeBoxCombo->count() != 0);
        selectedEdgeBoxCombo->removeItem(0);


    unitlengthBoxCombo->addItem("Bounding");
    selectedEdgeBoxCombo->addItem("Bounding");

    showEdgeRatiosCombo->addItem("All");
    showEdgeRatiosCombo->addItem("Bounding");
    showEdgeRatiosCombo->addItem("Segments");

    for(auto j = 1; j < i; j++)
    {
        auto string = QString("Segment ") + QString::number(j);
        unitlengthBoxCombo->addItem(string);
        selectedEdgeBoxCombo->addItem(string);
        showEdgeRatiosCombo->addItem(string);
    }

}


auto ControlsWidget::computeBoxes() -> void
{

   BoxAdjustMethod method;
   switch(boxAdjustMethodCombo->currentIndex())
   {
   case -1:
       return;
       break;
   case 0:
       method = BoxAdjustMethod::LEAST_SQUARES;
       break;
   }
   emit computeBoxes(method);
}

auto ControlsWidget::setDesieredEdgeRatio() -> void
{
    //qDebug() << "ControlsWidget::manualUpdateUnitLength()";
    emit setDesieredEdgeRatio(adjustSelectedEdgeRatioSpin->value());
}

auto ControlsWidget::manualUpdateSelectedLength() -> void
{
    //qDebug() << "ControlsWidget::manualUpdateUnitLength()";
    emit manualAdjustSelectedLengthEdge(adjustSelectedEdgeRatioSpin->value());
}


auto ControlsWidget::setUnitLengthEdge(double val) -> void
{
    adjustSelectedEdgeRatioSpin->setValue(val);
}

auto ControlsWidget::getActiveControlPanel() -> QWidget*
{
    switch(activePanel)
    {
        case 0: return &contoursPanel; break;
        case 1: return &primitivesPanel; break;
//        case 1: return &projectorPanel; break;
//        case 2: return &drawingPanel; break;
//        case 3: return &userViewPanel; break;
    }
//    auto temp = new QWidget;
//    return temp;
}

auto ControlsWidget::getActiveControlPanelId() -> int
{
    return activePanel;
}

//auto ControlsWidget::setCurrentGuide(int i) -> void
//{
//    //qDebug() << "setCurrentGuide";
////    projectedGuideComboBox->setCurrentIndex(i);
//}

//auto ControlsWidget::setCurrentSurfaceGuide(int i) -> void
//{
////    drawingSurfaceGuideComboBox->setCurrentIndex(i);
//}

auto ControlsWidget::setupSignals() -> void
{
    connect(&contoursPanel, SIGNAL(updateViewer()), this, SIGNAL(updateViewer()));
    connect(&primitivesPanel, SIGNAL(updateViewer()), this, SIGNAL(updateViewer()));
    connect(&primitivesPanel, SIGNAL(initViewer()), this, SIGNAL(initViewer()));
//    connect(&objectPanel, SIGNAL(setActiveMesh(int)), this, SLOT(setActiveMesh(int)));
//    connect(&objectPanel, SIGNAL(meshDeleted()), this, SLOT(meshDeleted()));
//    connect(&projectorPanel, SIGNAL(projectorsUpdated()), this, SLOT(updateProjectors()));
//    connect(&projectorPanel, SIGNAL(viewpointChanged(QVector3D, QVector3D)), this, SLOT(changeViewpoint(QVector3D, QVector3D)));
//    connect(&userViewPanel, SIGNAL(viewpointChanged(QVector3D, QVector3D)), this, SLOT(changeViewpoint(QVector3D, QVector3D)));
}

//auto ControlsWidget::changeViewpoint(QVector3D p, QVector3D l) -> void
//{
//    //qDebug() << "ControlsWidget::changeViewpoint(QVector3D p)";
////    emit viewpointChanged(p, l);
//}

//auto ControlsWidget::meshDeleted() -> void
//{
//    activeMesh = 0;
//    projectorPanel.updateDropdowns();
//    emit updateActiveMesh(activeMesh);
//    emit projectorsUpdated();
//}

//auto ControlsWidget::setActiveMesh(int id) -> void
//{
//    activeMesh = id;
//    emit updateActiveMesh(activeMesh);
//}


//auto ControlsWidget::updateProjectors() -> void
//{
//    emit projectorsUpdated();
//}

//auto ControlsWidget::addMesh(Mesh *m) -> void
//{
//    meshes.push_back(m);
//    objectPanel.addMesh(m);
//    projectorPanel.addMesh(m);
//    drawingPanel.addMesh(m);
//    objectPanelBtn->setChecked(true);

//    //activePanel = 0;
//   // emit addControlsPanel();
//}


auto ControlsWidget::toggleContoursPanel(bool checked) -> void
{
    //qDebug() << "ControlsWidget::toggleObjectPanel " << checked;
    emit removePanel();
    if(checked)
    {
       activePanel = 0;
//       projectorPanelBtn->setChecked(false);
//       drawingPanelBtn->setChecked(false);
//       userViewPanelBtn->setChecked(false);
       emit addControlsPanel();
    }
}

auto ControlsWidget::togglePrimitivesPanel(bool checked) -> void
{
    //qDebug() << "ControlsWidget::toggleObjectPanel " << checked;
    emit removePanel();
    if(checked)
    {
       activePanel = 1;
//       projectorPanelBtn->setChecked(false);
//       drawingPanelBtn->setChecked(false);
//       userViewPanelBtn->setChecked(false);
       emit addControlsPanel();
       emit getMesh();
    }
}

//auto ControlsWidget::toggleObjectPanel(bool checked) -> void
//{
//    //qDebug() << "ControlsWidget::toggleObjectPanel " << checked;
//    emit removePanel();
//    if(checked)
//    {
//       activePanel = 0;
//       projectorPanelBtn->setChecked(false);
//       drawingPanelBtn->setChecked(false);
//       userViewPanelBtn->setChecked(false);
//       emit addControlsPanel();
//    }

//}

//auto ControlsWidget::toggleProjectorPanel(bool checked) -> void
//{
//    //qDebug() << "ControlsWidget::toggleProjectorPanel " << checked;
//    emit removePanel();
//    if(checked)
//    {
//       activePanel = 1;
//       objectPanelBtn->setChecked(false);
//       drawingPanelBtn->setChecked(false);
//       userViewPanelBtn->setChecked(false);
//       emit addControlsPanel();
//    }
//}

//auto ControlsWidget::toggleDrawingPanel(bool checked) -> void
//{
//    emit removePanel();
//    if(checked)
//    {
//        activePanel = 2;
//        objectPanelBtn->setChecked(false);
//        projectorPanelBtn->setChecked(false);
//        userViewPanelBtn->setChecked(false);
//        emit addControlsPanel();
//    }
//}

//auto ControlsWidget::toggleUserViewPanel(bool checked) -> void
//{
//    emit removePanel();
//    if(checked)
//    {
//        activePanel = 3;
//        objectPanelBtn->setChecked(false);
//        projectorPanelBtn->setChecked(false);
//        drawingPanelBtn->setChecked(false);
//        emit addControlsPanel();
//    }
//}

auto ControlsWidget::getWidgets(std::vector<QWidget*>& widgets) -> void
{
    widgets.push_back(&contoursPanel);
    widgets.push_back(&primitivesPanel);
//    widgets.push_back(&objectPanel);
//    widgets.push_back(&projectorPanel);
//    widgets.push_back(&drawingPanel);
//    widgets.push_back(&userViewPanel);
}

//auto ControlsWidget::updateProjectorWidgets() -> void
//{
//   projectorPanel.updateProjectorWidgets();
//}

//auto ControlsWidget::addProjector() -> void
//{
//    projectorPanel.addProjector();
//}

//auto ControlsWidget::setCurrentView(QVector3D pos, QVector3D lookAt) -> void
//{
//    projectorPanel.setCurrentView(pos, lookAt);
//}

//auto ControlsWidget::changeToViewpoint(int i) -> void
//{
//    switch(i)
//    {
//        case 0: changeViewpoint(QVector3D(0, -1, 1), QVector3D(0,0,0)); break;
//        case 1: changeViewpoint(UserViewpoint::getInstance()->getPosition(), UserViewpoint::getInstance()->getLookAt()); break;
//        default: projectorPanel.changeToViewpoint(i-2); break;
//    }
//}

#endif
