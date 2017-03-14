#ifndef MAINWINDOW_CPP
#define MAINWINDOW_CPP

#include "widgets/mainwindow.h"

#include <QFileDialog>
#include <iostream>

//#include "widgets/controlswidget.h"
//#include "opengl/glscene.h"

//#include "..\common\display_error.h"
//#include "HPPC.hpp"


//#include "utils/matlabengine.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , viewerWidget(this)
    , activeStackedWidget(0)
    , stackedWidget(new QStackedWidget)

//   , drawingSurfaceProjector(new DrawingSurfaceProjector(0, &meshes, &drawingWindow))
//   , monitorGLWidget(&projectors, drawingSurfaceProjector, parent)
//   , drawingWindow(drawingSurfaceProjector)
//   , mainControls(&projectors, drawingSurfaceProjector, &drawingWindow, parent)
//
//   , showGridOnDrawing(false)
//   , currentGuide(0)
//   , currentSurfaceGuide(0)
//   , sceneID(0)
//   , drawingSurfaceGuide(DrawingSurfaceGuide::NONE)
//   , regionHighlight(GridType::NONE)
//   , strokeNumber(0)
{
    createMenuBar();
    createLayout();
//    //setupMatWindow();
    setupSignals();
    setAttribute(Qt::WA_QuitOnClose);

    std::vector<QWidget*> panelWidgets;
    controlsWidget.getWidgets(panelWidgets);
    for(auto it = panelWidgets.begin(); it != panelWidgets.end(); it++)
    {
        stackedWidget->addWidget(*it);
    }

    //auto engine = MatlabEngine::getInstance();
}

MainWindow::~MainWindow()
{
    //monitorGLWidget.~MonitorGLWidget();
    //std::cout << "Close Main Window" << std::endl;
    //mesh.reset();
//    mRegistration->Unregister();
//    mat_window->setAttribute(Qt::WA_QuitOnClose);
//    mat_window->close();

//    mat_window.reset(nullptr);
}

auto MainWindow::setupSignals() -> void
{
    connect(&controlsWidget, SIGNAL(setRenderMode(int)), &viewerWidget, SLOT(setRenderMode(int)));
    connect(&controlsWidget, SIGNAL(addControlsPanel()), this, SLOT(addControlsPanel()));
    connect(&controlsWidget, SIGNAL(removePanel()), this, SLOT(removePanel()));
    connect(&controlsWidget, SIGNAL(updateViewer()), &viewerWidget, SLOT(updateViewer()));
    connect(&controlsWidget, SIGNAL(boundingBoxToggled(bool)), &viewerWidget, SLOT(boundingBoxToggled(bool)));
    connect(&controlsWidget, SIGNAL(segmentBoxesToggled(bool)), &viewerWidget, SLOT(segmentBoxesToggled(bool)));
    connect(&controlsWidget, SIGNAL(ellipsesToggled()), &viewerWidget, SLOT(ellipsesToggled()));
    connect(&controlsWidget, SIGNAL(renderBoxFacesToggled(bool)), &viewerWidget, SLOT(renderBoxFacesToggled(bool)));
    connect(&controlsWidget, SIGNAL(renderBoxFacesToggled(bool)), &viewerWidget, SLOT(renderBoxFacesToggled(bool)));
    connect(&controlsWidget, SIGNAL(edgeRatiosToggled(bool)), &viewerWidget, SLOT(edgeRatiosToggled(bool)));
    connect(&controlsWidget, SIGNAL(setUnitLengthBox(int)), &viewerWidget, SLOT(setUnitLengthBox(int)));
    connect(&controlsWidget, SIGNAL(setUnitLengthEdge(int)), &viewerWidget, SLOT(setUnitLengthEdge(int)));
    connect(&controlsWidget, SIGNAL(setShowEdgeRatio(int)), &viewerWidget, SLOT(setShowEdgeRatio(int)));
    connect(&controlsWidget, SIGNAL(renderDiagLinesToggled(bool)), &viewerWidget, SLOT(renderDiagLinesToggled(bool)));
    connect(&controlsWidget, SIGNAL(renderThirdsDiagLinesToggled(bool)), &viewerWidget, SLOT(renderThirdsDiagLineToggled(bool)));
    connect(&controlsWidget, SIGNAL(renderTwoThirdsDiagLinesToggled(bool)), &viewerWidget, SLOT(renderTwoThirdsDiagLineToggled(bool)));
    connect(&controlsWidget, SIGNAL(renderThirdsLineToggled(bool)), &viewerWidget, SLOT(renderThirdsLineToggled(bool)));
    connect(&controlsWidget, SIGNAL(renderTwoThirdsLineToggled(bool)), &viewerWidget, SLOT(renderTwoThirdsLineToggled(bool)));
    connect(&controlsWidget, SIGNAL(renderMidLinesToggled(bool)), &viewerWidget, SLOT(renderMidLinesToggled(bool)));
    connect(&controlsWidget, SIGNAL(renderHiddenLinesToggled(bool)), &viewerWidget, SLOT(renderHiddenLinesToggled(bool)));
    connect(&controlsWidget, SIGNAL(renderPerspectiveLinesToggled(bool)), &viewerWidget, SLOT(renderPerspectiveLinesToggled(bool)));
    connect(&controlsWidget, SIGNAL(setSelectedEdge(int)), &viewerWidget, SLOT(setSelectedEdge(int)));
    connect(&controlsWidget, SIGNAL(manualAdjustSelectedLengthEdge(double)), &viewerWidget, SLOT(manualAdjustSelectedLengthEdge(double)));
    connect(&controlsWidget, SIGNAL(bbXRotChanged(double)), &viewerWidget, SLOT(bbXRotChanged(double)));
    connect(&controlsWidget, SIGNAL(bbYRotChanged(double)), &viewerWidget, SLOT(bbYRotChanged(double)));
    connect(&controlsWidget, SIGNAL(bbZRotChanged(double)), &viewerWidget, SLOT(bbZRotChanged(double)));
    connect(&controlsWidget, SIGNAL(setDesieredEdgeRatio(double)), &viewerWidget, SLOT(setDesieredEdgeRatio(double)));
    connect(&controlsWidget, SIGNAL(computeBoxes(BoxAdjustMethod)), &viewerWidget, SLOT(computeBoxes(BoxAdjustMethod)));
    connect(&controlsWidget, SIGNAL(setSelectedEdgeBox(int)), &viewerWidget, SLOT(setSelectedEdgeBox(int)));
    connect(&controlsWidget, SIGNAL(showObjectToggled(bool)), &viewerWidget, SLOT(showObjectToggled(bool)));
    connect(&controlsWidget, SIGNAL(printScreen()), &viewerWidget, SLOT(printScreen()));
    connect(&controlsWidget, SIGNAL(getMesh()), this, SLOT(getMesh()));
    connect(&controlsWidget, SIGNAL(saveJson()), this, SLOT(saveJson()));
    connect(&controlsWidget, SIGNAL(initViewer()), this, SLOT(initViewer()));
    connect(&controlsWidget, SIGNAL(zoomChanged(double)), &viewerWidget, SLOT(zoomChanged(double)));
    connect(&controlsWidget, SIGNAL(findRelations()), &viewerWidget, SLOT(findRelations()));
    connect(&controlsWidget, SIGNAL(acceptRelations()), &viewerWidget, SLOT(acceptRelations()));
    connect(&controlsWidget, SIGNAL(clearRelations()), &viewerWidget, SLOT(clearRelations()));
    connect(&controlsWidget, SIGNAL(generateCandidates()), &viewerWidget, SLOT(generateCandidates()));
    connect(&controlsWidget, SIGNAL(planesToggled()), &viewerWidget, SLOT(planesToggled()));
    connect(&controlsWidget, SIGNAL(candidatesToggled()), &viewerWidget, SLOT(candidatesToggled()));
    connect(&controlsWidget, SIGNAL(guidesToggled()), &viewerWidget, SLOT(guidesToggled()));
    connect(&controlsWidget, SIGNAL(runOptimisation()), &viewerWidget, SLOT(runOptimisation()));
    connect(&controlsWidget, SIGNAL(selectSegment(int)), &viewerWidget, SLOT(selectSegment(int)));
//    connect(&controlsWidget, SIGNAL(showClustersToggled(bool)), &viewerWidget, SLOT(showClustersToggled(bool)));
//    connect(&controlsWidget, SIGNAL(showNextCluster()), &viewerWidget, SLOT(showNextCluster()));
//    connect(&controlsWidget, SIGNAL(runOptimisationOnCluster()), &viewerWidget, SLOT(runOptimisationOnCluster()));
//    connect(&controlsWidget, SIGNAL(outputData()), &viewerWidget, SLOT(outputData()));

    connect(&viewerWidget, SIGNAL(setUnitLengthEdge(double)), &controlsWidget, SLOT(setUnitLengthEdge(double)));
    connect(&viewerWidget, SIGNAL(updateDropdowns(int)), &controlsWidget, SLOT(updateDropdowns(int)));

//    connect(&mainControls, SIGNAL(updateActiveMesh(int)), &monitorGLWidget, SLOT(setActiveMesh(int)) );
//    connect(&mainControls, SIGNAL(projectorsUpdated()), this, SLOT(updateProjectors()));
//    connect(&mainControls, SIGNAL(viewpointChanged(QVector3D, QVector3D)), &monitorGLWidget, SLOT(setEyePosAndLook(QVector3D,QVector3D)));
//    connect(&monitorGLWidget, SIGNAL(setCurrentView(QVector3D, QVector3D)), &mainControls, SLOT(setCurrentView(QVector3D, QVector3D)));
//    connect(&monitorGLWidget, SIGNAL(keyPressed(QKeyEvent*)), this, SLOT(keyPressed(QKeyEvent*)));
//    connect(&drawingWindow, SIGNAL(keyPressed(QKeyEvent*)), this, SLOT(keyPressed(QKeyEvent*)));
//    connect(&mainControls, SIGNAL(updateProjectorGuide(int)), this, SLOT(setCurrentGuide(int)));
//    connect(&mainControls, SIGNAL(updateDrawingSurfaceGuide(int)), this, SLOT(setDrawingSurfaceGuide(int)));
//    connect(&monitorGLWidget, SIGNAL(wheelMove(QWheelEvent*)), this, SLOT(wheelMove(QWheelEvent*)));
//    connect(&drawingWindow, SIGNAL(wheelMove(QWheelEvent*)), this, SLOT(wheelMove(QWheelEvent*)));

}


auto MainWindow::saveJson() -> void
{
    viewerWidget.writePrimitivesAndRelations();
}

auto MainWindow::initViewer() -> void
{
    qDebug() << "MainWindow::initViewer()";
    viewerWidget.initViewer();
}

auto MainWindow::getMesh() -> void
{
    auto mesh = viewerWidget.getMesh();
    controlsWidget.setMesh(mesh);
}

//auto MainWindow::setupMatWindow() -> void
//{
//    mLink = HPPC::CreateLink();
//    mat_window->onShowClicked();
//    mRegistration = mLink->RegisterWindow(mat_window->winId());
//}

//auto MainWindow::closeEvent(QCloseEvent *event) -> void
//{
////    std::cout << "Close Event" << std::endl;
////    mat_window->onHideClicked();
////    mat_window.reset(nullptr);
//}

void MainWindow::importMesh()
{
#ifdef __APPLE__
    static const auto location = QString("/Users/JamesHennessey/Dropbox/engd/multimodel_drawing_assistance/3D_models");
#else
    static const auto location = QString("C:/Users/James Hennesay/Dropbox/engd/multimodel_drawing_assistance/3D_models/");
#endif

    auto fileName = QFileDialog::getOpenFileName(this,
                                    QString("Import Mesh"),
                                    location);
    if (!fileName.isEmpty()) {
        viewerWidget.importMesh(fileName);
    }
}



//auto MainWindow::addMesh(Mesh *m) -> void
//{
//    //qDebug() << "MainWindow::addMesh(Mesh *m)";
//    m->setMeshId(meshes.size());
//    meshes.push_back(m);
//    monitorGLWidget.addMesh(m);
//    mainControls.addMesh(m);
//}

auto MainWindow::createMenuBar() -> void
{
    //Menu Bar Actions
    auto *fileMenu = new QMenu(tr("&File"), this);

//    auto *openAct = new QAction(tr("&Open..."), this);
//    openAct->setShortcut(tr("Ctrl+O"));
//    connect(openAct, SIGNAL(triggered()), this, SLOT(openScene()));

//    auto *saveAct = new QAction(tr("&Save..."), this);
//    saveAct->setShortcut(tr("Ctrl+S"));
//    connect(saveAct, SIGNAL(triggered()), this, SLOT(saveScene()));

    auto *importAct = new QAction(tr("&Import..."), this);
    importAct->setShortcut(tr("Ctrl+I"));
    connect(importAct, SIGNAL(triggered()), this, SLOT(importMesh()));

//    fileMenu->addAction(openAct);
//    fileMenu->addAction(saveAct);
    fileMenu->addAction(importAct);
    fileMenu->addSeparator();

    menuBar()->addMenu(fileMenu);
}

auto MainWindow::createLayout() -> void
{

    //delete controlPanel;

    widgetsLayout.setMargin(0);
    widgetsLayout.setHorizontalSpacing(20);
    widgetsLayout.setVerticalSpacing(0);
//    widgetsLayout.addWidget(&mainControls,    0, 0, 2, 1);
//    widgetsLayout.addWidget(&monitorGLWidget, 0, 1, 2, 8);

    widgetsLayout.addWidget(&controlsWidget,  0, 0, 2, 1);
    widgetsLayout.addWidget(&viewerWidget,    0, 1, 2, 8);

    auto *mainWidget = new QWidget(this);
    mainWidget->setLayout(&widgetsLayout);
    setCentralWidget(mainWidget);
}

auto MainWindow::addControlsPanel() -> void
{

    activeStackedWidget = controlsWidget.getActiveControlPanelId();
    stackedWidget->setCurrentIndex(activeStackedWidget);

    widgetsLayout.setMargin(0);
    widgetsLayout.setHorizontalSpacing(20);
    widgetsLayout.setVerticalSpacing(0);
    widgetsLayout.addWidget(&controlsWidget,    0, 0, 2, 1);
    widgetsLayout.addWidget(stackedWidget,      0, 1, 2, 2);
    widgetsLayout.addWidget(&viewerWidget,      0, 3, 2, 6);

    auto *mainWidget = new QWidget(this);
    mainWidget->setLayout(&widgetsLayout);
    setCentralWidget(mainWidget);
}

auto MainWindow::removePanel() -> void
{
    createLayout();
}


////auto MainWindow::keyPressEvent(QKeyEvent * event) -> void
////{
////    //std::cout << "Key Pressed" << std::endl;
////    //QGridLayout* widgetsLayout = new QGridLayout();
//////    widgetsLayout.setMargin(0);
//////    widgetsLayout.setHorizontalSpacing(20);
//////    widgetsLayout.setVerticalSpacing(0);
//////    //addWidget(QWidget * widget, int fromRow, int fromColumn, int rowSpan, int columnSpan)
//////    widgetsLayout.addWidget(mainControls, 0, 0, 2, 2);
//////    widgetsLayout.addWidget(monitorGLWidget, 0, 2, 2, 3);

////    //mainWidget->setLayout(widgetsLayout);
////    //setCentralWidget(mainWidget);
////}


//void MainWindow::keyPressed(QKeyEvent* event)
//{
//    qDebug() << "MainWindow::keyPressed";
//    //No projector
//    if(event->key() == Qt::Key_Q)
//    {
//        currentGuide = 0;
////        projectors[0]->setActiveCheckbox(false);
////        projectors[1]->setActiveCheckbox(false);
////        projectors[2]->setActiveCheckbox(false);
////        projectors[3]->setActiveCheckbox(false);
////        currentGridType = GridType::NONE;
////        updateProjectors();
//    }
//    // Regular Grid
//    else if(event->key() == Qt::Key_W)
//    {
//       //qDebug() << "Q KeyPress";
//        currentGuide = 1;
////        projectors[0]->setActiveCheckbox(true);
////        projectors[1]->setActiveCheckbox(false);
////        projectors[2]->setActiveCheckbox(false);
////        projectors[3]->setActiveCheckbox(true);
////        projectors[0]->renderModeChanged(2);
////        projectors[0]->setGridType(GridType::REGULAR);
////        projectors[3]->renderModeChanged(2);
////        projectors[3]->setGridType(GridType::REGULAR);
////        currentGridType = GridType::REGULAR;
////        updateProjectors();
//    }
//    // Crosshair Grid
//    else if(event->key() == Qt::Key_E)
//    {
//       //qDebug() << "Q KeyPress";
//        currentGuide = 2;
////       projectors[0]->setActiveCheckbox(true);
////       projectors[1]->setActiveCheckbox(false);
////       projectors[2]->setActiveCheckbox(false);
////       projectors[3]->setActiveCheckbox(true);
////       projectors[0]->renderModeChanged(2);
////       projectors[0]->setGridType(GridType::CROSSHAIR);
////       projectors[3]->renderModeChanged(2);
////       projectors[3]->setGridType(GridType::CROSSHAIR);
////       currentGridType = GridType::CROSSHAIR;
////       updateProjectors();
//    }
//    //Custom Grid
//    else if(event->key() == Qt::Key_R)
//    {
//       //qDebug() << "Q KeyPress";
//        currentGuide = 3;
////       projectors[0]->setActiveCheckbox(true);
////       projectors[1]->setActiveCheckbox(false);
////       projectors[2]->setActiveCheckbox(false);
////       projectors[3]->setActiveCheckbox(true);

////       projectors[0]->renderModeChanged(2);
////       projectors[0]->setGridType(GridType::CUSTOM);
////       projectors[3]->renderModeChanged(2);
////       projectors[3]->setGridType(GridType::CUSTOM);
////       currentGridType = GridType::CUSTOM;
////       updateProjectors();
//    }
//    // SUBDIVIDE Mode
//    else if(event->key() == Qt::Key_T)
//    {
//        currentGuide = 4;
////        projectors[0]->setActiveCheckbox(true);
////        projectors[1]->setActiveCheckbox(false);
////        projectors[2]->setActiveCheckbox(false);
////        projectors[3]->setActiveCheckbox(true);

////        projectors[0]->renderModeChanged(2);
////        projectors[0]->setGridType(GridType::SUBDIVIDE);
////        projectors[3]->renderModeChanged(2);
////        projectors[3]->setGridType(GridType::SUBDIVIDE);
////        currentGridType = GridType::SUBDIVIDE;
////        updateProjectors();
//    }
//    // Wireframe Mode
//    else if(event->key() == Qt::Key_Y)
//    {
//       //qDebug() << "Q KeyPress";
//        currentGuide = 5;
////       projectors[0]->setActiveCheckbox(true);
////       projectors[1]->setActiveCheckbox(true);
////       projectors[2]->setActiveCheckbox(true);
////       projectors[3]->setActiveCheckbox(false);
////       projectors[0]->renderModeChanged(1);
////       projectors[1]->renderModeChanged(1);
////       projectors[2]->renderModeChanged(1);
////       currentGridType = GridType::NONE;
////       updateProjectors();
//    }
//    // SEGMENTATION MODE
//    else if(event->key() == Qt::Key_U)
//    {
//        currentGuide = 6;
////       projectors[0]->setActiveCheckbox(true);
////       projectors[1]->setActiveCheckbox(true);
////       projectors[2]->setActiveCheckbox(true);
////       projectors[3]->setActiveCheckbox(false);
////       projectors[0]->renderModeChanged(0);
////       projectors[1]->renderModeChanged(0);
////       projectors[2]->renderModeChanged(0);
////       currentGridType = GridType::NONE;
////       updateProjectors();
//    }
//    else if(event->key() == Qt::Key_Z)
//    {
//        currentGuide = 0;
//        setDrawingSurfaceGuide(4);
//    }
//    else if(event->key() == Qt::Key_X)
//    {
//        currentGuide = 0;
//        setDrawingSurfaceGuide(5);
//    }
//    else if(event->key() == Qt::Key_C)
//    {
//        currentGuide = 0;
//        setDrawingSurfaceGuide(6);
//    }
//    else if(event->key() == Qt::Key_A)
//    {
//        currentGuide = 0;
//        setDrawingSurfaceGuide(7);
//    }
//    else if(event->key() == Qt::Key_S)
//    {
//        currentGuide = 0;
//        setDrawingSurfaceGuide(8);
//    }
//    else if(event->key() == Qt::Key_D)
//    {
//        currentGuide = 0;
//        setDrawingSurfaceGuide(9);
//    }
//    else if(event->key() == Qt::Key_F)
//    {
//        currentGuide = 0;
//        setDrawingSurfaceGuide(10);
//    }
//    else if(event->key() == Qt::Key_J)
//    {
//        currentGuide = 0;
//        setDrawingSurfaceGuide(11);
//    }
//    else if(event->key() == Qt::Key_K)
//    {
//        currentGuide = 0;
//        setDrawingSurfaceGuide(12);
//    }
//    else if(event->key() == Qt::Key_L)
//    {
//        currentGuide = 0;
//        setDrawingSurfaceGuide(13);
//    }
//    else if(event->key() == Qt::Key_P)
//    {
//        currentGuide = 0;
//        setDrawingSurfaceGuide(14);
//    }
//    else if(event->key() == Qt::Key_Left)
//    {
//        prevGuide();
//    }
//    else if(event->key() == Qt::Key_Right)
//    {
//        nextGuide();
//    }
//    else if(event->key() == Qt::Key_Up)
//    {
//        prevSurfaceGuide();
//    }
//    else if(event->key() == Qt::Key_Down)
//    {
//        nextSurfaceGuide();
//    }
//    else
//    {


//        if(event->key() == Qt::Key_G)
//        {
//            showGridOnDrawing = true;
//        }
//        else if(event->key() == Qt::Key_H)
//        {
//            showGridOnDrawing = false;
//            drawingWindow.removeOverlay();
//        }
////        qDebug() << "showGridOnDrawing " << showGridOnDrawing;
////        if(showGridOnDrawing)
////        {
////            if(currentGridType == GridType::NONE)
////                drawingWindow.removeOverlay();
////            else
////                drawingWindow.addOverlay(currentGridType);
////        }
//    }
//    updateGuide();



//}

//auto MainWindow::nextSurfaceGuide() -> void
//{
//    currentSurfaceGuide++;
//    if(currentSurfaceGuide > number_of_surface_guides)
//            currentSurfaceGuide = 0;
//    setDrawingSurfaceGuide(currentSurfaceGuide);
//    mainControls.setCurrentSurfaceGuide(currentSurfaceGuide);
//}

//auto MainWindow::prevSurfaceGuide() -> void
//{
//    currentSurfaceGuide--;
//    if(currentSurfaceGuide < 0)
//            currentSurfaceGuide = number_of_surface_guides;
//    setDrawingSurfaceGuide(currentSurfaceGuide);
//    mainControls.setCurrentSurfaceGuide(currentSurfaceGuide);

//}

//auto MainWindow::nextGuide() -> void
//{
//     currentGuide++;
//     if(currentGuide > number_of_guides)
//             currentGuide = 0;
//     //updateGuide();
//     mainControls.setCurrentGuide(currentGuide);
//}
//auto MainWindow::prevGuide() -> void
//{
//    currentGuide--;
//    if(currentGuide < 0)
//            currentGuide = number_of_guides;
//    //updateGuide();
//    mainControls.setCurrentGuide(currentGuide);
//}

//auto MainWindow::setCurrentGuide(int i) -> void
//{
//    currentGuide = i;

//    updateGuide();
//}

//auto MainWindow::setDrawingSurfaceGuide(int i) -> void
//{
//    switch(i)
//    {
//        case 0: drawingSurfaceGuide = DrawingSurfaceGuide::NONE; break;
//        case 1: drawingSurfaceGuide = DrawingSurfaceGuide::MIRROR; break;
//        case 2: drawingSurfaceGuide = DrawingSurfaceGuide::SHADOW; break;
//        case 3: drawingSurfaceGuide = DrawingSurfaceGuide::CONVEX_HULL; break;
//        case 4: drawingSurfaceGuide = DrawingSurfaceGuide::LINE1; break;
//        case 5: drawingSurfaceGuide = DrawingSurfaceGuide::LINE2; break;
//        case 6: drawingSurfaceGuide = DrawingSurfaceGuide::LINE3; break;
//        case 7: drawingSurfaceGuide = DrawingSurfaceGuide::BOX1; break;
//        case 12: drawingSurfaceGuide = DrawingSurfaceGuide::BOX7; break;
//        case 13: drawingSurfaceGuide = DrawingSurfaceGuide::BOX7; break;

//    }
//    currentSurfaceGuide = i;
//    updateGuide();
//}

//auto MainWindow::updateGuide() -> void
//{
//    qDebug() << "MainWindow::updateGuide(";
//    if(currentGuide == 0)
//    {
//        projectors[0]->setActiveCheckbox(false);
//        projectors[1]->setActiveCheckbox(false);
//        projectors[2]->setActiveCheckbox(false);
//        projectors[3]->setActiveCheckbox(false);
//        currentGridType = GridType::NONE;
//        regionHighlight = GridType::NONE;
//        updateProjectors();
//    }
//    // Regular Grid
//    else if(currentGuide == 1)
//    {
//       //qDebug() << "Q KeyPress";
//        projectors[0]->setActiveCheckbox(true);
//        projectors[1]->setActiveCheckbox(false);
//        projectors[2]->setActiveCheckbox(false);
//        projectors[3]->setActiveCheckbox(true);
//        projectors[0]->renderModeChanged(2);
//        projectors[0]->setGridType(GridType::REGULAR);
//        projectors[3]->renderModeChanged(2);
//        projectors[3]->setGridType(GridType::REGULAR);
//        currentGridType = GridType::REGULAR;
//        regionHighlight = GridType::NONE;
//        updateProjectors();
//    }
//    // Crosshair Grid
//    else if(currentGuide == 2)
//    {
//       //qDebug() << "Q KeyPress";
//       projectors[0]->setActiveCheckbox(true);
//       projectors[1]->setActiveCheckbox(false);
//       projectors[2]->setActiveCheckbox(false);
//       projectors[3]->setActiveCheckbox(true);
//       projectors[0]->renderModeChanged(2);
//       projectors[0]->setGridType(GridType::CROSSHAIR);
//       projectors[3]->renderModeChanged(2);
//       projectors[3]->setGridType(GridType::CROSSHAIR);
//       currentGridType = GridType::CROSSHAIR;
//       regionHighlight = GridType::NONE;
//       updateProjectors();
//    }
//    //Custom Grid
//    else if(currentGuide == 3)
//    {
//       //qDebug() << "Q KeyPress";
//       projectors[0]->setActiveCheckbox(true);
//       projectors[1]->setActiveCheckbox(false);
//       projectors[2]->setActiveCheckbox(false);
//       projectors[3]->setActiveCheckbox(true);

//       projectors[0]->renderModeChanged(2);
//       projectors[0]->setGridType(GridType::CUSTOM);
//       projectors[3]->renderModeChanged(2);
//       projectors[3]->setGridType(GridType::CUSTOM);
//       currentGridType = GridType::CUSTOM;
//       regionHighlight = GridType::NONE;
//       updateProjectors();
//    }
//    // SUBDIVIDE Mode
//    else if(currentGuide == 4)
//    {
//        projectors[0]->setActiveCheckbox(true);
//        projectors[1]->setActiveCheckbox(false);
//        projectors[2]->setActiveCheckbox(false);
//        projectors[3]->setActiveCheckbox(true);

//        projectors[0]->renderModeChanged(2);
//        projectors[0]->setGridType(GridType::SUBDIVIDE);
//        projectors[3]->renderModeChanged(2);
//        projectors[3]->setGridType(GridType::SUBDIVIDE);
//        currentGridType = GridType::SUBDIVIDE;
//        regionHighlight = GridType::NONE;
//        updateProjectors();
//    }
//    // Wireframe Mode
//    else if(currentGuide == 5)
//    {
//       //qDebug() << "Q KeyPress";
//       projectors[0]->setActiveCheckbox(true);
//       projectors[1]->setActiveCheckbox(true);
//       projectors[2]->setActiveCheckbox(true);
//       projectors[3]->setActiveCheckbox(false);
////       projectors[0]->renderModeChanged(1);
////       projectors[1]->renderModeChanged(1);
////       projectors[2]->renderModeChanged(1);
//       GridType t;
//       switch(sceneID)
//       {
//            case 0: t = GridType::TEAPOT1_WIRE; break;
//            //case 1: t = GridType::TEAPOT2_SEG; break;
//       }
//       projectors[0]->renderModeChanged(2);
//       projectors[0]->setGridType(t);
//       projectors[3]->renderModeChanged(2);
//       projectors[3]->setGridType(t);
//       //Still none as no guide on drawing surface
//       currentGridType = GridType::NONE;
//       regionHighlight = GridType::NONE;
//       updateProjectors();
//    }
//    // SEGMENTATION MODE
//    else if(currentGuide == 6)
//    {

//       projectors[0]->setActiveCheckbox(true);
//       projectors[1]->setActiveCheckbox(true);
//       projectors[2]->setActiveCheckbox(true);
//       projectors[3]->setActiveCheckbox(false);
////       projectors[0]->renderModeChanged(0);
////       projectors[1]->renderModeChanged(0);
////       projectors[2]->renderModeChanged(0);


//       GridType t;
//       switch(sceneID)
//       {
//            case 0: t = GridType::TEAPOT1_SEG; break;
//            //case 1: t = GridType::TEAPOT2_SEG; break;
//       }
//       projectors[0]->renderModeChanged(2);
//       projectors[0]->setGridType(t);
//       projectors[3]->renderModeChanged(2);
//       projectors[3]->setGridType(t);
//       //Still none as no guide on drawing surface
//       currentGridType = GridType::NONE;
//       regionHighlight = GridType::NONE;
//       updateProjectors();
//    }
//    //Suggestive contours
//    else if(currentGuide == 7)
//    {

//       projectors[0]->setActiveCheckbox(true);
//       projectors[1]->setActiveCheckbox(true);
//       projectors[2]->setActiveCheckbox(true);
//       projectors[3]->setActiveCheckbox(false);
////       projectors[0]->renderModeChanged(0);
////       projectors[1]->renderModeChanged(0);
////       projectors[2]->renderModeChanged(0);


//       GridType t;
//       switch(sceneID)
//       {
//            case 0: t = GridType::TEAPOT1_SC; break;
//            //case 1: t = GridType::TEAPOT2_SEG; break;
//       }
//       projectors[0]->renderModeChanged(2);
//       projectors[0]->setGridType(t);
//       projectors[3]->renderModeChanged(2);
//       projectors[3]->setGridType(t);
//       //Still none as no guide on drawing surface
//       currentGridType = GridType::NONE;
//       regionHighlight = GridType::NONE;
//       updateProjectors();
//    }
//    //Block-In
//    else if(currentGuide == 8)
//    {

//        projectors[0]->setActiveCheckbox(true);
//        projectors[1]->setActiveCheckbox(false);
//        projectors[2]->setActiveCheckbox(false);
//        projectors[3]->setActiveCheckbox(true);
////       projectors[0]->renderModeChanged(0);
////       projectors[1]->renderModeChanged(0);
////       projectors[2]->renderModeChanged(0);


//       GridType t;
//       switch(sceneID)
//       {
//            case 0: t = GridType::TEAPOT1_CONVEX_HULL; break;
//            //case 1: t = GridType::TEAPOT2_SEG; break;
//       }
//       projectors[0]->renderModeChanged(2);
//       projectors[0]->setGridType(t);
//       projectors[3]->renderModeChanged(2);
//       projectors[3]->setGridType(t);

//        regionHighlight = GridType::NONE;


//       currentGridType = t;
//       updateProjectors();
//    }
//    //RH Body
//    else if(currentGuide == 9)
//    {

//       projectors[0]->setActiveCheckbox(true);
//       projectors[1]->setActiveCheckbox(false);
//       projectors[2]->setActiveCheckbox(false);
//       projectors[3]->setActiveCheckbox(false);
////       projectors[0]->renderModeChanged(0);
////       projectors[1]->renderModeChanged(0);
////       projectors[2]->renderModeChanged(0);


//       GridType t;
//       switch(sceneID)
//       {
//            case 0: t = GridType::TEAPOT1_RH_BODY; break;
//            //case 1: t = GridType::TEAPOT2_SEG; break;
//       }
//       projectors[0]->renderModeChanged(2);
//       projectors[0]->setGridType(t);
//       projectors[3]->renderModeChanged(2);
//       projectors[3]->setGridType(t);
//       //Still none as no guide on drawing surface
//       currentGridType = GridType::NONE;
//       regionHighlight = GridType::TEAPOT1_RH_BODY;
//       updateProjectors();
//    }
//    //RH Spout
//    else if(currentGuide == 10)
//    {

//       projectors[0]->setActiveCheckbox(true);
//       projectors[1]->setActiveCheckbox(false);
//       projectors[2]->setActiveCheckbox(false);
//       projectors[3]->setActiveCheckbox(false);
////       projectors[0]->renderModeChanged(0);
////       projectors[1]->renderModeChanged(0);
////       projectors[2]->renderModeChanged(0);


//       GridType t;
//       switch(sceneID)
//       {
//            case 0: t = GridType::TEAPOT1_RH_SPOUT; break;
//            //case 1: t = GridType::TEAPOT2_SEG; break;
//       }
//       projectors[0]->renderModeChanged(2);
//       projectors[0]->setGridType(t);
//       projectors[3]->renderModeChanged(2);
//       projectors[3]->setGridType(t);
//       //Still none as no guide on drawing surface
//       currentGridType = GridType::NONE;
//       regionHighlight = GridType::TEAPOT1_RH_SPOUT;
//       updateProjectors();
//    }
//    //RH Spout
//    else if(currentGuide == 11)
//    {

//       projectors[0]->setActiveCheckbox(true);
//       projectors[1]->setActiveCheckbox(false);
//       projectors[2]->setActiveCheckbox(false);
//       projectors[3]->setActiveCheckbox(false);
////       projectors[0]->renderModeChanged(0);
////       projectors[1]->renderModeChanged(0);
////       projectors[2]->renderModeChanged(0);


//       GridType t;
//       switch(sceneID)
//       {
//            case 0: t = GridType::TEAPOT1_RH_LID; break;
//            //case 1: t = GridType::TEAPOT2_SEG; break;
//       }
//       projectors[0]->renderModeChanged(2);
//       projectors[0]->setGridType(t);
//       projectors[3]->renderModeChanged(2);
//       projectors[3]->setGridType(t);
//       //Still none as no guide on drawing surface
//       currentGridType = GridType::NONE;
//       updateProjectors();
//       regionHighlight = GridType::TEAPOT1_RH_LID;
//    }
//    else if(currentGuide == 12)
//    {

//       projectors[0]->setActiveCheckbox(true);
//       projectors[1]->setActiveCheckbox(false);
//       projectors[2]->setActiveCheckbox(false);
//       projectors[3]->setActiveCheckbox(false);
////       projectors[0]->renderModeChanged(0);
////       projectors[1]->renderModeChanged(0);
////       projectors[2]->renderModeChanged(0);


//       GridType t;
//       switch(sceneID)
//       {
//            case 0: t = GridType::TEAPOT1_RH_HANDLE; break;
//            //case 1: t = GridType::TEAPOT2_SEG; break;
//       }
//       projectors[0]->renderModeChanged(2);
//       projectors[0]->setGridType(t);
//       projectors[3]->renderModeChanged(2);
//       projectors[3]->setGridType(t);
//       //Still none as no guide on drawing surface
//       currentGridType = GridType::NONE;
//       regionHighlight = GridType::TEAPOT1_RH_HANDLE;
//       updateProjectors();
//    }

//    if(drawingSurfaceGuide != DrawingSurfaceGuide::NONE)
//    {
//        if(drawingSurfaceGuide != DrawingSurfaceGuide::MIRROR)
//        {
//            qDebug() << "current drawing surface guide " << currentSurfaceGuide;
//            drawingWindow.addOverlay(intToSurfaceGuide(currentSurfaceGuide));
//        }
//        else
//            drawingWindow.addOverlay(currentGridType);
//    }
//    else
//    {
//        drawingWindow.removeOverlay();
//    }
//}

//auto MainWindow::intToSurfaceGuide(int i) -> DrawingSurfaceGuide
//{
//    switch(i)
//    {
//        case 0: return DrawingSurfaceGuide::NONE; break;
//        case 1: return DrawingSurfaceGuide::MIRROR; break;
//        case 2: return DrawingSurfaceGuide::SHADOW; break;
//        case 3: return DrawingSurfaceGuide::CONVEX_HULL; break;
//        case 4: return DrawingSurfaceGuide::LINE1; break;
//        case 5: return DrawingSurfaceGuide::LINE2; break;
//        case 6: return DrawingSurfaceGuide::LINE3; break;
//        case 7: return DrawingSurfaceGuide::BOX1; break;
//        case 8: return DrawingSurfaceGuide::BOX2; break;
//        case 9: return DrawingSurfaceGuide::BOX3; break;
//        case 10: return DrawingSurfaceGuide::BOX4; break;
//        case 11: return DrawingSurfaceGuide::BOX5; break;
//        case 12: return DrawingSurfaceGuide::BOX6; break;
//        case 13: return DrawingSurfaceGuide::BOX7; break;
//        case 14: return DrawingSurfaceGuide::BOX8; break;
//    }
//}

//auto MainWindow::wheelMove(QWheelEvent *event) -> void
//{
//    if(regionHighlight != GridType::NONE)
//    {

//        auto pt =  event->angleDelta();
//        if(pt.y() > 0)
//            strokeNumber--;
//        else if(pt.y() < 0)
//            strokeNumber++;

//        if(regionHighlight == GridType::TEAPOT1_RH_BODY)
//        {

//            const static int number_body_strokes = 31;
//            if(strokeNumber < 0)
//                strokeNumber = number_body_strokes;
//            else if(strokeNumber > number_body_strokes)
//                strokeNumber = 0;

//            //qDebug() << "strokeNumber " << strokeNumber;
//            GridType t;
//            switch(strokeNumber)
//            {
//                case 0: t = GridType::TEAPOT1_RH_BODY; break;
//                case 1: t = GridType::TEAPOT1_RH_BODY_STROKE1; break;
//                case 2: t = GridType::TEAPOT1_RH_BODY_STROKE2; break;
//                case 3: t = GridType::TEAPOT1_RH_BODY_STROKE3; break;
//                case 4: t = GridType::TEAPOT1_RH_BODY_STROKE4; break;
//                case 5: t = GridType::TEAPOT1_RH_BODY_STROKE5; break;
//                case 6: t = GridType::TEAPOT1_RH_BODY_STROKE6; break;
//                case 7: t = GridType::TEAPOT1_RH_BODY_STROKE7; break;
//                case 8: t = GridType::TEAPOT1_RH_BODY_STROKE8; break;
//                case 9: t = GridType::TEAPOT1_RH_BODY_STROKE9; break;
//                case 10: t = GridType::TEAPOT1_RH_BODY_STROKE10; break;
//                case 11: t = GridType::TEAPOT1_RH_BODY_STROKE11; break;
//                case 12: t = GridType::TEAPOT1_RH_BODY_STROKE12; break;
//                case 13: t = GridType::TEAPOT1_RH_BODY_STROKE13; break;
//                case 14: t = GridType::TEAPOT1_RH_BODY_STROKE14; break;
//                case 15: t = GridType::TEAPOT1_RH_BODY_STROKE15; break;
//                case 16: t = GridType::TEAPOT1_RH_BODY_STROKE16; break;
//                case 17: t = GridType::TEAPOT1_RH_BODY_STROKE17; break;
//                case 18: t = GridType::TEAPOT1_RH_BODY_STROKE18; break;
//                case 19: t = GridType::TEAPOT1_RH_BODY_STROKE19; break;
//                case 20: t = GridType::TEAPOT1_RH_BODY_STROKE20; break;
//                case 21: t = GridType::TEAPOT1_RH_BODY_STROKE21; break;
//                case 22: t = GridType::TEAPOT1_RH_BODY_STROKE22; break;
//                case 23: t = GridType::TEAPOT1_RH_BODY_STROKE23; break;
//                case 24: t = GridType::TEAPOT1_RH_BODY_STROKE24; break;
//                case 25: t = GridType::TEAPOT1_RH_BODY_STROKE25; break;
//                case 26: t = GridType::TEAPOT1_RH_BODY_STROKE26; break;
//                case 27: t = GridType::TEAPOT1_RH_BODY_STROKE27; break;
//                case 28: t = GridType::TEAPOT1_RH_BODY_STROKE28; break;
//                case 29: t = GridType::TEAPOT1_RH_BODY_STROKE29; break;
//                case 30: t = GridType::TEAPOT1_RH_BODY_STROKE30; break;
//                case 31: t = GridType::TEAPOT1_RH_BODY_STROKE31; break;

//                default: t = GridType::NONE; break;

//            }

//            projectors[0]->renderModeChanged(2);
//            projectors[0]->setGridType(t);
//            projectors[3]->renderModeChanged(2);
//            projectors[3]->setGridType(t);
//            updateProjectors();
//        }
//    }
//}

#endif //MAINWINDOW_CPP


