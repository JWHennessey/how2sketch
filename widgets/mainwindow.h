#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QHBoxLayout>
#include <QtWidgets>
#include <QLabel>
#include "widgets/viewerwidget.h"
#include "mesh/mesh.h"
#include "widgets/controlswidget.h"

//#include "drawing_surface/drawingsurface.h"



//---CANDIDATE FOR DELETING
//#include "opengl/abstractprojector.h"
//#include "widgets/controlswidget.h"
//#include "widgets/monitorglwidget.h"
//#include "widgets/resizablestackedwidget.h"
//#include "opengl/drawingsurfaceprojector.h"

//static const int number_of_guides = 12;
//static const int number_of_surface_guides = 3




class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    //Signals and Slots can't use c++11 auto function definitions
private slots:
    void importMesh();
    //    void openScene();
    //    void saveScene();

    void addControlsPanel();
    void removePanel();
    void getMesh();
    void initViewer();
    void saveJson();
    //    void updateProjectors();
    //    void keyPressed(QKeyEvent* event);
    //    void wheelMove(QWheelEvent* event);
    //    void setCurrentGuide(int i);
    //    void setDrawingSurfaceGuide(int i);

private:
    //    //Private variables
    //    std::vector<Mesh*> meshes;
    //    std::vector<GLProjector*> projectors;
    //    DrawingSurfaceProjector* drawingSurfaceProjector;
    Mesh mesh;
    ViewerWidget viewerWidget;
    QGridLayout widgetsLayout;
    ControlsWidget controlsWidget;
    QStackedWidget* stackedWidget;
    //    MonitorGLWidget monitorGLWidget;
    //    DrawingSurface drawingWindow;
    //    GridType currentGridType;
    //    DrawingSurfaceGuide drawingSurfaceGuide;
    int activeStackedWidget;
    //    bool showGridOnDrawing;
    //    int currentGuide;
    //    int currentSurfaceGuide;
    //    int sceneID;
    //    GridType regionHighlight;
    //    int strokeNumber;

    //FUNCTIONS
    auto createLayout() -> void;
    auto setupSignals() -> void;
    auto createMenuBar() -> void;
    //    auto closeEvent(QCloseEvent *event) -> void;

    //    auto addMesh(Mesh* m) -> void;
    //    auto nextGuide() -> void;
    //    auto prevGuide() -> void;
    //    auto nextSurfaceGuide() -> void;
    //    auto prevSurfaceGuide() -> void;
    //    auto updateGuide() -> void;
    //    auto intToSurfaceGuide(int i) -> DrawingSurfaceGuide;
};

#endif // MAINWINDOW_H
