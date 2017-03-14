    #ifndef CONTROLSWIDGET_H
#define CONTROLSWIDGET_H

#include <QObject>
#include <QWidget>
#include <QVBoxLayout>
#include <QPushButton>
#include <QComboBox>
#include <QDoubleSpinBox>
#include "mesh/mesh.h"
#include "widgets/suggestivecontourspanel.h"
#include "widgets/primitivespanel.h"

//#include "widgets/objectspanelwidget.h"
//#include "widgets/projectorpanelwidget.h"
//#include "widgets/drawingpanelwidget.h"
//#include "widgets/userviewpanelwidget.h"
////#include "scribble/drawingwindow.h"
//#include "opengl/drawingsurfaceprojector.h"
//#include "drawing_surface/drawingsurface.h"


//class UserViewPanelWidget;


class ControlsWidget : public QWidget
{
    Q_OBJECT
public:
    ControlsWidget(QWidget *parent = 0);
    //auto addMesh(Mesh* m) -> void;
    auto getActiveControlPanel() -> QWidget*;
    auto getActiveControlPanelId() -> int;
    auto getWidgets(std::vector<QWidget*>& widgets) -> void;
    auto setMesh(Mesh* mesh) -> void;
//    auto updateProjectorWidgets() -> void;
//    auto addProjector() -> void;
//    auto setCurrentGuide(int i) -> void;
//    auto setCurrentSurfaceGuide(int i) -> void;

public slots:
      void setUnitLengthEdge(double val);
      void updateDropdowns(int i);
//    void setCurrentView(QVector3D pos, QVector3D lookAt);



private slots:
      void toggleContoursPanel(bool checked);
      void togglePrimitivesPanel(bool checked);
      void manualUpdateSelectedLength();
      void setDesieredEdgeRatio();
      void computeBoxes();


//    void toggleObjectPanel(bool checked);
//    void toggleProjectorPanel(bool checked);
//    void toggleDrawingPanel(bool checked);
//    void toggleUserViewPanel(bool checked);
//    void setActiveMesh(int id);
//    void updateProjectors();
//    void changeViewpoint(QVector3D p, QVector3D l);
//    void changeToViewpoint(int i);
//    void meshDeleted();

signals:
      void setRenderMode(int);
      void removePanel();
      void addControlsPanel();
      void updateViewer();
      void initViewer();
      void boundingBoxToggled(bool);
      void segmentBoxesToggled(bool);
      void showClustersToggled(bool);
      void showNextCluster();
      void renderBoxFacesToggled(bool);
      void renderMidLinesToggled(bool);
      void renderDiagLinesToggled(bool);
      void renderHiddenLinesToggled(bool);
      void renderPerspectiveLinesToggled(bool);
      void edgeRatiosToggled(bool);
      void setShowEdgeRatio(int);
      void setUnitLengthBox(int);
      void setUnitLengthEdge(int);
      void manualAdjustSelectedLengthEdge(double);
      void setSelectedEdge(int);
      void bbXRotChanged(double);
      void bbYRotChanged(double);
      void bbZRotChanged(double);
      void setDesieredEdgeRatio(double);
      void computeBoxes(BoxAdjustMethod);
      void setSelectedEdgeBox(int);
      void showObjectToggled(bool);
      void printScreen();
      void renderThirdsDiagLinesToggled(bool);
      void renderTwoThirdsDiagLinesToggled(bool);
      void renderThirdsLineToggled(bool);
      void renderTwoThirdsLineToggled(bool);
      void runOptimisationOnCluster();
      void outputData();
      void getMesh();
      void ellipsesToggled();
      void planesToggled();
      void saveJson();
      void zoomChanged(double);
      void findRelations();
      void acceptRelations();
      void clearRelations();
      void generateCandidates();
      void candidatesToggled();
      void runOptimisation();
      void guidesToggled();
      void selectSegment(int);
//    void updateActiveMesh(int i);
//    void projectorsUpdated();
//    void viewpointChanged(QVector3D p, QVector3D l);
//    void updateProjectorGuide(int i);
//    void updateDrawingSurfaceGuide(int i);

private:
    int activePanel;

//    QComboBox* viewComboBox;
//    QComboBox* projectedGuideComboBox;
//    QComboBox* drawingSurfaceGuideComboBox;
//    ObjectsPanelWidget objectPanel;
//    ProjectorPanelWidget projectorPanel;
//    DrawingPanelWidget drawingPanel;
//    UserViewPanelWidget userViewPanel;
    SuggestiveContoursPanel contoursPanel;
    PrimitivesPanel primitivesPanel;
    QVBoxLayout* items;
    QDoubleSpinBox* adjustUnitLengthEdgeSpin;
    QDoubleSpinBox* adjustSelectedEdgeRatioSpin;
    QComboBox* showEdgeRatiosCombo;
    QComboBox* unitlengthBoxCombo;
    QComboBox* selectedEdgeBoxCombo;
    QComboBox* boxAdjustMethodCombo;
//    QPushButton* objectPanelBtn;
//    QPushButton* projectorPanelBtn;
//    QPushButton* drawingPanelBtn;
//    QPushButton* userViewPanelBtn;
    auto init() -> void;
    auto setupSignals() -> void;


};

#endif // CONTROLSWIDGET_H
