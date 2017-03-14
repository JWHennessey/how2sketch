#ifndef VIEWERWIDGET_H
#define VIEWERWIDGET_H

#include <memory>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLBuffer>
#include <QOpenGLShaderProgram>
#include <QGLShader>
#include <QMatrix4x4>
#include "mesh/mesh.h"
#include "opengl/trackball.h"
#include "optimisation/primitiveoptimisation.h"

enum class RenderMode {PHONG, SEGMENTATION, SUGGESTIVE_CONTOURS};



class ViewerWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT

public:
    ViewerWidget(QWidget *parent = 0);
    ~ViewerWidget();
    auto importMesh(QString filename) -> void;
    auto getMesh() -> Mesh*;
    auto writePrimitivesAndRelations() -> void;

public slots:
    void cleanup();
    void setRenderMode(int i);
    void updateViewer();
    void initViewer();
    void boundingBoxToggled(bool checked);
    void segmentBoxesToggled(bool checked);
    void renderBoxFacesToggled(bool checked);
    void renderMidLinesToggled(bool checked);
    void renderDiagLinesToggled(bool checked);
    void renderThirdsDiagLineToggled(bool checked);
    void renderTwoThirdsDiagLineToggled(bool checked);
    void renderThirdsLineToggled(bool checked);
    void renderTwoThirdsLineToggled(bool checked);
    void renderHiddenLinesToggled(bool checked);
    void renderPerspectiveLinesToggled(bool checked);
    void edgeRatiosToggled(bool checked);
    void setShowEdgeRatio(int val);
    void setUnitLengthBox(int val);
    void setUnitLengthEdge(int val);
    void manualAdjustSelectedLengthEdge(double val);
    void setSelectedEdge(int val);
    void bbXRotChanged(double val);
    void bbYRotChanged(double val);
    void bbZRotChanged(double val);
    void setDesieredEdgeRatio(double val);
    void computeBoxes(BoxAdjustMethod method);
    void setSelectedEdgeBox(int val);
    void showObjectToggled(bool);
    void printScreen();
    void ellipsesToggled();
    void planesToggled();
    void zoomChanged(double val);
    void findRelations();
    void acceptRelations();
    void clearRelations();
    void generateCandidates();
    void candidatesToggled();
    void runOptimisation();
    void guidesToggled();
    void selectSegment(int);
    //void saveJson();
    //    void showClustersToggled(bool val);
//    void showNextCluster();
//    void runOptimisationOnCluster();
//    void outputData();

signals:
    void setUnitLengthEdge(double);
    void updateDropdowns(int);

protected:
    void initializeGL() Q_DECL_OVERRIDE;
    void paintGL() Q_DECL_OVERRIDE;

    void resizeGL(int width, int height) Q_DECL_OVERRIDE;
    void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseReleaseEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseMoveEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void wheelEvent(QWheelEvent * event) Q_DECL_OVERRIDE;

private:
    Mesh* mesh;
    Mesh* ellipseMesh;
    bool m_core;
    QMatrix4x4 m_proj;
    QMatrix4x4 m_view;
    QMatrix4x4 m_model;
    QMatrix4x4 m_viewport;
    PrimitiveOptimisation* optimisation;
    //Object Program
    QOpenGLVertexArrayObject m_objectVao;
    QOpenGLShaderProgram *m_objectProgram;
    QOpenGLBuffer m_vertexVbo;
    QOpenGLBuffer m_normalVbo;
    QOpenGLBuffer m_indexVbo;
    QOpenGLBuffer m_maxDirVbo;
    QOpenGLBuffer m_minDirVbo;
    QOpenGLBuffer m_maxCurveVbo;
    QOpenGLBuffer m_minCurveVbo;
    QOpenGLBuffer m_dCurvVbo;
    QOpenGLBuffer m_colourVbo;
    //Bounding Box Program
    QOpenGLShaderProgram *m_bbProgram;
    QOpenGLShader* m_bbGeometryShader;
    QOpenGLVertexArrayObject m_bbVao;
    QOpenGLBuffer m_bbVertexVbo;
    QOpenGLBuffer m_bbIndexVbo;
    bool renderBoundingBox;
    bool renderSegmentBoxes;
    bool renderDiagLines;
    bool renderThirdsDiagLine;
    bool renderTwoThirdsDiagLine;
    bool renderThirdsLine;
    bool renderTwoThirdsLine;
    bool renderMidLines;
    bool renderBoxFacesCheck;
    bool renderHiddenEdges;
    //Ellipse Program
    QOpenGLVertexArrayObject m_ellipsesVao;
    QOpenGLShaderProgram *m_ellipsesProgram;
    QOpenGLBuffer m_ellipsesVertexVbo;
    QOpenGLBuffer m_ellipsesIndexVbo;
    bool renderEllipsesCheck;
    //Planes Program
    QOpenGLVertexArrayObject m_planesVao;
    QOpenGLShaderProgram *m_planesProgram;
    QOpenGLBuffer m_planesVertexVbo;
    QOpenGLBuffer m_planesIndexVbo;
    bool renderPlanesCheck;
    unsigned short* planeFaces;
    int planeFaceCount;
    std::vector<int> planeCandidatePerPrimitve;
    //Candidates Program
    QOpenGLVertexArrayObject m_candidatesVao;
    QOpenGLShaderProgram *m_candidatesProgram;
    QOpenGLBuffer m_candidatesVertexVbo;
    QOpenGLBuffer m_candidatesIndexVbo;
    bool renderCandidatesCheck;
    unsigned short* candidateFaces;
    int candidateFaceCount;
    std::vector<int> completeCandidatePerPrimitve;
    //Candidates Program
    QOpenGLVertexArrayObject m_guidesVao;
    QOpenGLShaderProgram *m_guidesProgram;
    QOpenGLBuffer m_guidesVertexVbo;
    QOpenGLBuffer m_guidesIndexVbo;
    bool renderGuidesCheck;
    int number_total_guides;
    unsigned short* guideIndexes;
    //Show Cluster
//    bool renderClusters;
//    int clusterStartIndex;
    // Text Renderer
    QOpenGLShaderProgram *m_textProgram;
    QOpenGLBuffer m_textVertexVbo;
    QOpenGLBuffer m_textIndexVbo;
    QOpenGLBuffer m_textTextureCoordVbo;
    QOpenGLVertexArrayObject m_textVao;
    bool showObject;
    bool showEdgeRatios;
    bool saveImageFromFBO;
    int showEdgeRatioSetting;
    int unitLengthBox;
    int unitLengthEdge;
    int selectedEdge;
    int selectedEdgeBox;
    int selectedSegment;
//    float unitLength;
//    int unitLengthEdgeAVertex;
//    int unitLengthEdgeBVertex;
//    int selectedEdgeAVertex;
//    int selectedEdgeBVertex;
    //Perspective Lines (Uses BB Program but switches shader)
    QOpenGLShader* m_perspectiveGeometryShader;
    bool showPerspectiveLines;
//    QOpenGLShader* m_bbGeometryShader;
//    QOpenGLShader* m_bbGeometryShader;

    TrackBall m_rotationTrackball;
    //TrackBall m_scaleTrackball;
    int m_distExp;
    bool m_transparent;
    RenderMode m_renderMode;
    std::map<int, float> desiredRatiosForDisplayMap;
    std::map<int, std::vector<std::pair<int, float>>> desiredEdgeRatioMap;
    int relationCount;
    //Functions
    auto createEdgeMaps() -> void;
    auto setupObjectProgram() -> void;
    auto setupTextProgram() -> void;
    auto setupObjectShaders() -> void;
    auto setupPhongShader() -> void;
    auto setupSuggestiveContoursShader() -> void;
    auto setupSegmentationShader() -> void;
    auto setupObjectVertexAttribs() -> void;
    auto setupBoundingBoxProgram() -> void;
    auto setupEllipseProgram() -> void;
    auto setupPlanesProgram(std::vector<trimesh::point> points) -> void;
    auto setupCandidatesProgram(std::vector<trimesh::point> points) -> void;
    auto setupGuidesProgram(std::vector<trimesh::point> points) -> void;
    auto pixelPosToViewPos(const QPointF& p) -> QPointF;
    auto renderPlanes() -> void;
    auto renderCandidates() -> void;
    auto renderGuides() -> void;
    auto renderObject() -> void;
    auto renderToImage() -> void;
    auto renderBoxes() -> void;
    auto renderPhong() -> void;
    auto renderEllipses() -> void;
    auto renderSuggestiveContours() -> void;
    auto renderBoxFaces() -> void;
    auto renderBoxEdges() -> void;
    auto renderBoxHelper() -> void;
    auto renderRatioText() -> void;
    auto renderPerspectiveLines() -> void;
    auto setTextTextureCoords(int width, int height) -> void;
    auto updateSelectedEdgeRatio() -> void;
    auto displayText(const QString &text, const QColor &backgroundColor, const QColor &textColor, const QVector3D &pos) -> void;
    auto getRatioTextColour(int boxId, int unitEdge, int selectedBoxId, int selectedEdge, int edgeCount, float& ratio) -> QColor;
};

#endif //VIEWERWIDGET_H
