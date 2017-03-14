#ifndef DRAWINGSURFACE_H
#define DRAWINGSURFACE_H

#include <QWidget>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QMatrix4x4>
#include "drawing_surface/Stroke.h"
#include "Eigen/Dense"
#include "Eigen/Sparse"
#include <QMouseEvent>
#include <QLabel>
#include "opengl/drawingsurfaceprojector.h"

using MatrixXf = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic,Eigen::RowMajor>;
using MatrixXus = Eigen::Matrix<unsigned short,Eigen::Dynamic, Eigen::Dynamic,Eigen::RowMajor>;
class DrawingSurfaceProjector;
enum class DrawingSurfaceGuide {NONE, MIRROR, SHADOW, CONVEX_HULL, LINE1, LINE2, LINE3, BOX1, BOX2, BOX3, BOX4, BOX5, BOX6, BOX7, BOX8, BOX9};

class DrawingSurface : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    DrawingSurface(DrawingSurfaceProjector* dsp, QWidget *parent = 0);
    auto addOverlay(GridType t) -> void;
    auto addOverlay(DrawingSurfaceGuide t) -> void;
    auto removeOverlay() -> void;
    auto removeGuides() -> void;
    auto addGridLines() -> void;
    auto removeGridLines() -> void;
    auto getImage() -> QImage;
public slots:
    void clearImage();
protected:
    void initializeGL() Q_DECL_OVERRIDE;
    void initBackgroundGL();
    void paintBackground();
    void paintGL() Q_DECL_OVERRIDE;
    void resizeGL(int width, int height) Q_DECL_OVERRIDE;
    void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseMoveEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void mouseReleaseEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
    void keyPressEvent(QKeyEvent* event) Q_DECL_OVERRIDE;
    void keyReleaseEvent(QKeyEvent* event) Q_DECL_OVERRIDE;
    void wheelEvent(QWheelEvent* event) Q_DECL_OVERRIDE;

signals:
    void keyPressed(QKeyEvent* event);
    void wheelMove(QWheelEvent* event);
private:
    Stroke stroke;
    Stroke currentStroke;
    MatrixXf vertices;
    MatrixXf currentVertices;
    MatrixXf combinded;
    float* combinedData;
    QMatrix4x4 proj;
    QMatrix4x4 view;
    QOpenGLVertexArrayObject m_vao;
    QOpenGLVertexArrayObject m_backgroundVao;
    QOpenGLBuffer m_vertexVbo;
    QOpenGLBuffer m_currentVertexVbo;
    QOpenGLBuffer m_backgroundVertexVbo;
    QOpenGLBuffer m_backgroundTexCoordsVbo;
    QOpenGLShaderProgram *m_program;
    QOpenGLShaderProgram *m_backgroundProgram;
    QOpenGLTexture* texture;
    DrawingSurfaceProjector* drawingSurfaceProjector;
    bool hasOverlay;
    bool isDrawing;
    bool hasInitBackground;
    bool hasRenderedOnce;
    auto setupVertexAttribs() -> void;
    auto addPointToStroke(QPoint p) -> void;
    auto updateCurrentStroke() -> void;
    QLabel myLabel;
    int width;
    int height;


};

#endif // DRAWINGSURFACE_H
