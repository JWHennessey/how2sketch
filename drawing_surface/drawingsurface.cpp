#include "drawingsurface.h"
#include <QVector3D>


DrawingSurface::DrawingSurface(DrawingSurfaceProjector* dsp, QWidget *parent)
    : QOpenGLWidget(parent)
    , m_program(0)
    , isDrawing(false)
    , drawingSurfaceProjector(dsp)
    , hasOverlay(false)
    , hasInitBackground(false)
    , hasRenderedOnce(false)
    , width(1024)
    , height(768)
{
    resize(width, height);
    //setMouseTracking(true);
    auto format = QSurfaceFormat();
    format.setVersion(4,3);
    format.setProfile(QSurfaceFormat::CoreProfile);
   // QSurfaceFormat::setDefaultFormat(format);
    setFormat(format);

    stroke.curveType = CUBIC_BSPLINE;stroke.radius = currentStroke.radius = 0.9;
    stroke.taperStepSize = currentStroke.taperStepSize = 0.01;
    currentStroke.curveType = FOUR_POINT;
    stroke.useTexture = currentStroke.useTexture = false;

}


void DrawingSurface::initializeGL()
{

    //qDebug() << "initializeGL()";
    //makeCurrent();
    initializeOpenGLFunctions();
    if(!hasInitBackground)
    {
        initBackgroundGL();
    }


    proj.setToIdentity();
    proj.ortho(0.0f, (float)width, (float)height, 0.0f, -10.0f, 10.0f);


    view.setToIdentity();
    glViewport(0, 0, width, height);
    glClearColor(1,1,1,1);

    m_program = new QOpenGLShaderProgram;

    m_program->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/resources/shaders/stroke.vert");
    m_program->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/resources/shaders/stroke.frag");
    m_program->bindAttributeLocation("vertex", 0);
    m_program->bindAttributeLocation("new_vertex", 1);

    m_program->link();

    m_program->bind();
    m_program->setUniformValue("projMatrix", proj);
    m_program->setUniformValue("viewMatrix", view);

    if(!m_vao.isCreated())
    {
        m_vao.create();
    }

    QOpenGLVertexArrayObject::Binder vaoBinder(&m_vao);
    int maxRows = std::max(vertices.rows(), currentVertices.rows());

    //qDebug() << "init vertices.rows " << vertices.rows();

//    if(vertices.rows() > 0 || currentVertices.rows() > 0)
//    {
//        int total_verts = vertices.rows() * vertices.rows();

//        combinedData = (float*) std::malloc(total_verts * sizeof(float) * 3);

//        int cv_size = currentVertices.rows() * sizeof(float) * 3;
//        if(cv_size > 0)
//            std::memcpy(combinedData, currentVertices.data(), cv_size);

//        std::memcpy(combinedData + cv_size, vertices.data(), vertices.rows() * sizeof(float) * 3);

//        m_vertexVbo.create();
//        m_vertexVbo.bind();
//        m_vertexVbo.allocate(combinedData, total_verts * sizeof(GL_FLOAT) * 3);

//    }


    int total_verts = vertices.rows() + currentVertices.rows();
    combinded =  MatrixXf(total_verts, 3);
    if(vertices.rows() > 0 && currentVertices.rows() > 0)
    {
        combinded << vertices,
                     currentVertices;

    }
    else if(vertices.rows() > 0)
    {
        combinded << vertices;
    }
    else if(currentVertices.rows() > 0)
    {
        combinded << currentVertices;
    }

    //qDebug() << "combinded.rows() " << combinded.rows();
    m_vertexVbo.create();
    m_vertexVbo.bind();
    m_vertexVbo.allocate(combinded.data(), ( combinded.rows()) * sizeof(GL_FLOAT) * 3);


//    qDebug () << "init vertices.rows() " << vertices.rows();
//    m_vertexVbo.create();
//    m_vertexVbo.bind();
//    m_vertexVbo.allocate(vertices.data(), ( vertices.rows()) * sizeof(GL_FLOAT) * 3);


//    m_currentVertexVbo.create();
//    m_currentVertexVbo.bind();
//    m_currentVertexVbo.allocate(currentVertices.data(), (currentVertices.rows()) * sizeof(GL_FLOAT) * 3);

   setupVertexAttribs();

   m_program->release();
   //doneCurrent();

}

void DrawingSurface::initBackgroundGL()
{
    //qDebug() << "DrawingSurface::initBackgroundGL()";
    //makeCurrent();
    m_backgroundProgram = new QOpenGLShaderProgram;
    m_backgroundProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/resources/shaders/background_texture.vert");
    m_backgroundProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/resources/shaders/background_texture.frag");
    m_backgroundProgram->bindAttributeLocation("vertex", 0);
    m_backgroundProgram->bindAttributeLocation("tex_coord", 1);

    if(!m_backgroundVao.isCreated())
    {
        m_backgroundVao.create();
    }

    QOpenGLVertexArrayObject::Binder vaoBinder(&m_backgroundVao);

    static const GLfloat squareVertices[] = {
        -1.0f, -1.0f,
        1.0f, -1.0f,
        -1.0f, 1.0f,
        1.0f, 1.0f,
        };

    static const GLfloat textureVertices[] = {
        0.0f, 0.0f,
        1.0f, 0.0f,
        0.0f, 1.0f,
        1.0f, 1.0f,
        };

    m_backgroundVertexVbo.create();

    m_backgroundVertexVbo.bind();

    m_backgroundVertexVbo.allocate(&squareVertices, 4 * sizeof(GL_FLOAT) * 2);

    m_backgroundTexCoordsVbo.create();
    m_backgroundTexCoordsVbo.bind();
    m_backgroundTexCoordsVbo.allocate(&textureVertices, 4 * sizeof(GL_FLOAT) * 2);

    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
    m_backgroundVertexVbo.bind();
    f->glEnableVertexAttribArray(0);
    f->glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(GLfloat), 0);
    m_backgroundVertexVbo.release();
    m_backgroundTexCoordsVbo.bind();

    f->glEnableVertexAttribArray(1);
    f->glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(GLfloat), 0);
    m_backgroundTexCoordsVbo.release();

    m_backgroundProgram->release();

    hasInitBackground = true;
    //doneCurrent();
}

auto DrawingSurface::updateCurrentStroke() -> void
{

//    qDebug() << "Update Current Stroke";
    currentStroke.computeMesh(currentVertices);
    //currentStroke.clear();
    initializeGL();
    update();
}

auto DrawingSurface::setupVertexAttribs() -> void
{
    //makeCurrent();
    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
    m_vertexVbo.bind();
    f->glEnableVertexAttribArray(0);
    f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), 0);
    m_vertexVbo.release();

//    m_currentVertexVbo.bind();
//    f->glEnableVertexAttribArray(1);
//    f->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), 0);
//    m_currentVertexVbo.release();
    //doneCurrent();

}
void DrawingSurface::paintGL()
{


    //makeCurrent();

    //qDebug() << "DrawingSurface::paintGL()";

    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();

    f->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    f->glDisable(GL_DEPTH_TEST);
    f->glEnable(GL_BLEND);
    f->glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    f->glClearColor(1,1,1,1);
    if(hasOverlay)
    {
        paintBackground();
    }

    QOpenGLVertexArrayObject::Binder vaoBinder(&m_vao);
    m_program->bind();
    m_program->setUniformValue("isCurrent", false);
    f->glDrawArrays(GL_TRIANGLES, 0, vertices.rows());
    f->glDisable(GL_BLEND);
    m_program->setUniformValue("isCurrent", true);
    f->glDrawArrays(GL_TRIANGLES, vertices.rows(), combinded.rows());
    m_program->release();


  // IF USING THIS METHOD, YOU NEED TO UPDATE THE stroke.vert shader
//     //f->glDrawArrays(GL_TRIANGLES, 0, vertices.rows());
//     f->glDisable(GL_BLEND);
//     m_program->setUniformValue("isCurrent", true);
//     f->glDrawArrays(GL_TRIANGLES,0, currentVertices.rows());
//     m_program->release();

    if(!hasRenderedOnce) hasRenderedOnce = true;

    //doneCurrent();

}

void DrawingSurface::paintBackground()
{
    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
    QOpenGLVertexArrayObject::Binder vaoBinder(&m_backgroundVao);
    m_backgroundProgram->bind();
    texture->bind(0);
    m_program->setUniformValue("texture", 0);
    f->glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
    texture->release();
    m_backgroundProgram->release();
}

void DrawingSurface::resizeGL(int width, int height)
{

}

void DrawingSurface::mousePressEvent(QMouseEvent *event)
{
    isDrawing = true;
    stroke.clear();
    //addPointToStroke(event->pos());

    //lastPos = event->pos();
}

void DrawingSurface::mouseMoveEvent(QMouseEvent *event)
{
    if(isDrawing)
    {
       //qDebug() << event->pos();
       addPointToStroke(event->pos());
    }
}

void DrawingSurface::keyPressEvent(QKeyEvent* event)
{
    //qDebug() << "DrawingSurface::keyPressEvent";
    emit keyPressed(event);
}

void DrawingSurface::keyReleaseEvent(QKeyEvent* event)
{

}

void DrawingSurface::mouseReleaseEvent(QMouseEvent *event)
{
    //qDebug() << "Mouse Release";
    isDrawing = false;
    currentStroke.clearVertices();
    currentVertices = MatrixXf(0,0);
//    for(auto i = 0; i < 200; i++)
//        stroke.add(100, 100 + i * 2);
    stroke.computeMesh(vertices);
    //m_vertexVbo.release();
    //m_vertexVbo.destroy();
    initializeGL();
    update();


}

auto DrawingSurface::addPointToStroke(QPoint p) -> void
{
    auto x = p.x();
    auto y = p.y();
    //qDebug() << "addPointToStroke " << x << " " << y;
    if(x >= 0 && x <= width && y >= 0 && y <= height)
    {
        stroke.add((float)x, (float)y);
        currentStroke.add((float)x, (float)y);
    }
    updateCurrentStroke();
}


auto DrawingSurface::addOverlay(GridType t) -> void
{
    //qDebug() << "addOverlauy ";
    makeCurrent();
    bool foundOverlay = true;
    QImage overlay;
    switch(t)
    {
        case GridType::REGULAR:   overlay = QImage(":/resources/shaders/grid_pattern_regular.png"); break;
        case GridType::CROSSHAIR: overlay = QImage(":/resources/shaders/grid_pattern_crosshair.png"); break;
        case GridType::CUSTOM:    overlay = QImage(":/resources/shaders/grid_pattern_custom.png"); break;
        case GridType::SUBDIVIDE: overlay = QImage(":/resources/shaders/subdivide_grid_pattern.png"); break;
        case GridType::TEAPOT1_CONVEX_HULL: overlay = QImage(":/resources/shaders/teapot1_convexhull.png"); break;

        default: foundOverlay = false;
    }

   // qDebug() << "Overlay size " << overlay.size();
    if(foundOverlay)
    {
        texture = new QOpenGLTexture(overlay.mirrored(false, false));
        texture->setMinificationFilter(QOpenGLTexture::LinearMipMapLinear);
        texture->setMagnificationFilter(QOpenGLTexture::Linear);
        texture->setWrapMode(QOpenGLTexture::ClampToBorder);

        hasOverlay = true;


    }
    else
    {
        removeOverlay();
    }
    update();
    doneCurrent();

}


auto DrawingSurface::addOverlay(DrawingSurfaceGuide t) -> void
{
    //qDebug() << "addOverlauy ";
    makeCurrent();
    bool foundOverlay = true;
    QImage overlay;
    switch(t)
    {
        case DrawingSurfaceGuide::SHADOW:   overlay = QImage(":/resources/shaders/teapot1_shadow.png"); break;
        case DrawingSurfaceGuide::CONVEX_HULL: overlay = QImage(":/resources/shaders/teapot1_convexhull.png"); break;
        case DrawingSurfaceGuide::LINE1: overlay = QImage(":/resources/shaders/line1.png"); break;
        case DrawingSurfaceGuide::LINE2: overlay = QImage(":/resources/shaders/line2.png"); break;
        case DrawingSurfaceGuide::LINE3: overlay = QImage(":/resources/shaders/line3.png"); break;
        case DrawingSurfaceGuide::BOX1: overlay = QImage(":/resources/shaders/box1.png"); break;
        case DrawingSurfaceGuide::BOX2: overlay = QImage(":/resources/shaders/box2.png"); break;
        case DrawingSurfaceGuide::BOX3: overlay = QImage(":/resources/shaders/box3.png"); break;
        case DrawingSurfaceGuide::BOX4: overlay = QImage(":/resources/shaders/box4.png"); break;
        case DrawingSurfaceGuide::BOX5: overlay = QImage(":/resources/shaders/box5.png"); break;
        case DrawingSurfaceGuide::BOX6: overlay = QImage(":/resources/shaders/box6.png"); break;
        case DrawingSurfaceGuide::BOX7: overlay = QImage(":/resources/shaders/box7.png"); break;
        case DrawingSurfaceGuide::BOX8: overlay = QImage(":/resources/shaders/box8.png"); break;
        case DrawingSurfaceGuide::BOX9: overlay = QImage(":/resources/shaders/box9.png"); break;

        default: foundOverlay = false;
    }


   // qDebug() << "Overlay size " << overlay.size();
    if(foundOverlay)
    {

//        myLabel.setPixmap(QPixmap::fromImage(overlay));
//        myLabel.show();

        overlay.convertToFormat(QImage::Format_RGB32);
        texture = new QOpenGLTexture(overlay.mirrored(false, false));
        //texture->setMinificationFilter(QOpenGLTexture::LinearMipMapLinear);
        texture->setMagnificationFilter(QOpenGLTexture::Linear);
        texture->setWrapMode(QOpenGLTexture::ClampToBorder);

        hasOverlay = true;


    }
    else
    {
        removeOverlay();
    }
    update();
    doneCurrent();

}
auto DrawingSurface::removeOverlay() -> void
{
    //qDebug() << "removeOverlay";
    hasOverlay = false;
    update();
}
auto DrawingSurface::removeGuides() -> void
{

}

auto DrawingSurface::getImage() -> QImage
{
    //makeCurrent();
    QImage img;
//    if(hasRenderedOnce)
//        img = grabFramebuffer();

    //img.save("Test.png");
    img = img.convertToFormat(QImage::Format_ARGB32);
    return img;
}

auto DrawingSurface::addGridLines() -> void
{

}

auto DrawingSurface::removeGridLines() -> void
{

}

auto DrawingSurface::clearImage() -> void
{
    currentStroke.clearVertices();
    stroke.clearVertices();
    currentVertices = MatrixXf(0,0);
    vertices = MatrixXf(0,0);
    initializeGL();
    update();

}

auto DrawingSurface::wheelEvent(QWheelEvent* event) -> void
{
    emit wheelMove(event);
}
