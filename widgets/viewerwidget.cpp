#ifndef VIEWERWIDGET_CPP
#define VIEWERWIDGET_CPP

#include "widgets/viewerwidget.h"
#include "utils/contourparams.h"
#include <QMouseEvent>
#include <QOpenGLShaderProgram>
#include <QCoreApplication>
#include <QSurface>
#include <QGLFormat>
#include <math.h>
#include <iostream>
#include <thread>
#include <ctime>
#include <set>


ViewerWidget::ViewerWidget(QWidget* parent)
    : QOpenGLWidget(parent)
    , optimisation(nullptr)
    , m_indexVbo(QOpenGLBuffer::IndexBuffer)
    , m_bbIndexVbo(QOpenGLBuffer::IndexBuffer)
    , m_ellipsesIndexVbo(QOpenGLBuffer::IndexBuffer)
    , m_textIndexVbo(QOpenGLBuffer::IndexBuffer)
    , m_planesIndexVbo(QOpenGLBuffer::IndexBuffer)
    , m_candidatesIndexVbo(QOpenGLBuffer::IndexBuffer)
    , m_guidesIndexVbo(QOpenGLBuffer::IndexBuffer)
    , renderBoundingBox(false)
    , renderSegmentBoxes(false)
    , renderDiagLines(false)
    , renderThirdsDiagLine(false)
    , renderTwoThirdsDiagLine(false)
    , renderThirdsLine(false)
    , renderTwoThirdsLine(false)
    , renderHiddenEdges(false)
    , renderMidLines(true)
    , renderEllipsesCheck(false)
    , renderPlanesCheck(false)
    , renderCandidatesCheck(false)
    , renderGuidesCheck(false)
    , number_total_guides(0)
    , showObject(true)
    , showEdgeRatios(false)
    , saveImageFromFBO(false)
    , planeFaceCount(0)
    , showEdgeRatioSetting(0)
    , unitLengthBox(0)
    , unitLengthEdge(0)
    , selectedEdge(0)
    , selectedEdgeBox(0)
    , selectedSegment(0)
//    , unitLength(0)
//    , unitLengthEdgeAVertex(7) //Hardcoded for the box face indexes
//    , unitLengthEdgeBVertex(1) //Hardcoded for the box face indexes
//    , selectedEdgeAVertex(7)
//    , selectedEdgeBVertex(1)
    , showPerspectiveLines(false)
    , m_rotationTrackball(1.0f, QVector3D(0, -1, 0), TrackBall::Sphere)
    , m_distExp(600)
    , m_renderMode(RenderMode::PHONG)
    , renderBoxFacesCheck(false)
    , relationCount(0)
{
    //setUpdateBehavior(QOpenGLWidget::NoPartialUpdate);
    m_core = QCoreApplication::arguments().contains(QStringLiteral("--coreprofile"));

    // --transparent causes the clear color to be transparent. Therefore, on systems that
    // support it, the widget will become transparent apart from the logo.
    m_transparent = QCoreApplication::arguments().contains(QStringLiteral("--transparent"));
    if (m_transparent)
        setAttribute(Qt::WA_TranslucentBackground);


    m_view.setToIdentity();
    m_view.lookAt(camera_pos, QVector3D(0,0,0), QVector3D(0,1,0));

//    auto q = 0.5f;
//    m_view(0,3) = std::sin(q) / camera_pos[0];
//    m_view(2,3) = std::cos(q) / camera_pos[2];
//    qDebug() << m_view;

    m_model.setToIdentity();

}

ViewerWidget::~ViewerWidget()
{
    cleanup();
}

auto ViewerWidget::getMesh() -> Mesh*
{
    return mesh;
}

auto ViewerWidget::importMesh(QString filename) -> void
{
    cleanup();
    mesh = new Mesh(filename);

    connect(mesh, SIGNAL(updateViewer()), this, SLOT(updateViewer()));
    emit updateDropdowns(mesh->segmentSize());
    initializeGL();
    update();
}

auto ViewerWidget::writePrimitivesAndRelations() -> void
{
    mesh->writePrimitivesAndRelations();
}

auto ViewerWidget::generateCandidates() -> void
{
    optimisation = new PrimitiveOptimisation(mesh);
    auto mvp =  m_proj * m_view * m_model;
    optimisation->generateCandidates(m_viewport, mvp);
    std::map<QString, std::vector<PlanePartialPrimitive*> > partialPrimitives = optimisation->getPartialPrimitives();
    std::vector<trimesh::point> partialPrimitivesPoints;
    planeCandidatePerPrimitve.clear();
    int continuousPlaneCount = 0;
    for(auto pp_vec : partialPrimitives)
    {
        qDebug() << pp_vec.first << " no. plane candidates " << pp_vec.second.size();
        auto candidate_count = pp_vec.second.size() * 2;
        continuousPlaneCount += candidate_count;
        planeCandidatePerPrimitve.push_back(continuousPlaneCount);
        for(auto pp : pp_vec.second)
        {
            auto points = pp->getPoints();
            partialPrimitivesPoints.insert(std::end(partialPrimitivesPoints), std::begin(points), std::end(points));

        }

    }
    setupPlanesProgram(partialPrimitivesPoints);
    std::map<QString, std::vector<CandidatePrimitive*> > candidatesPrimitives = optimisation->getCandidatePrimitives();
    std::vector<trimesh::point> candidatePrimitivesPoints;
    completeCandidatePerPrimitve.clear();
    int continuousCandidateCount = 0;
    for(auto cp_vec : candidatesPrimitives)
    {
        qDebug() << cp_vec.first << " no. compelte candidates " << cp_vec.second.size();
        auto candidate_count = cp_vec.second.size() * 12;
        continuousCandidateCount += candidate_count;
        completeCandidatePerPrimitve.push_back(continuousCandidateCount);
        for(auto cp : cp_vec.second)
        {
            auto points = cp->getPoints();
            candidatePrimitivesPoints.insert(std::end(candidatePrimitivesPoints), std::begin(points), std::end(points));
        }
    }
    setupCandidatesProgram(candidatePrimitivesPoints);
}

auto ViewerWidget::runOptimisation() -> void
{
    if(optimisation == nullptr)
        return;


    optimisation->runOptimisation();

    std::map<QString, CandidatePrimitive* > candidatesPrimitives = optimisation->getFinalPrimitives();
    std::vector<trimesh::point> candidatePrimitivesPoints;
    std::vector<trimesh::point> allGuidePoints;
    completeCandidatePerPrimitve.clear();
    int continuousCandidateCount = 0;
    for(auto cp_vec : candidatesPrimitives)
    {
        auto candidate_count = 12;
        continuousCandidateCount += candidate_count;
        completeCandidatePerPrimitve.push_back(continuousCandidateCount);
        auto points = cp_vec.second->getPoints();
        candidatePrimitivesPoints.insert(std::end(candidatePrimitivesPoints), std::begin(points), std::end(points));
        auto guidePoints = cp_vec.second->getGuidePoints();
        allGuidePoints.insert(std::end(allGuidePoints), std::begin(guidePoints), std::end(guidePoints));
    }
    setupCandidatesProgram(candidatePrimitivesPoints);
    setupGuidesProgram(allGuidePoints);

}

auto ViewerWidget::findRelations() -> void
{

    auto segments = mesh->getMeshSegments();
    for(auto s : segments)
    {
        s->deselect();
        s->setAxis(-1);
        s->setFace(-1);
    }

    mesh->findRelations(relationCount);
    updateViewer();
    relationCount++;
}

auto ViewerWidget::clearRelations() -> void
{
    relationCount = 0;
}

auto ViewerWidget::acceptRelations() -> void
{

}

void ViewerWidget::cleanup()
{
    m_vertexVbo.destroy();
    m_normalVbo.destroy();
    m_indexVbo.destroy();
    m_maxDirVbo.destroy();
    m_minDirVbo.destroy();
    m_maxCurveVbo.destroy();
    m_minCurveVbo.destroy();
    m_dCurvVbo.destroy();
    m_objectVao.destroy();
    m_bbVertexVbo.destroy();
    m_bbIndexVbo.destroy();
    delete m_objectProgram;
    m_objectProgram = 0;
}

void ViewerWidget::initializeGL()
{

    qDebug() << " ViewerWidget::initializeGL()";
    initializeOpenGLFunctions();

    glClearColor(0.2, 0.2, 0.2, m_transparent ? 0 : 1);

    if(mesh == nullptr)
        return;

    setupObjectProgram();

    setupBoundingBoxProgram();

    setupTextProgram();

    setupEllipseProgram();

}

auto ViewerWidget::setupObjectProgram() -> void
{
    m_objectProgram = new QOpenGLShaderProgram;

    setupObjectShaders();

    m_objectProgram->bindAttributeLocation("vertex", 0);
    m_objectProgram->bindAttributeLocation("normal", 1);
    m_objectProgram->bindAttributeLocation("max_dir", 2);
    m_objectProgram->bindAttributeLocation("min_dir", 3);
    m_objectProgram->bindAttributeLocation("max_curve", 4);
    m_objectProgram->bindAttributeLocation("min_curve", 5);
    m_objectProgram->bindAttributeLocation("dcurv", 6);
    m_objectProgram->bindAttributeLocation("colour", 7);
    m_objectProgram->link();

    m_objectProgram->bind();

    if(!m_objectVao.isCreated())
    {
        m_objectVao.create();
    }

    QOpenGLVertexArrayObject::Binder vaoBinder(&m_objectVao);
    m_vertexVbo.create();
    m_vertexVbo.bind();
    m_vertexVbo.allocate(mesh->vData(), (mesh->vSize()) * sizeof(GL_FLOAT) * 3);

    m_normalVbo.create();
    m_normalVbo.bind();
    m_normalVbo.allocate(mesh->nData(), (mesh->nSize()) * sizeof(GL_FLOAT) * 3);

    m_indexVbo.create();
    m_indexVbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
    m_indexVbo.bind();
    m_indexVbo.allocate(mesh->fData(), mesh->fSize() * sizeof(GLushort) * 3);

    m_maxDirVbo.create();
    m_maxDirVbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
    m_maxDirVbo.bind();
    m_maxDirVbo.allocate(mesh->maxDirData(), mesh->maxDirSize() * sizeof(GL_FLOAT) * 3);

    m_minDirVbo.create();
    m_minDirVbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
    m_minDirVbo.bind();
    m_minDirVbo.allocate(mesh->minDirData(), mesh->minDirSize() * sizeof(GL_FLOAT) * 3);

    m_maxCurveVbo.create();
    m_maxCurveVbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
    m_maxCurveVbo.bind();
    m_maxCurveVbo.allocate(mesh->maxCurveData(), mesh->maxCurveSize() * sizeof(GL_FLOAT));

    m_minCurveVbo.create();
    m_minCurveVbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
    m_minCurveVbo.bind();
    m_minCurveVbo.allocate(mesh->minCurveData(), mesh->minCurveSize() * sizeof(GL_FLOAT));

    m_dCurvVbo.create();
    m_dCurvVbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
    m_dCurvVbo.bind();
    m_dCurvVbo.allocate(mesh->dCurveData(), mesh->dCurveSize() * sizeof(GL_FLOAT) * 4);

    m_colourVbo.create();
    m_colourVbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
    m_colourVbo.bind();
    m_colourVbo.allocate(mesh->cData(), mesh->cSize() * sizeof(GL_FLOAT) * 3);
    // Store the vertex attribute bindings for the program.
    setupObjectVertexAttribs();

    //Phong Params

    m_objectProgram->setUniformValue("Light.Intensity", QVector3D(0.7f, 0.7f, 0.7f));
    m_objectProgram->setUniformValue("Material.Ka", QVector3D(0.1f, 0.05f, 0.0f));
    m_objectProgram->setUniformValue("Material.Kd", QVector3D(.9f, .6f, .2f));
    m_objectProgram->setUniformValue("Material.Ks", QVector3D(.2f, .2f, .2f));
    m_objectProgram->setUniformValue("Material.Shininess", 1.f);

    m_objectProgram->release();
}

auto ViewerWidget::setupEllipseProgram() -> void
{

    ellipseMesh = new Mesh(QString("/Users/JamesHennessey/Dropbox/engd/multimodel_drawing_assistance/3D_models/circle.ply"));

    m_ellipsesProgram = new QOpenGLShaderProgram;

    m_ellipsesProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/resources/shaders/ellipseshader.vert");
    m_ellipsesProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/resources/shaders/ellipseshader.frag");

    m_ellipsesProgram->bindAttributeLocation("vertex", 0);

    m_ellipsesProgram->link();

    m_ellipsesProgram->bind();

    if(!m_ellipsesVao.isCreated())
    {
        m_ellipsesVao.create();
    }

    QOpenGLVertexArrayObject::Binder vaoBinder(&m_ellipsesVao);
    m_ellipsesVertexVbo.create();
    m_ellipsesVertexVbo.bind();
    m_ellipsesVertexVbo.allocate(ellipseMesh->vData(), (ellipseMesh->vSize()) * sizeof(GL_FLOAT) * 3);

    m_ellipsesIndexVbo.create();
    m_ellipsesIndexVbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
    m_ellipsesIndexVbo.bind();
    m_ellipsesIndexVbo.allocate(ellipseMesh->fData(), ellipseMesh->fSize() * sizeof(GLushort) * 3);

    // Store the vertex attribute bindings for the program.
    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
    m_ellipsesVertexVbo.bind();
    f->glEnableVertexAttribArray(0);
    f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), 0);
    m_ellipsesVertexVbo.release();

    m_ellipsesProgram->release();
}




auto ViewerWidget::setupPlanesProgram(std::vector<trimesh::point> points) -> void
{
    m_planesProgram = new QOpenGLShaderProgram;

    m_planesProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/resources/shaders/box.vert");
    m_planesProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/resources/shaders/box.frag");

    m_planesProgram->bindAttributeLocation("vertex", 0);

    m_planesProgram->link();

    m_planesProgram->bind();

    if(!m_planesVao.isCreated())
    {
        m_planesVao.create();
    }

    auto pointsSize = points.size();

    auto planeSize = pointsSize / 4;
//    qDebug() << "pointsSize " << pointsSize;
//    qDebug() << "planeSize " << planeSize;
    auto pointsData = points.data();

    QOpenGLVertexArrayObject::Binder vaoBinder(&m_planesVao);
    m_planesVertexVbo.create();
    m_planesVertexVbo.bind();
    m_planesVertexVbo.allocate(&pointsData[0], pointsSize * sizeof(GL_FLOAT) * 3);


    unsigned short planeIndicesTemplate[] = {0,1,2,
                                            0,2,3};

    planeFaceCount = planeSize * 2;

    planeFaces = (unsigned short*) std::malloc(planeFaceCount * 3 * sizeof(unsigned short));

    for(auto i = 1; i <= planeSize; i++)
    {
        auto step = (i - 1) * 6;
        for(auto j = 0; j < 6; j++)
        {
            auto index = (j + step);
            planeFaces[index] = (unsigned short) planeIndicesTemplate[j] + ((i - 1) * 4);
            //std::cout << planeFaces[index] << " ";
        }
        //std::cout << " " << std::endl;
    }

    m_planesIndexVbo.create();
    m_planesIndexVbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
    m_planesIndexVbo.bind();
    m_planesIndexVbo.allocate(planeFaces, planeFaceCount * sizeof(GLushort) * 3);

    // Store the vertex attribute bindings for the program.
    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
    m_planesVertexVbo.bind();
    f->glEnableVertexAttribArray(0);
    f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), 0);
    m_planesVertexVbo.release();

    m_planesProgram->release();
}


auto ViewerWidget::setupGuidesProgram(std::vector<trimesh::point> points) -> void
{
    m_guidesProgram = new QOpenGLShaderProgram;

    m_guidesProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/resources/shaders/box.vert");
    m_guidesProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/resources/shaders/box.frag");

    m_guidesProgram->bindAttributeLocation("vertex", 0);

    m_guidesProgram->link();

    m_guidesProgram->bind();

    if(!m_guidesVao.isCreated())
    {
        m_guidesVao.create();
    }

    auto pointsSize = points.size();
    auto pointsData = points.data();

    number_total_guides = pointsSize;

    qDebug() << "number_total_guides " << number_total_guides;


    QOpenGLVertexArrayObject::Binder vaoBinder(&m_guidesVao);
    m_guidesVertexVbo.create();
    m_guidesVertexVbo.bind();
    m_guidesVertexVbo.allocate(&pointsData[0], pointsSize * sizeof(GL_FLOAT) * 3);

    // Store the vertex attribute bindings for the program.
    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
    m_guidesVertexVbo.bind();
    f->glEnableVertexAttribArray(0);
    f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), 0);
    m_guidesVertexVbo.release();

    m_guidesProgram->release();


}

auto ViewerWidget::setupCandidatesProgram(std::vector<trimesh::point> points) -> void
{
    m_candidatesProgram = new QOpenGLShaderProgram;

    m_candidatesProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/resources/shaders/box.vert");
    m_candidatesProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/resources/shaders/box.frag");

    m_candidatesProgram->bindAttributeLocation("vertex", 0);

    m_candidatesProgram->link();

    m_candidatesProgram->bind();

    if(!m_candidatesVao.isCreated())
    {
        m_candidatesVao.create();
    }

    auto pointsSize = points.size();

    auto candidateSize = points.size() / 8;
    auto pointsData = points.data();

    QOpenGLVertexArrayObject::Binder vaoBinder(&m_candidatesVao);
    m_candidatesVertexVbo.create();
    m_candidatesVertexVbo.bind();
    m_candidatesVertexVbo.allocate(pointsData[0], pointsSize * sizeof(GL_FLOAT) * 3);


    unsigned short candidateIndicesTemplate[] = {0,7,1,
                                          0,6,7,
                                          0,2,1,
                                          2,1,3,
                                          0,6,2,
                                          6,4,2,
                                          2,4,3,
                                          4,5,3,
                                          7,5,3,
                                          7,3,1,
                                          6,7,5,
                                          4,6,5};
    candidateFaceCount = candidateSize * 12;

    candidateFaces = (unsigned short*) std::malloc(candidateFaceCount * 3 * sizeof(unsigned short));

    for(auto i = 1; i <= candidateSize; i++)
    {
        auto step = (i - 1) * 36;
        for(auto j = 0; j < 36; j++)
        {
            auto index = (j + step);
            candidateFaces[index] = (unsigned short) candidateIndicesTemplate[j] + ((i - 1) * 8);
        }
    }

    m_candidatesIndexVbo.create();
    m_candidatesIndexVbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
    m_candidatesIndexVbo.bind();
    m_candidatesIndexVbo.allocate(candidateFaces, candidateFaceCount * sizeof(GLushort) * 3);

    // Store the vertex attribute bindings for the program.
    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
    m_candidatesVertexVbo.bind();
    f->glEnableVertexAttribArray(0);
    f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), 0);
    m_candidatesVertexVbo.release();

    m_candidatesProgram->release();
}

auto ViewerWidget::setupBoundingBoxProgram() -> void
{
    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();

    m_bbProgram = new QOpenGLShaderProgram;

    m_bbProgram->addShaderFromSourceFile(QOpenGLShader::Vertex,   ":/resources/shaders/box.vert");
    m_bbProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/resources/shaders/box.frag");

    m_bbGeometryShader =  new QOpenGLShader(QOpenGLShader::Geometry);
    m_bbGeometryShader->compileSourceFile(":/resources/shaders/box.geom");

    m_perspectiveGeometryShader =  new QOpenGLShader(QOpenGLShader::Geometry);
    m_perspectiveGeometryShader->compileSourceFile(":/resources/shaders/perspectivelines.glsl");

    //m_bbProgram->addShader(m_bbGeometryShader);

    m_bbProgram->bindAttributeLocation("vertex", 0);

    m_bbProgram->link();

    m_bbProgram->bind();

    if(!m_bbVao.isCreated())
    {
        m_bbVao.create();
    }

    QOpenGLVertexArrayObject::Binder vaoBinder(&m_bbVao);
    m_bbVertexVbo.create();
    m_bbVertexVbo.bind();

//    if(renderClusters) // Setup cluster specific bounding boxes and segments
//    {
//        //qDebug() << "render clusters allocate 1";
//        auto bbPoints = mesh->clusterBoundingBoxPoints(clusterStartIndex);
//        auto data = bbPoints.data();
//        float* vertexData = data[0];
//        auto clusterSize = mesh->clusterBoundingBoxPointsSize(clusterStartIndex);

//        m_bbVertexVbo.allocate(vertexData, (clusterSize) * sizeof(GL_FLOAT) * 3);
//    }
//    else // Setup normal bounding box and segments
//    {
//    }

    m_bbVertexVbo.allocate(mesh->bBoxData(), (mesh->bBoxSize()) * sizeof(GL_FLOAT) * 3);

    m_bbIndexVbo.create();
    m_bbIndexVbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
    m_bbIndexVbo.bind();

//    if(renderClusters) // Setup cluster specific bounding boxes and segments
//    {
//        //qDebug() << "render clusters allocate 2";
//        auto cluster_no = mesh->noClusters(clusterStartIndex);
//        auto boundingBoxFaces = mesh->clusterFaceData(clusterStartIndex);
//        //    qDebug() << "viewer_widget bBoxSize " << mesh->bBoxSize();
//        //qDebug() << "viewer_widget cluster_no " << cluster_no;
//        m_bbIndexVbo.allocate(boundingBoxFaces, cluster_no * bb_faces_count * sizeof(unsigned short) * 3);
//    }
//    else // Setup normal bounding box and segments
//    {
//    }

    auto seg_no = mesh->segmentSize();
    auto boundingBoxFaces = mesh->bBoxFaceData();
    //    qDebug() << "viewer_widget bBoxSize " << mesh->bBoxSize();
    //    qDebug() << "viewer_widget seg_no " << seg_no;
    m_bbIndexVbo.allocate(boundingBoxFaces, seg_no * bb_faces_count * sizeof(unsigned short) * 3);

    f->glEnableVertexAttribArray(0);
    f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), 0);

    m_bbVertexVbo.release();
}

auto ViewerWidget::setupTextProgram() -> void
{

    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();


    static const QVector3D vertextData[] = {
        QVector3D(-1.0f, -1.0f,  0.0f),
        QVector3D( 1.0f, -1.0f,  0.0f),
        QVector3D( 1.0f,  1.0f,  0.0f),
        QVector3D(-1.0f,  1.0f,  0.0f)
    };

    // indices
    static const GLushort indices[] = {
        0,  1,  2,
        0,  2,  3
    };

    m_textProgram = new QOpenGLShaderProgram;
    m_textProgram->addShaderFromSourceFile(QOpenGLShader::Vertex,   ":/resources/shaders/text.vert");
    m_textProgram->addShaderFromSourceFile(QOpenGLShader::Fragment,   ":/resources/shaders/text.frag");
    m_textProgram->bindAttributeLocation("vertex", 0);
    m_textProgram->bindAttributeLocation("texCoord", 1);
    m_textProgram->link();

    m_textProgram->bind();
    m_textProgram->setUniformValue("texture", 0);

    if(!m_textVao.isCreated())
    {
        m_textVao.create();
    }

    QOpenGLVertexArrayObject::Binder vaoBinder(&m_textVao);

    m_textVertexVbo.create();
    m_textVertexVbo.bind();
    m_textVertexVbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
    m_textVertexVbo.allocate(vertextData, 4 * sizeof(float) * 3);

    m_textIndexVbo.create();
    m_textIndexVbo.bind();
    m_textIndexVbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
    m_textIndexVbo.allocate(indices, 6 * sizeof(GLushort));

    setTextTextureCoords(100,100);

    m_textVertexVbo.bind();
    f->glEnableVertexAttribArray(0);
    f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), 0);
    m_textVertexVbo.release();

    m_textProgram->release();

}

auto ViewerWidget::setTextTextureCoords(int width, int height) -> void
{
    const QVector2D textureData[] = {
        QVector2D(0.0f,  0.0f),
        QVector2D(1.0f,  0.0f),
        QVector2D(1.0f,  1.0f),
        QVector2D(0.0f,  1.0f)
    };
    m_textTextureCoordVbo.create();
    m_textTextureCoordVbo.bind();
    m_textTextureCoordVbo.setUsagePattern(QOpenGLBuffer::StaticDraw);
    m_textTextureCoordVbo.allocate(textureData, 4 * sizeof(float) * 2);

    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();

    m_textTextureCoordVbo.bind();
    f->glEnableVertexAttribArray(1);
    f->glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(GLfloat), 0);
    m_textTextureCoordVbo.release();

}

auto ViewerWidget::setupObjectShaders() -> void
{
    switch(m_renderMode)
    {
    case RenderMode::PHONG: setupPhongShader(); break;
    case RenderMode::SUGGESTIVE_CONTOURS: setupSuggestiveContoursShader(); break;
    case RenderMode::SEGMENTATION: setupSegmentationShader(); break;
    }
}

auto ViewerWidget::setRenderMode(int i) -> void
{
    switch(i)
    {
    case 0: m_renderMode = RenderMode::PHONG; break;
    case 1: m_renderMode = RenderMode::SEGMENTATION; break;
    case 2: m_renderMode = RenderMode::SUGGESTIVE_CONTOURS; break;
    }
    initializeGL();
    update();
}

auto ViewerWidget::setupPhongShader() -> void
{
    m_objectProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/resources/shaders/phongvshader.vert");
    m_objectProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/resources/shaders/phongfshader.frag");
}

auto ViewerWidget::setupSuggestiveContoursShader() -> void
{
    m_objectProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/resources/shaders/suggestive_contours.vert");
    m_objectProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/resources/shaders/suggestive_contours.frag");
}

auto ViewerWidget::setupSegmentationShader() -> void
{
    m_objectProgram->addShaderFromSourceFile(QOpenGLShader::Vertex, ":/resources/shaders/segmentation.vert");
    m_objectProgram->addShaderFromSourceFile(QOpenGLShader::Fragment, ":/resources/shaders/segmentation.frag");
}

auto ViewerWidget::setupObjectVertexAttribs() -> void
{
    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
    m_vertexVbo.bind();
    f->glEnableVertexAttribArray(0);
    f->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), 0);
    m_vertexVbo.release();

    m_normalVbo.bind();
    f->glEnableVertexAttribArray(1);
    f->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), 0);
    m_normalVbo.release();

    m_maxDirVbo.bind();
    f->glEnableVertexAttribArray(2);
    f->glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), 0);
    m_maxDirVbo.release();

    m_minDirVbo.bind();
    f->glEnableVertexAttribArray(3);
    f->glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), 0);
    m_minDirVbo.release();

    m_maxCurveVbo.bind();
    f->glEnableVertexAttribArray(4);
    f->glVertexAttribPointer(4, 1, GL_FLOAT, GL_FALSE, sizeof(GLfloat), 0);
    m_maxCurveVbo.release();

    m_minCurveVbo.bind();
    f->glEnableVertexAttribArray(5);
    f->glVertexAttribPointer(5, 1, GL_FLOAT, GL_FALSE, sizeof(GLfloat), 0);
    m_minCurveVbo.release();

    m_dCurvVbo.bind();
    f->glEnableVertexAttribArray(6);
    f->glVertexAttribPointer(6, 4, GL_FLOAT, GL_FALSE, sizeof(GLfloat), 0);
    m_dCurvVbo.release();

    m_colourVbo.bind();
    f->glEnableVertexAttribArray(7);
    f->glVertexAttribPointer(7, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), 0);
    m_colourVbo.release();
}

void ViewerWidget::paintGL()
{
    if(mesh == nullptr)
        return;


    //Save Image
    if(saveImageFromFBO)
    {
        renderToImage();
    }
    else // Normal Render
    {
        renderObject();
        renderBoxes();
        renderEllipses();
        renderPlanes();
        renderCandidates();
        renderGuides();
        renderPerspectiveLines();
        if(showEdgeRatios)
            renderRatioText();
    }


}

auto ViewerWidget::renderToImage() -> void
{

#ifdef __APPLE__
    static const auto location = QString("/Users/JamesHennessey/Dropbox/engd/multimodel_drawing_assistance/screengrabs/");
#else
    static const auto location = QString("C:/Users/James Hennesay/Dropbox/engd/multimodel_drawing_assistance/3D_models/");
#endif

    auto fileName = QFileDialog::getSaveFileName(this,
                                                 QString("Save File"),
                                                 location);
    if (!fileName.isEmpty()) {


         QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
        f->glClearColor(1.0, 1.0, 1.0, m_transparent ? 0 : 1);

        QOpenGLFramebufferObjectFormat fboFormat;
        fboFormat.setAttachment(QOpenGLFramebufferObject::Depth);
        qDebug() << "width " << width() << " " << height();
        QSize drawRectSize(width(), height());

        QOpenGLFramebufferObject fbo(drawRectSize, fboFormat);

        fbo.bind();

        renderObject();
        renderBoxes();
        renderPerspectiveLines();
        if(showEdgeRatios)
            renderRatioText();


        auto fboImage = fbo.toImage();
        qDebug() << "fboImage " << fboImage.width() << " " << fboImage.height();
        auto image = QImage(fboImage.constBits(), fboImage.width(), fboImage.height(), QImage::Format_ARGB32);

        image.save(fileName);

        fbo.release();

        saveImageFromFBO = false;

        f->glClearColor(0.2, 0.2, 0.2, m_transparent ? 0 : 1);

    }

    update();

}


auto ViewerWidget::zoomChanged(double val) -> void
{
    auto translate = camera_pos * val;
    m_view.lookAt(translate, QVector3D(0,0,0), QVector3D(0,1,0));
    update();
}

auto ViewerWidget::renderObject() -> void
{
    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();

    f->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    f->glEnable(GL_MULTISAMPLE);
    f->glDisable(GL_BLEND);
    f->glEnable(GL_DEPTH_TEST);
   // f->glEnable(GL_CULL_FACE);

    QOpenGLVertexArrayObject::Binder vaoBinder1(&m_objectVao);
    m_objectProgram->bind();

    //m_model.setToIdentity();
    auto rot = m_rotationTrackball.rotation();
    m_model.rotate(rot);
    //m_view.rotate(rot);



    //
    if(!showObject) return;

    switch(m_renderMode)
    {
    case RenderMode::PHONG:               renderPhong();              break;
    case RenderMode::SUGGESTIVE_CONTOURS: renderSuggestiveContours(); break;
    //Exactly the same params as phong
    case RenderMode::SEGMENTATION:        renderPhong();       break;
    }

    f->glDrawElements(GL_TRIANGLES, mesh->fSize() * 3, GL_UNSIGNED_SHORT, 0);
    // debugging f->glDrawArrays(GL_TRIANGLE_STRIP, 0, mesh->bBoxSize() * 3);
    m_objectProgram->release();
}

auto ViewerWidget::renderBoxes() -> void
{
    if(renderSegmentBoxes || renderBoundingBox)
    {
        if(renderBoxFacesCheck)
            renderBoxFaces();
        renderBoxEdges();
    }
}

auto ViewerWidget::renderBoxFaces() -> void
{
    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
    //Bounding Box

    QOpenGLVertexArrayObject::Binder vaoBinder2(&m_bbVao);
    m_bbProgram->bind();

    auto model_view = m_view * m_model;

    m_bbProgram->setUniformValue("projMatrix", m_proj);
    m_bbProgram->setUniformValue("viewMatrix", m_view);
    m_bbProgram->setUniformValue("modelMatrix", m_model);
    m_bbProgram->setUniformValue("mvMatrix", model_view);

    f->glDisable(GL_CULL_FACE);
    f->glEnable(GL_BLEND);
    f->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    m_bbProgram->setUniformValue("hasGeomShader", false);

    renderBoxHelper();

    m_bbProgram->release();
}

auto ViewerWidget::renderEllipses() -> void
{
    if(!renderEllipsesCheck)
        return;
    //qDebug() << "ViewerWidget::renderEllipses()";

    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
    //Bounding Box

    QOpenGLVertexArrayObject::Binder vaoBinder4(&m_ellipsesVao);
    m_ellipsesProgram->bind();

    auto model_view = m_view * m_model;

    m_ellipsesProgram->setUniformValue("projMatrix", m_proj);
    m_ellipsesProgram->setUniformValue("mvMatrix", model_view);


    f->glDisable(GL_CULL_FACE);
    f->glEnable(GL_BLEND);
    f->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


    auto segments = mesh->getMeshSegments();

    for(auto s : segments)
    {
        auto ellipses = s->getEllipses();
        for(auto e : ellipses)
        {
            m_ellipsesProgram->setUniformValue("ellipse_rotation", e->getRotation());
            auto translation = e->getTranslation();
            Eigen::Vector3f eigenTranslation(translation[0], translation[1],translation[2]);
            Eigen::Matrix3f rotation = s->getInvRotation();
            eigenTranslation = rotation * eigenTranslation;
            auto translationAdjusted = QVector3D(eigenTranslation[0],eigenTranslation[1],eigenTranslation[2]);
            m_ellipsesProgram->setUniformValue("translation", translationAdjusted);
            m_ellipsesProgram->setUniformValue("radius", e->getRadius());
            //Hardcoded no. faces
            f->glDrawElements(GL_TRIANGLES, 332 * 3, GL_UNSIGNED_SHORT, 0);
        }
    }

    m_ellipsesProgram->release();
}

auto ViewerWidget::renderPlanes() -> void
{
    if(!renderPlanesCheck || planeFaceCount <= 0)
        return;
    //qDebug() << "ViewerWidget::renderPlanes() " << planeFaceCount;
    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
    //Bounding Box

    QOpenGLVertexArrayObject::Binder vaoBinder4(&m_planesVao);
    m_planesProgram->bind();

    m_planesProgram->addShader(m_bbGeometryShader);
    m_planesProgram->link();

    auto model_view = m_view * m_model;

    m_planesProgram->setUniformValue("projMatrix", m_proj);
    m_planesProgram->setUniformValue("viewMatrix", m_view);
    m_planesProgram->setUniformValue("modelMatrix", m_model);
    m_planesProgram->setUniformValue("mvMatrix", model_view);

    f->glEnable(GL_MULTISAMPLE);
    f->glDisable(GL_CULL_FACE);
    f->glEnable(GL_BLEND);
    f->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    m_planesProgram->setUniformValue("showMidLines", renderMidLines);

    m_planesProgram->setUniformValue("showThirdsLine", renderThirdsLine);
    m_planesProgram->setUniformValue("showTwoThirdsLine", renderTwoThirdsLine);

    m_planesProgram->setUniformValue("showThirdsDiagLine", renderThirdsDiagLine);
    m_planesProgram->setUniformValue("showTwoThirdsDiagLine", renderTwoThirdsDiagLine);
    m_planesProgram->setUniformValue("showDiagLines", renderDiagLines);
    m_planesProgram->setUniformValue("hasGeomShader", true);

    m_planesProgram->setUniformValue("dashedLines", false);

    f->glDisable(GL_CULL_FACE);
    f->glEnable(GL_BLEND);
    f->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


   const static std::vector<QVector3D> box_colours = {QVector3D(0, 255, 0), QVector3D(0, 0, 255), QVector3D(0, 255, 255), QVector3D(255, 255, 0), QVector3D(200, 100, 40)};

    for(auto i = size_t(0); i < planeCandidatePerPrimitve.size(); i++)
    {
         auto start_index = 0;
         if(i > 0)
             start_index = planeCandidatePerPrimitve[i-1] + 1;

         auto no_faces = planeCandidatePerPrimitve[i];
         if(i > 0)
             no_faces -= planeCandidatePerPrimitve[i-1] + 1;

         auto box_colour = QVector3D(0, 255, 0);
         if(i < (size_t) box_colours.size())
             box_colour = box_colours[i];

         m_planesProgram->setUniformValue("box_colour", box_colour);

         //qDebug() << "no_faces " << no_faces << " start_index " << start_index ;

         f->glDrawElements(GL_TRIANGLES, no_faces * 3, GL_UNSIGNED_SHORT, (void*)(start_index * 3 * sizeof(GLushort)));
    }

    m_planesProgram->release();
}


auto ViewerWidget::renderGuides() -> void
{
    if(!renderGuidesCheck || number_total_guides == 0)
        return;


    //qDebug() << "ViewerWidget::renderGuides() " << number_total_guides;
    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
    //Bounding Box

    QOpenGLVertexArrayObject::Binder vaoBinder4(&m_guidesVao);
    m_guidesProgram->bind();

    //m_guidesProgram->addShader(m_bbGeometryShader);
    m_guidesProgram->link();

    auto model_view = m_view * m_model;

    m_guidesProgram->setUniformValue("projMatrix", m_proj);
    m_guidesProgram->setUniformValue("viewMatrix", m_view);
    m_guidesProgram->setUniformValue("modelMatrix", m_model);
    m_guidesProgram->setUniformValue("mvMatrix", model_view);

    f->glEnable(GL_MULTISAMPLE);
    f->glDisable(GL_CULL_FACE);
    f->glEnable(GL_BLEND);
    f->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    m_guidesProgram->setUniformValue("showMidLines", renderMidLines);

    m_guidesProgram->setUniformValue("showThirdsLine", renderThirdsLine);
    m_guidesProgram->setUniformValue("showTwoThirdsLine", renderTwoThirdsLine);

    m_guidesProgram->setUniformValue("showThirdsDiagLine", renderThirdsDiagLine);
    m_guidesProgram->setUniformValue("showTwoThirdsDiagLine", renderTwoThirdsDiagLine);
    m_guidesProgram->setUniformValue("showDiagLines", renderDiagLines);
    m_guidesProgram->setUniformValue("hasGeomShader", false);

    m_guidesProgram->setUniformValue("dashedLines", false);

    f->glDisable(GL_CULL_FACE);
    f->glEnable(GL_BLEND);
    f->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


    m_guidesProgram->setUniformValue("box_colour", QVector3D(255, 255, 0));
    //f->glDrawElements(GL_LINES, number_total_guides, GL_UNSIGNED_SHORT, (void*)(start_index * sizeof(GLushort)));
    f->glDrawArrays(GL_LINES, 0, number_total_guides);
    m_guidesProgram->release();
}


auto ViewerWidget::renderCandidates() -> void
{
    if(!renderCandidatesCheck || candidateFaceCount <= 0)
        return;
    //qDebug() << "ViewerWidget::renderPlanes() " << planeFaceCount;
    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
    //Bounding Box

    QOpenGLVertexArrayObject::Binder vaoBinder4(&m_candidatesVao);
    m_candidatesProgram->bind();

    m_candidatesProgram->addShader(m_bbGeometryShader);
    m_candidatesProgram->link();

    auto model_view = m_view * m_model;

    m_candidatesProgram->setUniformValue("projMatrix", m_proj);
    m_candidatesProgram->setUniformValue("viewMatrix", m_view);
    m_candidatesProgram->setUniformValue("modelMatrix", m_model);
    m_candidatesProgram->setUniformValue("mvMatrix", model_view);

    f->glEnable(GL_MULTISAMPLE);
    f->glDisable(GL_CULL_FACE);
    f->glEnable(GL_BLEND);
    f->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    m_candidatesProgram->setUniformValue("showMidLines", renderMidLines);

    m_candidatesProgram->setUniformValue("showThirdsLine", renderThirdsLine);
    m_candidatesProgram->setUniformValue("showTwoThirdsLine", renderTwoThirdsLine);

    m_candidatesProgram->setUniformValue("showThirdsDiagLine", renderThirdsDiagLine);
    m_candidatesProgram->setUniformValue("showTwoThirdsDiagLine", renderTwoThirdsDiagLine);
    m_candidatesProgram->setUniformValue("showDiagLines", renderDiagLines);
    m_candidatesProgram->setUniformValue("hasGeomShader", true);

    m_candidatesProgram->setUniformValue("dashedLines", false);

    f->glDisable(GL_CULL_FACE);
    f->glEnable(GL_BLEND);
    f->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


   const static std::vector<QVector3D> box_colours = {QVector3D(0, 255, 0), QVector3D(0, 0, 255), QVector3D(0, 255, 255), QVector3D(255, 255, 0), QVector3D(200, 100, 40)};

    for(auto i = size_t(0); i < completeCandidatePerPrimitve.size(); i++)
    {
         auto start_index = 0;
         if(i > 0)
             start_index = completeCandidatePerPrimitve[i-1] + 1;

         auto no_faces = completeCandidatePerPrimitve[i];
         if(i > 0)
             no_faces -= completeCandidatePerPrimitve[i-1] + 1;

         //qDebug() << "start_index " << start_index << " no_faces " << no_faces;

         auto box_colour = QVector3D(0, 255, 0);
         if(i < (size_t) box_colours.size())
             box_colour = box_colours[i];

         m_candidatesProgram->setUniformValue("box_colour", box_colour);

         f->glDrawElements(GL_TRIANGLES, no_faces * 3, GL_UNSIGNED_SHORT, (void*)(start_index * 3 * sizeof(GLushort)));
    }

    m_candidatesProgram->release();
}

auto ViewerWidget::renderBoxHelper() -> void
{


    auto planeColour = QVector3D(255, 165, 0);
    auto cuboidColour = QVector3D(0, 255, 100);
    auto tpColour = QVector3D(43, 82, 250);
    auto cylinderColour = QVector3D(250, 43, 189);
    const static std::vector<QVector3D> box_colours = {planeColour, tpColour, cuboidColour, cuboidColour, cylinderColour, cylinderColour, cuboidColour};

    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
    m_bbProgram->setUniformValue("render_mid_plane", false);
    if(renderBoundingBox)
    {
        m_bbProgram->setUniformValue("box_colour", QVector3D(0.8,0.8,0.8));
        f->glDrawElements(GL_TRIANGLES, bb_faces_count * 3, GL_UNSIGNED_SHORT, 0);
    }

//    if(selectedSegment > 0)
//    {
//        auto start_index = 36 * (selectedSegment);
//        auto box_colour = QVector3D(255, 255, 255);
//        m_bbProgram->setUniformValue("box_colour", box_colour);
//        f->glDrawElements(GL_TRIANGLES, bb_faces_count * 3, GL_UNSIGNED_SHORT, (void*)(start_index * sizeof(GLushort)));
//        return;
//    }

    if(renderSegmentBoxes)
    {
        //qDebug() << "mesh->segmentSize() " << mesh->segmentSize();
        for(auto i = 1; i <= mesh->segmentSize(); i++)
        {
            //qDebug() << i;
            //Segment is selected
            if(mesh->segmentIsSelected(i-1))
            {
                //qDebug() << i << " is selected";
                auto c = mesh->segmentColour(i-1);
                auto box_colour = QVector3D(255, 0, 0);
                auto start_index = (i * 36);

                auto segment = mesh->getSelectedSegment(i-1);
                //qDebug() << "Axis " << segment->getAxis() << " Face " << segment->getFace();
                if(segment->getAxis() > -1)
                {
                    m_bbProgram->setUniformValue("box_colour", box_colour);

                    if(segment->getAxis() == 1) //X
                    {
                        if(segment->getFace() == -1 || segment->getFace() == 1)
                        {
                            //Back
                            f->glDrawElements(GL_TRIANGLES, 2 * 3, GL_UNSIGNED_SHORT, (void*)(start_index * sizeof(GLushort)));
                        }
                        if(segment->getFace() == -1 || segment->getFace() == 2)
                        {
                            //Front
                            auto custom_start_index = (start_index + (6 * 3));
                            f->glDrawElements(GL_TRIANGLES, 2 * 3, GL_UNSIGNED_SHORT, (void*)(custom_start_index * sizeof(GLushort)));
                        }
                        if(segment->getFace() == 3)
                        {
//                            m_bbProgram->setUniformValue("render_mid_plane", true);
//                            m_bbProgram->setUniformValue("showMidLines", true);
//                            f->glDrawElements(GL_TRIANGLES, 2 * 3, GL_UNSIGNED_SHORT, (void*)(start_index * sizeof(GLushort)));
//                            auto custom_start_index = (start_index + (6 * 3));
//                            f->glDrawElements(GL_TRIANGLES, 2 * 3, GL_UNSIGNED_SHORT, (void*)(custom_start_index * sizeof(GLushort)));
//                            m_bbProgram->setUniformValue("render_mid_plane", false);
//                            m_bbProgram->setUniformValue("showMidLines", renderMidLines);
                        }

                    }
                    else if(segment->getAxis() == 2) //Y
                    {
                        if(segment->getFace() == -1 || segment->getFace() == 1)
                        {
                            //Left Side
                            auto custom_start_index = (start_index + (2 * 3));
                            f->glDrawElements(GL_TRIANGLES, 2 * 3, GL_UNSIGNED_SHORT, (void*)(custom_start_index * sizeof(GLushort)));
                        }
                        if(segment->getFace() == -1 || segment->getFace() == 2)
                        {
                            //Right
                            auto custom_start_index = (start_index + (10 * 3));
                            f->glDrawElements(GL_TRIANGLES, 2 * 3, GL_UNSIGNED_SHORT, (void*)(custom_start_index * sizeof(GLushort)));
                        }
                        if(segment->getFace() == 3)
                        {
                            //m_bbProgram->setUniformValue("render_mid_plane", true);
                            m_bbProgram->setUniformValue("render_mid_plane", true);
                            m_bbProgram->setUniformValue("showMidLines", true);
                            f->glDrawElements(GL_TRIANGLES, 2 * 3, GL_UNSIGNED_SHORT, (void*)(start_index * sizeof(GLushort)));
                            auto custom_start_index = (start_index + (6 * 3));
                            f->glDrawElements(GL_TRIANGLES, 2 * 3, GL_UNSIGNED_SHORT, (void*)(custom_start_index * sizeof(GLushort)));

                            custom_start_index = (start_index + (8 * 3));
                            f->glDrawElements(GL_TRIANGLES, 2 * 3, GL_UNSIGNED_SHORT, (void*)(custom_start_index * sizeof(GLushort)));
                            custom_start_index = (start_index + (4 * 3));
                            f->glDrawElements(GL_TRIANGLES, 2 * 3, GL_UNSIGNED_SHORT, (void*)(custom_start_index * sizeof(GLushort)));

                            m_bbProgram->setUniformValue("render_mid_plane", false);
                            m_bbProgram->setUniformValue("showMidLines", renderMidLines);

                            box_colour = QVector3D(255, 255, 255);
//                             if(i < (size_t) box_colours.size())
//                                 box_colour = box_colours[i];
                             auto start_index = (i * 36);
                             m_bbProgram->setUniformValue("box_colour", box_colour);
                             f->glDrawElements(GL_TRIANGLES, bb_faces_count * 3, GL_UNSIGNED_SHORT, (void*)(start_index * sizeof(GLushort)));



                        }
                    }
                    else if(segment->getAxis() == 3) //Z
                    {
                        if(segment->getFace() == -1 || segment->getFace() == 1)
                        {
                            //Top
                            auto custom_start_index = (start_index + (8 * 3));
                            f->glDrawElements(GL_TRIANGLES, 2 * 3, GL_UNSIGNED_SHORT, (void*)(custom_start_index * sizeof(GLushort)));
                        }
                        if(segment->getFace() == -1 || segment->getFace() == 2)
                        {
                            //Bottom
                            auto custom_start_index = (start_index + (4 * 3));
                            f->glDrawElements(GL_TRIANGLES, 2 * 3, GL_UNSIGNED_SHORT, (void*)(custom_start_index * sizeof(GLushort)));
                        }
                        if(segment->getFace() == 3)
                        {
                            //m_bbProgram->setUniformValue("render_mid_plane", true);
//                            m_bbProgram->setUniformValue("render_mid_plane", true);
//                            m_bbProgram->setUniformValue("showMidLines", true);
//                            f->glDrawElements(GL_TRIANGLES, bb_faces_count * 3, GL_UNSIGNED_SHORT, (void*)(start_index * sizeof(GLushort)));
//                            m_bbProgram->setUniformValue("render_mid_plane", false);
//                            m_bbProgram->setUniformValue("showMidLines", renderMidLines);
                        }
                    }
                }
                else
                {
                    box_colour = QVector3D(255, 255, 255);
//                     if(i < (size_t) box_colours.size())
//                         box_colour = box_colours[i];

//                     qDebug() << i << " " << box_colour;

                    m_bbProgram->setUniformValue("box_colour", box_colour);
                    f->glDrawElements(GL_TRIANGLES, bb_faces_count * 3, GL_UNSIGNED_SHORT, (void*)(start_index * sizeof(GLushort)));
                }

            }
            else
            {
                 m_bbProgram->setUniformValue("box_colour", QVector3D(255, 255, 255));

               auto box_colour = QVector3D(255, 0, 0);
                if(i < (size_t) box_colours.size())
                    box_colour = box_colours[i];


                //m_bbProgram->setUniformValue("box_colour", box_colour);

                auto start_index = (i * 36);
                //const static std::vector<QVector3D> box_colours = {QVector3D(244, 252, 23), QVector3D(23, 252, 239), QVector3D(23, 252, 77), QVector3D(23, 252, 77), QVector3D(252, 158, 23), QVector3D(252, 158, 23), QVector3D(23, 252, 77)};


                if(i == 1) box_colour = QVector3D(244, 252, 23);
                 if(i == 2) box_colour = QVector3D(23, 252, 239);


                qDebug() << i << " " << box_colour;
                if(i < (size_t) box_colours.size())
                    box_colour = box_colours[i];

                f->glDrawElements(GL_TRIANGLES, bb_faces_count * 3, GL_UNSIGNED_SHORT, (void*)(start_index * sizeof(GLushort)));

            }

        }
    }
}


auto ViewerWidget::selectSegment(int id) -> void
{
    qDebug() << "selectSegment " << id;
    selectedSegment = id;

}

auto ViewerWidget::renderRatioText() -> void
{
    auto vertices = mesh->bBoxData();
    auto boundingBoxFaces = mesh->bBoxFaceData();
    auto mvp = m_proj * m_view * m_model;
    int start_index = 0;
    int end_index = mesh->segmentSize();

    auto selectedEdgeMapped = mesh->edgeToEdgeId(selectedEdgeBox, selectedEdge);
    auto unitEdgeMapped = mesh->edgeToEdgeId(unitLengthBox, unitLengthEdge);


    auto unitLength = mesh->screenSpaceEdgeLength(m_viewport, mvp, unitLengthBox, unitLengthEdge);

    updateSelectedEdgeRatio();

    if(showEdgeRatioSetting == 1)
    {
        end_index = 1;
    }
    else if(showEdgeRatioSetting == 2)
    {
        start_index = 1;
    }
    else if(showEdgeRatioSetting > 2)
    {
        start_index = showEdgeRatioSetting - 2;
        end_index = showEdgeRatioSetting - 1;
    }

    //Number of edges on a box with triangle faces
    auto totalEdgeCount = 0;
    for(auto i = bb_faces_count * start_index; i < bb_faces_count * end_index; i++)
    {
        //This is a hack to get the right indexing...
        if(i % bb_faces_count == 0)
        {
            totalEdgeCount = 36 * i / bb_faces_count;
        }
        auto fIndex = i * 3;
        auto face = boundingBoxFaces[fIndex] * 3;
        auto v1 = QVector4D(vertices[face], vertices[face+1], vertices[face+2],1);
        face = boundingBoxFaces[fIndex+1]  * 3;
        auto v2 = QVector4D(vertices[face], vertices[face+1], vertices[face+2],1);
        face = boundingBoxFaces[fIndex+2]  * 3;
        auto v3 = QVector4D(vertices[face], vertices[face+1], vertices[face+2],1);
        auto ss1 = mvp * v1;
        ss1 /= ss1[3];

        auto ss2 = mvp * v2;
        ss2 /= ss2[3];

        auto ss3 = mvp * v3;
        ss3 /= ss3[3];

        auto pos1 =  (QVector2D(ss1[0], ss1[1]) + QVector2D(ss2[0], ss2[1])) * 0.5;
        auto pos2 = (QVector2D(ss2[0], ss2[1]) + QVector2D(ss3[0], ss3[1])) * 0.5;
        auto pos3 = (QVector2D(ss3[0], ss3[1]) + QVector2D(ss1[0], ss1[1])) * 0.5;

        ss1 = m_viewport * ss1;
        ss2 = m_viewport * ss2;
        ss3 = m_viewport * ss3;

        auto edge1 = QVector2D(ss1[0], ss1[1]) - QVector2D(ss2[0], ss2[1]);
        auto edge2 = QVector2D(ss2[0], ss2[1]) - QVector2D(ss3[0], ss3[1]);
        auto edge3 = QVector2D(ss3[0], ss3[1]) - QVector2D(ss1[0], ss1[1]);


        auto edge1_4d = v1 - v2;
        auto edge1_3d = QVector3D(edge1_4d[0], edge1_4d[1], edge1_4d[2]);
        auto edge2_4d = v2 - v3;
        auto edge2_3d = QVector3D(edge2_4d[0], edge2_4d[1], edge2_4d[2]);
        auto edge3_4d = v3 - v1;
        auto edge3_3d = QVector3D(edge3_4d[0], edge3_4d[1], edge3_4d[2]);

        QColor c = QColor(0,0,0);
        if(fabs(QVector3D::dotProduct(edge1_3d, edge2_3d)) < 1.0e-7)
        {
            auto ratio = edge1.length() / unitLength;
            c = getRatioTextColour(unitLengthBox, unitEdgeMapped, selectedEdgeBox, selectedEdgeMapped, totalEdgeCount, ratio);
            displayText(QString(QString::number(ratio,'f', 2)), c, QColor(255,255,255), QVector3D(pos1[0],pos1[1],1));
            totalEdgeCount++;

            ratio = edge2.length() / unitLength;
            c = getRatioTextColour(unitLengthBox, unitEdgeMapped, selectedEdgeBox, selectedEdgeMapped, totalEdgeCount, ratio);
            displayText(QString(QString::number(ratio,'f', 2)), c, QColor(255,255,255), QVector3D(pos2[0],pos2[1],1));
            totalEdgeCount++;
        }

        if(fabs(QVector3D::dotProduct(edge2_3d, edge3_3d)) < 1.0e-7)
        {
            auto ratio = edge2.length() / unitLength;
            c = getRatioTextColour(unitLengthBox, unitEdgeMapped, selectedEdgeBox, selectedEdgeMapped, totalEdgeCount, ratio);
            displayText(QString(QString::number(ratio,'f', 2)), c, QColor(255,255,255), QVector3D(pos2[0],pos2[1],1));
            totalEdgeCount++;

            ratio = edge3.length() / unitLength;
            c = getRatioTextColour(unitLengthBox, unitEdgeMapped, selectedEdgeBox, selectedEdgeMapped, totalEdgeCount, ratio);
            displayText(QString(QString::number(ratio,'f', 2)), c, QColor(255,255,255), QVector3D(pos3[0],pos3[1],1));
            totalEdgeCount++;
        }

        if(fabs(QVector3D::dotProduct(edge3_3d, edge1_3d)) < 1.0e-7)
        {
            auto ratio = edge3.length() / unitLength;
            c = getRatioTextColour(unitLengthBox, unitEdgeMapped, selectedEdgeBox, selectedEdgeMapped, totalEdgeCount, ratio);
            displayText(QString(QString::number(ratio,'f', 2)), c, QColor(255,255,255), QVector3D(pos3[0],pos3[1],1));
            totalEdgeCount++;

            ratio = edge1.length() / unitLength;
            c = getRatioTextColour(unitLengthBox, unitEdgeMapped, selectedEdgeBox, selectedEdgeMapped, totalEdgeCount, ratio);
            displayText(QString(QString::number(ratio,'f', 2)), c, QColor(255,255,255), QVector3D(pos1[0],pos1[1],1));
            totalEdgeCount++;
        }

    }
}


//auto ViewerWidget::showClustersToggled(bool val) -> void
//{
//   qDebug() << "ViewerWidget::showClustersToggled(bool val)";
//   renderClusters = val;
//   clusterStartIndex = 0;
//   //Setup bounding boxes to include all of the clusters
//   showNextCluster();
//}

//auto ViewerWidget::showNextCluster() -> void
//{
//    //qDebug() << "ViewerWidget::showNextCluster()";
//   clusterStartIndex++;
//   qDebug() << "ViewerWidget::showNextCluster() " << clusterStartIndex;
//   if(clusterStartIndex > mesh->totalNoClusters())
//   {
//       clusterStartIndex = 1;
//   }
//   setupBoundingBoxProgram();
//}

//auto ViewerWidget::runOptimisationOnCluster() -> void
//{
//    qDebug() << "runOptimisationOnCluster";
//    mesh->runOptimisationOnCluster(clusterStartIndex);
//    setupBoundingBoxProgram();
//}

//auto ViewerWidget::outputData() -> void
//{
//    auto model_view = m_view * m_model;
//    mesh->outputData(model_view);
//}

auto ViewerWidget::getRatioTextColour(int boxId, int unitEdge, int selectedBoxId, int selectedEdge, int edgeCount, float& ratio) -> QColor
{

    unitEdge += boxId * bb_faces_count;
    selectedEdge += selectedBoxId * bb_faces_count;
    if(unitEdge == edgeCount) return QColor(0,150, 0);

    //qDebug() << "edgeCount " << edgeCount;
    auto it = desiredRatiosForDisplayMap.find(edgeCount);
    if(it != desiredRatiosForDisplayMap.end())
    {
        ratio = it->second;
        return QColor(200,165, 0);
    }

    if(selectedEdge == edgeCount) return QColor(150, 0, 0);

    return QColor(0, 0, 0);
}


auto ViewerWidget::renderBoxEdges() -> void
{
    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
    //Bounding Box

    QOpenGLVertexArrayObject::Binder vaoBinder2(&m_bbVao);
    m_bbProgram->bind();

    m_bbProgram->addShader(m_bbGeometryShader);
    m_bbProgram->link();

    auto model_view = m_view * m_model;

    m_bbProgram->setUniformValue("projMatrix", m_proj);
    m_bbProgram->setUniformValue("viewMatrix", m_view);
    m_bbProgram->setUniformValue("modelMatrix", m_model);
    m_bbProgram->setUniformValue("mvMatrix", model_view);

    f->glEnable(GL_MULTISAMPLE);
    f->glDisable(GL_CULL_FACE);
    f->glEnable(GL_BLEND);
    f->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    m_bbProgram->setUniformValue("showMidLines", renderMidLines);

    m_bbProgram->setUniformValue("showThirdsLine", renderThirdsLine);
    m_bbProgram->setUniformValue("showTwoThirdsLine", renderTwoThirdsLine);

    m_bbProgram->setUniformValue("showThirdsDiagLine", renderThirdsDiagLine);
    m_bbProgram->setUniformValue("showTwoThirdsDiagLine", renderTwoThirdsDiagLine);
    m_bbProgram->setUniformValue("showDiagLines", renderDiagLines);
    m_bbProgram->setUniformValue("hasGeomShader", true);

    m_bbProgram->setUniformValue("dashedLines", false);

    renderBoxHelper();

    if(renderHiddenEdges)
    {
        f->glDepthFunc(GL_GREATER);
        m_bbProgram->setUniformValue("dashedLines", true);
        renderBoxHelper();
        m_bbProgram->setUniformValue("dashedLines", false);
        f->glDepthFunc(GL_LESS);
    }

    m_bbProgram->removeShader(m_bbGeometryShader);
    m_bbProgram->release();
}

auto ViewerWidget::renderPerspectiveLines() -> void
{
    if((renderSegmentBoxes || renderBoundingBox) && showPerspectiveLines)
    {
        QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();
        //Bounding Box

        QOpenGLVertexArrayObject::Binder vaoBinder2(&m_bbVao);
        m_bbProgram->bind();

        m_bbProgram->addShader(m_perspectiveGeometryShader);
        m_bbProgram->link();

        auto model_view = m_view * m_model;

        m_bbProgram->setUniformValue("projMatrix", m_proj);
        m_bbProgram->setUniformValue("viewMatrix", m_view);
        m_bbProgram->setUniformValue("modelMatrix", m_model);
        m_bbProgram->setUniformValue("mvMatrix", model_view);

        f->glDisable(GL_CULL_FACE);
        f->glEnable(GL_BLEND);
        f->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        m_bbProgram->setUniformValue("showMidLines", renderMidLines);
        m_bbProgram->setUniformValue("showDiagLines", renderDiagLines);
        m_bbProgram->setUniformValue("hasGeomShader", true);

        m_bbProgram->setUniformValue("dashedLines", true);

        renderBoxHelper();

        if(renderHiddenEdges)
        {
            f->glDepthFunc(GL_GREATER);
            renderBoxHelper();
            f->glDepthFunc(GL_LESS);
        }

        m_bbProgram->setUniformValue("dashedLines", false);

        m_bbProgram->removeShader(m_perspectiveGeometryShader);
        m_bbProgram->release();
    }
}

auto ViewerWidget::renderPhong() -> void
{
    auto model_view = m_view * m_model;
    m_objectProgram->setUniformValue("Light.Position", camera_pos * -1);
    m_objectProgram->setUniformValue("projMatrix", m_proj);
    m_objectProgram->setUniformValue("viewMatrix", m_view);
    m_objectProgram->setUniformValue("modelMatrix", m_model);
    m_objectProgram->setUniformValue("mvMatrix", model_view);
    m_objectProgram->setUniformValue("normalMatrix", model_view.normalMatrix());
}

auto ViewerWidget::renderSuggestiveContours() -> void
{
    auto model_view = m_view * m_model;
    m_objectProgram->setUniformValue("projMatrix", m_proj);
    m_objectProgram->setUniformValue("viewMatrix", m_view);
    m_objectProgram->setUniformValue("modelMatrix", m_model);
    m_objectProgram->setUniformValue("mvMatrix", model_view);
    m_objectProgram->setUniformValue("normalMatrix", model_view.inverted().transposed());

    m_objectProgram->setUniformValue("jeroenmethod", ContourParams::getInstance()->getJeroenMethod());
    m_objectProgram->setUniformValue("fz", mesh->featureSize());
    m_objectProgram->setUniformValue("c_limit", ContourParams::getInstance()->getCLimit());
    m_objectProgram->setUniformValue("sc_limit", ContourParams::getInstance()->getScLimit());
    m_objectProgram->setUniformValue("dwkr_limit", ContourParams::getInstance()->getDwkrLimit());
    m_objectProgram->setUniformValue("cam_pos", camera_pos);
}


auto ViewerWidget::displayText(const QString &text, const QColor &backgroundColor, const QColor &textColor, const QVector3D &pos) -> void
{
    // some magic number (margin, spacing, font size, etc..)
    int fontSize = 9;
    int text_width=text.size()*(fontSize/1.5)+5, text_height=fontSize+15;

    // create the QImage and draw txt into it
    QImage textimg(text_width, text_height, QImage::Format_ARGB32);
    {
        QPainter painter(&textimg);
        painter.fillRect(0, 0, text_width, text_height, backgroundColor);
        painter.setBrush(textColor);
        painter.setPen(textColor);
        painter.setFont(QFont("Sans", fontSize));
        painter.drawText(5, 15, text);
    }

    QOpenGLFunctions *f = QOpenGLContext::currentContext()->functions();

    QOpenGLVertexArrayObject::Binder vaoBinder(&m_textVao);
    m_textProgram->bind();

    textimg = textimg.mirrored();
    auto texture = new QOpenGLTexture(textimg);

    // set bilinear filtering mode for texture magnification and minification
    texture->setMinificationFilter(QOpenGLTexture::Nearest);
    texture->setMagnificationFilter(QOpenGLTexture::Linear);

    // set the wrap mode
    texture->setWrapMode(QOpenGLTexture::ClampToBorder);

    setTextTextureCoords(textimg.width(), textimg.height());

    auto widthNormalised  = (float)(textimg.width()  / ((float)width()));
    auto heightNormalised = (float)(textimg.height() / ((float)height()));

    auto matrix = QMatrix4x4();
    matrix.setToIdentity();
    matrix.translate(QVector3D(pos[0],pos[1],0));
    matrix.scale(widthNormalised, heightNormalised);
    matrix.translate(QVector3D(1.0,1.0,0));

    m_textProgram->setUniformValue("matrix", matrix);
    m_textProgram->setUniformValue("textTexture", 0);

    texture->bind(0);
    f->glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_SHORT, 0);
    m_textProgram->release();

    //      // very stupid debug. warning: do not use in animation loop
    //      QWidget *w = new QWidget();
    //      QLabel *l = new QLabel();
    //      l->setPixmap(QPixmap::fromImage(textimg));
    //      l->setParent(w);
    //      w->show();
}

void ViewerWidget::resizeGL(int w, int h)
{
    m_proj.setToIdentity();
    m_proj.perspective(45.0f, GLfloat(w) / h, 0.01f, 1000.0f);

    m_viewport.setToIdentity();
    m_viewport.translate(QVector3D((float)w * 0.5f, (float)h * 0.5f, 0.5f));
    m_viewport.scale(QVector3D((float)w * 0.5f, (float)h * 0.5f, 0.5f));
}

void ViewerWidget::mousePressEvent(QMouseEvent *event)
{
    if (event->buttons() & Qt::LeftButton) {
        m_rotationTrackball.push(pixelPosToViewPos(event->pos()), m_rotationTrackball.rotation().conjugate());
    }
    update();
}

void ViewerWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if (event->buttons() & Qt::LeftButton) {
        m_rotationTrackball.release(pixelPosToViewPos(event->pos()), m_rotationTrackball.rotation().conjugate(), m_model);
    }
    update();
}

void ViewerWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (event->buttons() & Qt::LeftButton)
    {
        m_rotationTrackball.move(pixelPosToViewPos(event->pos()), m_rotationTrackball.rotation().conjugate(), m_model);
    }
    else
    {
        m_rotationTrackball.release(pixelPosToViewPos(event->pos()), m_rotationTrackball.rotation().conjugate(), m_model);
    }
    desiredEdgeRatioMap.clear();
    desiredRatiosForDisplayMap.clear();
    update();
}

void ViewerWidget::wheelEvent(QWheelEvent * event)
{
    if (!event->isAccepted()) {
        m_distExp += event->delta();
        if (m_distExp < -8 * 120)
            m_distExp = -8 * 120;
        if (m_distExp > 10 * 120)
            m_distExp = 10 * 120;
        event->accept();
    }
}

QPointF ViewerWidget::pixelPosToViewPos(const QPointF& p)
{
    return QPointF(2.0 * float(p.x()) / width() - 1.0,
                   1.0 - 2.0 * float(p.y()) / height());
}

void ViewerWidget::updateViewer()
{
    update();
}

void ViewerWidget::initViewer()
{
    qDebug() << "ViewerWidget::initViewer()";
    mesh->updateBoundingBoxes();
    initializeGL();
    update();
}

auto ViewerWidget::ellipsesToggled() -> void
{
    renderEllipsesCheck = !renderEllipsesCheck;
}

auto ViewerWidget::planesToggled() -> void
{
    renderPlanesCheck = !renderPlanesCheck;
}


auto ViewerWidget::guidesToggled() -> void
{
    renderGuidesCheck = !renderGuidesCheck;
}

auto ViewerWidget::candidatesToggled() -> void
{
    renderCandidatesCheck = !renderCandidatesCheck;
}

auto ViewerWidget::boundingBoxToggled(bool checked) -> void
{
    renderBoundingBox = checked;
}

auto ViewerWidget::segmentBoxesToggled(bool checked) -> void
{
    renderSegmentBoxes = checked;
}

auto ViewerWidget::renderBoxFacesToggled(bool checked) -> void
{
    renderBoxFacesCheck = checked;
}

auto ViewerWidget::edgeRatiosToggled(bool checked) -> void
{
    showEdgeRatios = checked;
}

auto ViewerWidget::setSelectedEdge(int val) -> void
{
   selectedEdge = val;
   updateSelectedEdgeRatio();
}

auto ViewerWidget::updateSelectedEdgeRatio() -> void
{
    auto mvp = m_proj * m_view * m_model;
    auto unitLength = mesh->screenSpaceEdgeLength(m_viewport, mvp, unitLengthBox, unitLengthEdge);
    auto selectedEdgeLength = mesh->screenSpaceEdgeLength(m_viewport, mvp, selectedEdgeBox, selectedEdge);
    auto ratio = selectedEdgeLength / unitLength;
    emit setUnitLengthEdge(ratio);
}

auto ViewerWidget::manualAdjustSelectedLengthEdge(double val) -> void
{
    //would need to be updated now we have a selectedEdgeBox Param
//    mesh->manualAdjustBoundingBox(m_viewport, m_proj, m_view, m_model, unitLengthBox, unitLengthEdge, selectedEdge, val);
//    initializeGL();
//    update();
}

auto ViewerWidget::setDesieredEdgeRatio(double val) -> void
{
    //mesh->manualAdjustBoundingBox(m_viewport, m_proj, m_view, m_model, unitLengthBox, unitLengthEdge, selectedEdge, val);
   auto selectedEdgeMapped = mesh->edgeToEdgeId(selectedEdgeBox, selectedEdge);
   selectedEdgeMapped += selectedEdgeBox * bb_faces_count;
   desiredRatiosForDisplayMap.insert(std::pair<int, float>(selectedEdgeMapped, val));

   auto edgeId = selectedEdgeBox * bb_faces_count + selectedEdge;
   auto pair = std::pair<int, float>(edgeId, val);

   auto it = desiredEdgeRatioMap.find(selectedEdgeBox);
   if(it != desiredEdgeRatioMap.end())
   {
       it->second.push_back(pair);
   }
   else
   {
       std::vector<std::pair<int, float>> newVector;
       newVector.push_back(pair);
       desiredEdgeRatioMap.insert(std::pair<int, std::vector<std::pair<int, float>>>(selectedEdgeBox, newVector));
   }


   update();
}

auto ViewerWidget::computeBoxes(BoxAdjustMethod method) -> void
{
    mesh->computeBoxes(method, m_viewport, m_proj, m_view, m_model, unitLengthBox, unitLengthEdge, desiredEdgeRatioMap);
    desiredRatiosForDisplayMap.clear();
    desiredEdgeRatioMap.clear();
    initializeGL();
    update();
}

auto ViewerWidget::setUnitLengthBox(int val) -> void
{
    unitLengthBox = val;
    updateSelectedEdgeRatio();
}

auto ViewerWidget::setUnitLengthEdge(int val) -> void
{
    unitLengthEdge = val;
    updateSelectedEdgeRatio();
}

auto ViewerWidget::setSelectedEdgeBox(int val) -> void
{
    selectedEdgeBox = val;
    updateSelectedEdgeRatio();
}

auto ViewerWidget::setShowEdgeRatio(int val) -> void
{
    showEdgeRatioSetting = val;
    updateSelectedEdgeRatio();
}

auto ViewerWidget::renderMidLinesToggled(bool checked) -> void
{
    renderMidLines = checked;
}
auto ViewerWidget::renderDiagLinesToggled(bool checked) -> void
{
    renderDiagLines = checked;
}

auto ViewerWidget::renderThirdsDiagLineToggled(bool checked) -> void
{
    renderThirdsDiagLine = checked;
}

auto ViewerWidget::renderHiddenLinesToggled(bool checked) -> void
{
    renderHiddenEdges = checked;
}

auto ViewerWidget::renderPerspectiveLinesToggled(bool checked) -> void
{
    showPerspectiveLines = checked;
}

auto ViewerWidget::renderTwoThirdsDiagLineToggled(bool checked) -> void
{
    renderTwoThirdsDiagLine = checked;
}

auto ViewerWidget::renderThirdsLineToggled(bool checked) -> void
{
    qDebug() << "ViewerWidget::renderThirdsLineToggled( " << checked;
    renderThirdsLine = checked;
}

auto ViewerWidget::renderTwoThirdsLineToggled(bool checked) -> void
{
    renderTwoThirdsLine = checked;
}

auto ViewerWidget::bbXRotChanged(double val) -> void
{
    mesh->rotateBoundingBoxXAxis(val);
    initializeGL();
    update();
}

auto ViewerWidget::bbYRotChanged(double val) -> void
{
    mesh->rotateBoundingBoxYAxis(val);
    initializeGL();
    update();
}

auto ViewerWidget::bbZRotChanged(double val) -> void
{
    mesh->rotateBoundingBoxZAxis(val);
    initializeGL();
    update();
}

auto ViewerWidget::showObjectToggled(bool val) -> void
{
   showObject = val;
}

auto ViewerWidget::printScreen() -> void
{
    qDebug() << "ViewerWidget::printScreen()";

    saveImageFromFBO = true;
    update();

}



#endif // VIEWERWIDGET_CPP

