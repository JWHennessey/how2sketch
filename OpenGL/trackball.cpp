#include "trackball.h"

#include <cmath>

//============================================================================//
//                                  TrackBall                                 //
//============================================================================//

#define PI 3.14159265358979

TrackBall::TrackBall(TrackMode mode)
    : m_angularVelocity(0)
    , m_paused(false)
    , m_pressed(false)
    , m_mode(mode)
{
    m_axis = QVector3D(0,0,1);
    m_rotation = QQuaternion();
    m_lastTime = QTime::currentTime();
}


QQuaternion matrixToQuaternion(QMatrix4x4 m) {
    qDebug() << m;
    auto w = std::sqrt(1.0 + m(0,0) + m(1,1) + m(2,2)) / 2.0;
    double w4 = (4.0 * w);
    auto x = (m(2,1) - m(1,2)) / w4 ;
    auto y = (m(0,2) - m(2,0)) / w4 ;
    auto z = (m(1,0) - m(0,1)) / w4 ;
    return QQuaternion(w,x,y,z);
}


TrackBall::TrackBall(float angularVelocity, const QVector3D& axis, TrackMode mode)
    : m_axis(axis)
    , m_angularVelocity(angularVelocity)
    , m_paused(false)
    , m_pressed(false)
    , m_mode(mode)
{
    m_rotation = QQuaternion();
    m_lastTime = QTime::currentTime();
}

void TrackBall::push(const QPointF& p, const QQuaternion &)
{
    m_rotation = rotation();
    m_pressed = true;
    m_lastTime = QTime::currentTime();
    m_lastPos = p;
    m_angularVelocity = 0.0f;
}

void TrackBall::setRotation(QMatrix4x4 rotation)
{
    //m_rotation = matrixToQuaternion(rotation);
    qDebug() << m_rotation;
}

void TrackBall::move(const QPointF& p, const QQuaternion &transformation, QMatrix4x4 model)
{
    if (!m_pressed)
        return;

    QTime currentTime = QTime::currentTime();
    int msecs = m_lastTime.msecsTo(currentTime);
    if (msecs <= 20)
        return;

    switch (m_mode) {
    case Plane:
        {
            qDebug() << transformation;
            QLineF delta(m_lastPos, p);
            m_angularVelocity = 180*delta.length() / (PI*msecs);
            m_axis = QVector3D(delta.dy(), delta.dx(), 0.0f).normalized();
            m_axis = transformation.rotatedVector(m_axis);
            m_rotation = QQuaternion::fromAxisAndAngle(m_axis, 180 / PI * delta.length()) * m_rotation;
        }
        break;
    case Sphere:
        {
            QVector3D lastPos3D = QVector3D(m_lastPos.x(), m_lastPos.y(), 0.0f);
            float sqrZ = 1 - QVector3D::dotProduct(lastPos3D, lastPos3D);
            if (sqrZ > 0)
                lastPos3D.setZ(std::sqrt(sqrZ));
            else
                lastPos3D.normalize();

            QVector3D currentPos3D = QVector3D(p.x(), p.y(), 0.0f);
            sqrZ = 1 - QVector3D::dotProduct(currentPos3D, currentPos3D);
            if (sqrZ > 0)
                currentPos3D.setZ(std::sqrt(sqrZ));
            else
                currentPos3D.normalize();

            m_axis = QVector3D::crossProduct(lastPos3D, currentPos3D);
            float angle = 180 / PI * std::asin(std::sqrt(QVector3D::dotProduct(m_axis, m_axis)));

            m_angularVelocity = angle / msecs;
            m_axis.normalize();
            m_axis = transformation.rotatedVector(m_axis);
            m_rotation = QQuaternion::fromAxisAndAngle(m_axis, angle) * m_rotation;
        }
        break;
    }

    m_lastPos = p;
    m_lastTime = currentTime;
}

void TrackBall::release(const QPointF& p, const QQuaternion &transformation, QMatrix4x4 model)
{
    // Calling move() caused the rotation to stop if the framerate was too low.
    move(p, transformation, model);
    m_pressed = false;
}

void TrackBall::start()
{
    m_lastTime = QTime::currentTime();
    m_paused = false;
}

void TrackBall::stop()
{
    m_rotation = rotation();
    m_paused = true;
}

QQuaternion TrackBall::rotation()
{
    //if (m_paused || m_pressed)
     auto ret = m_rotation;
     m_rotation.setVector(QVector3D(0.0,0.0,0.0));
     m_rotation.setScalar(1.0);
     return ret;

//    QTime currentTime = QTime::currentTime();
//    float angle = m_angularVelocity * m_lastTime.msecsTo(currentTime);
//    return QQuaternion::fromAxisAndAngle(m_axis, angle) * m_rotation;
}
