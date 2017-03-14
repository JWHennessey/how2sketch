#ifndef TRACKBALL_H
#define TRACKBALL_H

#include <QtWidgets>

#include <QtGui/qvector3d.h>
#include <QtGui/qquaternion.h>

class TrackBall
{
public:
    enum TrackMode
    {
        Plane,
        Sphere,
    };
    TrackBall(TrackMode mode = Sphere);
    TrackBall(float angularVelocity, const QVector3D& axis, TrackMode mode = Sphere);
    // coordinates in [-1,1]x[-1,1]
    void push(const QPointF& p, const QQuaternion &transformation);
    void move(const QPointF& p, const QQuaternion &transformation, QMatrix4x4 model);
    void release(const QPointF& p, const QQuaternion &transformation, QMatrix4x4 model);
    void setRotation(QMatrix4x4 rotation);
    void start(); // starts clock
    void stop(); // stops clock
    QQuaternion rotation();
private:
    QQuaternion m_rotation;
    QVector3D m_axis;
    float m_angularVelocity;
    QPointF m_lastPos;
    QTime m_lastTime;
    bool m_paused;
    bool m_pressed;
    TrackMode m_mode;


};

#endif //trackball_h
