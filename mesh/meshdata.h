#ifndef MESHVIEW_H
#define MESHVIEW_H
#include <QMatrix4x4>

class MonitorGLWidget;
class MainWindow;

class MeshData
{
public:
    MeshData();
    auto getModel() -> QMatrix4x4;
    auto setXRotation(int angle) -> void;
    auto setYRotation(int angle) -> void;
    auto setZRotation(int angle) -> void;
    auto setRotation(int x, int y, int z) -> void;
    auto setXPos(float pos) -> void;
    auto setYPos(float pos) -> void;
    auto setZPos(float pos) -> void;
    auto setPosition(float x, float y, float z) -> void;
    auto setXYPos(float posX, float posY) -> void;
    auto setScale(float s) -> void;
    auto setScaleDirect(float s) -> void;
    auto needTextureUpdate() -> bool;
    auto setTextureUpdate(bool value) -> void;

private:
    int xRot;
    int yRot;
    int zRot;
    float xPos;
    float yPos;
    float zPos;
    float scale;
    QPoint lastPos;
    QMatrix4x4 model;
    bool updateTexture;
    //Functions
    auto updateModel() -> void;

friend class Mesh;
friend class MonitorGLWidget;
friend class MainWindow;

};

#endif // MESHVIEW_H
