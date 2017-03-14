#include "meshdata.h"

MeshData::MeshData() :
      xRot(0)
    , yRot(0)
    , zRot(0)
    , xPos(0.0f)
    , yPos(0.0f)
    , zPos(0.0f)
    , scale(1.0f)
    , updateTexture(false)
{
    updateModel();
}

auto MeshData::setXRotation(int angle) -> void
{
    xRot = angle;
    updateModel();
}
auto MeshData::setYRotation(int angle) -> void
{
   yRot = angle;
   updateModel();
}
auto MeshData::setZRotation(int angle) -> void
{
   zRot = angle;
   updateModel();
}

auto MeshData::setRotation(int x, int y, int z) -> void
{
    xRot = x;
    yRot = y;
    zRot = z;
    updateModel();
}

auto MeshData::getModel() -> QMatrix4x4
{
    return model;
}

auto MeshData::setXPos(float pos) -> void
{
    //qDebug() << "MeshData::setXPos(float pos)";
    xPos = pos;
    updateModel();
}

auto MeshData::setYPos(float pos) -> void
{
    //qDebug() << "MeshData::setYPos(float pos)";
    yPos = pos;
    updateModel();
}

auto MeshData::setZPos(float pos) -> void
{
    //qDebug() << "MeshData::setZPos(float pos)";
    zPos = pos;
    updateModel();
}

auto MeshData::setPosition(float x, float y, float z) -> void
{
    //qDebug() << "MeshData::setPosition(float pos)";
    xPos = x;
    yPos = y;
    zPos = z;
    updateModel();
}

auto MeshData::setXYPos(float posX, float posY) -> void
{
    //qDebug() << "MeshData::setXYPos(float pos)";
    xPos = posX;
    yPos = posY;
    updateModel();
}

auto MeshData::setScale(float s) -> void
{
    //qDebug() << "MeshData::setScale(float pos)";
    if(scale <= 1.0)
    {
       scale += (s * (1.0 / 64.0));
    }
    else
    {
        scale += s;
    }
    if(scale < 0.001)
    {
        scale = 0.001;
    }
    //qDebug() << scale;
    updateModel();
}

auto MeshData::setScaleDirect(float s) -> void
{
    //qDebug() << "MeshData::setScaleDirect(float pos)";
    scale = s;
    updateModel();
}


auto MeshData::updateModel() -> void
{
    //qDebug() << "MeshData::updateModel()";
    static const float tsf = 1000.0f;
    model.setToIdentity();
    model.translate(xPos / tsf, yPos / tsf, zPos / tsf);
    model.scale(scale);
    model.rotate(180.0f - (xRot / 16.0f), 1, 0, 0);
    model.rotate(yRot / 16.0f, 0, 1, 0);
    model.rotate(zRot / 16.0f, 0, 0, 1);
    updateTexture = true;

}

auto MeshData::needTextureUpdate() -> bool
{
    //qDebug() << "MeshData::needTextureUpdate() " << updateTexture;
    return updateTexture;
}

auto MeshData::setTextureUpdate(bool value) -> void
{
    //qDebug() << "setTextureUpdate(bool value) " << value;
    updateTexture = value;

}
