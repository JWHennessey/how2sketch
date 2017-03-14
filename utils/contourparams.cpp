#include "contourparams.h"

ContourParams* ContourParams::instance = nullptr;

ContourParams::ContourParams()
        : jeroenmethod(true)
        , cLimit(1.0f)
        , scLimit(1.0f)
        , dwkrLimit(1.0f)
{

}


auto ContourParams::getCLimit() -> float
{
        return cLimit;
}

auto ContourParams::setCLimit(float val) -> void
{
    cLimit = val;
}

auto ContourParams::getJeroenMethod() -> bool
{
    return jeroenmethod;
}

auto ContourParams::setJeroenMethod(bool val) -> void
{
 jeroenmethod = val;
}

auto ContourParams::getScLimit() -> float
{
    return scLimit;
}

auto ContourParams::setScLimit(float val) -> void
{
    scLimit = val;
}

auto ContourParams::getDwkrLimit() -> float
{
    return dwkrLimit;
}

auto ContourParams::setDwkrLimit(float val) -> void
{
    dwkrLimit = val;
}
