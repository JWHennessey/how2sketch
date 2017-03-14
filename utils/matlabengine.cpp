#include "matlabengine.h"


MatlabEngine* MatlabEngine::instance = nullptr;

MatlabEngine::MatlabEngine()
{
   bool loaded = engine = engOpen("");
   assert(loaded);
}

auto MatlabEngine::getEngine() -> Engine*
{
    return engine;
}
