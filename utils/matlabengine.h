#ifndef MATLABENGINE_H
#define MATLABENGINE_H

#include "matrix.h"
#include "mex.h"
#include "engine.h"

class MatlabEngine
{
public:
    static MatlabEngine* getInstance()
    {
        if(!instance)
            instance = new MatlabEngine();
        return instance;
    }
    auto getEngine() -> Engine*;
private:
    MatlabEngine();
    static MatlabEngine* instance;
    Engine* engine;
    MatlabEngine(MatlabEngine const&) = delete;
    void operator=(MatlabEngine const&) = delete;

};

#endif // MATLABENGINE_H
