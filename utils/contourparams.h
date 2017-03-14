#ifndef CONTOURPARAMS_H
#define CONTOURPARAMS_H


class ContourParams
{
public:
    static ContourParams* getInstance()
    {
        if(!instance)
            instance = new ContourParams();
        return instance;
    }
    auto getCLimit() -> float;
    auto setCLimit(float val) -> void;
    auto getJeroenMethod() -> bool;
    auto setJeroenMethod(bool val) -> void;
    auto getScLimit() -> float;
    auto setScLimit(float val) -> void;
    auto getDwkrLimit() -> float;
    auto setDwkrLimit(float val) -> void;
private:
    ContourParams();
    static ContourParams* instance;
    bool jeroenmethod;
    float cLimit;
    float scLimit;
    float dwkrLimit;

    //Needed for singleton
    ContourParams(ContourParams const&) = delete;
    void operator=(ContourParams const&) = delete;
};

#endif // CONTOURPARAMS_H
