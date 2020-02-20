#ifndef SIMULATIONDRIVER_H
#define SIMULATIONDRIVER_H

#include <QImage>
#include "charsystem.h"
#include "crowdpatch.h"

class SimulationDriver {

private:
    float dt;
    CharSystem chars;
    void advanceTimeStep();
    CrowdPatch testPatch;

public:
    SimulationDriver();
    void run(const int frameCount);
    float getDt();
    QImage display;
};

#endif // SIMULATIONDRIVER_H
