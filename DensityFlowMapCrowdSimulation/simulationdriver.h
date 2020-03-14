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
    void followTrajectories(std::vector<Trajectory> trajectories, int subFrame, int currframe);
    void smoothTrajectory(Trajectory T, int numControlPoints, int subFrame, int currFrame);
    bool minDist(Trajectory T1, Trajectory T2, glm::vec3 &cp1, glm::vec3 &cp2,
                  int &segFirst1, int &segFirst2, float &dist);
public:
    SimulationDriver();
    void run(const int frameCount);
    float getDt();
    QImage display;
};

#endif // SIMULATIONDRIVER_H
