#ifndef SIMULATIONDRIVER_H
#define SIMULATIONDRIVER_H

#include <iostream>
#include <fstream>
#include "charsystem.h"
#include "crowdpatch.h"
#include "patchgraph.h"

#define PERIOD 255.0
#define EPSILON 0.1

class SimulationDriver {
/*
 * The simulation driver stores all of the simulation data and runs
 * and saves out the simulation.
 */

private:
    // Test crowd patch, will eventually be graph of patches
    CrowdPatch testPatch;
    PatchGraph graph;

    // Helper function for collision avoidance
    bool minDist(Trajectory T1, Trajectory T2, glm::vec3 &cp1, glm::vec3 &cp2,
                  int &segFirst1, int &segFirst2, float &dist);

    // Animate characters along interpolated patch trajectories
    void followTrajectories(std::vector<Trajectory> trajectories,
                            int subFrame, int currframe,
                            std::ofstream &fs, int &count);
    bool smoothTrajectory(Trajectory T, int numControlPoints,
                          int subFrame, int currFrame,
                          std::ofstream &fs, int count);

public:
    // Constructor
    SimulationDriver();

    // Run Simulation
    void run(const int frameCount);

    // Display image
    QImage display;
};



#endif // SIMULATIONDRIVER_H
