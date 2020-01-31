#include "simulationdriver.h"
#include <iostream>

SimulationDriver::SimulationDriver() : dt(0.01)
{ }

void SimulationDriver::run(const int frameCount) {
    for (int frame = 1; frame <= frameCount; ++frame) {
        std::cout << "Frame " << frame << std::endl;
        // NOTE: Use substeps if animation period needs to be less than one frame

//        int numSubsteps = (int) ((float)(1.0 / 24.0) / dt);
//        for (int step = 1; step <= numSubsteps; ++step) {
//            std::cout << "Step " << step << std::endl;
//        }

        // TODO: This will be where points get displayed on screen and eventually point data is saved out
    }
}

float SimulationDriver::getDt() {
    return this->dt;
}
