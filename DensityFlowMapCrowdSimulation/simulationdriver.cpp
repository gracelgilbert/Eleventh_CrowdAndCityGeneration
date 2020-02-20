#include "simulationdriver.h"
#include <iostream>
#include <thread>         // std::this_thread::sleep_for
#include <chrono>         // std::chrono::seconds


#define PERIOD 255.0
#define EPSILON 0.001

SimulationDriver::SimulationDriver() : dt(0.01), chars(CharSystem())
{
    chars.fillChars(1);
    testPatch = CrowdPatch(glm::vec2(0.0, 0.0), 50.0, PERIOD, 1.0, glm::vec2(1.0, 1.0));
//    testPatch.leftIns.insert(glm::vec2(10.0, 0.0));
//    testPatch.rightIns.insert(glm::vec2(3.0, 5.0));
}

void SimulationDriver::run(const int frameCount) {
    display = QImage(400, 400, QImage::Format_RGB32);
    display.fill(Qt::red);

    int framesPerPeriod = (int) PERIOD;

    int numPeriods = std::floor((float) frameCount / (float) framesPerPeriod);


    // Initialize patches and graph

    for (int i = 0; i < numPeriods; ++i) {
        float error = 1.0;
        int attemptCount = 0;
        // Find optimal paths with current I/O
        // Calculate error
        while (error > EPSILON && attemptCount < 10) {
            // Find SCCs and error paths
            // Update I/O on error paths
            // Find optimal paths with current I/O
            // Calculate error

            // Increment attempt count
            attemptCount++;
        }
        // At this point, each crowd patch will contain initial trajectories from input to output points
        // Perform collision avoidance on these trajectories within each patch
        Trajectory testTrajectory = Trajectory();
        testTrajectory.insertControlPoint(glm::vec3(100.0, 80.0, 250.0), 0);
        testTrajectory.insertControlPoint(glm::vec3(100.0, 200.0, 255.0), 1);




        testPatch.trajectories.push_back(testTrajectory);



        for (int j = 0; j < framesPerPeriod; ++j) {
            int currFrame = framesPerPeriod * i + j;
            std::cout << "Frame " << j << std::endl;
//            float interpRatio = (float) j / (float) framesPerPeriod;

            for (Trajectory T : testPatch.trajectories) {
                // This will be spline interpolation, bezier curves?
                glm::vec3 cp1 = T.getControlPointAtIndex(0);
                glm::vec3 cp2 = T.getControlPointAtIndex(1);
                if (j < cp1[2] || j > cp2[2]) {
                    continue;
                }

                float interpRatio = (float) (j - cp1[2]) / (float) (cp2[2] - cp1[2]);


                glm::vec3 p = cp2 * interpRatio + cp1 * (1.f - interpRatio);
                std::cout << "p value " << p[2] << std::endl;
                // Check to make sure lies within trajectory
                display.setPixelColor(QPoint(p[0], p[1]), QColor(p[2], p[2], p[2]));
            }
        }

    }

}

float SimulationDriver::getDt() {
    return this->dt;
}

//void SimulationDriver::advanceTimeStep() {
//    for (int i = 0; i < this->chars.getNumChars(); ++i) {

//        float a = 1.5 * ((2.0 * static_cast <float> (std::rand()) / static_cast <float> (RAND_MAX)) - 1.0);
//        float b = 1.5 * ((2.0 * static_cast <float> (std::rand()) / static_cast <float> (RAND_MAX)) - 1.0);

//        this->chars.updateIndexPos(i, this->chars.getIndexPos(i) + glm::vec2(a, b));
//    }
//}

