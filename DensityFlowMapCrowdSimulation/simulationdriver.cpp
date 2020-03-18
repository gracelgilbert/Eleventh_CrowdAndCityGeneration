#include "simulationdriver.h"

// Constructor
SimulationDriver::SimulationDriver()
{
    testPatch = CrowdPatch(glm::vec2(0.0, 0.0), 200.0, PERIOD, 1.0, glm::vec2(1.0, 1.0));
}

// Helper functions
glm::vec3 mix(glm::vec3 a, glm::vec3 b, float t) {
    return (1.f - t) * a + t * b;
}

glm::vec2 vec3ToVec2(glm::vec3 v3) {
    return glm::vec2(v3[0], v3[1]);
}

void insertTestTrajectories(CrowdPatch &testPatch) {
    Trajectory testTrajectory = Trajectory();
    testTrajectory.insertControlPoint(glm::vec3(50.0, 70.0, 0.0), 0);
    testTrajectory.insertControlPoint(glm::vec3(160.0, 130.0, 130.0), 1);

    testTrajectory.insertControlPoint(glm::vec3(250.0, 150.0, 255.0), 2);

    Trajectory testT2 = Trajectory();
    testT2.insertControlPoint(glm::vec3(50.0, 200.0, 0.0), 0);
    testT2.insertControlPoint(glm::vec3(150.0, 110.0, 150.0), 1);
    testT2.insertControlPoint(glm::vec3(250.0, 120.0, 240.0), 2);

    testPatch.addTrajectory(testTrajectory);
    testPatch.addTrajectory(testT2);
}

// Animate characters along interpolated patch trajectories
void SimulationDriver::followTrajectories(std::vector<Trajectory> trajectories, int subFrame, int currFrame) {
    // iterate over all trajectories
    for (Trajectory T : trajectories) {
        int numControlPoints = T.getNumControlPoints();
        if (numControlPoints < 2) {
            // if fewer than 2 control points, skip
            continue;
        }

        glm::vec3 cpFirst = T.getControlPointAtIndex(0);
        glm::vec3 cpLast = T.getControlPointAtIndex(numControlPoints - 1);

        if (subFrame < cpFirst[2] || subFrame > cpLast[2]) {
            // If frame is outside trajectory time bounds, skip
            continue;
        }

        // Bezier interpolation
        this->smoothTrajectory(T, numControlPoints, subFrame, currFrame);
    }
}

void SimulationDriver::smoothTrajectory(Trajectory T, int numControlPoints, int subFrame, int currFrame) {
    for (int seg = 0; seg < numControlPoints - 1; ++seg) {
        // Check if current frame is within this segment otherwiese continue
        glm::vec3 cp1 = T.getControlPointAtIndex(seg);
        glm::vec3 cp2 = T.getControlPointAtIndex(seg + 1);
        if (subFrame < cp1[2] || subFrame > cp2[2]) {
            continue;
        }
        glm::vec3 cp0 = glm::vec3();
        glm::vec3 cp3 = glm::vec3();

        if (seg == 0) {
            // If first segment
            cp0 = cp1 + (cp1 - cp2);
        } else {
            cp0 = T.getControlPointAtIndex(seg - 1);
        }

        if (seg == numControlPoints - 2) {
            // If last segment
            cp3 = cp2 + (cp2 - cp1);
        } else {
            cp3 = T.getControlPointAtIndex(seg + 2);
        }

        // Bezier Control Points
        glm::vec3 b0 = cp1;
        glm::vec3 b1 = cp1 + ((1.f / 3.f) * ((cp2 - cp0) / 2.f));
        glm::vec3 b2 = cp2 - ((1.f / 3.f) * ((cp3 - cp1) / 2.f));
        glm::vec3 b3 = cp2;

        float interpRatio = (float) (subFrame - cp1[2]) / (float) (cp2[2] - cp1[2]);

        // Bezier interpolation
        glm::vec3 b01 = mix(b0, b1, interpRatio);
        glm::vec3 b11 = mix(b1, b2, interpRatio);
        glm::vec3 b21 = mix(b2, b3, interpRatio);

        glm::vec3 b02 = mix(b01, b11, interpRatio);
        glm::vec3 b12 = mix(b11, b21, interpRatio);

        glm::vec3 p = mix(b02, b12, interpRatio);

        display.setPixelColor(QPoint(p[0], p[1]), QColor(currFrame, currFrame, currFrame));
    }
}



// Run simulation
void SimulationDriver::run(const int frameCount) {
    display = QImage(400, 400, QImage::Format_RGB32);
    display.fill(Qt::red);

    int framesPerPeriod = (int) PERIOD;

    int numPeriods = std::floor((float) frameCount / (float) framesPerPeriod);


    // Initialize patches and graph

    // Optimize graph I/O points

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

    BoundaryPoint bp1 = BoundaryPoint(glm::vec3(testPatch.getOrigin()[0], 3.0 * testPatch.getWidth() / 4.0, 3.0), ENTRY, &testPatch);
    BoundaryPoint bp2 = BoundaryPoint(glm::vec3(testPatch.getOrigin()[0], testPatch.getWidth() / 4.0, 8.0), ENTRY, &testPatch);

    BoundaryPoint bp3 = BoundaryPoint(glm::vec3(testPatch.getOrigin()[0] + testPatch.getWidth() /  2.0, testPatch.getOrigin()[1] + testPatch.getWidth(), 200.0), EXIT, &testPatch);
    BoundaryPoint bp4 = BoundaryPoint(glm::vec3(testPatch.getOrigin()[0] + testPatch.getWidth(), testPatch.getWidth() / 4.0, 200.0), EXIT, &testPatch);

    testPatch.addEntry(&bp1);
    testPatch.addEntry(&bp2);

    testPatch.addExit(&bp3);
    testPatch.addExit(&bp4);

    testPatch.matchBoundaryPoints();



    // Temporary test trajectories:
//    insertTestTrajectories(testPatch);

    /*
     * At this point, each crowd patch will contain trajectories from input to output points
     * Perform collision avoidance on these trajectories within each patch
     */

    // Remove collisions
    testPatch.removeCollisions();

    // Add control points to straighten out spline
    for (int i = 0; i < (int) testPatch.getTrajectories().size(); ++i) {
        testPatch.getTrajectoryAt(i).straighten(2);
    }

    // Animate characters along the trajectories
    for (int p = 0; p < numPeriods; ++p) {
        // Interpolate and render points
        for (int j = 0; j < framesPerPeriod; ++j) {
            int currFrame = framesPerPeriod * p + j;
            this->followTrajectories(testPatch.getTrajectories(), j, currFrame);
        }
    }
}

