#include "simulationdriver.h"

// Constructor
SimulationDriver::SimulationDriver()
{
    testPatch = CrowdPatch(glm::vec2(0.0, 0.0), 50.0, PERIOD, 1.0, glm::vec2(1.0, 1.0));
    this->graph = PatchGraph();
    graph.tempFill();

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
void SimulationDriver::followTrajectories(std::vector<Trajectory> trajectories,
                                          int subFrame, int currFrame, std::ofstream &fs,
                                          int &count) {
    // iterate over all trajectories
    for (Trajectory T : trajectories) {
        int numControlPoints = T.getNumControlPoints();
        if (numControlPoints < 2) {
            // if fewer than 2 control points, skip
            continue;
        }

        glm::vec3 cpFirst = T.getControlPointAtIndex(0);
        glm::vec3 cpLast = T.getControlPointAtIndex(numControlPoints - 1);

        if (subFrame < cpFirst[2] || subFrame >= cpLast[2]) {
            // If frame is outside trajectory time bounds, skip
            continue;
        }

        // Bezier interpolation
        this->smoothTrajectory(T, numControlPoints, subFrame, currFrame, fs, count);
        count++;
    }
}

void SimulationDriver::smoothTrajectory(Trajectory T,
                                        int numControlPoints, int subFrame,
                                        int currFrame,
                                        std::ofstream &fs, int count) {
    for (int seg = 0; seg < numControlPoints - 1; ++seg) {
        // Check if current frame is within this segment otherwiese continue
        glm::vec3 cp1 = T.getControlPointAtIndex(seg);
        glm::vec3 cp2 = T.getControlPointAtIndex(seg + 1);
        if (subFrame < cp1[2] || subFrame >= cp2[2]) {
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

        fs << count << ":";
        fs << " " << p[0] << " " << 0 << " " << p[1] << "\n";
        display.setPixelColor(QPoint(p[0], p[1]), QColor(currFrame, currFrame, currFrame));
    }
}



// Run simulation
void SimulationDriver::run(const int frameCount) {
    display = QImage(400, 400, QImage::Format_RGB32);
    display.fill(Qt::red);
    std::srand(std::time(nullptr));


    int framesPerPeriod = (int) PERIOD;

    int numPeriods = std::floor((float) frameCount / (float) framesPerPeriod);


    // Initialize patches and graph

    // Optimize graph I/O points

    float error = 1.0;
    int attemptCount = 0;
    // Find optimal paths with current I/O:

    // 1. Calculate error of graph
    error = this->graph.calculateError();

    while (error > EPSILON && attemptCount < 2) {

        // 2. Find minima and maxima of error
        std::vector<CrowdPatch*> minima = this->graph.findMinima();
        std::vector<CrowdPatch*> maxima = this->graph.findMaxima();

        // 3a. Split minima into groups
        int groupSize = 3;
        for (int i = 0; i < floor(minima.size() / float(groupSize)); i += groupSize) {
            std::vector<CrowdPatch*> currGroup =  std::vector<CrowdPatch*>();
            currGroup.push_back(minima.at(i));
            currGroup.push_back(minima.at(i + 1));
            currGroup.push_back(minima.at(i + 2));

            // 4a. Find shortest paths between group of minima
            std::vector<CrowdPatch*> currPath = this->graph.getOptimaMinCostPath(currGroup);

            // 5a. Add boundary points along shortest path
            graph.addBPsAlongPath(currPath);
        }

        // 3b. Split minima into groups
        for (int i = 0; i < floor(maxima.size() / float(groupSize)); i += groupSize) {
            std::vector<CrowdPatch*> currGroup =  std::vector<CrowdPatch*>();
            currGroup.push_back(maxima.at(i));
            currGroup.push_back(maxima.at(i + 1));
            currGroup.push_back(maxima.at(i + 2));

            // 4b. Find shortest paths between group of minima
            std::vector<CrowdPatch*> currPath = this->graph.getOptimaMinCostPath(currGroup);

            // 5b. Remove boundary points along shortest path
        }
        for (int i = 0; i < XDIM; ++i) {
            for (int j = 0; j < YDIM; ++j) {
                CrowdPatch* currPatch = graph.getPatchAt(i, j);
                currPatch->matchBoundaryPoints();
            }
        }

        // 6. Calculate error of graph
        error = this->graph.calculateError();
        this->graph.resetTrajectories();

        // Increment attempt count
        attemptCount++;
    }


    for (int i = 0; i < XDIM; ++i) {
        for (int j = 0; j < YDIM; ++j) {
            CrowdPatch* currPatch = graph.getPatchAt(i, j);


//            float ox = currPatch->getOrigin()[0];
//            float oy = currPatch->getOrigin()[1];
//            float w = currPatch->getWidth();

//            BoundaryPoint bp1 = BoundaryPoint(glm::vec3(ox, oy + 3.0 * w / 4.0, 0.0), ENTRY, currPatch);
//            BoundaryPoint bp2 = BoundaryPoint(glm::vec3(ox, oy + w / 4.0, 50.0), ENTRY, currPatch);

//            BoundaryPoint bp3 = BoundaryPoint(glm::vec3(ox + w, oy + w / 2.0, 200.0), EXIT, currPatch);
//            BoundaryPoint bp4 = BoundaryPoint(glm::vec3(ox + w/2.0, oy + w, 250.0), EXIT, currPatch);

//            currPatch->addEntry(&bp1);
//            currPatch->addEntry(&bp2);

//            currPatch->addExit(&bp3);
//            currPatch->addExit(&bp4);

            currPatch->matchBoundaryPoints();
//            insertTestTrajectories(*currPatch);
            std::cout << currPatch->getTrajectories().size() << std::endl;


//            currPatch->removeCollisions();
            for (int p = 0; p < (int) currPatch->getTrajectories().size(); ++p) {
                currPatch->getTrajectoryAt(p)->addBump();
                currPatch->getTrajectoryAt(p)->straighten(1);
            }
        }
    }

//    BoundaryPoint bp1 = BoundaryPoint(glm::vec3(ox, oy + 3.0 * w / 4.0, 0.0), ENTRY, currPatch);
//    BoundaryPoint bp2 = BoundaryPoint(glm::vec3(ox, oy + w / 4.0, 50.0), ENTRY, currPatch);

//    BoundaryPoint bp3 = BoundaryPoint(glm::vec3(ox + w, oy + w / 2.0, 200.0), EXIT, currPatch);
//    BoundaryPoint bp4 = BoundaryPoint(glm::vec3(ox + w/2.0, oy + w, 250.0), EXIT, currPatch);
//    testPatch.addEntry(&bp1);
//    testPatch.addEntry(&bp2);

//    testPatch.addExit(&bp3);
//    testPatch.addExit(&bp4);

//    testPatch.matchBoundaryPoints();



    // Temporary test trajectories:
//    insertTestTrajectories(testPatch);

    /*
     * At this point, each crowd patch will contain trajectories from input to output points
     * Perform collision avoidance on these trajectories within each patch
     */

    // Remove collisions
//    testPatch.removeCollisions();

    // Add control points to straighten out spline
//    for (int i = 0; i < (int) testPatch.getTrajectories().size(); ++i) {
//        testPatch.getTrajectoryAt(i)->straighten(5);
//    }

    // Animate characters along the trajectories
    for (int p = 0; p < numPeriods; ++p) {
        // Interpolate and render points
        for (int j = 0; j < framesPerPeriod; ++j) {
            int currFrame = framesPerPeriod * p + j;
            std::string filename = "../../../../HoudiniProjects/Eleventh/geo/" + std::to_string(currFrame) + ".poly";
            std::ofstream fs;
            fs.open(filename);
            fs << "POINTS\n";
            int count = 1;
            for (int x = 0; x < XDIM; ++x) {
                for (int y = 0; y < YDIM; ++y) {
                    CrowdPatch* currPatch = graph.getPatchAt(x, y);
                    this->followTrajectories(currPatch->getTrajectories(), j, currFrame, fs, count);
                }
            }
//            this->followTrajectories(testPatch.getTrajectories(), j, currFrame, fs, count);

            fs << "POLYS\n";
            fs << "END\n";
            fs.close();

        }
    }
}

