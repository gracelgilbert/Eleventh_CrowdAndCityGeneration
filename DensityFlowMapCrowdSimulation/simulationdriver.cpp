#include "simulationdriver.h"
using namespace std;
// Constructor
SimulationDriver::SimulationDriver()
{
    testPatch = CrowdPatch(glm::vec2(0.0, 0.0), 50.0, PERIOD, 1.0, glm::vec2(1.0, 1.0));
    this->graph = PatchGraph();
//    graph.tempFill();

}

int Trajectory::idTracker = 0;

// Helper functions
glm::vec3 mix(glm::vec3 a, glm::vec3 b, float t) {
    return (1.f - t) * a + t * b;
}

glm::vec2 vec3ToVec2(glm::vec3 v3) {
    return glm::vec2(v3[0], v3[1]);
}

void insertTestTrajectories(CrowdPatch &testPatch) {
//    Trajectory testTrajectory = Trajectory();
//    testTrajectory.insertControlPoint(glm::vec3(50.0, 70.0, 0.0), 0);
//    testTrajectory.insertControlPoint(glm::vec3(160.0, 130.0, 130.0), 1);

//    testTrajectory.insertControlPoint(glm::vec3(250.0, 150.0, 255.0), 2);

//    Trajectory testT2 = Trajectory();
//    testT2.insertControlPoint(glm::vec3(50.0, 200.0, 0.0), 0);
//    testT2.insertControlPoint(glm::vec3(150.0, 110.0, 150.0), 1);
//    testT2.insertControlPoint(glm::vec3(250.0, 120.0, 240.0), 2);

//    testPatch.addTrajectory(testTrajectory);
//    testPatch.addTrajectory(testT2);
}

// Animate characters along interpolated patch trajectories
bool SimulationDriver::followTrajectory(Trajectory* T,
                                          int subFrame, int currFrame, std::ofstream &fs,
                                          int &count, glm::vec4& outputPoint) {
    // iterate over all trajectories
    int numControlPoints = T->getNumControlPoints();
    if (numControlPoints < 2) {
        // if fewer than 2 control points, skip
        return false;
    }


    // Bezier interpolation
    bool currFrameOn = this->smoothTrajectory(*T, numControlPoints, subFrame, currFrame, fs, count, outputPoint);
    if (currFrameOn) {
        count++;
    }
    else {
        return false;
    }
    return currFrameOn;
}

float myRand(float n){
    return ((sin(n) * 43758.5453123) - std::floor((sin(n) * 43758.5453123)));
}

float mix(float a, float b, float t) {
    return (1.f - t) * a + t * b;
}

float noise(float p){
    float fl = std::floor(p);
    float fc = p - fl;
    return mix(myRand(fl), myRand(fl + 1.0), fc);
}

bool SimulationDriver::smoothTrajectory(Trajectory T,
                                        int numControlPoints, int subFrame,
                                        int currFrame,
                                        std::ofstream &fs, int count, glm::vec4& outputPoint) {
    bool drawSomething = false;
    for (int seg = 0; seg < numControlPoints - 1; ++seg) {
//        cout << "Segment #" << seg << "============================" << endl;
        // Check if current frame is within this segment otherwiese continue
        glm::vec3 cp1 = T.getControlPointAtIndex(seg);
        glm::vec3 cp2 = T.getControlPointAtIndex(seg + 1);
        if (cp1[2] > cp2[2]) {
            if (currFrame <= cp1[2] && currFrame >= cp2[2]) {
                continue;
            }
            if (currFrame < cp2[2]) {
                subFrame += PERIOD;
            }

            cp2[2] += PERIOD;
        } else {
            if (subFrame < cp1[2] || subFrame >= cp2[2]) {
                continue;
            }
        }

        glm::vec3 cp0 = glm::vec3();
        glm::vec3 cp3 = glm::vec3();

        if (seg == 0) {
            // If first segment
//            glm::vec2 vel = T.getPreviousTrajectory()->getVelocity();
//            float time = T.getDuration();
//            glm::vec3 slope = glm::normalize(glm::vec3(vel[0], vel[1], time));
//            std::cout << slope[0] << ", " << slope[1] << ", " << slope[2] << std::endl;
            cp0 = cp1 + (cp1 - cp2);
//            slope = glm::vec3(5.0);

//            cp0 = cp1 - slope;
//            cp0 = cp1;
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

        if (std::isnan(p[0])) {
            p[0] = 0.0;
        }
        if (std::isnan(p[1])) {
            p[1] = 0.0;
        }
        float blue = 255.0 * (noise(T.getID()));
        display.setPixelColor(QPoint(p[0], p[1]), QColor(blue, currFrame, blue));
        outputPoint = glm::vec4(p[0], 0.0, p[1], T.getID());
        drawSomething = true;
        return drawSomething;
    }
    return drawSomething;
}


// Sort compare function
struct {
        bool operator()(glm::vec4 a, glm::vec4 b) const
        {
            return a[3] < b[3];
        }
    } customCompare;


// Run simulation
void SimulationDriver::run(const int frameCount) {
    display = QImage(512, 512, QImage::Format_RGB32);
    display.fill(Qt::red);
    std::srand(std::time(nullptr));

    QString filenameDensity = "../../../../Maps/PinkDensityMap.jpg";
    QString filenameFlow = "../../../../Maps/PinkFlowMap.jpg";


    QImage densityMap = QImage(filenameDensity);
    QImage flowMap = QImage(filenameFlow);

    // Initialize patches and graph:
    graph.fillWithMaps(densityMap, flowMap);

    // Optimize boundary points:

    // 1. Calculate error of graph
    float error = 1.0;
    int attemptCount = 0;
    error = this->graph.calculateError();
    while (error > EPSILON && attemptCount < 2) {
        // 2. Find minima and maxima of error
        std::vector<CrowdPatch*> minima = this->graph.findMinima();
        int groupSize = 4;

        // if not enough minima to make group, add points
        if (minima.size() < groupSize) {
            for (int dummy = 0; dummy < 3.0 * groupSize; ++dummy) {
                bool notDone = true;
                int counter = 0;
                while (notDone && counter < 50) {
                    int randIndexX = floor(XDIM * static_cast <float> (std::rand()) / (static_cast <float> (RAND_MAX)));
                    int randIndexY = floor(YDIM * static_cast <float> (std::rand()) / (static_cast <float> (RAND_MAX)));
                    CrowdPatch* currPatch = graph.getPatchAt(randIndexX, randIndexY);
                    if (currPatch != nullptr) {
                        minima.push_back(currPatch);
                        notDone = false;
                    }
                    counter++;
                }

            }
        }

//         3a. Split minima into groups
        if (minima.size() >= groupSize) {
            for (int i = 0; i < minima.size() - groupSize + 1; i += groupSize) {
                std::vector<CrowdPatch*> currGroupFront =  std::vector<CrowdPatch*>();
                for (int s = 0; s < groupSize; ++s) {
                    if (i + s < minima.size()) {
                        currGroupFront.push_back(minima.at(i + s));
                    }
                }

                // 4a. Find shortest paths between group of minima
                std::vector<CrowdPatch*> currPathFront = this->graph.getOptimaMinCostPath(currGroupFront);

                // 5a. Add boundary points along shortest path
                float random = static_cast <float> (std::rand()) / (static_cast <float> (RAND_MAX));
                if (random > 0.0) {
                    graph.addBPsAlongPath(currPathFront);
                }

            }
        }


        this->graph.resetTrajectories();
        // Create temporary matching for error calculation
        for (int i = 0; i < XDIM; ++i) {
            for (int j = 0; j < YDIM; ++j) {
                CrowdPatch* currPatch = graph.getPatchAt(i, j);
                if (currPatch != nullptr) {
                    currPatch->matchBoundaryPoints();
                }
            }
        }

        // 6. Calculate error of graph
        error = this->graph.calculateError();
        this->graph.resetTrajectories();

        std::vector<CrowdPatch*> maxima = this->graph.findMaxima();

        if (maxima.size() < groupSize) {
            for (int dummy = 0; dummy < 3.0 * groupSize; ++dummy) {
                bool notDone = true;
                int counter = 0;
                while (notDone && counter < 50) {
                    int randIndexX = floor(XDIM * static_cast <float> (std::rand()) / (static_cast <float> (RAND_MAX)));
                    int randIndexY = floor(YDIM * static_cast <float> (std::rand()) / (static_cast <float> (RAND_MAX)));
                    CrowdPatch* currPatch = graph.getPatchAt(randIndexX, randIndexY);
                    if (currPatch != nullptr) {
                        maxima.push_back(currPatch);
                        notDone = false;
                    }
                    counter++;
                }

            }
        }

        // 3b. Split maxima into groups
        if (maxima.size() >= groupSize) {
            for (int i = 0; i < maxima.size() - groupSize + 1; i += groupSize) {
                std::vector<CrowdPatch*> currGroup =  std::vector<CrowdPatch*>();
                for (int s = 0; s < groupSize; ++s) {
                    if (i + s < maxima.size()) {
                        currGroup.push_back(maxima.at(i + s));
                    }
                }

                // 4b. Find shortest paths between group of maxima
                std::vector<CrowdPatch*> currPath = this->graph.getOptimaMinCostPath(currGroup);
                if (currPath.size() > 1) {
                    if (currPath.at(0) != currPath.at(currPath.size() - 1)) {
                        std::cout << "non cyclic path!!!" << std::endl;
                        continue;
                    }
                }

                // 5b. Remove boundary points along shortest path
                graph.removeBPsAlongPath(currPath);
//                break;

            }
        }
        this->graph.resetTrajectories();
        // Create temporary matching for error calculation
        for (int i = 0; i < XDIM; ++i) {
            for (int j = 0; j < YDIM; ++j) {
                CrowdPatch* currPatch = graph.getPatchAt(i, j);
                if (currPatch != nullptr) {
                    currPatch->matchBoundaryPoints();
                }
            }
        }

        // 6. Calculate error of graph
        error = this->graph.calculateError();
        this->graph.resetTrajectories();

        // Increment attempt count
        attemptCount++;
    }

    // Match boundary points :
    for (int i = 0; i < XDIM; ++i) {
        for (int j = 0; j < YDIM; ++j) {
            CrowdPatch* currPatch = graph.getPatchAt(i, j);
            if (currPatch == nullptr) {
                continue;
            }

            currPatch->matchBoundaryPoints();
//            insertTestTrajectories(*currPatch);
//            Trajectory t = Trajectory();
//            t.insertControlPoint(glm::vec3(currPatch->getOrigin()[0], currPatch->getOrigin()[1], 0), 0);
//            t.insertControlPoint(glm::vec3(currPatch->getOrigin()[0] + currPatch->getWidth(),
//                                 currPatch->getOrigin()[1] + currPatch->getWidth(),
//                    currPatch->getPeriod()), 1);
//            t.setParent(currPatch);
//            currPatch->addTrajectory(t);
        }
    }

    // Set up trajectory previous and next links
    for (int i = 0; i < XDIM; ++i) {
        for (int j = 0; j < YDIM; ++j) {
            CrowdPatch* currPatch = graph.getPatchAt(i, j);
            if (currPatch == nullptr) {
                continue;
            }
            for (int p = 0; p < (int) currPatch->getTrajectories().size(); ++p) {
                Trajectory* currTrajectory = currPatch->getTrajectoryAt(p);

                BoundaryPoint* entrance = currTrajectory->getEntryPoint();
                BoundaryPoint* prevExit = entrance->getNeighbor();
                Trajectory* prevTrajectory = prevExit->getTrajectory();
                currTrajectory->setPreviousTrajectory(prevTrajectory);

                BoundaryPoint* exit = currTrajectory->getExitPoint();
                BoundaryPoint* nextEntrance = exit->getNeighbor();
                Trajectory* nextTrajectory = nextEntrance->getTrajectory();
                currTrajectory->setNextTrajectory(nextTrajectory);

            }
        }
    }


    // Split trajectories so non overlap period
    for (int i = 0; i < XDIM; ++i) {
        for (int j = 0; j < YDIM; ++j) {
            CrowdPatch* currPatch = graph.getPatchAt(i, j);
            if (currPatch == nullptr) {
                continue;
            }
            int initialTrajectorySize = currPatch->getTrajectories().size();

            for (int p = 0; p < initialTrajectorySize; ++p) {
                Trajectory* splitPointer = new Trajectory();
                bool split = currPatch->getTrajectoryAt(p)->split(splitPointer);
                if (split) {
                    currPatch->addTrajectory(splitPointer);
                }
            }
        }
    }


    // Curve trajectories and set IDs
    for (int i = 0; i < XDIM; ++i) {
        for (int j = 0; j < YDIM; ++j) {
            CrowdPatch* currPatch = graph.getPatchAt(i, j);
            if (currPatch == nullptr) {
                continue;
            }
            for (int p = 0; p < currPatch->getTrajectories().size(); ++p) {
                Trajectory* currTrajectory = currPatch->getTrajectories().at(p);
                currTrajectory->curve();

                if (currTrajectory->getTimeAtIndex(0) < 0.001 ||
                        currTrajectory->getStarting()) {
                    continue;
                }

                Trajectory* prevTrajectory = currTrajectory->getPreviousTrajectory();
                bool go = true;
                while (go) {
                    if (prevTrajectory == nullptr) {
                        go = false;
                    } else if (prevTrajectory == currTrajectory) {
                        go = false;
                    } else {
                        if (prevTrajectory->getTimeAtIndex(0) < 0.001 ||
                                prevTrajectory->getStarting()) {
                            currTrajectory->setID(prevTrajectory->getID());
                            go = false;
                        } else {
                            prevTrajectory = prevTrajectory->getPreviousTrajectory();
                        }
                    }
                }
            }
        }
    }

    /*
     * At this point, each crowd patch will contain curved trajectories from input to output points with proper IDs
     */

    // Animate characters along the trajectories
    for (int currFrame = 0; currFrame < frameCount; ++currFrame) {

        // Update IDs if starting new period
        if (currFrame % PERIOD == 0 && currFrame > 0) {
            std::cout << currFrame << std::endl;
            // Update starting IDs over period
            for (int i = 0; i < XDIM; ++i) {
                for (int j = 0; j < YDIM; ++j) {
                    CrowdPatch* currPatch = graph.getPatchAt(i, j);
                    if (currPatch == nullptr) {
                        continue;
                    }
                    for (int p = 0; p < currPatch->getTrajectories().size(); ++p) {
                        Trajectory* currTrajectory = currPatch->getTrajectories().at(p);


                        if (currTrajectory->getTimeAtIndex(0) < 0.001 ||
                                currTrajectory->getStarting()) {
                            Trajectory* prevTrajectory = currTrajectory->getPreviousTrajectory();
                            if (prevTrajectory != nullptr) {
                                currTrajectory->setID(prevTrajectory->getID());
                            }
                        }

                    }
                }
            }
            for (int i = 0; i < XDIM; ++i) {
                for (int j = 0; j < YDIM; ++j) {
                    CrowdPatch* currPatch = graph.getPatchAt(i, j);
                    if (currPatch == nullptr) {
                        continue;
                    }
                    for (int p = 0; p < currPatch->getTrajectories().size(); ++p) {
                        Trajectory* currTrajectory = currPatch->getTrajectories().at(p);
                        if (currTrajectory->getTimeAtIndex(0) < 0.001 ||
                                currTrajectory->getStarting()) {
                            continue;
                        }

                        Trajectory* prevTrajectory = currTrajectory->getPreviousTrajectory();
                        bool go = true;
                        while (go) {
                            if (prevTrajectory == nullptr) {
                                go = false;
                            } else if (prevTrajectory == currTrajectory) {
                                go = false;
                            } else {
                                if (prevTrajectory->getTimeAtIndex(0) < 0.001 ||
                                        prevTrajectory->getStarting()) {
                                    currTrajectory->setID(prevTrajectory->getID());
                                    go = false;
                                } else {
                                    prevTrajectory = prevTrajectory->getPreviousTrajectory();
                                }
                            }
                        }

                    }
                }
            }
        }

        std::string filename = "../../../../HoudiniProjects/Eleventh/geo/" + std::to_string(currFrame + 1) + ".poly";
        std::ofstream fs;
        fs.open(filename);
        fs << "POINTS\n";
        int count = 1;
        std::vector<glm::vec4> outputPoints;
        for (int x = 0; x < XDIM; ++x) {
            for (int y = 0; y < YDIM; ++y) {
                CrowdPatch* currPatch = graph.getPatchAt(x, y);
                if (currPatch == nullptr) {
                    continue;
                }
                for (int t = 0; t < currPatch->getTrajectories().size(); ++t) {
                    Trajectory* T = currPatch->getTrajectories().at(t);
                    glm::vec4 outputPoint;
                    bool output = this->followTrajectory(T, int(currFrame) % int(PERIOD), int(currFrame) % int(PERIOD), fs, count, outputPoint);
                    if (output) {
                        outputPoints.push_back(outputPoint);
                    }
                }
            }
        }
        std::sort(outputPoints.begin(), outputPoints.end(), customCompare);

        for (int o = 0; o < outputPoints.size(); ++o) {
            glm::vec4 p = outputPoints.at(o);
            fs << o + 1 << ":";
            fs << " " << p[0] << " " << 0 << " " << p[2] << "\n";
        }

        fs << "POLYS\n";
        fs << "END\n";
        fs.close();

    }
}

