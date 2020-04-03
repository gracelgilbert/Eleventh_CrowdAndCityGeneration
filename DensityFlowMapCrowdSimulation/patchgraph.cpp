#include "patchgraph.h"
#define PERIOD 255.0

// Constructors
PatchGraph::PatchGraph() : numPatches(0),
    totalError(0), maxDensity(0.5)
{
    graph.fill(nullptr);
}


void PatchGraph::tempFill() {
    for (int i = 0; i < XDIM; ++i) {
        for (int j = 0; j < YDIM; ++j) {
            CrowdPatch* currPatch = new CrowdPatch(glm::vec2(i * 50, j * 50.0), 50.0, PERIOD, 1.0, glm::vec2(-1.0, 1.0));
            this->setPatchAt(currPatch, i, j);
        }
    }
    this->getPatchAt(0, 0)->setRight(this->getPatchAt(1, 0));
    this->getPatchAt(0, 0)->setDown(this->getPatchAt(0, 1));

    this->getPatchAt(1, 0)->setLeft(this->getPatchAt(0, 0));
    this->getPatchAt(1, 0)->setDown(this->getPatchAt(1, 1));

    this->getPatchAt(0, 1)->setRight(this->getPatchAt(1, 1));
    this->getPatchAt(0, 1)->setUp(this->getPatchAt(0, 0));

    this->getPatchAt(1, 1)->setLeft(this->getPatchAt(0, 1));
    this->getPatchAt(1, 1)->setUp(this->getPatchAt(1, 0));

}


// Getters
CrowdPatch* PatchGraph::getPatchAt(int x, int y) {
    int index = x + y * XDIM;
    return graph.at(index);
}

int PatchGraph::getNumPatches() {
    return this->numPatches;
}

float PatchGraph::getMaxDensity() {
    return this->maxDensity;
}

// Setters
void PatchGraph::setPatchAt(CrowdPatch* patch, int x, int y) {
    int index = x + y * XDIM;
    graph.at(index) = patch;
}

void PatchGraph::setMaxDensity(float density) {
    this->maxDensity = density;
}


// Operations
float PatchGraph::calculateDensityError() {
    float accSqDiff = 0.0;
    int patchCount = 0;
    for (int i = 0; i < int(graph.size()); ++i) {
        if (graph.at(i) != nullptr) {
            CrowdPatch* cp = graph.at(i);
            float diff = cp->getDesiredDensity() - cp->calculateDensity();
            accSqDiff += diff * diff;
            patchCount++;
        }
    }
    float avgSqDiff = accSqDiff / float(patchCount); // or numPatches
    return sqrt(avgSqDiff) / this->maxDensity;
}

float PatchGraph::calculateFlowError() {
    float accFlowDot = 0.0;
    int patchCount = 0;
    for (int i = 0; i < int(graph.size()); ++i) {
        if (graph.at(i) != nullptr) {
            CrowdPatch* cp = graph.at(i);
            accFlowDot += 1.0 - glm::dot(cp->getDesiredDirection(), cp->calculateFlow());
            patchCount++;
        }
    }
    return accFlowDot * (1.f / float(patchCount));
}

float PatchGraph::calculateError() {
    for (int i = 0; i < int(graph.size()); ++i) {
        if (graph.at(i) != nullptr) {
            CrowdPatch* cp = graph.at(i);
            cp->calculateError();
        }
    }
    float de = this->calculateDensityError();
    float fe = this->calculateFlowError();
    this->totalError = de + fe;
    return this->totalError;
}


std::vector<CrowdPatch*> PatchGraph::getOptimaMinCostPath(std::vector<CrowdPatch *> optima) {
    std::vector<CrowdPatch*> path = std::vector<CrowdPatch*>();
    if (optima.size() < 1) {
        return path;
    }
    for (int i = 0; i < int(optima.size()); ++i) {
        CrowdPatch* a = optima.at(i);
        CrowdPatch* b = optima.at((i + 1) % optima.size());
        std::vector<CrowdPatch*> partialPath = this->getPairMinCostPath(a, b);
        for (int j = 0; j < int(partialPath.size()); ++j) {
            path.push_back(partialPath.at(j));
        }
    }
    path.push_back(optima.at(0));
    return path;
}

void updateNeighbor(CrowdPatch* current, CrowdPatch* n) {
    if (n != nullptr) {
        if (!n->isVisited()) {
            float currDist = n->getDistance();
            float newDist = current->getDistance() + current->calculateNeighborWeight(n);
            if (newDist < currDist) {
                n->setDistance(newDist);
                n->setParent(current);
            }
        }
    }
}

std::vector<CrowdPatch*> PatchGraph::getPairMinCostPath(CrowdPatch *cpSource, CrowdPatch *cpTarget) {
    // TODO: implement shortest path finding algorithm

    // Fill set of all unvisited nodes
    std::set<CrowdPatch*> unvisitedSet = std::set<CrowdPatch*>();
    for (int i = 0; i < int(this->graph.size()); ++i) {
        if (this->graph.at(i) != nullptr) {
            unvisitedSet.insert(this->graph.at(i));
        }
    }

    // Set source to distance of 0.0, and current node to source
    cpSource->setDistance(0.0);
    cpSource->setParent(cpSource);
    CrowdPatch* current = cpSource;

    while (current != cpTarget) {
        // Update current node's neighbors
        CrowdPatch* left = current->getLeft();
        updateNeighbor(current, left);

        CrowdPatch* right = current->getRight();
        updateNeighbor(current, right);

        CrowdPatch* up = current->getUp();
        updateNeighbor(current, up);

        CrowdPatch* down = current->getDown();
        updateNeighbor(current, down);

        // current node has now been visited
        current->setVistState(true);
        unvisitedSet.erase(current);

        // Check to see if all unvisited nodes are distance infinity and if so, break
        bool allInfinity = true;
        float minDist = FLT_MAX;
        CrowdPatch* minPatch = nullptr;

        for (CrowdPatch* cp : unvisitedSet) {
            if (cp->getDistance() < minDist) {
                allInfinity = false;
                minDist = cp->getDistance();
                minPatch = cp;
            }
        }
        if (allInfinity) {
            break;
        }
        current = minPatch;
    }

    std::vector<CrowdPatch*> path = std::vector<CrowdPatch*>();
    CrowdPatch* currPathNode = cpTarget;
    while (currPathNode != cpSource) {
        path.insert(path.begin(), currPathNode->getParent());
        currPathNode = currPathNode->getParent();
    }
    for (int x = 0; x < XDIM; ++x) {
        for (int y = 0; y < YDIM; ++y) {
            // Looking at patch (x, y)
            CrowdPatch* currPatch = this->getPatchAt(x, y);
            currPatch->reset();
        }
    }
    return path;
}

std::vector<CrowdPatch*> PatchGraph::findMinima() {
    std::vector<CrowdPatch*> minima = std::vector<CrowdPatch*>();
    for (int x = 0; x < XDIM; ++x) {
        for (int y = 0; y < YDIM; ++y) {
            // Looking at patch (x, y)
            CrowdPatch* currPatch = this->getPatchAt(x, y);
            if (currPatch != nullptr) {
                // Chacke if patch has negative error, meaning low density
                float currError = currPatch->getError();
                if (currError < 0) {
                    // Check neighors to see if any have smaller error
                    float leftError = FLT_MAX;
                    float rightError = FLT_MAX;
                    float upError = FLT_MAX;
                    float downError = FLT_MAX;
                    CrowdPatch* left = currPatch->getLeft();
                    if (left != nullptr) {
                        leftError = left->getError();
                    }

                    CrowdPatch* right = currPatch->getRight();
                    if (right != nullptr) {
                        rightError = right->getError();
                    }

                    CrowdPatch* up = currPatch->getUp();
                    if (up != nullptr) {
                        upError = up->getError();
                    }

                    CrowdPatch* down = currPatch->getDown();
                    if (down != nullptr) {
                        downError = down->getError();
                    }

                    // If current patch's error is smaller than all neighbors, it is a local minima
                    if (currError <= leftError && currError <= rightError
                            && currError <= upError && currError <= downError) {
                        minima.push_back(currPatch);
                    }
                }
            }
        }
    }
    return minima;
}

std::vector<CrowdPatch*> PatchGraph::findMaxima() {
    std::vector<CrowdPatch*> maxima = std::vector<CrowdPatch*>();
    for (int x = 0; x < XDIM; ++x) {
        for (int y = 0; y < YDIM; ++y) {
            // Looking at patch (x, y)
            CrowdPatch* currPatch = this->getPatchAt(x, y);
            if (currPatch != nullptr) {
                // Chacke if patch has positive error, meaning high density
                float currError = currPatch->getError();
                if (currError > 0) {
                    // Check neighors to see if any have larger error
                    float leftError = -FLT_MAX;
                    float rightError = -FLT_MAX;
                    float upError = -FLT_MAX;
                    float downError = -FLT_MAX;
                    CrowdPatch* left = currPatch->getLeft();
                    if (left != nullptr) {
                        leftError = left->getError();
                    }

                    CrowdPatch* right = currPatch->getRight();
                    if (right != nullptr) {
                        rightError = right->getError();
                    }

                    CrowdPatch* up = currPatch->getUp();
                    if (up != nullptr) {
                        upError = up->getError();
                    }

                    CrowdPatch* down = currPatch->getDown();
                    if (down != nullptr) {
                        downError = down->getError();
                    }

                    // If current patch's error is smaller than all neighbors, it is a local minima
                    if (currError >= leftError && currError >= rightError
                            && currError >= upError && currError >= downError) {
                        maxima.push_back(currPatch);
                    }
                }
            }
        }
    }

    return maxima;
}

void PatchGraph::addBPsAlongPath(std::vector<CrowdPatch*> path) {
    for (int i = 0; i < int(path.size()) - 1; ++i) {
        CrowdPatch* cp1 = path.at(i);
        CrowdPatch* cp2 = path.at(i + 1);
        float cp1xOrigin = cp1->getOrigin()[0];
        float cp2xOrigin = cp2->getOrigin()[0];

        float cp1yOrigin = cp1->getOrigin()[1];
        float cp2yOrigin = cp2->getOrigin()[1];

        // Add exit point to cp1 and entry point to cp2
        glm::vec3 bpPos = glm::vec3();
//        std::cout << std::rand() % 100 << std::endl;
        if (cp1->getLeft() == cp2) {
            // cp2 | cp1
            float x = cp2xOrigin + cp2->getWidth();
            float y = cp2yOrigin + cp2->getWidth() * static_cast <float> (std::rand()) / (static_cast <float> (RAND_MAX));
            float t = cp2->getPeriod() * static_cast <float> (std::rand()) / (static_cast <float> (RAND_MAX));

            bpPos = glm::vec3(x, y, t);
        }
        else if (cp1->getRight() == cp2) {
            // cp1 | cp2
            float x = cp1xOrigin + cp1->getWidth();
            float y = cp1yOrigin + cp1->getWidth() * static_cast <float> (std::rand()) / (static_cast <float> (RAND_MAX));
            float t = cp1->getPeriod() * static_cast <float> (std::rand()) / (static_cast <float> (RAND_MAX));

            bpPos = glm::vec3(x, y, t);
        }
        else if (cp1->getUp() == cp2) {
            // cp2
            // cp1

            float x = cp2xOrigin + cp2->getWidth() * static_cast <float> (std::rand()) / (static_cast <float> (RAND_MAX));
            float y = cp2yOrigin + cp2->getWidth();
            float t = cp2->getPeriod() * static_cast <float> (std::rand()) / (static_cast <float> (RAND_MAX));

            bpPos = glm::vec3(x, y, t);
        }
        else if (cp1->getDown() == cp2) {
            // cp1
            // cp2
            float x = cp1xOrigin + cp1->getWidth() * static_cast <float> (std::rand()) / (static_cast <float> (RAND_MAX));
            float y = cp1yOrigin + cp1->getWidth();
            float t = cp1->getPeriod() * static_cast <float> (std::rand()) / (static_cast <float> (RAND_MAX));

            bpPos = glm::vec3(x, y, t);

        } else {
            continue;
        }
        BoundaryPoint* exit = new BoundaryPoint(bpPos, EXIT, cp1);
        BoundaryPoint* entry = new BoundaryPoint(bpPos, ENTRY, cp2);
        exit->setNeighbor(entry);
        entry->setNeighbor(exit);
        cp1->addExit(exit);
        cp2->addEntry(entry);
    }
}

void PatchGraph::removeBPsAlongPath(std::vector<CrowdPatch*> path) {

}



void PatchGraph::resetTrajectories() {
    for (int x = 0; x < XDIM; ++x) {
        for (int y = 0; y < YDIM; ++y) {
            // Looking at patch (x, y)
            CrowdPatch* currPatch = this->getPatchAt(x, y);
            currPatch->clearVecs();
            for (BoundaryPoint* bp: currPatch->getEntryBPs()) {
                bp->setTrajectory(nullptr);
            }
            for (BoundaryPoint* bp: currPatch->getExitBPs()) {
                bp->setTrajectory(nullptr);
            }
        }
    }

}























