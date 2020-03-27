#include "patchgraph.h"
#define PERIOD 255.0

// Constructors
PatchGraph::PatchGraph() : numPatches(0),
    totalError(0), maxDensity(0.5)
{
    graph.fill(nullptr);
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
            accFlowDot += 1.0 - glm::dot(cp->getDesiredDensity(), cp->calculateDensity());
            patchCount++;
        }
    }
    return accFlowDot * (1.f / float(patchCount));
}

float PatchGraph::calculateError() {
    this->totalError = this->calculateDensityError() +
            this->calculateFlowError();
    return this->totalError;
}


std::vector<CrowdPatch*> PatchGraph::getOptimaMinCostPath(std::vector<CrowdPatch *> optima) {
    std::vector<CrowdPatch*> path;
    for (int i = 0; i < int(optima.size()); ++i) {
        CrowdPatch* a = optima.at(i);
        CrowdPatch* b = optima.at((i + 1) % optima.size());
        std::vector<CrowdPatch*> partialPath = this->getPairMinCostPath(a, b);
        for (int j = 0; j < int(partialPath.size()); ++j) {
            path.push_back(partialPath.at(j));
        }
    }
    return path;
}

std::vector<CrowdPatch*> PatchGraph::getPairMinCostPath(CrowdPatch *cpSource, CrowdPatch *cpTarget) {
    // TODO: implement shortest path finding algorithm
    return std::vector<CrowdPatch*>();
}

void PatchGraph::tempFill() {
    for (int i = 0; i < XDIM; ++i) {
        for (int j = 0; j < YDIM; ++j) {
            CrowdPatch* currPatch = new CrowdPatch(glm::vec2(i * 200.0 + 50.0, j * 200.0 + 50.0), 200.0, PERIOD, 1.0, glm::vec2(1.0, -1.0));
            this->setPatchAt(currPatch, i, j);
        }
    }
}

