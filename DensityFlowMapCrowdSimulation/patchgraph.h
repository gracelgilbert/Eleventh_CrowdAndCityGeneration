#ifndef PATCHGRAPH_H
#define PATCHGRAPH_H

#include <array>
#include "crowdpatch.h"
#include "math.h"

#define XDIM 1
#define YDIM 1

class PatchGraph {

private:
    int numPatches;
//    CrowdPatch graph[XDIM * YDIM];
    std::array<CrowdPatch*, XDIM * YDIM> graph;

    float totalError;
    float maxDensity;

    float calculateDensityError();
    float calculateFlowError();

public:
    // Constructors
    // Eventually will pass in an image to the constructor to fill the graph
    PatchGraph();

    // Getters
    CrowdPatch* getPatchAt(int x, int y);
    int getNumPatches();
    float getTotalError();
    float getMaxDensity();

    // Setters
    void setPatchAt(CrowdPatch* patch, int x, int y);
    void setMaxDensity(float density);

    // Operations
    float calculateError();

    std::vector<CrowdPatch*> getOptimaMinCostPath(std::vector<CrowdPatch*> optima);

    std::vector<CrowdPatch*> getPairMinCostPath(CrowdPatch* cpSource, CrowdPatch* cpTarget);

    void tempFill();

};


#endif // PATCHGRAPH_H
