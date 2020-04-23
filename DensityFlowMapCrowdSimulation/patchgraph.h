#ifndef PATCHGRAPH_H
#define PATCHGRAPH_H

#include <array>
#include <set>
#include "crowdpatch.h"
#include "math.h"
#include <time.h>

#define XDIM 90
#define YDIM 90

class PatchGraph {

private:
    int numPatches;
    std::array<CrowdPatch*, XDIM * YDIM> graph;

    float totalError;
    float maxDensity;

    float calculateDensityError();
    float calculateFlowError();

public:
    void tempFill();

    void fillWithMaps(QImage densityMap, QImage flowMap);


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

    void resetTrajectories();

    std::vector<CrowdPatch*> getOptimaMinCostPath(std::vector<CrowdPatch*> optima);

    std::vector<CrowdPatch*> getPairMinCostPath(CrowdPatch* cpSource, CrowdPatch* cpTarget);

    std::vector<CrowdPatch*> findMinima();

    std::vector<CrowdPatch*> findMaxima();

    void addBPsAlongPath(std::vector<CrowdPatch*> path);

    bool isValidRemovalPath(std::vector<CrowdPatch*> path);

    void removeBPsAlongPath(std::vector<CrowdPatch*> path);





};


#endif // PATCHGRAPH_H
