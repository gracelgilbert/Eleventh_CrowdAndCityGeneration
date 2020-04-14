#ifndef CROWDPATCH_H
#define CROWDPATCH_H

#include <iostream>
#include <set>
#include "la.h"
#include "boundarypoint.h"
#include "trajectory.h"

class BoundaryPoint;

class Trajectory;

class CrowdPatch;

struct Neighbor {
    CrowdPatch* cp;
    float weight;
};

class CrowdPatch {

/*
 * A crowd patch is a small section of the overall crowd. Crowd patches link together
 * in a graph to form the entire crowd. Each patch fills a 2D square and represents a period
 * of time of cyclical animation.
 *
 * Each patch has input and output points that are connected to form trajectories of animation.
 *
 * Each patch has a desired density and direction that is achieved by optimizing the number of
 * input and output points on the patch. The density and flow of the patch is calculated based
 * on the number of characters within it and their velocities.
 */

private:
    // Dimensions and position
    glm::vec2 origin;
    float width;
    float period;

    // Contents
    std::vector<int> S;
    std::vector<int> D;

    // state
    bool visited;
    float distance;

    // Target density and direction
    float desiredDensity;
    glm::vec2 desiredDirection;

    // Error
    float error;
    float calculateDensityError();
    float calculateFlowError();

    // Entry and exit boundary points
    std::vector<BoundaryPoint*> entryBPs;
    std::vector<BoundaryPoint*> exitBPs;

    // Trajectories
    std::vector<Trajectory> trajectories;

    // Neighbors
    CrowdPatch* left;
    CrowdPatch* right;
    CrowdPatch* up;
    CrowdPatch* down;

    std::vector<BoundaryPoint*> leftEntries;
    std::vector<BoundaryPoint*> rightEntries;
    std::vector<BoundaryPoint*> upEntries;
    std::vector<BoundaryPoint*> downEntries;

    std::vector<BoundaryPoint*> leftExits;
    std::vector<BoundaryPoint*> rightExits;
    std::vector<BoundaryPoint*> upExits;
    std::vector<BoundaryPoint*> downExits;

    CrowdPatch* parent;

public:
    // Constructors and destructors
    CrowdPatch();
    CrowdPatch(glm::vec2 origin, float width, float period, float desiredDensity, glm::vec2 desiredDirection);
    ~CrowdPatch();

    // Getters
    glm::vec2 getOrigin();
    float getWidth();
    float getPeriod();

    std::vector<int> getS();
    std::vector<int> getD();

    bool isVisited();
    float getDistance();
    CrowdPatch* getParent();

    float getError();

    float getDesiredDensity();
    glm::vec2 getDesiredDirection();

    std::vector<BoundaryPoint*> getEntryBPs();
    std::vector<BoundaryPoint*> getExitBPs();

    std::vector<Trajectory> getTrajectories();
    Trajectory* getTrajectoryAt(int index);

    // Setters
    void setDesiredDensity(float d);
    void setDesiredDirection(glm::vec2 dir);
    void setVistState(bool state);
    void setDistance(float distance);
    void setParent(CrowdPatch* p);

    // Modifiers
    void addIndexS(int index);
    void addIndexD(int index);

    void removeIndexS(int index);
    void removeIndexD(int index);

    void addEntry(BoundaryPoint* bp);
    void addExit(BoundaryPoint* bp);

    void removeEntry(BoundaryPoint* bp);
    void removeExit(BoundaryPoint* bp);

    bool isInS(int index);
    bool isInD(int index);

    void addTrajectory(Trajectory T);

    void reset();

    // Operations
    void removeCollisions();
    bool minDist(Trajectory T1, Trajectory T2,
                 glm::vec3 &cp1, glm::vec3 &cp2,
                 int &segFirst1, int &segFirst2,
                 float &dist);

    void matchBoundaryPoints();

    float calculateDensity();
    glm::vec2 calculateFlow();

    // Graph node behaviors
    CrowdPatch* getLeft();
    CrowdPatch* getRight();
    CrowdPatch* getUp();
    CrowdPatch* getDown();

    void setLeft(CrowdPatch* l);
    void setRight(CrowdPatch* r);
    void setUp(CrowdPatch* u);
    void setDown(CrowdPatch* d);

    // Calculate the weight of the "edge" from this to cp
    float calculateError();
    float calculateNeighborWeight(CrowdPatch* cp);

    void clearVecs();

    std::vector<BoundaryPoint*> getLeftEntries();
    std::vector<BoundaryPoint*> getRightEntries();
    std::vector<BoundaryPoint*> getUpEntries();
    std::vector<BoundaryPoint*> getDownEntries();

    std::vector<BoundaryPoint*> getLeftExits();
    std::vector<BoundaryPoint*> getRightExits();
    std::vector<BoundaryPoint*> getUpExits();
    std::vector<BoundaryPoint*> getDownExits();





    void addLeftEntry(BoundaryPoint* bp);

    void addRightEntry(BoundaryPoint* bp);

    void addUpEntry(BoundaryPoint* bp);

    void addDownEntry(BoundaryPoint* bp);

    void addLeftExit(BoundaryPoint* bp);

    void addRightExit(BoundaryPoint* bp);

    void addUpExit(BoundaryPoint* bp);

    void addDownExit(BoundaryPoint* bp);

    void removeFromVector(std::vector<BoundaryPoint*> bpVec, BoundaryPoint* bp);

    void removeLeftEntry(BoundaryPoint* bp);
    void removeRightEntry(BoundaryPoint* bp);
    void removeUpEntry(BoundaryPoint* bp);
    void removeDownEntry(BoundaryPoint* bp);

    void removeLeftExit(BoundaryPoint* bp);
    void removeRightExit(BoundaryPoint* bp);
    void removeUpExit(BoundaryPoint* bp);
    void removeDownExit(BoundaryPoint* bp);

};

#endif // CROWDPATCH_H
