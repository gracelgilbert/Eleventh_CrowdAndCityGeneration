#ifndef CROWDPATCH_H
#define CROWDPATCH_H

#include <iostream>
#include <set>
#include "la.h"
#include "boundarypoint.h"
#include "trajectory.h"

class BoundaryPoint;

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

    // Target density and direction
    float desiredDensity;
    glm::vec2 desiredDirection;

    // Entry and exit boundary points
    std::vector<BoundaryPoint*> entryBPs;
    std::vector<BoundaryPoint*> exitBPs;

    // Trajectories
    std::vector<Trajectory> trajectories;

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

    float getDesiredDensity();
    glm::vec2 getDesiredDirection();

    std::vector<BoundaryPoint*> getEntryBPs();
    std::vector<BoundaryPoint*> getExitBPs();

    std::vector<Trajectory> getTrajectories();
    Trajectory getTrajectoryAt(int index);

    // Setters
    void setDesiredDensity(float d);
    void setDesiredDirection(glm::vec2 dir);

    // Modifiers
    void addIndexS(int index);
    void addIndexD(int index);

    void removeIndexS(int index);
    void removeIndexD(int index);

    void addEntry(BoundaryPoint* bp);
    void addExit(BoundaryPoint* bp);

    bool isInS(int index);
    bool isInD(int index);

    void addTrajectory(Trajectory T);

    // Operations
    void removeCollisions();
    bool minDist(Trajectory T1, Trajectory T2,
                 glm::vec3 &cp1, glm::vec3 &cp2,
                 int &segFirst1, int &segFirst2,
                 float &dist);

    void matchBoundaryPoints();

};




#endif // CROWDPATCH_H
