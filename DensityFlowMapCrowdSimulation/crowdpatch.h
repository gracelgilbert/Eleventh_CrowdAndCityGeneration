#ifndef CROWDPATCH_H
#define CROWDPATCH_H

#include "la.h"
#include "trajectory.h"
#include "set"
#include "boundarypoint.h"

class BoundaryPoint;

class CrowdPatch {
private:
    glm::vec2 origin;
    float width;
    float period;
    std::vector<int> S;
    std::vector<int> D;
    float desiredDensity;
    glm::vec2 desiredDirection;

    std::vector<BoundaryPoint*> entryBPs;
    std::vector<BoundaryPoint*> exitBPs;


public:
    CrowdPatch();
    CrowdPatch(glm::vec2 origin, float width, float period, float desiredDensity, glm::vec2 desiredDirection);

    ~CrowdPatch();
    glm::vec2 getOrigin();
    float getWidth();
    float getPeriod();
    std::vector<int> getS();
    std::vector<int> getD();
    float getDesiredDensity();
    glm::vec2 getDesiredDirection();

    // TODO: Make private and add getter/setters and inserts
    std::vector<Trajectory> trajectories;

    void removeCollisions();

    bool minDist(Trajectory T1, Trajectory T2,
                 glm::vec3 &cp1, glm::vec3 &cp2,
                 int &segFirst1, int &segFirst2,
                 float &dist);

    void setDesiredDensity(float d);
    void setDesiredDirection(glm::vec2 dir);

    void addIndexS(int index);
    void addIndexD(int index);

    void removeIndexS(int index);
    void removeIndexD(int index);

    bool isInS(int index);
    bool isInD(int index);

    std::vector<BoundaryPoint*> getEntryBPs();
    std::vector<BoundaryPoint*> getExitBPs();


};




#endif // CROWDPATCH_H
