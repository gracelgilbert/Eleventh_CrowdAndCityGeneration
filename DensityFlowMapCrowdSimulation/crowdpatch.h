#ifndef CROWDPATCH_H
#define CROWDPATCH_H

#include "la.h"
#include "trajectory.h"
#include "set"

class CrowdPatch {
private:
    glm::vec2 origin;
    float width;
    float period;
    std::vector<int> S;
    std::vector<int> D;
    float desiredDensity;
    glm::vec2 desiredDirection;



//    CrowdPatch leftPatch;
//    CrowdPatch rightPatch;
//    CrowdPatch upPatch;
//    CrowdPatch downPatch;

public:
    CrowdPatch();
    CrowdPatch(glm::vec2 origin, float width, float period, float desiredDensity, glm::vec2 desiredDirection);

    std::vector<glm::vec2> leftIns;
    std::vector<glm::vec2> leftOuts;
    std::vector<glm::vec2> rightIns;
    std::vector<glm::vec2> rightOuts;
    std::vector<glm::vec2> upIns;
    std::vector<glm::vec2> upOuts;
    std::vector<glm::vec2> downIns;
    std::vector<glm::vec2> downOuts;


    glm::vec2 getOrigin();
    float getWidth();
    float getPeriod();
    std::vector<int> getS();
    std::vector<int> getD();
    float getDesiredDensity();
    glm::vec2 getDesiredDirection();

    // TODO: Make private and add getter/setters and inserts
    std::vector<Trajectory> trajectories;



    void setDesiredDensity(float d);
    void setDesiredDirection(glm::vec2 dir);

    void addIndexS(int index);
    void addIndexD(int index);

    void removeIndexS(int index);
    void removeIndexD(int index);

    bool isInS(int index);
    bool isInD(int index);

};




#endif // CROWDPATCH_H
