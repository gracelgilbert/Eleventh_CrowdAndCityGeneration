#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "la.h"

class Trajectory {
private:
    std::vector<glm::vec3> controlPoints;
public:
    Trajectory();
    std::vector<glm::vec3>& getControlPoints();
    glm::vec3 getControlPointAtIndex(int index);
    int getNumControlPoints();

    glm::vec2 getPositionAtIndex(int index);
    float getTimeAtIndex(int index);

    void insertControlPoint(glm::vec3 cp, int insertPos);

    glm::vec2 getVelocity();
    float getSpeed();

    void straighten(int amt);


};

#endif // TRAJECTORY_H
