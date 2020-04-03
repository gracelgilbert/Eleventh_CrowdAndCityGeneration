#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "la.h"
#include "crowdpatch.h"

class CrowdPatch;

class Trajectory {

/*
 * A trajectory is a sequence of control points that represents the path of a character.
 * Crowd patches are filled with trajectories and players are animated along the interpolated
 * control points over the period of the crowd patch
 */

private:
    // Store control points
    std::vector<glm::vec3> controlPoints;

    CrowdPatch* parent;
public:
    // Constructor
    Trajectory();

    // Getters
    std::vector<glm::vec3>& getControlPoints();
    glm::vec3 getControlPointAtIndex(int index);
    int getNumControlPoints();
    glm::vec2 getPositionAtIndex(int index);
    float getTimeAtIndex(int index);
    CrowdPatch* getParent();

    // Setters
    void setParent(CrowdPatch* p);

    // Modifiers
    void insertControlPoint(glm::vec3 cp, int insertPos);

    // Calculations
    float getDuration();
    glm::vec2 getVelocity();
    float getSpeed();
    glm::vec2 getDirection();

    // Operations
    void straighten(int amt);
    void addBump();

};

#endif // TRAJECTORY_H
