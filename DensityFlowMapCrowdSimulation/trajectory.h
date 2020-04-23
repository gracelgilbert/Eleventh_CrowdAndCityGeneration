#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "la.h"
#include "crowdpatch.h"

class CrowdPatch;
class BoundaryPoint;

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
    static int idTracker;
    int ID;
    Trajectory* previousTrajectory;
    Trajectory* nextTrajectory;
    BoundaryPoint* entryPoint;
    BoundaryPoint* exitPoint;
    bool startingTrajectory;
public:
    // Constructor
    Trajectory();

    bool swap;

    // Getters
    std::vector<glm::vec3>& getControlPoints();
    glm::vec3 getControlPointAtIndex(int index);
    int getNumControlPoints();
    glm::vec2 getPositionAtIndex(int index);
    float getTimeAtIndex(int index);
    CrowdPatch* getParent();
    int getID();
    Trajectory* getPreviousTrajectory();
    Trajectory* getNextTrajectory();
    BoundaryPoint* getEntryPoint();
    BoundaryPoint* getExitPoint();
    bool getStarting();

    // Setters
    void setParent(CrowdPatch* p);
    void setID(int id);
    void setPreviousTrajectory(Trajectory* t);
    void setNextTrajectory(Trajectory* t);
    void setEntryPoint(BoundaryPoint* bp);
    void setExitPoint(BoundaryPoint* bp);
    void setStarting(bool start);

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
    bool split(Trajectory* split);
    void curve();

};

#endif // TRAJECTORY_H
