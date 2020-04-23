#include "trajectory.h"
#include <iostream>

// Constructor
Trajectory::Trajectory() : controlPoints(std::vector<glm::vec3>()),
                                         parent(nullptr),
                                         previousTrajectory(nullptr),
                                         nextTrajectory(nullptr),
                                         entryPoint(nullptr),
                                         exitPoint(nullptr),
                                         swap(true),
                                         startingTrajectory(false)
{
    this->ID = idTracker;
    idTracker++;
}

// Getters
std::vector<glm::vec3>& Trajectory::getControlPoints() {
    return this->controlPoints;
}

glm::vec3 Trajectory::getControlPointAtIndex(int index) {
    return this->controlPoints.at(index);
}

int Trajectory::getNumControlPoints() {
    return this->controlPoints.size();
}

glm::vec2 Trajectory::getPositionAtIndex(int index) {
    return glm::vec2(this->controlPoints.at(index)[0], this->controlPoints.at(index)[1]);
}

float Trajectory::getTimeAtIndex(int index) {
    return this->controlPoints.at(index)[2];
}

CrowdPatch* Trajectory::getParent() {
    return this->parent;
}

int Trajectory::getID() {
    return this->ID;
}

Trajectory* Trajectory::getPreviousTrajectory() {
    return this->previousTrajectory;
}

Trajectory* Trajectory::getNextTrajectory() {
    return this->nextTrajectory;
}

BoundaryPoint* Trajectory::getEntryPoint() {
    return this->entryPoint;
}

BoundaryPoint* Trajectory::getExitPoint() {
    return this->exitPoint;
}

bool Trajectory::getStarting() {
    return this->startingTrajectory;
}

// Setters
void Trajectory::setParent(CrowdPatch *p) {
    this->parent = p;
}

void Trajectory::setID(int id) {
    this->ID = id;
}

void Trajectory::setPreviousTrajectory(Trajectory *t) {
    this->previousTrajectory = t;
}

void Trajectory::setNextTrajectory(Trajectory *t) {
    this->nextTrajectory = t;
}

void Trajectory::setEntryPoint(BoundaryPoint *bp) {
    this->entryPoint = bp;
}

void Trajectory::setExitPoint(BoundaryPoint *bp) {
    this->exitPoint = bp;
}

void Trajectory::setStarting(bool start) {
    this->startingTrajectory = start;
}

// Modifiers
void Trajectory::insertControlPoint(glm::vec3 cp, int insertPos) {
    this->controlPoints.insert(controlPoints.begin() + insertPos, cp);
}

// Calculations
float Trajectory::getDuration() {
    int numControlPoints = this->getNumControlPoints();
    return (this->getTimeAtIndex(numControlPoints - 1) - this->getTimeAtIndex(0));
}

glm::vec2 Trajectory::getVelocity() {
    int numControlPoints = this->getNumControlPoints();
    return (this->getPositionAtIndex(numControlPoints - 1) - this->getPositionAtIndex(0))
            / this->getDuration();
}

float Trajectory::getSpeed() {
    return glm::length(this->getVelocity());
}

glm::vec2 Trajectory::getDirection() {
    return glm::normalize(this->getVelocity());
}

// Operations
void Trajectory::straighten(int amt) {

    int numControlPoints = this->getNumControlPoints();
    for (int i = 0; i < numControlPoints - 1; ++i) {
        // Curr segment is segment i
        int currIndex = i * (amt + 1);
        glm::vec3 cpFirst = this->getControlPointAtIndex(currIndex);
        glm::vec3 cpSecond = this->getControlPointAtIndex(currIndex + 1);
//        if (cpFirst[2] > cpSecond[2]) {
//            cpSecond[2] += this->parent->getPeriod();
//        }

        float dist = glm::length(cpSecond - cpFirst); // Length of segment
        glm::vec3 dir = glm::normalize(cpSecond - cpFirst); // Direction of segment
        float interval = dist / (amt + 1.f); // Distance between new segments
        for (int j = 1; j <= amt; ++j) {
            glm::vec3 newCp = cpSecond - interval * j * dir;
//            if (newCp[2] >= this->parent->getPeriod()) {
//                newCp[2] -= this->parent->getPeriod();
//            }
            this->insertControlPoint(newCp, currIndex + 1);
        }
    }
}

void Trajectory::addBump() {
    int numControlPoints = this->getNumControlPoints();
    glm::vec3 cpFirst = this->getControlPointAtIndex(0);
    glm::vec3 cpLast = this->getControlPointAtIndex(numControlPoints - 1);

    glm::vec2 pos1 = this->getPositionAtIndex(0);
    glm::vec2 pos2 = this->getPositionAtIndex(numControlPoints - 1);

    if (abs(pos1[0] - pos2[0]) < 0.01) {
        // Vertical, share x
        glm::vec2 patchCenter = this->parent->getOrigin() + 0.5f * glm::vec2(this->parent->getWidth());
        float newY = (pos1[1] + pos2[1]) * 0.5;
        glm::vec2 dirToCenter = glm::normalize(patchCenter - glm::vec2(pos1[0], newY));
        glm::vec2 newPos = glm::vec2(pos1[0], newY) + dirToCenter * this->parent->getWidth() * 0.25f;
        float newTime = 0.5 * (cpFirst[2] + cpLast[2]);
//        if (cpFirst[2] > cpLast[2]) {
//            newTime = 0.5 * (cpFirst[2] + this->parent->getPeriod() + cpLast[2]);
//            if (newTime >= this->parent->getPeriod()) {
//                newTime -= this->parent->getPeriod();
//            }
//        }
        this->insertControlPoint(glm::vec3(newPos[0], newPos[1], newTime), 1);
    }
    else if (abs(pos1[1] - pos2[1]) < 0.01) {
        // Horizontal, share y
        glm::vec2 patchCenter = this->parent->getOrigin() + 0.5f * glm::vec2(this->parent->getWidth());
        float newX = (pos1[0] + pos2[0]) * 0.5;
        glm::vec2 dirToCenter = glm::normalize(patchCenter - glm::vec2(newX, pos1[1]));
        glm::vec2 newPos = glm::vec2(newX, pos1[1]) + dirToCenter * this->parent->getWidth() * 0.25f;
        float newTime = 0.5 * (cpFirst[2] + cpLast[2]);
//        if (cpFirst[2] > cpLast[2]) {
//            newTime = 0.5 * (cpFirst[2] + this->parent->getPeriod() + cpLast[2]);
//            if (newTime >= this->parent->getPeriod()) {
//                newTime -= this->parent->getPeriod();
//            }
//        }
        this->insertControlPoint(glm::vec3(newPos[0], newPos[1], newTime), 1);
    }
}

bool Trajectory::split(Trajectory* split) {
    if (this->getNumControlPoints() > 2) {
        return false;
    }
    glm::vec2 cpFirst = this->getPositionAtIndex(0);
    glm::vec2 cpSecond = this->getPositionAtIndex(1);
    float tFirst = this->getTimeAtIndex(0);
    float tSecond = this->getTimeAtIndex(1);
    float period = this->parent->getPeriod();

    if (tFirst > tSecond) {
        float laterSecond = tSecond + period;

//        glm::vec2 direction = glm::normalize(this->getVelocity());

        float timeToPeriod = period - tFirst;
        float totalTime = laterSecond - tFirst;

        glm::vec2 posAtPeriod = (timeToPeriod / totalTime) * cpSecond +
                (1 - (timeToPeriod / totalTime)) * cpFirst;

//        Trajectory* split = new Trajectory();
        split->insertControlPoint(this->getControlPointAtIndex(0), 0);
        split->insertControlPoint(glm::vec3(posAtPeriod[0], posAtPeriod[1], period), 1);

        split->setParent(this->parent);
        split->setPreviousTrajectory(this->getPreviousTrajectory());
        split->setNextTrajectory(this);
        this->getPreviousTrajectory()->setNextTrajectory(split);

        this->controlPoints.at(0) = glm::vec3(posAtPeriod[0], posAtPeriod[1], 0);
        this->setStarting(true);
        this->setPreviousTrajectory(split);

//        this->parent->addTrajectory(split);
        return true;
    } else {
        return false;
    }

//    int numControlPoints = this->getNumControlPoints();
//    for (int i = 0; i < numControlPoints - 1; ++i) {
//        glm::vec3 cpFirst = this->getControlPointAtIndex(i);
//        glm::vec3 cpSecond = this->getControlPointAtIndex(i + 1);

//        if (cpFirst[2] > cpSecond[2]) {

//        }
//    }
}

void Trajectory::curve() {
    glm::vec2 prevVel = glm::normalize(this->getPreviousTrajectory()->getVelocity());
//    float prevTime = this->getDuration();
//    glm::vec3 frontSlope = glm::normalize(glm::vec3(prevVel[0], prevVel[1], prevTime));
//    frontSlope = glm::vec3(1.0);

    glm::vec2 nextVel = glm::normalize(this->getNextTrajectory()->getVelocity());
//    float nextTime = this->getDuration();
//    glm::vec3 backSlope = glm::normalize(glm::vec3(nextVel[0], nextVel[1], nextTime));
//    backSlope = glm::vec3(1.0);

    glm::vec3 cp0 = this->getControlPointAtIndex(0);
    glm::vec3 cp1 = this->getControlPointAtIndex(this->getNumControlPoints() - 1);
    float length = glm::length(cp1 - cp0);

    float scale = 0.01f;
    glm::vec3 frontSlope = glm::vec3(prevVel[0] * length * scale, prevVel[1] * length * scale, this->getDuration() * 0.33f);
    glm::vec3 backSlope = glm::vec3(nextVel[0] * length * scale, nextVel[1] * length * scale, this->getDuration() * 0.33f);


    glm::vec3 frontCP = cp0 + frontSlope;
    glm::vec3 backCP = cp1 - backSlope;
//    frontCP[2] = cp0[2] + this->getDuration() * 0.33f;
//    backCP[2] = cp1[2] - this->getDuration() * 0.33f;

    this->insertControlPoint(backCP, 1);
    this->insertControlPoint(frontCP, 1);


}



