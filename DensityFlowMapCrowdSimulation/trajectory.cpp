#include "trajectory.h"
#include <iostream>

// Constructor
Trajectory::Trajectory() : controlPoints(std::vector<glm::vec3>()) {
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

// Modifiers
void Trajectory::insertControlPoint(glm::vec3 cp, int insertPos) {
    this->controlPoints.insert(controlPoints.begin() + insertPos, cp);
}

// Calculations
glm::vec2 Trajectory::getVelocity() {
    int numControlPoints = this->getNumControlPoints();
    return (this->getPositionAtIndex(numControlPoints - 1) - this->getPositionAtIndex(0))
            / (this->getTimeAtIndex(numControlPoints - 1) - this->getTimeAtIndex(0));
}

float Trajectory::getSpeed() {
    return glm::length(this->getVelocity());
}

// Operations
void Trajectory::straighten(int amt) {

    int numControlPoints = this->getNumControlPoints();
    for (int i = 0; i < numControlPoints - 1; ++i) {
        // Curr segment is segment i
        int currIndex = i * (amt + 1);
        glm::vec3 cpFirst = this->getControlPointAtIndex(currIndex);
        glm::vec3 cpSecond = this->getControlPointAtIndex(currIndex + 1);

        float dist = glm::length(cpSecond - cpFirst); // Length of segment
        glm::vec3 dir = glm::normalize(cpSecond - cpFirst); // Direction of segment
        float interval = dist / (amt + 1.f); // Distance between new segments
        for (int j = 1; j <= amt; ++j) {
            glm::vec3 newCp = cpSecond - interval * j * dir;
            this->insertControlPoint(newCp, currIndex + 1);
        }
    }
}
