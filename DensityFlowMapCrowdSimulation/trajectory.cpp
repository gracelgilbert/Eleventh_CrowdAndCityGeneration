#include "trajectory.h"
#include <iostream>


Trajectory::Trajectory() : controlPoints(std::vector<glm::vec3>()) {

}

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

void Trajectory::insertControlPoint(glm::vec3 cp, int insertPos) {
    this->controlPoints.insert(controlPoints.begin() + insertPos, cp);
}

glm::vec2 Trajectory::getVelocity() {

    int numControlPoints = this->getNumControlPoints();
    return (this->getPositionAtIndex(numControlPoints - 1) - this->getPositionAtIndex(0))
            / (this->getTimeAtIndex(numControlPoints - 1) - this->getTimeAtIndex(0));

}

float Trajectory::getSpeed() {
    return glm::length(this->getVelocity());
}


void Trajectory::straighten(int amt) {

    int numControlPoints = this->getNumControlPoints();
//    std::cout << numControlPoints << std::endl;
    for (int i = 0; i < numControlPoints - 1; ++i) {
        // Curr segment is segment i
        int currIndex = i * (amt + 1);
        glm::vec3 cpFirst = this->getControlPointAtIndex(currIndex);
        glm::vec3 cpSecond = this->getControlPointAtIndex(currIndex + 1);

        float dist = glm::length(cpSecond - cpFirst);
        glm::vec3 dir = glm::normalize(cpSecond - cpFirst);
        float interval = dist / (amt + 1.f);
        for (int j = 1; j <= amt; ++j) {
            glm::vec3 newCp = cpSecond - interval * j * dir;
            this->insertControlPoint(newCp, currIndex + 1);
        }
    }
}
