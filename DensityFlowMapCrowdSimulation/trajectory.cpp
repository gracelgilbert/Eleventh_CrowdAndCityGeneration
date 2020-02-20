#include "trajectory.h"


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
