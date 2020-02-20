#include "crowdpatch.h"

CrowdPatch::CrowdPatch() :
    origin(glm::vec2()),
    width(1.0),
    period(1.0),
    S(std::vector<int>()),
    D(std::vector<int>()),
    desiredDensity(1.0),
    desiredDirection(glm::vec2(1.0))
{ }

CrowdPatch::CrowdPatch(glm::vec2 origin, float width, float period,
                       float desiredDensity, glm::vec2 desiredDirection) :
    origin(origin),
    width(width),
    period(period),
    S(std::vector<int>()),
    D(std::vector<int>()),
    desiredDensity(desiredDensity),
    desiredDirection(glm::vec2(desiredDirection))
{ }

glm::vec2 CrowdPatch::getOrigin() {
    return this->origin;
}

float CrowdPatch::getWidth() {
    return this->width;
}

float CrowdPatch::getPeriod() {
    return this->period;
}

std::vector<int> CrowdPatch::getS() {
    return this->S;
}

std::vector<int> CrowdPatch::getD() {
    return this->D;
}

float CrowdPatch::getDesiredDensity() {
    return desiredDensity;
}

glm::vec2 CrowdPatch::getDesiredDirection() {
    return glm::normalize(desiredDirection);
}

void CrowdPatch::setDesiredDensity(float d) {
    this->desiredDensity = d;
}

void CrowdPatch::setDesiredDirection(glm::vec2 dir) {
    this->desiredDirection = glm::normalize(dir);
}

void CrowdPatch::addIndexS(int index) {
    this->S.push_back(index);
}

void CrowdPatch::addIndexD(int index) {
    this->D.push_back(index);
}

void CrowdPatch::removeIndexS(int index) {
    for (auto it = S.begin(); it != S.end(); ) {
        if (*it == index) {
            it = S.erase(it);
        } else {
            ++it;
        }
    }
}

void CrowdPatch::removeIndexD(int index) {
    for (auto it = D.begin(); it != D.end(); ) {
        if (*it == index) {
            it = D.erase(it);
        } else {
            ++it;
        }
    }
}

bool CrowdPatch::isInS(int index) {
    for (auto it = S.begin(); it != S.end(); ) {
        if (*it == index) {
            return true;
        } else {
            ++it;
        }
    }
    return false;
}

bool CrowdPatch::isInD(int index) {
    for (auto it = D.begin(); it != D.end(); ) {
        if (*it == index) {
            return true;
        } else {
            ++it;
        }
    }
    return false;
}













