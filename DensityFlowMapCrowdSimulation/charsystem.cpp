#include "charsystem.h"
#include <stdlib.h>

CharSystem::CharSystem() : positions(std::vector<glm::vec2>()),
    velocities(std::vector<glm::vec2>()),
    numChars(0)
{ }

std::vector<glm::vec2> CharSystem::getPositions() {
    return this->positions;
}

std::vector<glm::vec2> CharSystem::getVelocities() {
    return this->velocities;
}

glm::vec2 CharSystem::getIndexPos(int i) {
    return this->positions.at(i);
}

int CharSystem::getNumChars() {
    return this->numChars;
}

glm::vec2 CharSystem::getIndexVel(int i) {
    return this->velocities.at(i);
}

void CharSystem::updateIndexPos(int i, glm::vec2 newPos) {
    this->positions.at(i) = newPos;
}

void CharSystem::updateIndexVel(int i, glm::vec2 newVel) {
    this->velocities.at(i) = newVel;
}

void CharSystem::addChar(glm::vec2 pos, glm::vec2 vel) {
    this->positions.push_back(pos);
    this->velocities.push_back(vel);
    this->numChars++;
}

void CharSystem::fillChars(int numChars) {
//    for (int i = 0; i < numChars; ++i) {
//        glm::vec2 pos = glm::vec2(std::rand() % 400, std::rand() % 400);
//        glm::vec2 vel = glm::vec2(std::rand() % 400, std::rand() % 400);
//        this->addChar(pos, vel);
//    }
}


