#ifndef CHARSYSTEM_H
#define CHARSYSTEM_H

#include "la.h"
class CharSystem {
private:

    std::vector<glm::vec2> positions;
    std::vector<glm::vec2> velocities;
    int numChars;

public:

    CharSystem();
    std::vector<glm::vec2> getPositions();
    std::vector<glm::vec2> getVelocities();

    glm::vec2 getIndexPos(int i);
    glm::vec2 getIndexVel(int i);
    int getNumChars();

    void updateIndexPos(int i, glm::vec2 newPos);
    void updateIndexVel(int i, glm::vec2 newVel);

    void addChar(glm::vec2 pos, glm::vec2 vel);
    void fillChars(int numChars);
};



#endif // CHARSYSTEM_H
