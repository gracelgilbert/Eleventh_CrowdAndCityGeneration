#include "boundarypoint.h"
#include "math.h"

BoundaryPoint::BoundaryPoint(glm::vec3 pos, TYPE type, CrowdPatch* parentPatch) :
    position(pos),
    type(type),
    status(UNMATCHED),
    preferenceList(std::vector<BoundaryPoint>()),
    currentMatch(nullptr),
    parentPatch(parentPatch)
{ }

bool BoundaryPoint::operator <(const BoundaryPoint &bp2) const {
    return this->position[0] < bp2.position[0];
}

glm::vec3 BoundaryPoint::getPos() {
    return this->position;
}

TYPE BoundaryPoint::getType() {
    return this->type;
}

STATUS BoundaryPoint::getStatus() {
    return this->status;
}

std::vector<BoundaryPoint> BoundaryPoint::getPrefList() {
    return this->preferenceList;
}

BoundaryPoint BoundaryPoint::getPointAt(int i) {
    return this->preferenceList.at(i);
}

BoundaryPoint* BoundaryPoint::getCurrentMatch() {
    return this->currentMatch;
}

void BoundaryPoint::setPos(glm::vec3 position) {
    this->position = position;
}

void BoundaryPoint::setType(TYPE newType) {
    this->type = newType;
}

void BoundaryPoint::insertBoundaryPoint(BoundaryPoint bp) {
    this->preferenceList.push_back(bp);
}


float BoundaryPoint::findPenalty(BoundaryPoint *bp) {
    glm::vec2 cpOrigin = this->parentPatch->getOrigin();
    float cpWidth = this->parentPatch->getWidth();

    float xmin = cpOrigin[0];
    float xmax = cpOrigin[0] + cpWidth;

    float ymin = cpOrigin[1];
    float ymax = cpOrigin[1] + cpWidth;

    if (abs(this->position[0] - xmin) < 0.001) {
        // this point is on xmin
        if (abs(bp->getPos()[0] - xmin) < 0.001) {
            // and bp is on xmin
            return 4.0;
        }
    }

    if (abs(this->position[0] - xmax) < 0.001) {
        // this point is on xmax
        if (abs(bp->getPos()[0] - xmax) < 0.001) {
            // and bp is on xmax
            return 4.0;
        }
    }


}

float BoundaryPoint::calculateRating(BoundaryPoint* bp) {
    Trajectory tempT = Trajectory();
    if (this->type == ENTRY) {
        tempT.insertControlPoint(this->position, 0);
        tempT.insertControlPoint(bp->getPos(), 1);
    } else {
        tempT.insertControlPoint(bp->getPos(), 0);
        tempT.insertControlPoint(this->position, 1);
    }
    glm::vec3 vel = tempT.getVelocity();
    glm::vec3 dir = glm::normalize(vel);
    glm::vec3 desiredVel = this->parentPatch->getDesiredDirection();
    float difference = glm::length(desiredVel - dir);

    float uMatch = atan(difference);

    float p = this->findPenalty(bp);

    return uMatch + p;
}

void BoundaryPoint::fillRatingsMap() {
    if (this->type == ENTRY) {
        for (BoundaryPoint* bp : this->parentPatch->getExitBPs()) {
            this->ratingMap.insert(std::pair<BoundaryPoint*,float>(bp, this->calculateRating(bp)));
        }
    } else {
        for (BoundaryPoint* bp : this->parentPatch->getEntryBPs()) {
            this->ratingMap.insert(std::pair<BoundaryPoint*,float>(bp, this->calculateRating(bp)));
        }
    }
}

void BoundaryPoint::sortPreferenceList() {
    // sort preference list with custom compare function

}

bool BoundaryPoint::setCurrentMatch(BoundaryPoint *newMatch) {
    if (newMatch->type != this->type) {
        this->currentMatch = newMatch;
        return true;
    } else {
        return false;
    }
}

bool BoundaryPoint::createTrajectory(Trajectory& T) {
    if (this->type == ENTRY) {
        T.insertControlPoint(this->position, 0);
        T.insertControlPoint(this->currentMatch->position, 1);
        return true;
    } else {
        return false;
    }
}
