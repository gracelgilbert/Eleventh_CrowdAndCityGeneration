#include "boundarypoint.h"

// Constructor
BoundaryPoint::BoundaryPoint(glm::vec3 pos, TYPE type, CrowdPatch* parentPatch) :
    position(pos),
    type(type),
    parentPatch(parentPatch),
    neighbor(nullptr),
    trajectory(nullptr),
    status(UNMATCHED),
    currentMatch(nullptr),
    counter(0),
    done(false),
    preferenceList(std::vector<BoundaryPoint*>())
{ }

// Destructor
BoundaryPoint::~BoundaryPoint() {
}

// Operators
bool BoundaryPoint::operator<(const BoundaryPoint &bp2) const {
    return this->position[0] < bp2.position[0];
}

// Getters
glm::vec3 BoundaryPoint::getPos() {
    return this->position;
}

TYPE BoundaryPoint::getType() {
    return this->type;
}

CrowdPatch* BoundaryPoint::getParent() {
    return this->parentPatch;
}

BoundaryPoint* BoundaryPoint::getNeighbor() {
    return this->neighbor;
}

Trajectory* BoundaryPoint::getTrajectory() {
    return this->trajectory;
}

STATUS BoundaryPoint::getStatus() {
    return this->status;
}

BoundaryPoint* BoundaryPoint::getCurrentMatch() {
    return this->currentMatch;
}

std::vector<BoundaryPoint*> BoundaryPoint::getPrefList() {
    return this->preferenceList;
}

BoundaryPoint BoundaryPoint::getPointAt(int i) {
    return *this->preferenceList.at(i);
}

// Get rating of bp from map
float BoundaryPoint::getRating(BoundaryPoint *bp) const {
    return this->ratingMap.at(bp);
}

// Setters
void BoundaryPoint::setPos(glm::vec3 position) {
    this->position = position;
}

void BoundaryPoint::setType(TYPE newType) {
    this->type = newType;
}

void BoundaryPoint::setNeighbor(BoundaryPoint *n) {
    this->neighbor = n;
}

void BoundaryPoint::setParent(CrowdPatch *p) {
    this->parentPatch = p;
}

void BoundaryPoint::setTrajectory(Trajectory *t) {
    this->trajectory = t;
}

void BoundaryPoint::setStatus(STATUS newStatus) {
    this->status = newStatus;
}

void BoundaryPoint::setDone(bool done) {
    this->done = done;
}

bool BoundaryPoint::setCurrentMatch(BoundaryPoint *newMatch) {
    if (newMatch == nullptr) {
        this->currentMatch = nullptr;
        return true;
    }
    if (newMatch->type != this->type) {
        this->currentMatch = newMatch;
        return true;
    } else {
        return false;
    }
}

// Sort compare function
struct doCompare
   {
       doCompare( const BoundaryPoint& bp ) : m_bp(bp) { } // only if you really need the object state
       const BoundaryPoint& m_bp;

       bool operator()(BoundaryPoint* & i1, BoundaryPoint* & i2  )
       {
            // comparison code using m_info
           return m_bp.getRating(i1) < m_bp.getRating(i2);
       }
   };

// Modifiers
void BoundaryPoint::insertBoundaryPoint(BoundaryPoint* bp) {
    this->preferenceList.push_back(bp);
}

void BoundaryPoint::sortPreferenceList() {
    // sort preference list with custom compare function
    std::sort(this->preferenceList.begin(), this->preferenceList.end(), doCompare(*this));
}

void BoundaryPoint::incrementCounter() {
    this->counter++;
    if (this->counter >= int(this->preferenceList.size()) - 1) {
        this->done = true;
    }
}

// Helper
bool isOn(BoundaryPoint* bp, float wall, int coord) {
    return abs(bp->getPos()[coord] - wall) < 0.001;
}

// Operations
float BoundaryPoint::findPenalty(BoundaryPoint *bp) {
    glm::vec2 cpOrigin = this->parentPatch->getOrigin();
    float cpWidth = this->parentPatch->getWidth();

    float xmin = cpOrigin[0];
    float xmax = cpOrigin[0] + cpWidth;

    float ymin = cpOrigin[1];
    float ymax = cpOrigin[1] + cpWidth;

    if (isOn(this, xmin, 0)) {
        // this point is on xmin
        if (isOn(bp, xmax, 0)) {
            // if bp is on opposite wall
            return 0.0;
        }

        if (isOn(bp, xmin, 0)) {
            // if bp is on xmin
            return 8.0;
        }

        if (isOn(bp, ymin, 1) || isOn(bp, ymax, 1)) {
            return 2.0;
        }
    }


    if (isOn(this, xmax, 0)) {
        // this point is on xmax
        if (isOn(bp, xmin, 0)) {
            // if bp is on opposite wall
            return 0.0;
        }

        if (isOn(bp, xmax, 0)) {
            // if bp is also on xmax
            return 8.0;
        }

        if (isOn(bp, ymin, 1) || isOn(bp, ymax, 1)) {
            return 2.0;
        }
    }

    if (isOn(this, ymin, 1)) {
        // this point is on ymin
        if (isOn(bp, ymax, 1)) {
            // if bp is on opposite wall
            return 0.0;
        }

        if (isOn(bp, ymin, 1)) {
            // if bp is also on ymin
            return 8.0;
        }

        if (isOn(bp, xmin, 0) || isOn(bp, xmax, 0)) {
            return 2.0;
        }
    }


    if (isOn(this, ymax, 1)) {
        // this point is on ymax
        if (isOn(bp, ymin, 1)) {
            // if bp is on opposite wall
            return 0.0;
        }

        if (isOn(bp, ymax, 1)) {
            // if bp is also on ymax
            return 8.0;
        }

        if (isOn(bp, xmin, 0) || isOn(bp, xmax, 0)) {
            return 2.0;
        }
    }


    return 0.0;



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
    glm::vec2 vel = tempT.getVelocity();
    glm::vec2 dir = glm::normalize(vel);
    glm::vec2 desiredVel = this->parentPatch->getDesiredDirection();
    float difference = glm::length(desiredVel - dir);

    float uMatch = atan(difference);

    float p = this->findPenalty(bp);

    p += tempT.getSpeed() / 2.0;

    return uMatch + p;
}

void BoundaryPoint::fillRatingsMap() {
    if (this->type == ENTRY) {
        for (BoundaryPoint* bp : this->parentPatch->getExitBPs()) {
            this->insertBoundaryPoint(bp);
            this->ratingMap.insert(std::pair<BoundaryPoint*,float>(bp, this->calculateRating(bp)));
        }
    } else {
        for (BoundaryPoint* bp : this->parentPatch->getEntryBPs()) {
            this->insertBoundaryPoint(bp);
            this->ratingMap.insert(std::pair<BoundaryPoint*,float>(bp, this->calculateRating(bp)));
        }
    }
}

void BoundaryPoint::resetCounter() {
    this->counter = 0;
}

//bool BoundaryPoint::createTrajectory(Trajectory *T) {
//    if (this->type == ENTRY) {
//        if (this->currentMatch == nullptr) {
//            return false;
//        }
//        T.insertControlPoint(this->position, 0);
//        T.insertControlPoint(this->currentMatch->position, 1);
//        T.setEntryPoint(this);
//        T.setExitPoint(this->currentMatch);
//        this->setTrajectory(&T);
//        this->currentMatch->setTrajectory(&T);
//        return true;
//    } else {
//        return false;
//    }
//}

Trajectory* BoundaryPoint::createTrajectory() {
    if (this->type == ENTRY) {
        if (this->currentMatch == nullptr) {
            return nullptr;
        }
        Trajectory* T = new Trajectory();
        T->insertControlPoint(this->position, 0);
        T->insertControlPoint(this->currentMatch->position, 1);
        T->setEntryPoint(this);
        T->setExitPoint(this->currentMatch);
//        if (this->position[2] < 0.0001) {
//            T->setStarting(true);
//        }
        this->setTrajectory(T);
        if (this->currentMatch->getTrajectory() != nullptr) {
            int dhuaisdf = 1;
            dhuaisdf++;
        }
        this->currentMatch->setTrajectory(T);
        return T;
    } else {
        return nullptr;
    }
}

BoundaryPoint* BoundaryPoint::getNextProposal() {
    if (counter < int(this->preferenceList.size())) {
        incrementCounter();
        return this->preferenceList.at(counter - 1);
    } else {
        return nullptr;
    }
}

void BoundaryPoint::clear() {
    this->trajectory = nullptr;
    this->status = UNMATCHED;
    this->currentMatch = nullptr;
    this->counter = 0;
    this->done = false;
    this->preferenceList.clear();
    this->ratingMap.clear();

}
