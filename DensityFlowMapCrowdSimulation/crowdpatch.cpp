#include "crowdpatch.h"
#include <glm/gtx/matrix_transform_2d.hpp>
// Constructors
CrowdPatch::CrowdPatch() :
    origin(glm::vec2()),
    width(1.0),
    period(1.0),
    S(std::vector<int>()),
    D(std::vector<int>()),
    visited(false),
    distance(FLT_MAX),
    desiredDensity(1.0),
    desiredDirection(glm::vec2(1.0)),
    error(1.0),
    entryBPs(std::vector<BoundaryPoint*>()),
    exitBPs(std::vector<BoundaryPoint*>()),
    trajectories(std::vector<Trajectory>()),
    parent(nullptr),
    left(nullptr),
    right(nullptr),
    up(nullptr),
    down(nullptr)
{ }

CrowdPatch::CrowdPatch(glm::vec2 origin, float width, float period,
                       float desiredDensity, glm::vec2 desiredDirection) :
    origin(origin),
    width(width),
    period(period),
    S(std::vector<int>()),
    D(std::vector<int>()),
    visited(false),
    distance(FLT_MAX),
    desiredDensity(desiredDensity),
    desiredDirection(glm::vec2(desiredDirection)),
    error(1.0),
    entryBPs(std::vector<BoundaryPoint*>()),
    exitBPs(std::vector<BoundaryPoint*>()),
    trajectories(std::vector<Trajectory>()),
    parent(nullptr),
    left(nullptr),
    right(nullptr),
    up(nullptr),
    down(nullptr)
{ }

// Destructor
CrowdPatch::~CrowdPatch() {
//    for (BoundaryPoint* bp : this->entryBPs) {
////        delete(bp);
//    }

//    for (BoundaryPoint* bp : this->exitBPs) {
////        delete(bp);
//    }
}

// Getters
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

bool CrowdPatch::isVisited() {
    return this->visited;
}

float CrowdPatch::getDistance() {
    return this->distance;
}

CrowdPatch* CrowdPatch::getParent() {
    return this->parent;
}

float CrowdPatch::getError() {
    return this->error;
}

float CrowdPatch::getDesiredDensity() {
    return desiredDensity;
}

glm::vec2 CrowdPatch::getDesiredDirection() {
    if (glm::length(desiredDirection) < 0.01) {
        return glm::vec2(0.0);
    }
    return glm::normalize(desiredDirection);
}

std::vector<BoundaryPoint*> CrowdPatch::getEntryBPs() {
    return this->entryBPs;
}

std::vector<BoundaryPoint*> CrowdPatch::getExitBPs() {
    return this->exitBPs;
}

std::vector<Trajectory> CrowdPatch::getTrajectories() {
    return this->trajectories;
}

Trajectory* CrowdPatch::getTrajectoryAt(int index) {
    return &this->trajectories.at(index);
}

// Setters

void CrowdPatch::setDesiredDensity(float d) {
    this->desiredDensity = d;
}

void CrowdPatch::setDesiredDirection(glm::vec2 dir) {
    this->desiredDirection = glm::normalize(dir);
}

void CrowdPatch::setVistState(bool state) {
    this->visited = state;
}

void CrowdPatch::setDistance(float distance) {
    this->distance = distance;
}

void CrowdPatch::setParent(CrowdPatch *p) {
    this->parent = p;
}

// Modifiers

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

void CrowdPatch::removeEntry(BoundaryPoint *bp) {
    for (int i = 0; i < this->entryBPs.size(); ++i) {
        if (this->entryBPs.at(i) == bp) {
            entryBPs.erase(entryBPs.begin() + i);
            return;
        }
    }
}

void CrowdPatch::removeExit(BoundaryPoint *bp) {
    for (int i = 0; i < this->exitBPs.size(); ++i) {
        if (this->exitBPs.at(i) == bp) {
            exitBPs.erase(exitBPs.begin() + i);
            return;
        }
    }
}

void CrowdPatch::addEntry(BoundaryPoint *bp) {
    this->entryBPs.push_back(bp);
}

void CrowdPatch::addExit(BoundaryPoint *bp) {
    this->exitBPs.push_back(bp);
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

void CrowdPatch::addTrajectory(Trajectory T) {
    this->trajectories.push_back(T);
}

void CrowdPatch::reset() {
    this->visited = false;
    this->distance = FLT_MAX;
    this->parent = nullptr;
}

// Operations

void CrowdPatch::removeCollisions() {
    int numTrajectories = this->trajectories.size();
    if (numTrajectories > 1) {
        // Remove Collisions:
//        float M[numTrajectories * numTrajectories];
        float minAllowedDistance = 40.f;
        int counter = 0;
        float minOverallDist = 0.f;

        while (minOverallDist < minAllowedDistance && counter < 100) {
            // Will get out minimum distance and the trajectory points that got this distance
            float minDist = FLT_MAX; // Min dist between two trajectories
            glm::vec3 minCPx = glm::vec3(0.0); // Position/time on trajectory x of min dist
            glm::vec3 minCPy = glm::vec3(0.0); // Position/time on trajectory y of min dist
            int minx = 0; // index of trajectory x with min dist
            int miny = 0; // index of trajectory y with min dist
            int minSegFirstx = 0; // index of first cp of segment on trajectory x with min dist
            int minSegFirsty = 0; // index of first cp of segment on trajectory y with min dist


            // Iterate over all trajectories
            for (int x = 0; x < numTrajectories; ++x) {
                for (int y = 0; y < numTrajectories; ++y) {
                    // Find current shortest distance between trajectory x and y
//                    int arrayIndex = x + numTrajectories * y;
                    if (y <= x) {
                        // trajectories are the same, minimum distance is 0
                        // Or already checked this pair
                        continue;
//                        M[arrayIndex] = 0;
                    } else {
                        // Get the two trajectories
                        Trajectory T1 = this->trajectories.at(x);
                        Trajectory T2 = this->trajectories.at(y);
                        // Set up variables to get out of trajectory pair
                        glm::vec3 cpx; // position of collision
                        glm::vec3 cpy;
                        int segFirstx = 0; // index of segment of collision
                        int segFirsty = 0;
                        float currTrajectoryPairDist = 0.f; // distance of collision

                        if (this->minDist(T1, T2, cpx, cpy, segFirstx, segFirsty, currTrajectoryPairDist)) {
//                        M[arrayIndex] = currTrajectoryPairDist;
                            if (currTrajectoryPairDist < minDist) {
                                minDist = currTrajectoryPairDist;
                                minCPx = cpx;
                                minCPy = cpy;
                                minx = x;
                                miny = y;
                                minSegFirstx = segFirstx;
                                minSegFirsty = segFirsty;
                            }
                        }
                    }
                }
            }
            minOverallDist = minDist;

            if (minOverallDist < minAllowedDistance) {
                glm::vec3 dPxy = glm::normalize(minCPx - minCPy);
                Trajectory xTraj = this->trajectories.at(minx);
                Trajectory yTraj = this->trajectories.at(miny);

                float alpha = std::min(minAllowedDistance * 2.0, 2.0);

                float xSpeed = xTraj.getSpeed();
                float ySpeed = yTraj.getSpeed();

                float wx = (ySpeed / (xSpeed + ySpeed));
                float wy = (xSpeed / (xSpeed + ySpeed));

                float phi =  (static_cast <float> (std::rand()) / (static_cast <float> (RAND_MAX))) - 0.5;
                glm::mat3 rot = glm::rotate(glm::mat3(), phi);

                glm::vec3 Fx = rot * dPxy * alpha;  // /** minAllowedDistance * minAllowedDistance*/;

                glm::vec3 pxNew = minCPx + Fx * wx;
                glm::vec3 pyNew = minCPy - Fx * wy;

                float timex = xTraj.getTimeAtIndex(minSegFirstx) + xTraj.getTimeAtIndex(minSegFirstx + 1);
                timex *= 0.5;
                float timey = yTraj.getTimeAtIndex(minSegFirsty) + yTraj.getTimeAtIndex(minSegFirsty + 1);
                timey *= 0.5;
//                pxNew[2] = 0.5 * (pxNew[2] + timex);
//                pyNew[2] = 0.5 * (pyNew[2] + timey);
                pxNew[2] = timex;
                pyNew[2] = timey;


                this->trajectories.at(minx).insertControlPoint(pxNew, minSegFirstx + 1);
                this->trajectories.at(miny).insertControlPoint(pyNew, minSegFirsty + 1);
            }
            counter++;
        }
    }
}

bool CrowdPatch::minDist(Trajectory T1, Trajectory T2,
                         glm::vec3 &cp1, glm::vec3 &cp2,
                         int &segFirst1, int &segFirst2,
                         float &dist) {
    int period = this->getPeriod();

    int numControlPoints1 = T1.getNumControlPoints();
    int numControlPoints2 = T2.getNumControlPoints();

    glm::vec3 T1Start = T1.getControlPointAtIndex(0);
    glm::vec3 T2Start = T2.getControlPointAtIndex(0);

    glm::vec3 T1End = T1.getControlPointAtIndex(numControlPoints1 - 1);
    glm::vec3 T2End = T2.getControlPointAtIndex(numControlPoints2 - 1);

    // If trajectories don't overlap at all, return false

//    if (T1Start[2] > T1End[2]) {
//        if (!(T2Start[2] <= T1End[2] || T2Start[2] >= T1Start[2] ||
//                T2End[2] <= T1End[2] || T2End[2] >= T1Start[2])) {
//            return false;
//        }
//    }

//    if (T2Start[2] > T2End[2]) {
//        if (!(T1Start[2] <= T2End[2] || T1Start[2] >= T2Start[2] ||
//                T1End[2] <= T2End[2] || T1End[2] >= T2Start[2])) {
//            return false;
//        }
//    }

//    if (T1Start[2] < T1End[2] && T2Start[2] < T2End[2]) {
        if (T1Start[2] > T2End[2] || T2Start[2] > T1End[2]) {
            return false;
        }
//    }



    float minDist = FLT_MAX;
    for (int s1 = 0; s1 < numControlPoints1 - 1; ++s1) {
        for (int s2 = 0; s2 < numControlPoints2 - 1; ++s2) {
            // Iterate over all segment pairs
            // Find endpoints of both segments
            glm::vec2 seg1Start = T1.getPositionAtIndex(s1);
            glm::vec2 seg1End = T1.getPositionAtIndex(s1 + 1);
            float ts1 = T1.getTimeAtIndex(s1);
            float te1 = T1.getTimeAtIndex(s1 + 1);


            glm::vec2 seg2Start = T2.getPositionAtIndex(s2);
            glm::vec2 seg2End = T2.getPositionAtIndex(s2 + 1);
            float ts2 = T2.getTimeAtIndex(s2);
            float te2 = T2.getTimeAtIndex(s2 + 1);



            // If segment pair does not overlap, skip over it:
//            if (ts1 > te2 || ts2 > te1) {
//                continue;
//            }
//            if (ts1 > te1) {
//                if (!(ts2 <= te1 || ts2 >= ts1 ||
//                        te2 <= te1 || te2 >= ts1)) {
//                    continue;
//                }
//            }

//            if (ts2 > te2) {
//                if (!(ts1 <= te2 || ts1 >= ts2 ||
//                        te1 <= te2 || te1 >= ts2)) {
//                    continue;
//                }
//            }

//            if (ts1 <= te1 && ts2 <= te2) {
                if (ts1 > te2 || ts2 > te1) {
                    continue;
                }
//            }



            float ts = std::max(ts1, ts2);
            float te = std::min(te1, te2);

//            if (ts1 <= te1 && ts2 <= te2) {
//                // Both are start then finish
//                ts = std::max(ts1, ts2);
//                te = std::min(te1, te2);
//            }
//            else if (ts1 > te1) {
//                if (ts2 > te2) {
//                    // Both are finish then start
//                    te1 += period;
//                    te2 += period;
//                    ts = std::max(ts1, ts2);
//                    te = std::min(te1, te2);
//                } else {
//                    // Only first section is over period
//                    te1 += period;


//                }
//            } else {
//                // Only second section is over period
//            }

            glm::vec2 v1 = (1.f / (te1 - ts1)) * (seg1End - seg1Start);
            glm::vec2 v2 = (1.f / (te2 - ts2)) * (seg2End - seg2Start);


            glm::vec2 ps1 = seg1Start + ts * v1;
            glm::vec2 ps2 = seg2Start + ts * v2;

            glm::vec2 pe1 = seg1Start + te * v1;
            glm::vec2 pe2 = seg2Start + te * v2;

            glm::vec2 w = ps1 - ps2;
            glm::vec2 dv = v1 - v2;

            float dist = 0.f;

            glm::vec3 tempcp1 = glm::vec3(0.0);
            glm::vec3 tempcp2 = glm::vec3(0.0);

            if (glm::length2(dv) < 0.001) {
                dist = glm::length(ps1 - ps2);
                tempcp1 = glm::vec3(ps1[0], ps1[1], ts);
                tempcp2 = glm::vec3(ps2[0], ps2[1], ts);
            } else {
                float tc = glm::dot(-w, dv) / (glm::length(dv) * glm::length(dv));
                if (0 <= tc && tc <= te - ts) {
                    dist = glm::length(w + tc * dv);
                    tempcp1 = glm::vec3((seg1Start + tc * v1)[0], (seg1Start + tc * v1)[1], tc);
                    tempcp2 = glm::vec3((seg2Start + tc * v2)[0], (seg2Start + tc * v2)[1], tc);

                } else {
                    float dist00 = glm::length(ps1 - ps2);
                    float dist11 = glm::length(pe1 - pe2);
                    dist = std::min(dist00, dist11);
                    if (dist00 < dist11) {
                        tempcp1 = glm::vec3(ps1[0], ps1[1], ts);
                        tempcp2 = glm::vec3(ps2[0], ps2[1], ts);
                    } else {
                        tempcp1 = glm::vec3(pe1[0], pe1[1], te);
                        tempcp2 = glm::vec3(pe2[0], pe2[1], te);
                    }
                }
            }

            if (dist < minDist) {
                minDist = dist;
                cp1 = tempcp1;
                cp2 = tempcp2;
                segFirst1 = s1;
                segFirst2 = s2;
            }
        }
    }
    dist = minDist;
    return true;

}

void CrowdPatch::matchBoundaryPoints() {
    for (BoundaryPoint* bp : entryBPs) {
        bp->fillRatingsMap();
        bp->sortPreferenceList();
    }
    for (BoundaryPoint* bp : exitBPs) {
        bp->fillRatingsMap();
        bp->sortPreferenceList();
    }
    if (entryBPs.size() != exitBPs.size()) {
        return;
        std::cout << "help" << std::endl;
    }

    // Perform matching
    bool notDone = true;
    while (notDone) {
        notDone = false;
        for (BoundaryPoint* i : this->entryBPs) {
            if (i->getStatus() == UNMATCHED) {
                BoundaryPoint* o = i->getNextProposal();
                if (o != nullptr) {
                    // i proposes to o, next on preference list
                    notDone = true; // if there was a next BP to propose to for any i, then not done
                    if (o->getStatus() == UNMATCHED) {
                        // o is unmatched
                        i->setCurrentMatch(o);
                        i->setStatus(MATCHED);
                        o->setCurrentMatch(i);
                        o->setStatus(MATCHED);
                    } else {
                        // o is matched already
                        BoundaryPoint* iPrime = o->getCurrentMatch();
                        if (!iPrime) {
                            std::cout << "error: o is MATCHED and match is null" << std::endl;
                            continue;
                        }
                        float iRank = o->getRating(i);
                        float iPrimeRank = o->getRating(iPrime);
                        if (iRank < iPrimeRank) {
                            // o is with i prime but prefers i
                            i->setCurrentMatch(o);
                            i->setStatus(MATCHED);
                            o->setCurrentMatch(i);
                            o->setStatus(MATCHED);

                            iPrime->setCurrentMatch(nullptr);
                            iPrime->setStatus(UNMATCHED);
                        }
                    }
                }
            }
        }
    }

    for (BoundaryPoint* bp : entryBPs) {
        Trajectory T = Trajectory();
        if (bp->createTrajectory(T)) {
            T.setParent(this);
            this->trajectories.push_back(T);
        }
    }
}

float CrowdPatch::calculateDensity() {
    // iterate over patch's period with small dt, find density at that dt and sum
    float volume = this->width * this->width * this->period;
    float density = 0.f;

    float dt = 1.0;
    for (int t = 0; t < this->period; t += dt) {
        float tDensity = 0.f;
        for (Trajectory T : this->trajectories) {
            int numControlPoints = T.getNumControlPoints();
            if (t > T.getTimeAtIndex(0) && t < T.getTimeAtIndex(numControlPoints - 1)) {
                tDensity++;
            }
        }
        density += tDensity * dt;
    }
    return density / volume;
}

glm::vec2 CrowdPatch::calculateFlow() {
    if (this->trajectories.size() < 1) {
        return glm::vec2(0.0);
    }
    glm::vec2 flow = glm::vec2(0.f);
    for (Trajectory T : this->trajectories) {
        glm::vec2 di = T.getDirection();
        float ti = T.getDuration();
        flow += ti * di;
    }
    return glm::normalize(flow);
}

CrowdPatch* CrowdPatch::getLeft() {
    return this->left;
}

CrowdPatch* CrowdPatch::getRight() {
    return this->right;
}

CrowdPatch* CrowdPatch::getUp() {
    return this->up;
}

CrowdPatch* CrowdPatch::getDown() {
    return this->down;
}

void CrowdPatch::setLeft(CrowdPatch* l) {
    this->left = l;
}

void CrowdPatch::setRight(CrowdPatch* r) {
    this->right = r;
}

void CrowdPatch::setUp(CrowdPatch* u) {
    this->up = u;
}

void CrowdPatch::setDown(CrowdPatch* d) {
    this->down = d;
}

float CrowdPatch::calculateDensityError() {
    return this->calculateDensity() - this->getDesiredDensity();
}

float CrowdPatch::calculateFlowError() {
    glm::vec2 desiredDir = this->getDesiredDirection();
    glm::vec2 flow = this->calculateFlow();
    float dot = glm::dot(flow, desiredDir);
    if (glm::abs(dot) < 0.01) {
        dot = 0;
    }
    return 0.5 * glm::abs(1.f - dot);
}

float CrowdPatch::calculateError() {
    float ep = this->calculateDensityError();
    float ed = this->calculateFlowError();
    float sign = 1.0;
    if (ep < 0) {
        sign = -1.0;
    }
    this->error = ep + sign * ed;
    return this->error;
}

float CrowdPatch::calculateNeighborWeight(CrowdPatch *cp) {
    float neighborError = cp->calculateError();
    float myError = this->calculateError();
    return glm::abs(neighborError - myError);
}


void CrowdPatch::clearVecs() {
//    this->entryBPs.clear();
//    this->exitBPs.clear();
    this->trajectories.clear();
}



std::vector<BoundaryPoint*> CrowdPatch::getLeftEntries() {
    return this->leftEntries;
}

std::vector<BoundaryPoint*> CrowdPatch::getRightEntries() {
    return this->rightEntries;
}

std::vector<BoundaryPoint*> CrowdPatch::getUpEntries() {
    return this->upEntries;
}

std::vector<BoundaryPoint*> CrowdPatch::getDownEntries() {
    return this->downEntries;
}


std::vector<BoundaryPoint*> CrowdPatch::getLeftExits() {
    return this->leftExits;
}

std::vector<BoundaryPoint*> CrowdPatch::getRightExits() {
    return this->rightExits;
}

std::vector<BoundaryPoint*> CrowdPatch::getUpExits() {
    return this->upExits;
}

std::vector<BoundaryPoint*> CrowdPatch::getDownExits() {
    return this->downExits;
}





void CrowdPatch::addLeftEntry(BoundaryPoint* bp) {
    this->leftEntries.push_back(bp);
}

void CrowdPatch::addRightEntry(BoundaryPoint* bp) {
    this->rightEntries.push_back(bp);
}

void CrowdPatch::addUpEntry(BoundaryPoint* bp) {
    this->upEntries.push_back(bp);
}

void CrowdPatch::addDownEntry(BoundaryPoint* bp) {
    this->downEntries.push_back(bp);
}


void CrowdPatch::addLeftExit(BoundaryPoint* bp) {
    this->leftExits.push_back(bp);
}

void CrowdPatch::addRightExit(BoundaryPoint* bp) {
    this->rightExits.push_back(bp);
}

void CrowdPatch::addUpExit(BoundaryPoint* bp) {
    this->upExits.push_back(bp);
}

void CrowdPatch::addDownExit(BoundaryPoint* bp) {
    this->downExits.push_back(bp);
}

void CrowdPatch::removeFromVector(std::vector<BoundaryPoint *> bpVec, BoundaryPoint *bp) {
    for (int i = 0; i < bpVec.size(); ++i) {
            if (bpVec.at(i) == bp) {
                bpVec.erase(bpVec.begin() + i);
                return;
            }
        }
}

void CrowdPatch::removeLeftEntry(BoundaryPoint* bp) {
    for (int i = 0; i < this->leftEntries.size(); ++i) {
        if (this->leftEntries.at(i) == bp) {
            leftEntries.erase(leftEntries.begin() + i);
            return;
        }
    }
}

void CrowdPatch::removeRightEntry(BoundaryPoint* bp) {
    for (int i = 0; i < this->rightEntries.size(); ++i) {
        if (this->rightEntries.at(i) == bp) {
            rightEntries.erase(rightEntries.begin() + i);
            return;
        }
    }
}

void CrowdPatch::removeUpEntry(BoundaryPoint* bp) {
    for (int i = 0; i < this->upEntries.size(); ++i) {
        if (this->upEntries.at(i) == bp) {
            upEntries.erase(upEntries.begin() + i);
            return;
        }
    }
}

void CrowdPatch::removeDownEntry(BoundaryPoint* bp) {
    for (int i = 0; i < this->downEntries.size(); ++i) {
        if (this->downEntries.at(i) == bp) {
            downEntries.erase(downEntries.begin() + i);
            return;
        }
    }
}

void CrowdPatch::removeLeftExit(BoundaryPoint* bp) {
    for (int i = 0; i < this->leftExits.size(); ++i) {
        if (this->leftExits.at(i) == bp) {
            leftExits.erase(leftExits.begin() + i);
            return;
        }
    }
}

void CrowdPatch::removeRightExit(BoundaryPoint* bp) {
    for (int i = 0; i < this->rightExits.size(); ++i) {
        if (this->rightExits.at(i) == bp) {
            rightExits.erase(rightExits.begin() + i);
            return;
        }
    }
}

void CrowdPatch::removeUpExit(BoundaryPoint* bp) {
    for (int i = 0; i < this->upExits.size(); ++i) {
        if (this->upExits.at(i) == bp) {
            upExits.erase(upExits.begin() + i);
            return;
        }
    }
}

void CrowdPatch::removeDownExit(BoundaryPoint* bp) {
    for (int i = 0; i < this->downExits.size(); ++i) {
        if (this->downExits.at(i) == bp) {
            downExits.erase(downExits.begin() + i);
            return;
        }
    }
}









