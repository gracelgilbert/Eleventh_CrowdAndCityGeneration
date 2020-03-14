#include "crowdpatch.h"
#include <iostream>

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

CrowdPatch::~CrowdPatch() {
    for (BoundaryPoint* bp : this->entryBPs) {
        delete(bp);
    }

    for (BoundaryPoint* bp : this->exitBPs) {
        delete(bp);
    }
}

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

std::vector<BoundaryPoint*> CrowdPatch::getEntryBPs() {
    return this->entryBPs;
}

std::vector<BoundaryPoint*> CrowdPatch::getExitBPs() {
    return this->exitBPs;
}

void CrowdPatch::removeCollisions() {
    int numTrajectories = this->trajectories.size();
    if (numTrajectories > 1) {
        // Remove Collisions:
        float M[numTrajectories * numTrajectories];
        float minAllowedDistance = 25.f;
        int counter = 0;
        float minOverallDist = 0.f;

        while (minOverallDist < minAllowedDistance && counter < 100) {
            // Will get out minimum distance and the trajectory points that got this distance
            float minDist = FLT_MAX;
            glm::vec3 minCPx = glm::vec3(0.0);
            glm::vec3 minCPy = glm::vec3(0.0);
            int minx = 0;
            int miny = 0;
            int minSegFirstx = 0;
            int minSegFirsty = 0;


            // Iterate over all trajectories
            for (int x = 0; x < numTrajectories; ++x) {
                for (int y = 0; y < numTrajectories; ++y) {
                    // Find current shortest distance between trajectory x and y
                    int arrayIndex = x + numTrajectories * y;
                    if (x == y) {
                        // trajectories are the same, minimum distance is 0
                        M[arrayIndex] = 0;
                    } else {
                        Trajectory T1 = this->trajectories.at(x);
                        Trajectory T2 = this->trajectories.at(y);
                        glm::vec3 cpx;
                        glm::vec3 cpy;
                        int segFirstx = 0;
                        int segFirsty = 0;
                        float currTrajectoryPairDist = 0.f;

                        if (this->minDist(T1, T2, cpx, cpy, segFirstx, segFirsty, currTrajectoryPairDist))
                        M[arrayIndex] = currTrajectoryPairDist;
                        if (M[arrayIndex] < minDist) {
                            minDist = M[arrayIndex];
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
            minOverallDist = minDist;

            if (minOverallDist < minAllowedDistance) {
                glm::vec3 dPxy = glm::normalize(minCPx - minCPy);
                Trajectory xTraj = this->trajectories.at(minx);
                Trajectory yTraj = this->trajectories.at(miny);

                float xSpeed = xTraj.getSpeed();
                float ySpeed = yTraj.getSpeed();

                glm::vec3 Fx = dPxy * minAllowedDistance / 0.5f /** minAllowedDistance * minAllowedDistance*/;

                glm::vec3 pxNew = minCPx + Fx * (ySpeed / (xSpeed + ySpeed));
                glm::vec3 pyNew = minCPy - Fx * (xSpeed / (xSpeed + ySpeed));

                float timex = xTraj.getTimeAtIndex(minSegFirstx) + xTraj.getTimeAtIndex(minSegFirstx + 1);
                timex *= 0.5;
                float timey = yTraj.getTimeAtIndex(minSegFirsty) + yTraj.getTimeAtIndex(minSegFirsty + 1);
                timey *= 0.5;
                pxNew[2] = 0.5 * (pxNew[2] + timex);
                pyNew[2] = 0.5 * (pyNew[2] + timey);

                this->trajectories.at(minx).insertControlPoint(pxNew, minSegFirstx + 1);
                this->trajectories.at(miny).insertControlPoint(pyNew, minSegFirsty + 1);
                std::cout << pxNew[0] << ", " << pxNew[1] << ", " << pxNew[2] << std::endl;
            }
            counter++;
        }
    }
}


bool CrowdPatch::minDist(Trajectory T1, Trajectory T2,
                         glm::vec3 &cp1, glm::vec3 &cp2,
                         int &segFirst1, int &segFirst2,
                         float &dist) {

    int numControlPoints1 = T1.getNumControlPoints();
    int numControlPoints2 = T2.getNumControlPoints();

    glm::vec3 cpFirst1 = T1.getControlPointAtIndex(0);
    glm::vec3 cpFirst2 = T2.getControlPointAtIndex(0);

    glm::vec3 cpLast1 = T1.getControlPointAtIndex(numControlPoints1 - 1);
    glm::vec3 cpLast2 = T2.getControlPointAtIndex(numControlPoints2 - 1);

    // If trajectories don't overlap at all, return infinity
    if (cpFirst1[2] > cpLast2[2] || cpFirst2[2] > cpLast1[2]) {
        return false;
    }

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
            if (ts1 > te2 || ts2 > te1) {
                continue;
            }

            float ts = std::max(ts1, ts2);
            float te = std::min(te1, te2);

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










