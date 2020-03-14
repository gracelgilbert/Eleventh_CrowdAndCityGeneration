#ifndef BOUNDARYPOINT_H
#define BOUNDARYPOINT_H

#include "la.h"
#include "trajectory.h"
#include "crowdpatch.h"
#include <map>

class CrowdPatch;

enum TYPE {
    ENTRY,
    EXIT
};

enum STATUS {
    MATCHED,
    UNMATCHED
};

class BoundaryPoint {

private:
    glm::vec3 position;
    TYPE type;
    STATUS status;
    std::vector<BoundaryPoint> preferenceList;
    BoundaryPoint* currentMatch;
    CrowdPatch* parentPatch;
    std::map<BoundaryPoint*, float> ratingMap;


public:
    BoundaryPoint(glm::vec3 pos, TYPE type, CrowdPatch* parentPatch);

    bool operator<(const BoundaryPoint &bp2) const;

    glm::vec3 getPos();
    TYPE getType();
    STATUS getStatus();
    std::vector<BoundaryPoint> getPrefList();
    BoundaryPoint getPointAt(int i);
    BoundaryPoint* getCurrentMatch();

    void setPos(glm::vec3 position);
    void setType(TYPE newType);

    void insertBoundaryPoint(BoundaryPoint bp);

    void sortPreferenceList();

    bool setCurrentMatch(BoundaryPoint* newMatch);

    float findPenalty(BoundaryPoint* bp);

    bool createTrajectory(Trajectory& T);

    float calculateRating(BoundaryPoint* bp);

    void fillRatingsMap();

};





#endif // BOUNDARYPOINT_H
