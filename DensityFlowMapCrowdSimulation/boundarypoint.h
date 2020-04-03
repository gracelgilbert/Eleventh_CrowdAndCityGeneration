#ifndef BOUNDARYPOINT_H
#define BOUNDARYPOINT_H

#include <map>
#include <math.h>
#include "la.h"
#include "trajectory.h"
#include "crowdpatch.h"

class CrowdPatch;
class Trajectory;

// Specifies type of boundary point, entry or exit
enum TYPE {
    ENTRY,
    EXIT
};

// Specifies state of boundary point, matched or free
enum STATUS {
    MATCHED,
    UNMATCHED
};

class BoundaryPoint {
/*
 * A boundary point is either an input or output point of a crowd patch. They have a position
 * on the border of their parent patch.
 *
 * Entry and exit boundary points are paired up to form trajectories using stable matching.
 * Each one computes a rating for all boundary points of opposite type based how close
 * the trajectory's velocity would align with the parent patch's desired direction. The rating
 * is penalized if the two points are on adjacent or the same borders of the patch.
 *
 * The boundary point then sorts the opposite type points according to their ratings, and this
 * is the preference list used in the stable matching.
 */
private:
    // Descriptors
    glm::vec3 position;
    TYPE type;
    CrowdPatch* parentPatch;
    BoundaryPoint* neighbor;
    Trajectory* trajectory;

    // State
    STATUS status;
    BoundaryPoint* currentMatch;
    int counter;
    bool done;

    // Preference list and ratings
    std::vector<BoundaryPoint*> preferenceList;
    std::map<BoundaryPoint*, float> ratingMap;

public:
    // Constructors and destructors
    BoundaryPoint(glm::vec3 pos, TYPE type, CrowdPatch* parentPatch);
    ~BoundaryPoint();

    // Operators
    bool operator<(const BoundaryPoint &bp2) const;

    // Getters
    glm::vec3 getPos();
    TYPE getType();
    CrowdPatch* getParent();
    BoundaryPoint* getNeighbor();
    Trajectory* getTrajectory();

    STATUS getStatus();
    BoundaryPoint* getCurrentMatch();

    std::vector<BoundaryPoint*> getPrefList();
    BoundaryPoint getPointAt(int i);

    // Get rating of bp from map
    float getRating(BoundaryPoint* bp) const;

    // Setters
    void setPos(glm::vec3 position);
    void setType(TYPE newType);
    void setNeighbor(BoundaryPoint* n);
    void setParent(CrowdPatch* p);
    void setTrajectory(Trajectory* t);

    void setStatus(STATUS newStatus);
    bool setCurrentMatch(BoundaryPoint* newMatch);

    // Modifiers
    void insertBoundaryPoint(BoundaryPoint* bp);
    void sortPreferenceList();
    void incrementCounter();

    // Operations
    float findPenalty(BoundaryPoint* bp);
    float calculateRating(BoundaryPoint* bp);
    void fillRatingsMap();
    bool createTrajectory(Trajectory& T);
    BoundaryPoint* getNextProposal();

};

#endif // BOUNDARYPOINT_H
