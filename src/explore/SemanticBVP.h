#ifndef SEMANTICBVP_H
#define SEMANTICBVP_H

#include <vector>
#include <queue>
#include <iostream>

#include "BVP.h"
#include "Doors.h"
#include "SomeKernels.h"
#include "Configuration.h"
#include "Grid.h"

#include <iomanip>
#include <cmath>
#include <iostream>
#include <utility>

class SemanticBVP : public VoronoiBVP
{
public:
    SemanticBVP(Configuration* config);

    void initialize(Grid* grid);
    void updateRoomsGraph(Doors& doors);

    void updateGridSegments(Grid *grid, const robotCell &robotInVoronoi, Segment *seg);
    void updateVisited(Grid *grid, const robotCell &robotInVoronoi);
    void updateParity(Grid *grid, const robotCell &robotInVoronoi, Doors& doors);
    void updateDistancesToGoalNumber(Grid *grid, const robotCell &robotInVoronoi, Doors& doors);

    bool computeDistanceToUnknownCellsUsingNumbers(Grid *grid, const robotCell &curPose);
    void segmentClassification(Grid *grid, const robotCell &curPose, std::set<Door *> &unionDoors);

    bool findLocalGoal(Grid *grid, const robotCell &robotInVoronoi);
    void printOrientRobotSawDoors();

    void extendVoronoi(std::list<std::pair<Cell*, int>> cellAndSegmentsGoals, Grid *grid);

    float orientation_attrativeness(Cell* goal);
    float orientation_Robot_attractiveness(Cell* goal);

    Segment* checkingLabelNeighboors(Cell *robotCell, Grid *grid);

    Segment* searchForSegment(int index);

    Segment *searchNearestSegmentofGoal(Cell* cellGoal, Grid *grid);

    robotCell findNearestCenterCell(Grid *grid, const robotCell &rc, bool checkInUnexplored = false);

    std::vector<int> goalNumber;

    bool founded_;

private:

    int max_goal_radius;

    Configuration* config_;

    Cell* goal;

    CKernel* visitedKernel;
    CKernel* circularKernel;
    CKernel* gaussianKernel;

    robotCell previous;

    Segment *seg;
    Grid mygrid;

    std::vector<float> histogramDoorsOrientation;
    std::vector<float> histogramRobotOrientation;

    std::list<std::pair<Cell*, Segment*>> cellSegmentOfGoals;
    std::list<std::pair<std::string,int> > validDoors;
    std::list<float> historyRobotOrientation;
    std::list<float> historyDoorsOrientation;
    std::list<Segment*> allSegments;

    int threshold_segment_;
    int sizeOffset;
    int min_vector_direction_;
    int amountOrientation;
    int amountOrientation2;
    int amountHistoryRobotOrientations;
    int amountHistoryDoorsOrientations;
    int totalAmountDoorsOrient;

    float w_alpha_;
    float w_parity_;
    float w_direction_;
    float w_difference_;
    float w_distance_;
    float w_orientation_;
    float w_semantic_;
    float w_geometric_;
    float w_tollerance_;
    float w_distanceGoal_;
    float w_historyOrientation_;
};

#endif // SEMANTICBVP_H
