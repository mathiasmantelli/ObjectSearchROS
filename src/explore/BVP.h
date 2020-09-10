#ifndef BVP_H
#define BVP_H

#include "Grid.h"

class BVP
{
public:
    BVP();

    void initializePotential(Grid *grid, const robotCell &curPose, const int radius);
    double iteratePotential(Grid *grid, const bbox &curLimits);
    void updateGradient(Grid *grid, const bbox &curLimits);
};

class BVP_mod
{
public:
    BVP_mod();

    void initializePotential(Grid *grid, const robotCell &curPose, const int radius);
    double iteratePotential(Grid *grid, const bbox &curLimits);
    void updateGradient(Grid *grid, const bbox &curLimits);
};

class VoronoiBVP
{
public:
    VoronoiBVP();

    robotCell findNearestCenterCell(Grid *grid, const robotCell &rc, bool checkInUnexplored=false);
    bool computeDistanceToUnknownCells(Grid *grid, const robotCell &curPose);
    bool findLocalGoal(Grid *grid, const robotCell &robotInVoronoi);

    void initializePotential(Grid *grid, const robotCell &curPose);
    double iteratePotential(Grid *grid, const robotCell &curPose);
    void updateGradient(Grid *grid, const robotCell &curPose);

    int auxCounter;
    int halfInnerWindow;
    int minHalfInnerWindowSize;
    int localGoalRadius;
    int minGoalRadius;

    int segmentCounter;

    robotCell localGoal;


    //eight-neighbor offset
    int offset[8][2];

};

#endif // BVP_H
