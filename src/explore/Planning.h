#ifndef __PLANNING_H__
#define __PLANNING_H__

class Planning;

#include <pthread.h>
#include <queue>
#include "Robot.h"
#include "Grid.h"

#include "BVP.h"
#include "SemanticBVP.h"
#include "Configuration.h"

class Planning {
	public:
        Planning(Configuration* config);
        ~Planning();

        bool run();

        void initialize();

        void setNewRobotPose(Pose p);
        void setGrid(Grid* g);
        void setLocalRadius(int r);

        bool reached_door(int goal_number);

        Grid* grid;

        Doors doors;


        ExplorationKind exploration_kind() const;

        bool reached;

private:

        BVP bvp;
        BVP_mod bvp_mod;
        VoronoiBVP voronoiBVP;
        SemanticBVP* semanticBVP;


        void expandObstacles();
        void markBorders();
        void updateCellsTypes();
        void updateCellsTypesUsingSLAM();
        void updateCenterCells(bool removeSpurs, bool voronoiInUnexplored, bool useVoronoiHelper);

        int iteration;
        int radius;

        robotCell curPose;
        bbox curLimits;
        robotCell prevPose;

        robotCell newPose;
        bbox newLimits;

        ExplorationKind exploration_kind_;
        int goal_number;

        float last_norm;

};


#endif /* __PLANNING_H__ */
