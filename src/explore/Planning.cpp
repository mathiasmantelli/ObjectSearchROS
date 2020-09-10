#include <stack>
#include <math.h>
#include <iomanip>
#include <sstream>

#include "Planning.h"

#include "thinning.h"

//////////////////////
// Métodos Públicos //
//////////////////////
Planning::Planning(Configuration* config)
{
    if(config->GetString("exploration_kind") == "SEMANTIC_exp")
        exploration_kind_ = SEMANTIC_exp;
    else if(config->GetString("exploration_kind") == "VORONOIBVP_exp")
        exploration_kind_ = VORONOIBVP_exp;
    else if(config->GetString("exploration_kind") == "BVP_exp")
        exploration_kind_ = BVP_exp;
    else if(config->GetString("exploration_kind") == "BVP_MOD_exp")
        exploration_kind_ = BVP_MOD_exp;

    semanticBVP = new SemanticBVP(config);

    iteration=0;

    newPose.x = 0;
    newPose.y = 0;

    prevPose.x = -100;
    prevPose.y = -100;

    newLimits.minX = newLimits.minY = 1000;
    newLimits.maxX = newLimits.maxY = -1000;

    last_norm = 0;

    goal_number = config->GetInt("goal_number");

    reached = false;
}

Planning::~Planning()
{}

void Planning::setGrid(Grid *g)
{
    grid = g;
}

void Planning::setLocalRadius(int r)
{
    radius = 1.2*r*grid->getMapScale();
}

void Planning::setNewRobotPose(Pose p)
{

    newPose.x = (int)(p.x*grid->getMapScale());
    newPose.y = (int)(p.y*grid->getMapScale());
    newPose.theta = p.theta;

    newLimits.minX = std::min(newLimits.minX,newPose.x-radius);
    newLimits.maxX = std::max(newLimits.maxX,newPose.x+radius);
    newLimits.minY = std::min(newLimits.minY,newPose.y-radius);
    newLimits.maxY = std::max(newLimits.maxY,newPose.y+radius);
}

void Planning::initialize()
{
    semanticBVP->initialize(grid);

}

bool Planning::reached_door(int goal_number)
{
    Door *d;
    std::map<std::string,Door*>::iterator itDoor;

    //Checking whether the 'query door' is already known.
    for (itDoor=doors.doorsMap.begin(); itDoor!=doors.doorsMap.end(); ++itDoor){
        d = itDoor->second;
        if(d->number == goal_number)
           return true;
    }
    return false;
}

bool Planning::run()
{
    curPose = newPose;
    curLimits = newLimits;

    markBorders();
//    updateCellsTypes();
    updateCellsTypesUsingSLAM();
    expandObstacles();

    updateCenterCells(true,true,false);

    if(this->reached_door(goal_number))
    {
        reached = true;
        return false;
    }

    /************************************************************/
    /******** Explore using traditional potential (BVP) *********/
    /************************************************************/
    if(exploration_kind_ == BVP_exp)
    {
        bvp.initializePotential(grid,curPose,radius);

        for(int i=0; i<100; i++)
            bvp.iteratePotential(grid,curLimits) ;

        bvp.updateGradient(grid,curLimits);

        if(grid->getCell(curPose.x,curPose.y)->norm < 0.0000000001 && last_norm != 0)
            return false;
        else if(grid->getCell(curPose.x,curPose.y)->norm >= 0.0000000001)
            last_norm = grid->getCell(curPose.x,curPose.y)->norm;
    }

    /************************************************************/
    /************* Explore using MODIFIED potential *************/
    /************************************************************/
    else if(exploration_kind_ == BVP_MOD_exp)
    {
        semanticBVP->updateVisited(grid,curPose);

        bvp_mod.initializePotential(grid,curPose,radius);

        float variance = 0;
        for(int i=0; i<100; i++)
            variance += bvp_mod.iteratePotential(grid,curLimits) ;

        bvp_mod.updateGradient(grid,curLimits);

        if(grid->getCell(curPose.x,curPose.y)->norm < 0.0000000001 && last_norm != 0)
            return false;
        else if(grid->getCell(curPose.x,curPose.y)->norm >= 0.0000000001)
            last_norm = grid->getCell(curPose.x,curPose.y)->norm;
    }

    /************************************************************/
    /********** Explore using potential over Voronoi  ***********/
    /************************************************************/
    else if(exploration_kind_ == VORONOIBVP_exp)
    {
        if(voronoiBVP.computeDistanceToUnknownCells(grid,curPose)) return false;

        grid->robotVoronoiCell = voronoiBVP.findNearestCenterCell(grid,curPose);

        if(grid->robotVoronoiCell.x != UNDEF && grid->robotVoronoiCell.y != UNDEF){
            if( voronoiBVP.findLocalGoal(grid,grid->robotVoronoiCell) == true){
                voronoiBVP.initializePotential(grid,curPose);
                for(int i=0; i<100; i++)
                    voronoiBVP.iteratePotential(grid,curPose);
                voronoiBVP.updateGradient(grid,curPose);
            }
        }
    }

    /************************************************************/
    /***** Explore using Semantic information over Voronoi ******/
    /************************************************************/
    else if(exploration_kind_ == SEMANTIC_exp)
    {
        if(doors.newObservation){
            semanticBVP->updateRoomsGraph(doors);
        }

        if(prevPose.x!=curPose.x || prevPose.y!=curPose.y)
        {
            prevPose=curPose;
            semanticBVP->updateVisited(grid,curPose);
            semanticBVP->segmentClassification(grid, curPose, doors.unionDoors);
            semanticBVP->updateParity(grid,curPose,doors);
            semanticBVP->updateDistancesToGoalNumber(grid,curPose,doors);
        }

        if(semanticBVP->computeDistanceToUnknownCellsUsingNumbers(grid,curPose)) return false;

        grid->robotVoronoiCell = semanticBVP->findNearestCenterCell(grid,curPose);

        if(grid->robotVoronoiCell.x != UNDEF && grid->robotVoronoiCell.y != UNDEF){
            if( semanticBVP->findLocalGoal(grid,grid->robotVoronoiCell) == true){
                semanticBVP->initializePotential(grid,curPose);
                for(int i=0; i<100; i++)
                    semanticBVP->iteratePotential(grid,curPose);
                semanticBVP->updateGradient(grid,curPose);
            }
        }
    }

    iteration++;
    grid->planIteration = iteration;

    return true;
}

void Planning::markBorders()
{
    Cell* c;

    for(int i=curLimits.minX;i<=curLimits.maxX;i++){
        c = grid->getCell(i,curLimits.minY);
        c->border = iteration;

        c = grid->getCell(i,curLimits.maxY);
        c->border = iteration;
    }

    for(int j=curLimits.minY;j<=curLimits.maxY;j++){
        c = grid->getCell(curLimits.minX,j);
        c->border = iteration;

        c = grid->getCell(curLimits.maxX,j);
        c->border = iteration;
    }

}

void Planning::updateCellsTypes()
{
    int count=0;
    Cell* c;
    for(int i=newPose.x-radius;i<=newPose.x+radius;i++){
        for(int j=newPose.y-radius;j<=newPose.y+radius;j++){

            c = grid->getCell(i,j);
            if(c->type==UNEXPLORED){
                c->pot = 0.0;
                if(c->himm>8){
                    c->type = OCCUPIED;
                    c->pot = 1.0;
                }
                else if(c->himm<5){
                    c->type = FREE;
                    count++;
                }
            }
            else if(c->type == OCCUPIED && c->himm<5){
                c->type = FREE;
                count++;
            }
            else if((c->type == FREE || c->type == NEAROBSTACLE) && c->himm>8){
                c->type = OCCUPIED;
                c->pot = 1.0;
            }
        }
    }
}

void Planning::updateCellsTypesUsingSLAM()
{
    std::cout << "AQUI\n\n\n";
    int largeRadius = 3*radius;
    int count=0;
    Cell* c;
    for(int i=newPose.x-largeRadius;i<=newPose.x+largeRadius;i++){
        for(int j=newPose.y-largeRadius;j<=newPose.y+largeRadius;j++){

            c = grid->getCell(i,j);
            if(c->slamValue<0){
                c->type = UNEXPLORED;
                c->pot = 0.0;
            }else if(c->slamValue < 70){
                c->type = FREE;
                count++;
            }else if(c->slamValue > 80){
                c->type = OCCUPIED;
                c->pot = 1.0;
            }

//            if(c->type==UNEXPLORED){
//                c->pot = 0.0;
//                if(c->slamValue>80){
//                    c->type = OCCUPIED;
//                    c->pot = 1.0;
//                }
//                else if(c->slamValue>0 && c->slamValue<50){
//                    c->type = FREE;
//                    count++;
//                }
//            }
//            else if(c->type == OCCUPIED){
//                if(c->slamValue<0){
//                    c->type = UNEXPLORED;
//                }else if(c->slamValue<50){
//                    c->type = FREE;
//                    count++;
//                }
//            }
//            else if(c->type == FREE){
//                if(c->slamValue<0){
//                    c->type = UNEXPLORED;
//                }else if(c->slamValue>80){
//                    c->type = OCCUPIED;
//                    c->pot = 1.0;
//                }
//            }
        }
    }

//    std::cout << "COUNT " << count << std::endl;
}

void Planning::expandObstacles()
{
    int width=2;
    Cell *c, *n;

    for(int i=curPose.x-radius;i<=curPose.x+radius;i++){
        for(int j=curPose.y-radius;j<=curPose.y+radius;j++){
            c = grid->getCell(i,j);
            if(c->type == NEAROBSTACLE)
                c->type = FREE;
        }
    }

    for(int i=curPose.x-radius;i<=curPose.x+radius;i++){
        for(int j=curPose.y-radius;j<=curPose.y+radius;j++){
            c = grid->getCell(i,j);

            if(c->type == OCCUPIED){
                for(int x=i-width;x<=i+width;x++){
                    for(int y=j-width;y<=j+width;y++){
                        n = grid->getCell(x,y);
                        if(n->type == FREE){
                            n->type = NEAROBSTACLE;
                        }
                    }
                }
            }

        }
    }

//    for(int i=curPose.x-radius;i<=curPose.x+radius;i++){
//        for(int j=curPose.y-radius;j<=curPose.y+radius;j++){
//            c = grid->getCell(i,j);

//            if(c->type == NEAROBSTACLE)
//                c->type = OCCUPIED;
//        }
//    }
}

void Planning::updateCenterCells(bool removeSpurs, bool voronoiInUnexplored, bool useVoronoiHelper)
{
    Cell *c;
    int externalBorder = 1;

    int width = curLimits.maxX - curLimits.minX+1 + 2*externalBorder;
    int height = curLimits.maxY - curLimits.minY+1 + 2*externalBorder;
    unsigned char* map;
    map = new unsigned char[width*height*3];
    unsigned char* result;
    result = new unsigned char[width*height*3];

    int counter=0;
    int cellsT=0, cellsF=0;
    // x esq->dir - y cima->baixo
    for (int y = curLimits.maxY+externalBorder; y >= curLimits.minY-externalBorder; y--){
        for (int x = curLimits.minX-externalBorder; x <= curLimits.maxX+externalBorder; x++){

            c=grid->getCell(x,y);

            if(c->type == OCCUPIED || c->type == NEAROBSTACLE ||
               (!voronoiInUnexplored && c->type == UNEXPLORED) ||
               (useVoronoiHelper && c->type == UNEXPLORED && c->isVoronoiHelper) ||
               x==curLimits.maxX+externalBorder || x==curLimits.minX-externalBorder ||
               y==curLimits.maxY+externalBorder || y==curLimits.minY-externalBorder )
                //0x00
            {
                map[counter]=0x00;
                map[counter+1]=0x00;
                map[counter+2]=0x00;
            }
            else
                //0xFF
            {
                map[counter]=0xFF;
                map[counter+1]=0xFF;
                map[counter+2]=0xFF;
            }
            counter+=3;
        }
    }

    Thinning t;
    t.thinningGUOandHALL(map, result, width, height);
    if(removeSpurs)
        t.spurRemoval(20, result, width, height);

    // x esq->dir - y cima->baixo
    // Let's copy the thinning to the map
    counter=0;
    cellsT=0, cellsF=0;
    for (int y = curLimits.maxY+externalBorder; y >= curLimits.minY-externalBorder; y--){
        for (int x = curLimits.minX-externalBorder; x <= curLimits.maxX+externalBorder; x++){

            c=grid->getCell(x,y);

            if(result[counter]==0xFF)
            {
                c->isCenter=true;
                c->isEndPoint=false;
            }
            else if(result[counter]==0x80)
            {
                c->isCenter=false;
                c->isEndPoint=true;
            }
            else
            {
                c->isCenter=false;
                c->isEndPoint=false;
            }

            counter+=3;
        }
    }
    delete [] map;
    delete [] result;

}

ExplorationKind Planning::exploration_kind() const
{
    return exploration_kind_;
}
