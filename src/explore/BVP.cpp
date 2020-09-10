#include "BVP.h"

#include <cstdlib>
#include <cmath>

#include <queue>
#include <iostream>

///////////////////////
/// TRADITIONAL BVP ///
///////////////////////

BVP::BVP()
{

}

void BVP::initializePotential(Grid *grid, const robotCell &curPose, const int radius)
{
    Cell *c;
    for(int i=curPose.x-radius;i<=curPose.x+radius;i++){
        for(int j=curPose.y-radius;j<=curPose.y+radius;j++){
            c = grid->getCell(i,j);

            if(c->type == OCCUPIED || c->type == NEAROBSTACLE)
                c->pot = 1.0;
            else if(c->type == UNEXPLORED)
                c->pot = 0.0;

        }
    }
}

double BVP::iteratePotential(Grid *grid, const bbox &curLimits)
{
    double totalVariation = 0.0;

    Cell *c,*l,*r,*u,*d;

    double edc;

    for(int i=curLimits.minX;i<=curLimits.maxX;i++){
        for(int j=curLimits.minY;j<=curLimits.maxY;j++){
            c = grid->getCell(i,j);
            if(c->type==FREE){

                double prev = c->pot;

                l=grid->getCell(i-1,j);
                r=grid->getCell(i+1,j);
                u=grid->getCell(i,j+1);
                d=grid->getCell(i,j-1);

                c->pot = 0.25*(l->pot + r->pot + d->pot + u->pot);

                edc = c->pref*(fabs(l->pot - r->pot) + fabs(d->pot - u->pot))/8.0;
                c->pot = c->pot - edc;

                totalVariation += fabs(c->pot - prev);
            }
        }
    }
    return totalVariation;
}

void BVP::updateGradient(Grid *grid, const bbox &curLimits)
{
    Cell *c,*l,*r,*u,*d;

    for(int i=curLimits.minX;i<=curLimits.maxX;i++){
        for(int j=curLimits.minY;j<=curLimits.maxY;j++){
            c = grid->getCell(i,j);
            if(c->type==FREE){
                l=grid->getCell(i-1,j);
                r=grid->getCell(i+1,j);
                u=grid->getCell(i,j+1);
                d=grid->getCell(i,j-1);

                c->dirX = l->pot - r->pot;
                c->dirY = d->pot - u->pot;

                c->norm = sqrt(c->dirX*c->dirX + c->dirY*c->dirY);
                if(c->norm ==0){
                    c->dirX = 0.0;
                    c->dirY = 0.0;
                }else{
                    c->dirX *= 1.0/c->norm;
                    c->dirY *= 1.0/c->norm;
                }

            }else{
                c->dirX = 0.0;
                c->dirY = 0.0;
            }
        }
    }
}

///////////////////
///   BVP MOD   ///
///////////////////

BVP_mod::BVP_mod()
{

}

void BVP_mod::initializePotential(Grid *grid, const robotCell &curPose, const int radius)
{
    Cell *c;
    for(int i=curPose.x-radius;i<=curPose.x+radius;i++){
        for(int j=curPose.y-radius;j<=curPose.y+radius;j++){
            c = grid->getCell(i,j);

            if(c->type == OCCUPIED || c->type == NEAROBSTACLE)
                c->pot = 1.0;
            else if(c->type == FREE && !c->isVisited)
                c->pot = 0.0;

        }
    }
}

double BVP_mod::iteratePotential(Grid *grid, const bbox &curLimits)
{
    double totalVariation = 0.0;

    Cell *c,*l,*r,*u,*d;

    double edc;

    for(int i=curLimits.minX;i<=curLimits.maxX;i++){
        for(int j=curLimits.minY;j<=curLimits.maxY;j++){
            c = grid->getCell(i,j);
            if(c->type==FREE && c->isVisited){

                double prev = c->pot;

                l=grid->getCell(i-1,j);
                r=grid->getCell(i+1,j);
                u=grid->getCell(i,j+1);
                d=grid->getCell(i,j-1);

                c->pot = 0.25*(l->pot + r->pot + d->pot + u->pot);

                edc = c->pref*(fabs(l->pot - r->pot) + fabs(d->pot - u->pot))/8.0;
                c->pot = c->pot - edc;

                totalVariation += fabs(c->pot - prev);
            }
        }
    }
    return totalVariation;
}

void BVP_mod::updateGradient(Grid *grid, const bbox &curLimits)
{
    Cell *c,*l,*r,*u,*d;

    for(int i=curLimits.minX;i<=curLimits.maxX;i++){
        for(int j=curLimits.minY;j<=curLimits.maxY;j++){
            c = grid->getCell(i,j);
            if(c->type==FREE && c->isVisited){
                l=grid->getCell(i-1,j);
                r=grid->getCell(i+1,j);
                u=grid->getCell(i,j+1);
                d=grid->getCell(i,j-1);

                c->dirX = l->pot - r->pot;
                c->dirY = d->pot - u->pot;

                double norm = sqrt(c->dirX*c->dirX + c->dirY*c->dirY);
                if(norm==0){
                    c->dirX = 0.0;
                    c->dirY = 0.0;
                }else{
                    c->dirX *= 1.0/norm;
                    c->dirY *= 1.0/norm;
                }

            }else{
                c->dirX = 0.0;
                c->dirY = 0.0;
            }
        }
    }
}

///////////////////
/// VORONOI BVP ///
///////////////////

VoronoiBVP::VoronoiBVP():
    offset{{-1,  1}, { 0,  1}, { 1,  1}, { 1,  0}, { 1, -1}, { 0, -1}, {-1, -1}, {-1,  0}}
{
    auxCounter=0;
    minHalfInnerWindowSize = 30;
    halfInnerWindow = minHalfInnerWindowSize;
    localGoalRadius = 20;
    minGoalRadius = 10;

    segmentCounter = 0;


    localGoal.x = localGoal.y = UNDEF;
}

robotCell VoronoiBVP::findNearestCenterCell(Grid *grid, const robotCell &rc, bool checkInUnexplored)
{
    Cell *root, *c, *nc;
    std::queue<Cell*> cellQueue;

    robotCell ret;
    ret.x = ret.y = UNDEF;

    auxCounter++;
    root->planCounter=auxCounter;
    cellQueue.push(root);

    while(!cellQueue.empty())
    {
        c=cellQueue.front();

        if(c->isCenter)
            break;

        for(int n=0;n<8;n++)
        {
            nc = grid->getCell(c->x+offset[n][0],c->y+offset[n][1]);

            if(nc->type == OCCUPIED ||
              (!checkInUnexplored && nc->type == UNEXPLORED) ||
              nc->planCounter == auxCounter)
                 continue;

            cellQueue.push(nc);
            nc->planCounter = auxCounter;
        }
        cellQueue.pop();

    }

    if(!c->isCenter){
    //    std::cout << "Can't find Voronoi!" << std::endl;
        return ret;
    }

    ret.x = c->x;
    return ret;

}

bool VoronoiBVP::findLocalGoal(Grid *grid, const robotCell &robotInVoronoi)
{
    Cell *c, *nc, *goal;
    std::queue<Cell*> cellQueue;
    int smallest=100000000;

    Cell *root = grid->getCell(robotInVoronoi.x,robotInVoronoi.y);

    auxCounter++;
    root->planCounter=auxCounter;
    cellQueue.push(root);

    // find smallest distance at the Voronoi diagram
    while(!cellQueue.empty()){
        c=cellQueue.front();

        for(int n=0;n<8;n++)
        {
            nc = grid->getCell(c->x+offset[n][0],c->y+offset[n][1]);
            if(!nc->isCenter || nc->type==UNEXPLORED ||
               (root->distanceFromGoal-nc->distanceFromGoal)>localGoalRadius ||
              //sqrt(pow(root->x-nc->x,2.0)+pow(root->y-nc->y,2.0))>localGoalRadius ||
                nc->planCounter == auxCounter)
                continue;

            if(nc->distanceFromGoal<=smallest && nc->distanceFromGoal>-1)
            {
                if(abs(root->distanceFromGoal-nc->distanceFromGoal) > minGoalRadius){
                    smallest=nc->distanceFromGoal;
                    goal=nc;
                }
                cellQueue.push(nc);
                nc->planCounter = auxCounter;
            }
        }
        cellQueue.pop();
    }


    if(!goal->isCenter){
        localGoal.x = localGoal.y = UNDEF;
        return false;
    }

    localGoal.x = goal->x;
    localGoal.y = goal->y;
    grid->goalVoronoiCell = localGoal;
    return true;
}


bool VoronoiBVP::computeDistanceToUnknownCells(Grid *grid, const robotCell &curPose)
{
    Cell *c, *nc;
    std::queue<Cell*> cellQueue;
    std::queue<Cell*> goalsQueue;

    // Find nearest center cell
    robotCell robotInVoronoi = findNearestCenterCell(grid,curPose);
    if(robotInVoronoi.x == UNDEF && robotInVoronoi.y == UNDEF)
        return false;

    Cell *root = grid->getCell(robotInVoronoi.x,robotInVoronoi.y);

    auxCounter++;
    root->planCounter=auxCounter;
    cellQueue.push(root);

    //Find all center cells in the border of visited space
    while(!cellQueue.empty()){
        c=cellQueue.front();
        bool add = false;

        for(int n=0;n<8;n++)
        {
            nc = grid->getCell(c->x+offset[n][0],c->y+offset[n][1]);
            if(nc->type == UNEXPLORED && nc->isCenter){
                add = true;
                continue;
            }

            if(!nc->isCenter || nc->planCounter == auxCounter)
                continue;

            nc->planCounter = auxCounter;
            nc->distanceFromGoal=0;
            cellQueue.push(nc);
        }
        cellQueue.pop();

        if(add)
            goalsQueue.push(c);
    }

    auxCounter++;

//    std::cout << "goals " << goalsQueue.size() << std::endl;

    // check if exploration is complete
    if(goalsQueue.empty()){
        std::cout << "Exploration complete! No unknown cells" << std::endl;
        return true;
    }

    cellQueue = goalsQueue;

    //Update the distanceFromGoal of all center cells
    while(!cellQueue.empty()){
        c=cellQueue.front();
        c->planCounter=auxCounter;

        for(int n=0;n<8;n++)
        {
            nc = grid->getCell(c->x+offset[n][0],c->y+offset[n][1]);
            if(!nc->isCenter || /*nc->type==UNEXPLORED || */nc->planCounter == auxCounter)
                continue;

            nc->distanceFromGoal=c->distanceFromGoal+1;
            nc->planCounter = auxCounter;
            cellQueue.push(nc);

        }
        cellQueue.pop();
    }

    return false;

}


void VoronoiBVP::initializePotential(Grid *grid, const robotCell &curPose)
{
    Cell *c;
    for(int i=curPose.x-2*halfInnerWindow;i<=curPose.x+2*halfInnerWindow;i++){
        for(int j=curPose.y-2*halfInnerWindow;j<=curPose.y+2*halfInnerWindow;j++){
            if(i>=curPose.x-halfInnerWindow && i<=curPose.x+halfInnerWindow &&
               j>=curPose.y-halfInnerWindow && j<=curPose.y+halfInnerWindow )
                continue;

            c = grid->getCell(i,j);

            c->pot = 0.5;
        }
    }

    for(int i=curPose.x-halfInnerWindow-1;i<=curPose.x+halfInnerWindow+1;i++){
        c = grid->getCell(i,curPose.y-halfInnerWindow-1);
        c->pot = 1.0;
        c = grid->getCell(i,curPose.y+halfInnerWindow+1);
        c->pot = 1.0;
    }
    for(int j=curPose.y-halfInnerWindow-1;j<=curPose.y+halfInnerWindow+1;j++){
        c = grid->getCell(curPose.x-halfInnerWindow-1,j);
        c->pot = 1.0;
        c = grid->getCell(curPose.x+halfInnerWindow+1,j);
        c->pot = 1.0;
    }

    for(int i=curPose.x-halfInnerWindow;i<=curPose.x+halfInnerWindow;i++){
        for(int j=curPose.y-halfInnerWindow;j<=curPose.y+halfInnerWindow;j++){
            c = grid->getCell(i,j);

            if(c->type == OCCUPIED || c->type == NEAROBSTACLE)
                c->pot = 1.0;
        }
    }

    c = grid->getCell(localGoal.x, localGoal.y);
    c->pot = 0.0;

}

double VoronoiBVP::iteratePotential(Grid *grid, const robotCell &curPose)
{
    Cell *c,*l,*r,*u,*d;
    double totalVariation = 0.0;

    for(int i=curPose.x-halfInnerWindow;i<=curPose.x+halfInnerWindow;i++){
        for(int j=curPose.y-halfInnerWindow;j<=curPose.y+halfInnerWindow;j++){
            c = grid->getCell(i,j);

            if(c->type == FREE && !(c->x == localGoal.x && c->y == localGoal.y))
            {
                double prev = c->pot;

                l=grid->getCell(i-1,j);
                r=grid->getCell(i+1,j);
                u=grid->getCell(i,j+1);
                d=grid->getCell(i,j-1);

                c->pot = 0.25*(l->pot + r->pot + d->pot + u->pot);

                totalVariation += fabs(c->pot - prev);
            }
        }
    }
    return totalVariation;
}

void VoronoiBVP::updateGradient(Grid *grid, const robotCell &curPose)
{
    Cell *c,*l,*r,*u,*d;

    for(int i=curPose.x-halfInnerWindow;i<=curPose.x+halfInnerWindow;i++){
        for(int j=curPose.y-halfInnerWindow;j<=curPose.y+halfInnerWindow;j++){
            c = grid->getCell(i,j);

            if(c->type==FREE){
                l=grid->getCell(i-1,j);
                r=grid->getCell(i+1,j);
                u=grid->getCell(i,j+1);
                d=grid->getCell(i,j-1);

                c->dirX = l->pot - r->pot;
                c->dirY = d->pot - u->pot;

                double norm = sqrt(c->dirX*c->dirX + c->dirY*c->dirY);
                if(norm==0){
                    c->dirX = 0.0;
                    c->dirY = 0.0;
                }else{
                    c->dirX *= 1.0/norm;
                    c->dirY *= 1.0/norm;
                }

            }else{
                c->dirX = 0.0;
                c->dirY = 0.0;
            }
        }
    }

}
