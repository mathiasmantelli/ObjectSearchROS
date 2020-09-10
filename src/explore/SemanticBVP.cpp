#include "SemanticBVP.h"

//eight-neighbor offset
int offset[][8]={{-1,  1},
                 { 0,  1},
                 { 1,  1},
                 { 1,  0},
                 { 1, -1},
                 { 0, -1},
                 {-1, -1},
                 {-1,  0}
                };

SemanticBVP::SemanticBVP(Configuration* config)
{    
    founded_ = false;
    config_ = config;

    goal = NULL;
    max_goal_radius = 20;
    amountOrientation = 45;
    amountOrientation2 = 20;
    totalAmountDoorsOrient = 0;
    amountHistoryRobotOrientations = 600000;
    amountHistoryDoorsOrientations = 6000;
    previous.x = -1;
    previous.y = -1;
    histogramDoorsOrientation.resize(180/amountOrientation);
    histogramRobotOrientation.resize(360/amountOrientation2);
}

void SemanticBVP::initialize(Grid *grid)
{
    int goal;
    if(config_->GetInt("goal_number", goal))
        goalNumber.push_back(goal);

    threshold_segment_ = config_->GetInt("threshold_segment");

    min_vector_direction_ = config_->GetInt("min_vector_direction");

    //    w_parity_ = config_->GetFloat("w_parity");
    //    w_direction_ = config_->GetFloat("w_direction");
    //    w_difference_ = config_->GetFloat("w_difference");
    w_alpha_ = config_->GetFloat("w_alpha");
    //w_distance_ = config_->GetFloat("w_distance");
    w_orientation_ = config_->GetFloat("w_orientation");
    w_geometric_ = 1.0 - w_alpha_;
    w_tollerance_ = config_->GetFloat("w_tollerance");
    w_distanceGoal_ = config_->GetFloat("w_distanceGoal");
    w_historyOrientation_ = config_->GetFloat("w_historyOrientation");

    double radius;
    radius=1.2*(double)grid->getMapScale();

    visitedKernel = new CCircular();
    visitedKernel->initializeKernel(&radius);
    //    visitedKernel->print();

    radius=2.4*(double)grid->getMapScale();

    circularKernel = new CCircular();
    circularKernel->initializeKernel(&radius);
    //    circularKernel->print();

    gaussianKernel = new CGaussianC();
    gaussianKernel->initializeKernel(&radius);
    gaussianKernel->setMaxValueToMask(0.05);
    //    gaussianKernel->print();

    //    goalNumber.push_back(240);
    //    goalNumber.push_back(207);

    sizeOffset = 8;

    allSegments.clear();
    validDoors.clear();

}

void SemanticBVP::updateRoomsGraph(Doors &doors)
{
    std::list<Door*> validDoors;

    for (std::list<std::pair<std::string,int> >::iterator itRecent=doors.recentDoors.begin(); itRecent != doors.recentDoors.end(); ++itRecent)
        if((*itRecent).second >= doors.minValidObservations)
            validDoors.push_back(doors.doorsMap[(*itRecent).first]);

    float diff=0;
    float diff2=0;

    int N=validDoors.size();
    std::list<Door*>::iterator door1 = validDoors.begin();
    int firstDoor = (*door1)->number;
    std::list<Door*>::iterator door2 = std::next(door1);
    for (int i=0; i<N-1; i++)
    {
        Door* d1 = *door1;
        Door* d2 = *door2;

        diff += d1->number - d2->number;
        diff2 += firstDoor-d2->number;

        if(d1->number > d2->number)
            d1->addNeighbor(d2);
        else if(d1->number < d2->number)
            d2->addNeighbor(d1);

        door1++;
        door2++;

    }

    doors.newObservation = false;
}

void SemanticBVP::updateGridSegments(Grid *grid, const robotCell &robotInVoronoi, Segment *seg)
{
    Cell *c, *nc;
    std::queue<Cell*> cellQueue;

    int x=0;
    int y=0;

    Cell *root = grid->getCell(robotInVoronoi.x,robotInVoronoi.y);
    if(root->type != FREE)
        return;

    int width=visitedKernel->width();
    int height=visitedKernel->height();
    int halfWidth=(width)/2;
    int halfHeight=(height)/2;

    auxCounter++;
    root->planCounter=auxCounter;
    cellQueue.push(root);

    while(!cellQueue.empty())
    {
        c=cellQueue.front();

        x = c->x - robotInVoronoi.x + halfWidth;
        y = c->y - robotInVoronoi.y + halfHeight;

        if(visitedKernel->m_kernelMask[x+y*width] > 0)
            if(!c->segments_)
                c->segments_ = seg;

        // Check neighbors
        for(int n=0;n<sizeOffset;n++)
        {
            nc = grid->getCell(c->x+offset[n][0],c->y+offset[n][1]);

            if((nc->type == FREE || nc->type == NEAROBSTACLE) &&
                    nc->planCounter != auxCounter &&
                    x+offset[n][0] >= 0 && x+offset[n][0] < width &&
                    y+offset[n][1] >= 0 && y+offset[n][1] < height )
            {
                nc->planCounter = auxCounter;
                cellQueue.push(nc);
            }
        }
        cellQueue.pop();
    }
}

void SemanticBVP::updateVisited(Grid *grid, const robotCell &robotInVoronoi)
{
    Cell *c, *nc;
    std::queue<Cell*> cellQueue;

    int x=0;
    int y=0;

    Cell *root = grid->getCell(robotInVoronoi.x,robotInVoronoi.y);
    if(root->type != FREE)
        return;

    int width=visitedKernel->width();
    int height=visitedKernel->height();
    int halfWidth=(width)/2;
    int halfHeight=(height)/2;

    auxCounter++;
    root->planCounter=auxCounter;
    cellQueue.push(root);

    while(!cellQueue.empty())
    {
        c=cellQueue.front();

        x = c->x - robotInVoronoi.x + halfWidth;
        y = c->y - robotInVoronoi.y + halfHeight;

        if(visitedKernel->m_kernelMask[x+y*width] > 0)
            c->isVisited = true;

        // Check neighbors
        for(int n=0;n<sizeOffset;n++)
        {
            nc = grid->getCell(c->x+offset[n][0],c->y+offset[n][1]);

            if((nc->type == FREE || nc->type == NEAROBSTACLE) &&
                    nc->planCounter != auxCounter &&
                    x+offset[n][0] >= 0 && x+offset[n][0] < width &&
                    y+offset[n][1] >= 0 && y+offset[n][1] < height )
            {
                nc->planCounter = auxCounter;
                cellQueue.push(nc);
            }
        }
        cellQueue.pop();
    }
}

//This function assign an ID and a kind for each segment. It also deals with merging when two segments with same kind and different IDs touches each other.
void SemanticBVP::segmentClassification(Grid *grid, const robotCell &robotInVoronoi, std::set<Door*> &validDoors){
    Cell *c, *nc;
    std::queue<Cell*> cellQueue;
    std::vector<Cell*> hist;

    int x=0;
    int y=0;
    int cont=0;
    int kind;

    Cell* robotCell = grid->getCell(robotInVoronoi.x,robotInVoronoi.y);
    if(robotCell->type != FREE)
        return;

    int width=circularKernel->width();
    int height=circularKernel->height();
    int halfWidth=(width)/2;
    int halfHeight=(height)/2;


    Segment* currSegment = robotCell->segments_;
    Segment* neighSegment = nullptr;

    auxCounter++;
    robotCell->planCounter=auxCounter;

    hist.push_back(robotCell);
    for(int n = 0; n < sizeOffset; n++)
        hist.push_back(grid->getCell(robotCell->x+offset[n][0],robotCell->y+offset[n][1]));

    cellQueue.push(robotCell);

    while(!cellQueue.empty())
    {
        c=cellQueue.front();

        x = c->x - robotInVoronoi.x + halfWidth;
        y = c->y - robotInVoronoi.y + halfHeight;

        // Check neighbors
        for(int n=0;n<sizeOffset;n++)
        {
            nc = grid->getCell(c->x+offset[n][0],c->y+offset[n][1]);

            if((nc->type == FREE || nc->type == NEAROBSTACLE) &&
                    nc->planCounter != auxCounter &&
                    //               nc->isVisited == true &&
                    x+offset[n][0] >= 0 && x+offset[n][0] < width &&
                    y+offset[n][1] >= 0 && y+offset[n][1] < height )
            {
                nc->planCounter = auxCounter;
                cont++;
                cellQueue.push(nc);
            }
        }
        cellQueue.pop();
    }

    kind = int(floor(cont/threshold_segment_));

    //(cont <= thresholdSegment) ? kind = 1: kind = 2;

    if(!currSegment)//if the robot's pose doesn't have a segment, then we check whether one of its neighboors have the same kind of segment. If so, the robot's pose receives it.
    {
        for(int n=0;n<sizeOffset;n++)
        {
            nc = grid->getCell(robotCell->x+offset[n][0],robotCell->y+offset[n][1]);
            if(nc->segments_)
                if(nc->segments_->kindSegment == kind)
                    currSegment = nc->segments_;
        }
    }

    if(!currSegment)//if the robot's pose doesn't have a segment, neither its neighboors, then we create a new one.
    {
        currSegment = new Segment(min_vector_direction_);
        currSegment->idSegment = ++segmentCounter;
        currSegment->kindSegment = kind;
        allSegments.push_back(currSegment);
    }
    else //otherwise, it means that one of the robot's neighboor has a segment of the same kind of it.
    {
        for(int n=0;n<sizeOffset;n++)
        {
            nc = grid->getCell(robotCell->x+offset[n][0],robotCell->y+offset[n][1]);
            if(nc->segments_)
            {
                if(nc->segments_->kindSegment == currSegment->kindSegment && nc->segments_->getRealID() != currSegment->getRealID()) //Checking whether the neighboor segment has the same kind, but different ID. That means that a loop was closure
                {
                    neighSegment = currSegment;

                    while(neighSegment->father)
                        neighSegment = neighSegment->father;

                    neighSegment->father = nc->segments_;
                    nc->segments_->copyDoorsSon2Father(neighSegment->doorsOfSegment);

                }
            }
        }

    }

    if(currSegment->kindSegment == kind)
        this->updateGridSegments(grid, robotInVoronoi, currSegment);


    //ADDING DOORS
    totalAmountDoorsOrient = 0;
    for(std::set<Door*>::iterator itValid = validDoors.begin(); itValid != validDoors.end(); ++itValid){
        currSegment->addDoor((*itValid));
        float orientation = robotInVoronoi.theta; //in degrees
        if(orientation < 0) orientation += 180;
        if(currSegment->getHaveDoors()){
            int index;
            if(orientation > 23 && orientation <= 68)
                index = 1;
            else
                if(orientation > 68 && orientation <= 113)
                    index = 2;
                else
                    if(orientation > 113 && orientation <= 158)
                        index = 3;
                    else
                        index = 0;


            if(historyDoorsOrientation.size() == amountHistoryDoorsOrientations)
                historyDoorsOrientation.pop_front();
            historyDoorsOrientation.push_back(index);
            totalAmountDoorsOrient += 1;
        }
    }

    if(histogramDoorsOrientation.size() > 0){
        for(int i = 0; i < histogramDoorsOrientation.size(); i++)
            histogramDoorsOrientation[i] = 0;

        int total = 0;
        for (list<float>::iterator i = historyDoorsOrientation.begin(); i != historyDoorsOrientation.end(); ++i){

            histogramDoorsOrientation[*i] += 1;
            total++;
        }

        for(int i = 0; i < histogramDoorsOrientation.size(); i++)
            if(histogramDoorsOrientation[i] > 0)
                histogramDoorsOrientation[i] /= total;
    }
    validDoors.clear();
}

//CHECK WHETHER THE ROBOT IS TOUCHING A LABELLED/UNLABELLED SEGMENT - just consider the actual id
Segment *SemanticBVP::checkingLabelNeighboors(Cell* robotCell, Grid *grid){
    Cell *nc, *rc;
    Segment *robotSeg, *temp;
    int rcID, ncID, cont;

    std::cout<<std::endl<<"-----------------CHECKINGNEIGHBOORS----------------- "<<std::endl;
    rc = grid->getCell(robotCell->x,robotCell->y);
    robotSeg = rc->segments_;
    temp = nullptr;
    cont = 0;

    for(int n=0;n<sizeOffset;n++){
        nc = grid->getCell(robotCell->x+offset[n][0],robotCell->y+offset[n][1]);
        if(nc->segments_){
            rcID = robotSeg->getRealID();
            ncID = nc->segments_->getRealID();
            if((ncID < segmentCounter) && (ncID != rcID)){//checking whether the kernel surrounding the robot is touching a cell that is already labeled and is different from the robot's one.
                if(!temp) temp = nc->segments_;
                else if(nc->segments_->getRealID() < temp->getRealID()) temp = nc->segments_;
            }else cont++;
        }
    }
    if(cont == sizeOffset) return robotSeg;
    return temp;
}

Segment* SemanticBVP::searchForSegment(int index){
    std::list<Segment*>::iterator it;
    for (it = allSegments.begin(); it != allSegments.end(); it++)
    {
        if((*it)->idSegment == index)
            return (*it);

    }
    return nullptr;
}

void SemanticBVP::updateParity(Grid *grid, const robotCell &robotInVoronoi, Doors &doors)
{
    if(doors.numOddRoomsNearRobot+doors.numEvenRoomsNearRobot == 0)
        return;

    int numOdds = doors.numOddRoomsNearRobot;
    int numEvens = doors.numEvenRoomsNearRobot;

    Cell *c, *nc;
    std::queue<Cell*> cellQueue;

    int x=0;
    int y=0;

    Cell *root = grid->getCell(robotInVoronoi.x,robotInVoronoi.y);

    int width=circularKernel->width();
    int height=circularKernel->width();
    int halfWidth=(width)/2;
    int halfHeight=(height)/2;

    auxCounter++;
    root->planCounter=auxCounter;
    cellQueue.push(root);

    while(!cellQueue.empty()){
        c=cellQueue.front();

        x = c->x - robotInVoronoi.x + halfWidth;
        y = c->y - robotInVoronoi.y + halfHeight;

        if(circularKernel->m_kernelMask[x+y*width] > 0){
            if(numOdds>0)
            {
                c->isNearOddRoom += numOdds*gaussianKernel->m_kernelMask[x+y*width];
                if(c->isNearOddRoom > 1) c->isNearOddRoom = 1;
            }
            else if(numEvens>0)
            {
                c->isNearEvenRoom += numEvens*gaussianKernel->m_kernelMask[x+y*width];
                if(c->isNearEvenRoom > 1) c->isNearEvenRoom = 1;
            }
        }

        // Check neighbors
        for(int n=0;n<sizeOffset;n++)
        {
            nc = grid->getCell(c->x+offset[n][0],c->y+offset[n][1]);
            if((nc->type == FREE || nc->type == NEAROBSTACLE) &&
                    nc->planCounter != auxCounter &&
                    x+offset[n][0] >= 0 && x+offset[n][0] < width &&
                    y+offset[n][1] >= 0 && y+offset[n][1] < height ){

                nc->planCounter = auxCounter;
                cellQueue.push(nc);
            }
        }
        cellQueue.pop();
    }
}

void SemanticBVP::updateDistancesToGoalNumber(Grid *grid, const robotCell &robotInVoronoi, Doors &doors)
{

    if(doors.numOddRoomsNearRobot+doors.numEvenRoomsNearRobot == 0)
        return;

    int currentGoal = goalNumber.at(0);
    grid->currentDoorGoal = currentGoal;

    std::cout << "GOAL NUMBER " << currentGoal << std::endl;

    //double distToGoal = doors.avgDoorNumberNearRobot;

    Door *d;
    std::map<std::string,Door*>::iterator itDoor;

    //Checking whether the 'query door' is already known.
    for (itDoor=doors.doorsMap.begin(); itDoor!=doors.doorsMap.end(); ++itDoor){
        d = itDoor->second;
        if(d->number == currentGoal){
            goalNumber.erase(goalNumber.begin());
            founded_ = true;
        }
    }

    //std::cout << "Avg Door Number " << doors.avgDoorNumberNearRobot << std::endl;

    Cell *c, *nc;
    std::queue<Cell*> cellQueue;

    int x=0;
    int y=0;

    Cell *root = grid->getCell(robotInVoronoi.x,robotInVoronoi.y);

    int width=gaussianKernel->width();
    int height=gaussianKernel->width();
    int halfWidth=(width)/2;
    int halfHeight=(height)/2;

    auxCounter++;
    root->planCounter=auxCounter;
    cellQueue.push(root);

    while(!cellQueue.empty()){
        c=cellQueue.front();

        x = c->x - robotInVoronoi.x + halfWidth;
        y = c->y - robotInVoronoi.y + halfHeight;

        if(circularKernel->m_kernelMask[x+y*width] > 0){
            //            c->sumDtGN += distToGoal;
            c->countDtGN += 1.0;
            c->differenceToGoalNumber = doors.avgDoorNumberNearRobot;//c->sumDtGN/c->countDtGN;
        }

        // Check neighbors
        for(int n=0;n<sizeOffset;n++)
        {
            nc = grid->getCell(c->x+offset[n][0],c->y+offset[n][1]);
            if((nc->type == FREE || nc->type == NEAROBSTACLE) &&
                    nc->planCounter != auxCounter &&
                    x+offset[n][0] >= 0 && x+offset[n][0] < width &&
                    y+offset[n][1] >= 0 && y+offset[n][1] < height ){

                nc->planCounter = auxCounter;
                cellQueue.push(nc);
            }
        }
        cellQueue.pop();
    }
}

//Diego
bool SemanticBVP::computeDistanceToUnknownCellsUsingNumbers(Grid *grid, const robotCell &curPose)
{
    Cell *c, *nc, *endVoronoiCell;
    int countEndVoronoiCells, indexEndVoronoiCells;
    std::queue<Cell*> cellQueue, test, goalsQueue;

    std::list<std::pair<Cell*, int>> endVoronoiCells;
    std::list<std::pair<Cell*, Segment*>> cellAndSegmentsGoals;

    // Find nearest center cell
    robotCell robotInVoronoi = findNearestCenterCell(grid,curPose);
    if(robotInVoronoi.x == UNDEF && robotInVoronoi.y == UNDEF)
        return false;

    Cell *root = grid->getCell(robotInVoronoi.x,robotInVoronoi.y);

    //Registering the history with the last 'amounthistoryorientations' orientations. Instead of +180, I'm using *-1 to mirror the robot's orientation
    if(historyRobotOrientation.size() == amountHistoryRobotOrientations)
        historyRobotOrientation.pop_front();
    (curPose.theta < 0) ? historyRobotOrientation.push_back(curPose.theta + 360) : historyRobotOrientation.push_back(curPose.theta);

    //Cleaning the robot's orientation histogram to the next iteration
    int index, total = 0;
    for(int i = 0; i < histogramRobotOrientation.size(); i++)
        histogramRobotOrientation[i] = 0;

    //Computing the robot's orientation histogram
    for (list<float>::iterator i = historyRobotOrientation.begin(); i != historyRobotOrientation.end(); ++i){
        index = *i/amountOrientation2;
        histogramRobotOrientation[index] += 1;
        total++;
    }

    //Translating all the observed orientations into percentage (how likely an orientation is to be observed)
    for(int i = 0; i < histogramRobotOrientation.size(); i++)
        histogramRobotOrientation[i] /= total;

    auxCounter++;
    root->planCounter=auxCounter;
    cellQueue.push(root);

    root->distanceFromRobot = 0;
    grid->goals.clear();

    endVoronoiCells.clear();

    //Find all center cells in the border of visited space
    while(!cellQueue.empty()){
        c=cellQueue.front();
        bool add = false;
        countEndVoronoiCells = 0;

        for(int n=0;n<sizeOffset;n++)
        {
            nc = grid->getCell(c->x+offset[n][0],c->y+offset[n][1]);
            if(!nc->isVisited && nc->isCenter && c->isVisited){
                add = true;
                //                continue;
            }

            //checking whether it is a voronoi cell in an final line
            if(nc->isCenter){
                countEndVoronoiCells++;
                indexEndVoronoiCells = n;
                endVoronoiCell = nc;
            }

            if(!nc->isCenter || nc->planCounter == auxCounter)
                continue;

            nc->planCounter = auxCounter;
            nc->distanceFromGoal=0;
            nc->distanceFromRobot=c->distanceFromRobot+1;
            cellQueue.push(nc);
        }

        //checking whether it is a voronoi cell in an final line
        if(countEndVoronoiCells == 1){
            indexEndVoronoiCells += 4;
            if(indexEndVoronoiCells > 7) indexEndVoronoiCells -= 8; //computing the opposite offset index to continue the voronoi line
      //      std::cout << "CHECKING - CELL:" << endVoronoiCell->x << "x" << endVoronoiCell->y << " | INDEX: " << indexEndVoronoiCells << std::endl;
            endVoronoiCells.push_back(std::make_pair(endVoronoiCell, indexEndVoronoiCells));
        //    std::cout << "AN END VORONOI CELL WAS INCLUDED" << std::endl;
        }

        cellQueue.pop();

        if(add){
            goalsQueue.push(c);
            grid->goals.push_back(c);

            //            std::cout << "cell: [" << c->x << ", " << c->y << "]";
            //            std::cout << " | distanceFromRobot: " << c->distanceFromRobot << std::endl;

            int dx = 0;
            int dy = 0;

            for(int n=0;n<sizeOffset;n++)
            {
                nc = grid->getCell(c->x+offset[n][0],c->y+offset[n][1]);
                if(!nc->isVisited && nc->isCenter)
                {
                    dx += nc->x - c->x;
                    dy += nc->y - c->y;
                }
            }

            c->frontier_direction = atan2(dy,dx);
        }
    }

    //    std::cout << std::endl;


    //MERGING THE GOAL CELLS WITH ITS NEAREST SEGMENTS
    test = goalsQueue;
    //std::cout << "-----------WHILE COMECANDO--------------" << std::endl;
    extendVoronoi(endVoronoiCells, grid);   //##################################################################################################################################
    while(!test.empty()){
        Segment* temp = searchNearestSegmentofGoal(test.front(), grid);
        if(temp != nullptr){
            cellAndSegmentsGoals.push_back(std::pair<Cell*, Segment*>(test.front(), temp));
            //            std::cout << "cell: [" << test.front()->x << ", " << test.front()->y << "]";
            //            std::cout << std::setprecision(3) << " getGrowingAngle: " << temp->getGrowing_Angle() << std::endl;
        }
        test.pop();
    }
    //    std::cout << std::endl;

    //MEASURING THE DISTANCE BETWEEN THE CANDIDATES USING THE UNVISITED VORONOI
    std::queue<Cell*> myQueue, empty;
    for (std::list<std::pair<Cell*, Segment*>>::iterator cellSegment = cellAndSegmentsGoals.begin(); cellSegment!=cellAndSegmentsGoals.end(); cellSegment++){
        auxCounter++;
        (*cellSegment).first->distanceFromNearestCandidate = 0;
        c = (*cellSegment).first;
        for(int n = 0 ; n < sizeOffset ; n++)//getting the first voronoi cell that is within the unvisited area
        {
            nc = grid->getCell(c->x + offset[n][0],c->y + offset[n][1]);
            if(!nc->isVisited && nc->isCenter && c->isVisited){
                nc->planCounter = auxCounter;
                nc->distanceFromNearestCandidate = c->distanceFromNearestCandidate + 1;
                myQueue.push(nc);
                n = sizeOffset;
            }
        }

        while(!myQueue.empty()){
            c = myQueue.front();
            bool add = false;
            for(int n = 0 ; n < sizeOffset ; n++)
            {
                nc = grid->getCell(c->x + offset[n][0],c->y + offset[n][1]);
                if(nc->isVisited && nc->isCenter && !c->isVisited){ //a new candidate goal was found
                    if(!grid->sameCell(nc,(*cellSegment).first)){
                        (*cellSegment).first->distanceFromNearestCandidate = c->distanceFromNearestCandidate + 1;
                        myQueue = empty; //reseting the queue
                        add = true;
                        break;
                    }
                    continue;
                }

                if(!nc->isCenter || nc->planCounter == auxCounter)
                    continue;

                nc->planCounter = auxCounter;
                nc->distanceFromNearestCandidate = c->distanceFromNearestCandidate + 1;
                myQueue.push(nc);
            }
            if(!add) //if the queue is empty
                myQueue.pop();
        }
    }

    //CHECKING WHETHER THE EXPLORATION IS COMPLETE
    if(goalsQueue.empty()){
        std::cout << "Exploration complete! No unknown cells" << std::endl;
        return true;
    }

    //DECIDING THE BEST GOAL
    int smallestDistance = -1;
    int smallestDifference = -1;
    int smallestDistanceGoal = -1;
    for (std::list<std::pair<Cell*, Segment*>>::iterator cellSegment = cellAndSegmentsGoals.begin(); cellSegment!=cellAndSegmentsGoals.end(); cellSegment++){
        if(((*cellSegment).first->distanceFromRobot < smallestDistance) || smallestDistance == -1)
            smallestDistance = (*cellSegment).first->distanceFromRobot;

        if((std::abs((*cellSegment).first->differenceToGoalNumber) < smallestDifference) || smallestDifference == -1)
            smallestDifference = std::abs((*cellSegment).first->differenceToGoalNumber);

        if((*cellSegment).first->distanceFromNearestCandidate < smallestDistanceGoal || smallestDistanceGoal == -1)
            smallestDistanceGoal = (*cellSegment).first->distanceFromNearestCandidate;
    }
    //    std::cout << "SmallestDistance: " << smallestDistance << " | SmallestDifference: " << smallestDifference << std::endl;
    //    std::cout << std::endl << "\t      Grow.Dir. |  Parity | Orient. | Dist.  | M. Sem | M. Geo | Attract." << std::endl;
    for (std::list<std::pair<Cell*, Segment*>>::iterator cellSegment = cellAndSegmentsGoals.begin(); cellSegment!=cellAndSegmentsGoals.end(); cellSegment++){
        std::vector<float> elements;

        float pathDistance = 1;
        float numberDifference = 1;

        //0 - Growing direction
        elements.push_back((*cellSegment).second->Growing_Attractiveness(grid->currentDoorGoal, (*cellSegment).first->frontier_direction, w_tollerance_));

        //1 - Parity
        elements.push_back((*cellSegment).second->Parity_Attractiveness(grid->currentDoorGoal, w_tollerance_));

        //2 - Orientation (observed doors)
        elements.push_back(orientation_attrativeness((*cellSegment).first));

        //3 - Distance of frontiers
        if((*cellSegment).first->distanceFromRobot > 0 && smallestDistance > 0)
            pathDistance = 1.0 - std::pow(1. - ((float)smallestDistance / (float)(*cellSegment).first->distanceFromRobot), 4);
        elements.push_back(pathDistance);

        //4 - Orientation (robot's ones)
        elements.push_back(orientation_Robot_attractiveness((*cellSegment).first));

        //5 - Difference between the goal door and the observed ones
        if(std::abs((*cellSegment).first->differenceToGoalNumber) > 0 && smallestDifference > 0)
            numberDifference = 1.0 - std::pow(1. - ((float)smallestDifference / (float)std::abs((*cellSegment).first->differenceToGoalNumber)), 1);
        elements.push_back(numberDifference);

        //6 - Distance of the nearest candidate considering Voronoi
        if((*cellSegment).first->distanceFromNearestCandidate > 0 && smallestDistanceGoal > 0)
            pathDistance = 1.0 - std::pow(1. - ((float)smallestDistanceGoal / (float)(*cellSegment).first->distanceFromNearestCandidate), 4);
        elements.push_back(pathDistance);

        /*
        0 - Growing Dir.
        1 - Parity
        2 - Orientation
        3 - Distance
        4 - History of Robot's Orientation
        5 - Difference between door numbers (we are not using this one)
        6 - Distance to other goal by Voronoi (we are not using this one)
        */
        float multi_semantic = elements[0] * elements[1] * w_alpha_;
        float multi_geometric = ((elements[2] * w_orientation_ * elements[3] +
                                  elements[3] * w_distanceGoal_ +
                                  elements[4] * w_historyOrientation_ * elements[3]) /
                                  (w_orientation_ + w_distanceGoal_ + w_historyOrientation_)) * w_geometric_;
        (*cellSegment).first->attractiveness = multi_semantic + multi_geometric;
//        std::cout << "                         ----VALUES --- [" << (*cellSegment).first->x <<", " << (*cellSegment).first->y << "]" << std::endl;
//        std::cout << "Growing Dir.: " << elements[0] << " | ";
//        std::cout << "Parity: " << elements[1] << std::endl;
//        std::cout << "Orient. Doors: " << elements[2] << " | ";
//        std::cout << "Distance: " << elements[3] << " | ";
//        std::cout << "Orient. Robot: " << elements[4] << std::endl;
//        std::cout << "Semantic: " << multi_semantic << " | ";
//        std::cout << "Geometric: " << multi_geometric << " | ";
//        std::cout << "Total Att: " << (*cellSegment).first->attractiveness << std::endl;
//        std::cout << std::endl;
    }
//    std::cout << "-----------------------------------------------" << std::endl;

    // esse aux queue é gambiara
    std::queue<Cell*> auxQueue;
    float biggestAttractiveness = -1;
    for (std::list<std::pair<Cell*, Segment*>>::iterator cellSegment = cellAndSegmentsGoals.begin(); cellSegment!=cellAndSegmentsGoals.end(); cellSegment++){
        if((*cellSegment).first->attractiveness > biggestAttractiveness){
            if(auxQueue.size()>0) auxQueue.pop();
            biggestAttractiveness = (*cellSegment).first->attractiveness;
            goal = (*cellSegment).first;  //goal is the candidate cell that has the highest attractiveness factor
            auxQueue.push((*cellSegment).first);
        }
    }
    grid->setMyGoal(goal);

    cellQueue = auxQueue;

    auxCounter++;

    //Update the distanceFromGoal of all center cells
    while(!cellQueue.empty()){
        c=cellQueue.front();
        c->planCounter=auxCounter;

        for(int n=0;n<sizeOffset;n++)
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

bool SemanticBVP::findLocalGoal(Grid *grid, const robotCell &robotInVoronoi)
{
    Cell *root, *c, *nc, *local_goal = NULL;

    std::vector<Cell*> actualCells;
    std::vector<Cell*> futureCells;

    std::vector<Cell*> candidateCells;

    robotCell ret;
    ret.x = ret.y = UNDEF;

    root=grid->getCell(robotInVoronoi.x,robotInVoronoi.y);

    auxCounter++;
    root->planCounter=auxCounter;

    actualCells.push_back(root);

    int max_radius = 0;


    if(goal == NULL)
        return false;
    std::cout << "GOAL " << goal->x << " - " << goal->y << std::endl;


    while(max_radius <= max_goal_radius)
    {
        max_radius++;

        futureCells.clear();

        for(int c1=0; c1<actualCells.size();c1++)
        {
            c = actualCells[c1];

            if(sqrt(pow(root->x - c->x, 2) + pow(root->y - c->y, 2)) <= max_radius)
            {
                if(c->x == goal->x && c->y == goal->y)
                {
                    localGoal.x = goal->x;
                    localGoal.y = goal->y;
                    grid->goalVoronoiCell = localGoal;
                    std::cout << "EXACT GOAL " << goal->x << " - " << goal->y << std::endl;

                    return true;
                }

                if(c->isCenter)
                {
                    if(local_goal)
                    {
                        if(local_goal->distanceFromGoal > c->distanceFromGoal)
                            local_goal = c;
                    }
                    else local_goal = c;
                }

                for(int n=0;n<sizeOffset;n++)
                {
                    nc = grid->getCell(c->x+offset[n][0],c->y+offset[n][1]);
                    if(nc->type == OCCUPIED || nc->planCounter == auxCounter)
                        continue;

                    futureCells.push_back(nc);
                    nc->planCounter = auxCounter;

                }
            }
            else
            {
                futureCells.push_back(c);
            }
        }

        actualCells = futureCells;
    }

    if(!local_goal)
    {
        localGoal.x = localGoal.y = UNDEF;
        return false;
    }

    localGoal.x = local_goal->x;
    localGoal.y = local_goal->y;
    std::cout << localGoal.x << " - " << localGoal.y << std::endl;
    grid->goalVoronoiCell = localGoal;
    return true;
}

Segment* SemanticBVP::searchNearestSegmentofGoal(Cell *cellGoal, Grid *grid){
    Segment* auxSegment = nullptr;
    Cell *c, *nc;
    std::queue<Cell*> cellQueue;

    Cell *root = grid->getCell(cellGoal->x,cellGoal->y);
    if(root->type != FREE)
        return auxSegment;

    auxCounter++;
    root->planCounter=auxCounter;
    cellQueue.push(root);

    while(!cellQueue.empty())
    {
        c=cellQueue.front();

        // Check neighbors
        for(int n=0;n<sizeOffset;n++)
        {
            nc = grid->getCell(c->x+offset[n][0],c->y+offset[n][1]);

            if((nc->type == FREE || nc->type == NEAROBSTACLE) && nc->planCounter != auxCounter)
            {
                nc->planCounter = auxCounter;
                cellQueue.push(nc);
                if(nc->segments_ != nullptr){
                    //                    std::cout<<"The segment is:"<<nc->segments_->idSegment<<std::endl;
                    return nc->segments_;
                }
            }
        }
        cellQueue.pop();
    }
    return auxSegment;
}

robotCell SemanticBVP::findNearestCenterCell(Grid *grid, const robotCell &rc, bool checkInUnexplored)
{
    Cell *root, *c, *nc;

    std::vector<Cell*> actualCells;
    std::vector<Cell*> futureCells;

    std::vector<Cell*> candidateCells;

    robotCell ret;
    ret.x = ret.y = UNDEF;

    root=grid->getCell(rc.x,rc.y);

    auxCounter++;
    root->planCounter=auxCounter;

    actualCells.push_back(root);

    int max_radius = 0;
    bool found = false;

    while(!found)
    {
        max_radius++;
        futureCells.clear();

        for(int c1=0; c1<actualCells.size();c1++)
        {
            c = actualCells[c1];

            if(sqrt(pow(root->x - c->x, 2) + pow(root->y - c->y, 2)) <= max_radius)
            {

                if(c->isCenter)
                {
                    found = true;
                    candidateCells.push_back(c);
                }

                if(!found)
                {
                    for(int n=0;n<8;n++)
                    {
                        nc = grid->getCell(c->x+offset[n][0],c->y+offset[n][1]);
                        if(nc->type == OCCUPIED ||
                                (!checkInUnexplored && nc->type == UNEXPLORED) ||
                                nc->planCounter == auxCounter)
                            continue;

                        futureCells.push_back(nc);
                        nc->planCounter = auxCounter;

                    }
                }
            }
            else
            {
                futureCells.push_back(c);
            }
        }

        actualCells = futureCells;
    }

    if(candidateCells.size() == 0)
        return ret;

    c = candidateCells[0];

    float small = sqrt(pow(root->x - c->x, 2) + pow(root->y - c->y, 2));

    for(int c1=1; c1<candidateCells.size();c1++)
    {
        nc = candidateCells[c1];
        if(sqrt(pow(root->x - nc->x, 2) + pow(root->y - nc->y, 2)) < small)
        {
            c = nc;
            small = sqrt(pow(root->x - nc->x, 2) + pow(root->y - nc->y, 2));
        }
    }

    ret.x = c->x;
    ret.y = c->y;
    return ret;
}

void SemanticBVP::printOrientRobotSawDoors(){
    std::cout<< "---------------------------------------------------------" << std::endl;
    for (int i = 0; i < histogramDoorsOrientation.size(); i++){
        std::cout << "Angle: " << i*amountOrientation << " | # observed doors: " << histogramDoorsOrientation[i] << " | Probability: ";
        if(totalAmountDoorsOrient > 0 && histogramDoorsOrientation[i] > 0)
            std::cout << (float)(histogramDoorsOrientation[i])/totalAmountDoorsOrient*100.0 << " %" << std::endl;
        else
            std::cout <<  "0 %" << std::endl;
    }
    std::cout << "TOTAL: " << totalAmountDoorsOrient << std::endl;
}

float SemanticBVP::orientation_attrativeness(Cell* goal){
    int orientation = RAD2DEG(goal->frontier_direction);
    if(orientation < 0) orientation += 180;
    int index;
    if(orientation == 180) orientation = 0;
    if(orientation > 23 && orientation <= 68)
        index = 1;
    else
        if(orientation > 68 && orientation <= 113)
            index = 2;
        else
            if(orientation > 113 && orientation <= 158)
                index = 3;
            else
                index = 0;
    //std::cout << "#################################################### Orient: " << orientation << " | Index: " << index << " | Perct.: " << histogramDoorsOrientation[orientation] << std::endl;
    return histogramDoorsOrientation[index];
}

float SemanticBVP::orientation_Robot_attractiveness(Cell* goal){
    int angle = RAD2DEG(goal->frontier_direction);
    if(angle < 0) angle += 360;
    angle /= amountOrientation2;
    return histogramRobotOrientation[angle];
}

//FUNCTION TO FIX THE PROBLEM OF THE ROBOT GOING BACK AND FORTH TO A CANDIDATE THAT IS IN THE END OF A PATH
void SemanticBVP::extendVoronoi(std::list<std::pair<Cell *, int> > endVoronoiCells, Grid *grid){
    Cell* nc;

    std::pair<Cell*, int> myPair;
    int size = endVoronoiCells.size();
    while(size > 0){
        myPair = endVoronoiCells.front();
        nc = grid->getCell(myPair.first->x+offset[myPair.second][0], myPair.first->y+offset[myPair.second][1]);
        if(nc->type == FREE){
            nc->isCenter = true;
            nc->isVisited = true;
            endVoronoiCells.push_back(std::make_pair(nc, myPair.second));
            size++;
 //           std::cout << "EXTENDING VORONOI: " << myPair.first->x << "x" << myPair.first->y << " - INDEX:" << myPair.second << std::endl;
        }
        endVoronoiCells.pop_front();
        size--;
    }

}
