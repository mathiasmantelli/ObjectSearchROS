#include <cstdio>
#include <GL/glut.h>
#include <iostream>
#include <sstream>
#include <iomanip>

#include "Grid.h"
#include "math.h"
#include "SemanticBVP.h"
#include "cmath"

Segment::Segment(int min_vector_direction)
{
    att_odd_ = 0;
    att_even_ = 0;
    idSegment = -1;
    kindSegment = -1;
    father = nullptr;
    doorsOfSegment.clear();
    min_vector_direction_ = min_vector_direction;

    haveDoors = false;
    amountOrientation = 45;
}

Cell::Cell ()
{
    segments_ = nullptr;

}

Grid::Grid ()
{
    mapScale_ = 10;
    mapWidth_ = mapHeight_ = 2000;
    numCellsInRow_=mapWidth_;
    halfNumCellsInRow_=mapWidth_/2;

    cells_ = new Cell[mapWidth_*mapHeight_];

    for (unsigned int j = 0; j < numCellsInRow_; ++j)
    {
        for (unsigned int i = 0; i < numCellsInRow_; ++i)
        {
            unsigned int c = j*numCellsInRow_ + i;
            cells_[c].x = -halfNumCellsInRow_ + 1 + i;
            cells_[c].y =  halfNumCellsInRow_ - j;

            cells_[c].himm=7;
            cells_[c].slamValue=-1;
            cells_[c].pot=0.5;
            cells_[c].pot=0.5;
            cells_[c].pref = 0.0;
            cells_[c].border = 0;
            cells_[c].type = UNEXPLORED;

            cells_[c].norm = 0;

            cells_[c].isCenter = false;
            cells_[c].isVisitedCenter = false;
            cells_[c].isEndPoint = false;
            cells_[c].isVoronoiHelper = false;

            cells_[c].planCounter = 0;
            cells_[c].isVisited = false;
            cells_[c].isNearEvenRoom = 0.0;
            cells_[c].isNearOddRoom = 0.0;
            cells_[c].differenceToGoalNumber = 0.0;
            cells_[c].sumDtGN = 0.0;
            cells_[c].countDtGN = 0.0;

            cells_[c].distWalls = -1;
            cells_[c].distanceFromGoal = -1;
            cells_[c].dirX = 0.0;
            cells_[c].dirY = 0.0;

            //Diego
//            cells_[c].bel_even=0;
//            cells_[c].bel_odd=0;

        }
    }

    // Mark voronoi helper cells
    for (unsigned int j = 0; j < numCellsInRow_; j=j+2){
        for (unsigned int i = 0; i < numCellsInRow_; i=i+2){
            cells_[j*numCellsInRow_ + i].isVoronoiHelper=true;
        }
    }

    himm_count=0; //keeping the global count of HIMM synchronous

    numViewModes=9;
    viewMode=0;

    showValues=false;
    showArrows=false;

    robotVoronoiCell.x = robotVoronoiCell.y = UNDEF;
    goalVoronoiCell.x = goalVoronoiCell.y = UNDEF;

    SLAMlimits.minX = SLAMlimits.minY =  1000000;
    SLAMlimits.maxX = SLAMlimits.maxY = -1000000;

    myGoal = NULL;
}

Cell* Grid::getCell (int x, int y)
{
    int i=x+halfNumCellsInRow_-1;
    int j=halfNumCellsInRow_-y;
    return &(cells_[j*numCellsInRow_ + i]);
}

int Grid::getMapScale()
{
    return mapScale_;
}

int Grid::getMapWidth()
{
    return mapWidth_;
}

int Grid::getMapHeight()
{
    return mapHeight_;
}

void Grid::draw(int xi, int yi, int xf, int yf)
{
    glLoadIdentity();

    for(int i=xi; i<=xf; ++i){
        for(int j=yi; j<=yf; ++j){
            drawCell(i+j*numCellsInRow_);
        }
    }

    //        if(robotVoronoiCell.x != UNDEF && robotVoronoiCell.y != UNDEF)
    //            drawCellWithColor(robotVoronoiCell.x, robotVoronoiCell.y, 1.0, 0.0, 0.0);
    for(int i=0; i<goals.size(); i++)
        drawCellWithColor(goals[i]->x, goals[i]->y, 0.6, 0.0, 0.2);
    if(myGoal != NULL){
        std::cout << "myGoal " << myGoal->x << " " << myGoal->y << std::endl;
        drawCellWithColor(myGoal->x, myGoal->y, 1.0, 0.0, 0.0);
        drawCellWithColor(myGoal->x-1, myGoal->y, 0.0, 0.0, 0.0);
        drawCellWithColor(myGoal->x+1, myGoal->y, 0.0, 0.0, 0.0);
        drawCellWithColor(myGoal->x, myGoal->y-1, 0.0, 0.0, 0.0);
        drawCellWithColor(myGoal->x, myGoal->y+1, 0.0, 0.0, 0.0);
    }

    if(showArrows){
        glPointSize(2);
        for(int i=xi; i<=xf; ++i){
            for(int j=yi; j<=yf; ++j){
                drawVector(i+j*numCellsInRow_);
            }
        }
    }

    if(showValues){
        for(int i=xi; i<=xf; i++){
            for(int j=yi; j<=yf; j++){
                int n=i+j*numCellsInRow_;
                if(cells_[n].isCenter && cells_[n].distanceFromGoal>=0)
                    drawText(n);
            }
        }
    }
}

void Grid::drawCell(unsigned int n)
{
    float aux;

    if(viewMode==0){
//        std::cout<<std::endl<<"VIEWMODE 0"<<std::endl;
        if(cells_[n].type == UNEXPLORED){
            glColor3f(0.6,0.6,0.6);
        }else if(cells_[n].isVisited){
            glColor3f(1.0,0.95,0.4);
        }else if(cells_[n].type == FREE){
            glColor3f(1.0,1.0,1.0);
        }else if(cells_[n].type == OCCUPIED){
            glColor3f(0.0,0.0,0.0);
        }else if(cells_[n].type == NEAROBSTACLE){
            glColor3f(0.0,0.0,0.5);
        }

        if(cells_[n].isCenter){ //voronoi
            glColor3f(0.0,0.8,0.0);
        }


        //        if(cells_[n].border > planIteration-5)
        //            glColor3f(1.0,0.0,0.0);
    }else if(viewMode==1){
//        std::cout<<std::endl<<"VIEWMODE 1"<<std::endl;
        if(cells_[n].type == UNEXPLORED){
            glColor3f(0.5,0.8,0.5);
        }else if(cells_[n].type == FREE || cells_[n].type == NEAROBSTACLE){
            if(cells_[n].segments_){
                aux = cells_[n].segments_->getAtt_Even();
                glColor3f(aux,aux,aux);
            }else
                glColor3f(.5,.5,0);
        }else if(cells_[n].type == OCCUPIED){
            glColor3f(0.0,0.0,1.0);
        }
    }else if(viewMode==2){
//        std::cout<<std::endl<<"VIEWMODE 2"<<std::endl;
        if(cells_[n].type == UNEXPLORED){
            glColor3f(0.5,0.5,0.8);
        }else if(cells_[n].type == FREE || cells_[n].type == NEAROBSTACLE){
            if(cells_[n].segments_){
                aux = cells_[n].segments_->getAtt_Odd();
                glColor3f(aux,aux,aux);
            }else
                glColor3f(.5,0.5,0);

        }else if(cells_[n].type == OCCUPIED){
            glColor3f(0.0,0.0,1.0);
        }
        //        if(cells_[n].isCenter){
        //            glColor3f(0.0,0.8,0.0);
        //        }else{
        //            aux = cells_[n].localPot;
        //            glColor3f(aux,aux,aux);
        //        }
    }else if(viewMode==3){
//        std::cout<<std::endl<<"VIEWMODE 3"<<std::endl;
        if(cells_[n].type == UNEXPLORED){
            glColor3f(0.5,0.5,0.5);
        }else if(cells_[n].type == FREE || cells_[n].type == NEAROBSTACLE){
            if(cells_[n].countDtGN>2){
                int diff = cells_[n].differenceToGoalNumber - currentDoorGoal;
                //std::cout<<"*****DIFF:"<<diff<<std::endl;
                if(diff >= 0) //update the distance - goal right here. Calculate the average and then the difference between it and the goal.
                    glColor3f(1.0,1.0-diff/15.0,1.0);
                else
                    glColor3f(diff/15.0+1.0,1.0,1.0);
            }else{
                glColor3f(0.3,0.3,0.3);
            }
        }else if(cells_[n].type == OCCUPIED){
            glColor3f(0.0,0.0,0.0);
        }
    }else if(viewMode==4){
//        aux=(16.0-cells_[n].himm)/16.0;
//        glColor3f(aux,aux,aux);
        if(cells_[n].slamValue == -1)
            glColor3f(0.6,0.6,0.8);
        else{
            aux =(100.0-cells_[n].slamValue)/100.0;
            glColor3f(aux,aux,aux);
        }
//        std::cout<<std::endl<<"VIEWMODE 4"<<std::endl;
    }else if(viewMode==5){
//        std::cout<<std::endl<<"VIEWMODE 5"<<std::endl;
        if(cells_[n].type == UNEXPLORED){
            glColor3f(0.6,0.6,0.6);
        }else if(cells_[n].type == OCCUPIED){
            glColor3f(0.0,0.0,0.0);
        }else{
//            float diff = cells_[n].segments_->bel_even-cells_[n].segments_->bel_odd;

//            if(diff>0) diff=0;
//            aux=(cells_[n].segments_->bel_even)+((diff)*std::abs(diff)*std::abs(diff));

//            aux=(0.5+(aux/2));

//            glColor3f(1-aux,1,1);
        }
    }else if(viewMode==6){
//        std::cout<<std::endl<<"VIEWMODE 6"<<std::endl;
        if(cells_[n].type == UNEXPLORED){
            glColor3f(0.6,0.6,0.6);
        }else if(cells_[n].type == OCCUPIED){
            glColor3f(0.0,0.0,0.0);
        }else{
            aux = cells_[n].pot;
//            float diff = cells_[n].segments_->bel_odd-cells_[n].segments_->bel_even;

//            if(diff>0) diff=0;
//            aux=(cells_[n].segments_->bel_odd)+((diff)*std::abs(diff)*std::abs(diff));

//            aux=(0.5+(aux/2));

            glColor3f(aux,aux,aux);
        }
    }else if(viewMode==7){//Printing corridor and non-corridor
//        std::cout<<std::endl<<"VIEWMODE 7"<<std::endl;
        if(cells_[n].type == UNEXPLORED){
            glColor3f(0.6,0.6,0.6);
        }else if((cells_[n].type == FREE)){
            glColor3f(1.0,1.0,1.0);
            if(cells_[n].segments_ != nullptr){
                int kind = cells_[n].segments_->getRealKind();
                if(kind == 1){
                    glColor3f(0.4,1.0,0.4);
                }else if(kind == 2){
                    glColor3f(1.0,0.4,0.4);
                }else if(kind == 0){
                    glColor3f(0.4,0.4,1.0);
                }
            }
        }else if(cells_[n].type == OCCUPIED){
            glColor3f(0.0,0.0,0.0);
        }else if(cells_[n].type == NEAROBSTACLE){
            glColor3f(0.0,0.0,0.5);
        }

    }else if(viewMode==8){//Printing the segments according to their IDs
//        std::cout<<std::endl<<"VIEWMODE 8"<<std::endl;
        if(cells_[n].type == UNEXPLORED){
            glColor3f(0.6,0.6,0.6);
        }else if(cells_[n].type == FREE){
            if(cells_[n].segments_ != nullptr){
                int realID = cells_[n].segments_->getRealID();
                if(realID == -1){
                    glColor3f(1.0,1.0,1.0);
                }else{
                    float var =  (std::fmod(realID,2.0) == 0)?1:0;
                    glColor3f(var,1.0 - realID/20.0,realID/20.0);
                }
            }else{
                glColor3f(1,1,1);
            }
        }else if(cells_[n].type == OCCUPIED){
            glColor3f(0.0,0.0,0.0);
        }else if(cells_[n].type == NEAROBSTACLE){
            glColor3f(0.0,0.0,0.5);
        }
    }

    glBegin( GL_QUADS );
    {
        glVertex2f(cells_[n].x+1, cells_[n].y+1);
        glVertex2f(cells_[n].x+1, cells_[n].y  );
        glVertex2f(cells_[n].x  , cells_[n].y  );
        glVertex2f(cells_[n].x  , cells_[n].y+1);
    }
    glEnd();
}

void Grid::drawCellWithColor(int x, int y, float r, float g, float b)
{
    Cell* c = getCell(x,y);
//    std::cout << "x " << x << " e y " << y << std::endl;

    glColor3f(r,g,b);

    glBegin( GL_QUADS );
    {
        glVertex2f(c->x+1, c->y+1);
        glVertex2f(c->x+1, c->y  );
        glVertex2f(c->x  , c->y  );
        glVertex2f(c->x  , c->y+1);
    }
    glEnd();
}

void Grid::drawVector(unsigned int n)
{
    if(cells_[n].type == FREE){
        if(cells_[n].dirX == 0.0 && cells_[n].dirY == 0.0)
            glColor3f(1.0,0.7,0.0);
        else
            glColor3f(1.0,0.0,0.0);
        glLineWidth(1);
        glBegin( GL_LINES );
        {
            glVertex2f(cells_[n].x+0.5, cells_[n].y+0.5);
            glVertex2f(cells_[n].x+0.5+cells_[n].dirX, cells_[n].y+0.5+cells_[n].dirY);
        }
        glEnd();
        glBegin( GL_POINTS );
        {
            glVertex2f(cells_[n].x+0.5+cells_[n].dirX, cells_[n].y+0.5+cells_[n].dirY);
        }
        glEnd();
    }
}

void Grid::drawText(unsigned int n)
{
    glRasterPos2f(cells_[n].x+0.25, cells_[n].y+0.25);
    std::stringstream s;
    glColor3f(0.5f, 0.0f, 0.0f);
    //    s << std::setprecision(1) << std::fixed << cells_[n].pot;
    s << cells_[n].distanceFromGoal;
//    s << cells_[n].distanceFromRobot;


    std::string text=s.str();
    for (unsigned int i=0; i<text.size(); i++)
    {
        glutBitmapCharacter(GLUT_BITMAP_HELVETICA_10, text[i]);
    }
}

void Grid::setMyGoal(Cell *highAttractivity){
    myGoal = highAttractivity;
}

bool Grid::sameCell(Cell* c1,Cell* c2){
    if(c1->x == c2->x && c1->y == c2->y) return true;
    else return false;
}


int Segment::getRealID(){
    if(this->father) return this->father->getRealID();
    else return this->idSegment;
}

int Segment::getRealKind(){
    if(this->father) return this->father->getRealKind();
    else return this->kindSegment;
}

double Segment::getGrowing_Angle(){
    if(this->father) return this->father->getGrowing_Angle();
    else return this->growing_angle_;
}

std::map<std::string, Door*> Segment::getRealDoors(){
    if(this->father) return this->father->getRealDoors();
    else return this->doorsOfSegment;
}

Segment* Segment::getFather(){
    if(this->father) return this->father->getFather();
    else return this;
}

bool Segment::getHaveDoors(){
    if(this->father) return this->father->getHaveDoors();
    else return this->haveDoors;
}

void Segment::addDoor(Door* door){
    if(this->father)
        this->father->addDoor(door);
    else{
        haveDoors = true;
        std::map<std::string,Door*>::iterator itDoor = this->doorsOfSegment.find(door->name);
        if (itDoor == this->doorsOfSegment.end())
        {
           if(this->doorsOfSegment.size() == 0)
               bigger_door = smaller_door = door->number;
           else if(bigger_door < door->number)
                bigger_door = door->number;
           else if(smaller_door > door->number)
                smaller_door = door->number;

           this->doorsOfSegment.insert(std::make_pair(door->name, door));
        }
    }
}

void Segment::copyDoorsSon2Father(std::map<std::string, Door*> &doorsOfSegment){
    if(this->father)
        this->father->copyDoorsSon2Father(doorsOfSegment);
    else
        for(std::map<std::string, Door*>::iterator itValid = doorsOfSegment.begin(); itValid != doorsOfSegment.end(); ++itValid)
            this->addDoor((*itValid).second);
    doorsOfSegment.clear();
}

void Segment::printSegment(){
    bool test = false;
    if(this->father) test = true;
    int countdoors = 0;
    std::map<std::string,Door*> tes = this->getRealDoors();

    std::cout<<"------------------------ SEGMENT - "<<this->idSegment<<" ------------------------"<<std::endl;
    if(test){
        std::cout<<"| It has a father: "<<this->getRealID();
        for(int i = 0; i < 38; i++) std::cout<<" ";
        std::cout<<" |"<<std::endl;

        std::cout<<"| Its kind: "<<this->kindSegment<<" and its father's: "<<this->getRealKind();
        for(int i = 0; i < 25; i++) std::cout<<" ";
        std::cout<<" |"<<std::endl;
        std::cout<<"| Doors:";
    }else{
        std::cout<<"| It doesn't have a father.";
        for(int i = 0; i < 33; i++) std::cout<<" ";
        std::cout<<" |"<<std::endl;
        std::cout<<"| Its kind: "<<this->kindSegment;
        for(int i = 0; i < 47; i++) std::cout<<" ";
        std::cout<<" |"<<std::endl;
        std::cout<<"| Doors:";
    }
    if(!haveDoors){
        std::cout<<" no doors";
        for(int i = 0; i < 44; i++) std::cout<<" ";
        std::cout<<"|"<<std::endl;
    }else{
        for (std::map<std::string,Door*>::iterator itDoor = tes.begin(); itDoor != tes.end(); ++itDoor){
            if(countdoors % 5 == 0 && countdoors != 0) std::cout<<"|       ";
            std::cout << " " << (*itDoor).first << "-(" << (*itDoor).second->numObservations<< ")";
            countdoors++;
            if(countdoors % 5 == 0) std::cout<<"   |"<<std::endl;
        }
    }


    std::cout<<" |"<<std::endl;
    std::cout<<"-------------------------------------------------------------"<<std::endl<<std::endl;
}

double Segment::Parity_Attractiveness(int doorNumber, float w_tollerance){
    if(this->father) return this->father->Parity_Attractiveness(doorNumber, w_tollerance);

//    float bel_Odd = 0, bel_Even = 0;
    float attract = 0, same_parity = 0, diff_parity = 0, total_doors = 0;

//    //############# Thres com numero minimo de vezes que a porta tem que ser vista para existir
//    float threshouldDoor = 5;

//    //############# Peso para cada door na belief
//    float weightDoor = 0.1;

    for(std::map<std::string,Door*>::iterator itDoor = this->doorsOfSegment.begin(); itDoor != this->doorsOfSegment.end(); itDoor++){
//        float doorBel = 0;
//        if((*itDoor).second->numObservations > threshouldDoor) doorBel = 1;

//        if(std::stoi((*itDoor).first) % 2 != 0)
//            bel_Odd += weightDoor * doorBel;
//        else
//            bel_Even += weightDoor * doorBel;

        int curr_door = std::stoi((*itDoor).first);
        if((curr_door % 2 == 0 && doorNumber % 2 == 0) || (curr_door % 2 != 0 && doorNumber % 2 != 0))
            same_parity++;
        else
            diff_parity++;
        total_doors++;
    }

    attract = (same_parity - diff_parity) / max(w_tollerance, total_doors);
    return 0.5 + attract/2.0;

//    if(bel_Odd > 1 ) bel_Odd = 1;
//    if(bel_Even > 1 ) bel_Even = 1;

//    float diff = bel_Odd - bel_Even;
//    if(diff>0) diff=0;
//    this->att_odd_ = (0.5 + (bel_Odd + (diff * std::pow(std::abs(diff), 1)))/2.0);

//    diff = bel_Even - bel_Odd;
//    if(diff>0) diff=0;
//    this->att_even_ = (0.5 + (bel_Even + (diff * std::pow(std::abs(diff), 1)))/2.0);

//    //odd
//    if(doorNumber % 2 != 0)
//        return this->att_odd_;
//    else//even
//        return this->att_even_;
}

double Segment::Growing_Attractiveness(int doorNumber, double frontier_angle, float w_tollerance)
{
    if(this->father) return this->father->Growing_Attractiveness(doorNumber, frontier_angle, w_tollerance);

    if(this->doorsOfSegment.size() < 2) return 0.5;

    float big_dx = 0;
    float big_dy = 0;
    float small_doors = 0, big_doors = 0, total_doors = 0;
    float big_small_factor = 0;
    float diff_angle_factor = 0;
    int scale_vector = 1, amount_vectors = 0;

    for(std::map<std::string,Door*>::iterator itDoor1 = this->doorsOfSegment.begin(); itDoor1 != this->doorsOfSegment.end(); itDoor1++)
    {
        for(std::map<std::string,Door*>::iterator itDoor2 = this->doorsOfSegment.begin(); itDoor2 != this->doorsOfSegment.end(); itDoor2++)
        {
            if((*itDoor1).first == (*itDoor2).first || (*itDoor1).second->number <= (*itDoor2).second->number) continue;
            float dx = (*itDoor1).second->map_pose.x - (*itDoor2).second->map_pose.x;
            float dy = (*itDoor1).second->map_pose.y - (*itDoor2).second->map_pose.y;

            double angle = atan2(dy,dx);

            big_dx += (cos(angle) * scale_vector);
            big_dy += (sin(angle) * scale_vector);

            amount_vectors++;

        }        
//        ((*itDoor1).second->number > doorNumber)? big_small_factor -= 1.0/saturate : big_small_factor += 1.0/saturate;
        ((*itDoor1).second->number > doorNumber)? big_doors++ : small_doors++;
        total_doors++;
    }

    //big_dx and big_dy are the size of the difference among all door vectors. Total.
    float big_angle = atan2(big_dy,big_dx);

    if(big_angle > 2*PI) big_angle -= 2*PI;
    if(big_angle < 0) big_angle += 2*PI;

    this->growing_angle_ = big_angle;

    if(amount_vectors < min_vector_direction_)
        amount_vectors = min_vector_direction_;

    //find norm
//    float  norm;
//    if(cos(big_angle) != 0)
//        norm = (cos(big_angle)*big_dx)/amount_vectors;
//    else
//        norm = (sin(big_angle)*big_dy)/amount_vectors;

//    if(isnan(norm)) norm = 0;

    float diff = atan2(sin(big_angle - frontier_angle), cos(big_angle - frontier_angle));

    if(big_small_factor > 1) big_small_factor = 1;
    if(big_small_factor < -1) big_small_factor = -1;

    diff_angle_factor = 1 + fabs((diff/PI)) * -2;

    float factor = (small_doors - big_doors) / std::max(total_doors, w_tollerance);
//    std::cout << "doorNbr:" << doorNumber
//              << " | smll_drs:" << small_doors
//              << " | bg_drs:" << big_doors
//              << " | total:" << total_doors
//              << " | F:" << factor
//              << " | D:" << diff_angle_factor
//              << " | (D*F)/2:" << (factor * diff_angle_factor) / 2.0 << std::endl;

    return ((factor * diff_angle_factor) / 2.0) + 0.5;

}

double Segment::getAtt_Odd(){
    if(this->father) return this->father->getAtt_Odd();
    return this->att_odd_;
}
double Segment::getAtt_Even(){
    if(this->father) return this->father->getAtt_Even();
    return this->att_even_;
}



//https://gist.github.com/Thileban/88f425a9a8ea947b78beb4f9f2dd7da9
void Segment::myRansac(){
    int i,j,k,n;
    double x[n],y[n],a,b;

    std::map<std::string,Door*> doors;
    doors = this->getRealDoors();

    n = doors.size();
    double xsum=0,x2sum=0,ysum=0,xysum=0;                //variables for sums/sigma of xi,yi,xi^2,xiyi etc
    std::map<std::string, Door*>::iterator it;
    for(it = doors.begin(); it != doors.end(); it++){
        xsum = xsum + it->second->pose.x;                        //calculate sigma(xi)
        ysum = ysum + it->second->pose.y;                        //calculate sigma(yi)
        x2sum = x2sum + pow(it->second->pose.x,2);                //calculate sigma(x^2i)
        xysum = xysum + it->second->pose.x * it->second->pose.y;                    //calculate sigma(xi*yi)
    }

    a=(n*xysum-xsum*ysum)/(n*x2sum-xsum*xsum);            //calculate slope
    b=(x2sum*ysum-xsum*xysum)/(x2sum*n-xsum*xsum);            //calculate intercept
    double y_fit[n];                        //an array to store the new fitted values of y
    for (i=0;i<n;i++)
        y_fit[i]=a*it->second->pose.x+b;                    //to calculate y(fitted) at given x points
//    std::cout<<"S.no"<<setw(5)<<"x"<<setw(19)<<"y(observed)"<<setw(19)<<"y(fitted)"<<endl;
//    std::cout<<"-----------------------------------------------------------------\n";
//    for (i=0;i<n;i++)
//        std::cout<<it->second->name<<"."<<setw(8)<<it->second->pose.x<<setw(15)<<it->second->pose.y<<setw(18)<<y_fit[i]<<endl;//print a table of x,y(obs.) and y(fit.)
//    std::cout<<"\nThe linear fit line is of the form:\n\n"<<a<<"x + "<<b<<endl;        //print the best fit line

}
