#ifndef __GRID_H__
#define __GRID_H__

#include <string>
#include <set>
#include <vector>
#include <math.h>
#include "Doors.h"
#include "Utils.h"


#define UNDEF -10000000

enum CellType {OCCUPIED, UNEXPLORED, FREE, NEAROBSTACLE};

typedef struct
{
    int x,y;
    float theta;
} robotCell;

typedef struct
{
    int minX, maxX, minY, maxY;
} bbox;

class Segment
{
public:
    Segment(int min_vector_direction_);

    int idSegment, kindSegment;
    int min_vector_direction_;

    Segment* father;
    std::map<std::string,Door*> doorsOfSegment;


    int getRealID();
    int getRealKind();
    std::map<std::string, Door *> getRealDoors();
    Segment* getFather();
    float getBelDoor();
    double getAtt_Odd();
    double getAtt_Even();
    double getGrowing_Angle();
    bool getHaveDoors();

    void addDoor(Door *door);
    void copyDoorsSon2Father(std::map<std::string, Door *> &doorsOfSegment);
    void printSegment();
    void myRansac();

    double Parity_Attractiveness(int doorNumber, float w_tollerance_);

    double Growing_Attractiveness(int doorNumber, double frontier_angle, float w_tollerance_);

private:
    double att_odd_, att_even_;
    double growing_angle_;
    bool haveDoors;
    int bigger_door, smaller_door, amountOrientation;


};

class Cell
{
public:
    Cell();
    int x,y;
    int himm, himm_count;
    int slamValue;
    double pref;
    double distWalls;

    double pot, dirX, dirY, norm;

    CellType type;
    int border;
    bool isCenter, isVisitedCenter, isEndPoint, isVoronoiHelper;

    int distanceFromRobot;
    int distanceFromNearestCandidate;

    float attractiveness;

    float frontier_direction;

    float att_even;
    float att_odd;
    std::set<std::string> doors;
    int planCounter, distanceFromGoal;

    bool isVisited;
    double isNearOddRoom, isNearEvenRoom;
    double sumDtGN, countDtGN, differenceToGoalNumber;

    Segment* segments_;

};

class Grid
{
public:
    Grid();
    Cell* getCell(int x, int y);



    int getMapScale();
    int getMapWidth();
    int getMapHeight();
    bool sameCell(Cell* c1,Cell* c2);
    void setMyGoal(Cell* highAttractivity);

    void draw(int xi, int yi, int xf, int yf);

    int numViewModes;
    int viewMode;
    bool showValues;
    bool showArrows;

    int himm_count;

    int planIteration;

    int currentDoorGoal;

    robotCell robotVoronoiCell;
    robotCell goalVoronoiCell;
    std::vector<Cell*> goals;

    bbox SLAMlimits;

private:
    int mapScale_; // Number of cells per meter
    int mapWidth_, mapHeight_; // in cells
    int numCellsInRow_, halfNumCellsInRow_;

    Cell* cells_, *myGoal;

    void drawCell(unsigned int i);
    void drawCellWithColor(int x, int y, float r, float g, float b);
    void drawVector(unsigned int i);
    void drawText(unsigned int n);
};

#endif // __GRID_H__
