#ifndef DOORS_H
#define DOORS_H

#include <string>
#include <set>
#include <map>
#include <list>

#include "Utils.h"

class Door
{
public:
    Door(std::string);
    bool addNeighbor(Door*);

    std::string name;
    int number;
    Pose pose;
    Pose sumPoses;

    Pose map_pose;
    Pose sum_map_poses;

    int numObservations;

    std::vector<Door*> neighbors;

};

class Doors
{
public:
    Doors();

    std::set<Door*> updateRight(const Pose &robotPose, const std::set<std::string> &observedDoorsRight);
    std::set<Door *> updateLeft(const Pose &robotPose, const std::set<std::string> &observedDoorsLeft);
    void updateRecent(const std::set<std::string> &observedDoorsLeft, const std::set<std::string> &observedDoorsRight);

    void print();
    void draw(double scale);
    void drawGraph(double scale);


    std::map<std::string,Door*> doorsMap;
    std::list<std::pair<std::string,int> > recentDoors;

    std::set<Door*> unionDoors;

    int numOddRoomsNearRobot;
    int numEvenRoomsNearRobot;
    double avgDoorNumberNearRobot;
    bool newObservation;
    int minValidObservations;

private:
    int sizeOfRecentDoors;
};

#endif // DOORS_H
