#pragma once

#ifndef ROBOT_H
#define ROBOT_H

#include <vector>
//using namespace std;

#include "Doors.h"
#include "Grid.h"
#include "Planning.h"
#include "Utils.h"
#include "Configuration.h"

//#include "PioneerBase_ARIA.h"
#include "PioneerBase_ROS.h"

class Robot
{
public:
    Robot(Configuration* config);
    ~Robot();

    void initialize(ConnectionMode cmode, LogMode lmode, std::string fname);
    void run();

    void move(MovingDirection dir);
    void draw(double xRobot, double yRobot, double angRobot);

    void startingMovement();

    const Pose& getCurrentPose();
    const Pose& getAbsCurrentPose();
    const std::string& getSrcPath();

    void drawPath();
    void printNearbyDoors();
    bool isReady();
    bool isRunning();

    float ComputePathSize();

    Grid* grid;
    Planning* plan;
    MotionMode motionMode_;
    int viewMode;
    int numViewModes;
    std::set<std::string> nearbyDoors; 
    std::set<std::string> nearbyDoorsLeft;
    std::set<std::string> nearbyDoorsRight;

    std::set<Door*> unionDoors;

    Configuration* configuration;

    bool limit_frames;

protected:

    Pose currentPose_;
    Pose absCurrentPose_;
    std::vector<Pose> path_;

    bool ready_;
    bool running_;

    // ARIA stuff
//    PioneerBase_ARIA base;
    PioneerBase_ROS base;

    // Log stuff
    LogFile* logFile_;
    LogMode logMode_;
    void writeOnLog();
    bool readFromLog();

    // Navigation stuff
    float avoidDistance_, smallDistance_, largeDistance_;
    bool closeToWall_;
    float oldCTE_, incCTE_;
    void wanderAvoidingCollisions();
    void wallFollow();
    void wallFollowPID();
    bool isFollowingLeftWall_;

    void followPotentialField(bool local);

    // Mapping stuff
    int minX_, minY_, maxX_, maxY_;
    bool doWallThickening;
    float getOccupancyFromLogOdds(float logodds);

    void updateGridUsingHIMM();
    void updateGridUsingHIMM2();
    void updateGridUsingLogOdds();
    void updateGridUsingLogOddsSonar();

    void drawPotGradient(double scale);

    Timer controlTimer;
    void waitTime(float t);

};

#endif // ROBOT_H
