#include "Robot.h"

#include <unistd.h>
#include <GL/glut.h>
#include <cmath>
#include <iostream>
#include <iomanip>

//////////////////////////////////////
///// CONSTRUCTORS & DESTRUCTORS /////
//////////////////////////////////////

//Robot::Robot(int argc, char **argv):
Robot::Robot(Configuration* config):
    base()
{
    configuration = config;

    ready_ = false;
    running_ = true;

    grid = new Grid();

    plan = new Planning(config);
    plan->setGrid(grid);
    plan->setLocalRadius(base.getMaxLaserRange());

    base.setGrid(grid);

    // variables used for mapping
    doWallThickening=false;
    minX_=minY_=INT_MAX;
    maxX_=maxY_=INT_MIN;

    // variables used for navigation
    avoidDistance_ = 0.4;
    largeDistance_ = 0.5;
    smallDistance_ = 0.4;
    closeToWall_ = false;
    incCTE_ = oldCTE_ = 0.0;
    isFollowingLeftWall_=false;

    // variables used for visualization
    viewMode=0;
    numViewModes=4;

    limit_frames = false;
}

Robot::~Robot()
{
    //base.closeARIAConnection();
    if(grid!=NULL)
        delete grid;
}

////////////////////////////////////
///// INITIALIZE & RUN METHODS /////
////////////////////////////////////

void Robot::initialize(ConnectionMode cmode, LogMode lmode, std::string fname)
{
    // initialize logfile
    logMode_ = lmode;

    // initialize ARIA
    if(logMode_!=PLAYBACK){
        bool success = base.initialize();
//        bool success = base.initialize(cmode,lmode,fname);
        if(!success){
            printf("Could not connect to robot... exiting\n");
            exit(0);
        }

//        base.reset_pose(configuration->GetInt("init_x"),configuration->GetInt("init_y"),configuration->GetInt("init_th"));
    }

//    ready_ = true;
    controlTimer.startLap();
}

void Robot::startingMovement()
{
    float startingTheta = getCurrentPose().theta;

    base.setWheelsVelocity(50,-50);

    sleep(5);

    while(fabs(startingTheta - getCurrentPose().theta) > 10){}

    base.setWheelsVelocity(0,0);
}

void Robot::run()
{
    controlTimer.waitTime(0.1);

    if(logMode_==PLAYBACK){
        bool hasEnded = base.readFromLog();
        if(hasEnded){
            std::cout << "PROCESS COMPLETE. CLOSING PROGRAM." << std::endl;
            exit(0);
        }
    }else{
        bool success = base.readOdometryAndSensors();
        if(!success){
            usleep(50000);
            return;
        }

        if(logMode_==RECORDING)
            base.writeOnLog();
    }

    //currentPose_ = base.getOdometry();
    currentPose_ = base.getSLAMPose();
    absCurrentPose_ = base.getAbsoluteTruePose();

//    std::cout << "ABS " << base.getAbsoluteTruePose()
//              << " - REL " << base.getRelativeTruePose()
//              << " - ODOM " << currentPose_;

    nearbyDoorsLeft = base.getNearbyDoorsLeft();
    nearbyDoorsRight = base.getNearbyDoorsRight();

//    this->printNearbyDoors();

    std::set<Door*> doorsLeft;
    std::set<Door*> doorsRight;

    if(!nearbyDoorsLeft.empty())
        doorsLeft = plan->doors.updateLeft(currentPose_,nearbyDoorsLeft);
    if(!nearbyDoorsRight.empty())
        doorsRight = plan->doors.updateRight(currentPose_,nearbyDoorsRight);

    //Merging the left and right doors into one unique list. We don't have to know how many times the doors have been seen
    unionDoors.clear();
    std::set_union(doorsLeft.begin(), doorsLeft.end(),
                   doorsRight.begin(), doorsRight.end(),
                   std::inserter(unionDoors,unionDoors.begin()));


    plan->doors.unionDoors = this->unionDoors;


//    plan->doors.print();

    // Mapping
//    updateGridUsingHIMM();

    plan->setNewRobotPose(currentPose_);

    // Save path traversed by the robot
    if(base.isMoving() || logMode_== PLAYBACK){
        path_.push_back(base.getOdometry());
    }

    // Navigation
    switch(motionMode_){
        case POTFIELD:
            followPotentialField(false);
            break;
        case LOCALPOTFIELD:
            followPotentialField(true);
            break;
        case ENDING:
            running_=false;
            break;
        default:
            break;
    }

    base.resumeMovement();

    if(base.getLaserReadings().size()>0)
        ready_ = true;

    usleep(50000);
}

//////////////////////////////
///// NAVIGATION METHODS /////
//////////////////////////////

void Robot::move(MovingDirection dir)
{
    switch(dir){
        case FRONT:
            std::cout << "moving front" << std::endl;
            break;
        case BACK:
            std::cout << "moving back" << std::endl;
            break;
        case LEFT:
            std::cout << "turning left" << std::endl;
            break;
        case RIGHT:
            std::cout << "turning right" << std::endl;
            break;
        case STOP:
            std::cout << "stopping robot" << std::endl;
    }

    if(motionMode_==MANUAL_SIMPLE)
        base.setMovementSimple(dir);
    else if(motionMode_==MANUAL_VEL)
        base.setMovementVel(dir);
    else if(motionMode_=WALLFOLLOW)
        if(dir==LEFT)
            isFollowingLeftWall_=true;
        else if(dir==RIGHT)
            isFollowingLeftWall_=false;
}

void Robot::followPotentialField(bool local)
{
    float linVel=0.1;
    float angVel=0.0;

    Cell* c;
    int robotX=currentPose_.x*grid->getMapScale();
    int robotY=currentPose_.y*grid->getMapScale();

    c = grid->getCell(robotX,robotY);

    float dirX, dirY;
    if(local){
        dirX = c->dirX;
        dirY = c->dirY;
    }else{
        dirX = c->dirX;
        dirY = c->dirY;
    }

    if(dirX == 0.0 && dirY == 0.0){
        linVel = angVel = 0.0;
    }else{

        float phi = RAD2DEG(atan2(dirY,dirX)) - currentPose_.theta;
        phi = normalizeAngleDEG(phi);

//        std::cout << "PHI " << phi << std::endl;

        if(phi<-90.0){
            linVel = 0.0;
            angVel = -0.5;
        }else if(phi>90.0){
            linVel = 0.0;
            angVel = 0.5;
        }else{
            angVel = (phi/90.0)*(linVel*3.0);

        }

    }


    base.setLinAngVelocity(linVel,angVel);
}

///////////////////////////
///// MAPPING METHODS /////
///////////////////////////

void Robot::updateGridUsingHIMM()
{
    int scale = grid->getMapScale();

    float alpha = 0.1; //10 cm
    float beta = 10.0;  //0.5 degrees

    int kernel=1;

    int robotX=currentPose_.x*scale;
    int robotY=currentPose_.y*scale;

    float maxRange = base.getMaxLaserRange();
    int maxRangeInt = maxRange*scale;

    float robotAngle = currentPose_.theta;

    Cell* r=grid->getCell(robotX,robotY);
    r->himm -= 1;
    if(r->himm<0)
        r->himm=0;

    const std::vector<float>& laser = base.getLaserReadings();

    for(int i=robotX-maxRangeInt; i<=robotX+maxRangeInt; i++){
        for(int j=robotY-maxRangeInt; j<=robotY+maxRangeInt; j++){

            Cell* c = grid->getCell(i,j);

            float r = sqrt(pow(i-robotX,2.0)+pow(j-robotY,2.0))/scale;
            float phi = RAD2DEG(atan2(j-robotY,i-robotX)) - robotAngle;
            phi = normalizeAngleDEG(phi);

            int k = base.getNearestLaserBeam(phi);

            if(phi < 90.0+beta && phi > -90.0-beta){
                if(laser[k] < maxRange && fabs(r-laser[k])<alpha){
                    c->himm += 3;
                    if(c->himm>15)
                        c->himm=15;
                    c->slamValue=90;

                    for(int i2=i-kernel;i2<=i+kernel;i2++)
                        for(int j2=j-kernel;j2<=j+kernel;j2++){
                            Cell* t = grid->getCell(i2,j2);
                            t->himm += 3;
                            if(t->himm>15)
                                t->himm=15;
                            t->slamValue=90;
                        }

                }
                else if(r <= laser[k] && r < maxRange){
                    c->himm -= 1;
                    if(c->himm<0)
                        c->himm=0;
                    c->slamValue=20;

                }
            }

        }
    }
}

/////////////////////////////////////////////////////
////// METHODS FOR READING & WRITING ON LOGFILE /////
/////////////////////////////////////////////////////

// Prints to file the data that we would normally be getting from sensors, such as the laser and the odometry.
// This allows us to later play back the exact run.
void Robot::writeOnLog()
{
    logFile_->writePose("Odometry",currentPose_);
    logFile_->writeSensors("Sonar",base.getSonarReadings());
    logFile_->writeSensors("Laser",base.getLaserReadings());
}

// Reads back into the sensor data structures the raw readings that were stored to file
// While there is still information in the file, it will return 0. When it reaches the end of the file, it will return 1.
bool Robot::readFromLog() {

    if(logFile_->hasEnded())
        return true;

    base.setOdometry(logFile_->readPose("Odometry"));
    base.setSonarReadings(logFile_->readSensors("Sonar"));
    base.setLaserReadings(logFile_->readSensors("Laser"));

    return false;
}

////////////////////////
///// DRAW METHODS /////
////////////////////////

void Robot::draw(double xRobot, double yRobot, double angRobot)
{


    double scale = grid->getMapScale();
    glTranslatef(xRobot,yRobot,0.0);

    drawPotGradient(scale);

    glRotatef(angRobot,0.0,0.0,1.0);
    glScalef(1.0/scale,1.0/scale,1.0/scale);

    // sonars and lasers draw in cm
    if(viewMode==1)
        base.drawSonars(true);
    else if(viewMode==2)
        base.drawSonars(false);
    else if(viewMode==3)
        base.drawLasers(true);
    else if(viewMode==4)
        base.drawLasers(false);

    // robot draw in cm
    base.drawBase();

    glScalef(scale,scale,scale);
    glRotatef(-angRobot,0.0,0.0,1.0);
    glTranslatef(-xRobot,-yRobot,0.0);
}

void Robot::drawPotGradient(double scale)
{
    Cell* c;
    int robotX=currentPose_.x*scale;
    int robotY=currentPose_.y*scale;
    c = grid->getCell(robotX,robotY);

    glColor3f(0.0,0.6,0.2);
    glLineWidth(3);
    glBegin( GL_LINE_STRIP );
    {
        glVertex2f(0, 0);
        glVertex2f(5*c->dirX, 5*c->dirY);
    }
    glEnd();

    glColor3f(0.0,0.2,0.6);
    glLineWidth(3);
    glBegin( GL_LINE_STRIP );
    {
        glVertex2f(0, 0);
        glVertex2f(5*c->dirX, 5*c->dirY);
    }
    glEnd();
}

/////////////////////////
///// OTHER METHODS /////
/////////////////////////

bool Robot::isReady()
{
    return ready_;
}

bool Robot::isRunning()
{
    return running_;
}

float Robot::ComputePathSize()
{
    float path_size = 0;

    if(path_.size() > 1)
        for(unsigned int i=0;i<path_.size()-1; i++)
            path_size += sqrt(pow(path_[i].x-path_[i+1].x, 2) + pow(path_[i].y-path_[i+1].y, 2));

    return path_size;
}

const Pose& Robot::getCurrentPose()
{
    return currentPose_;
}

const Pose& Robot::getAbsCurrentPose()
{
    return absCurrentPose_;
}

const std::string& Robot::getSrcPath()
{
    return base.srcPath;
}

void Robot::drawPath()
{
    if(grid->viewMode != 7 && grid->viewMode != 8){
        double scale = grid->getMapScale();

        if(path_.size() > 1){
            glScalef(scale,scale,scale);
            glLineWidth(3);
            glBegin( GL_LINE_STRIP );
            {
                for(unsigned int i=0;i<path_.size()-1; i++){
                    glColor3f(1.0,0.0,1.0);

                    glVertex2f(path_[i].x, path_[i].y);
                    glVertex2f(path_[i+1].x, path_[i+1].y);
                }
            }
            glEnd();
            glLineWidth(1);
            glScalef(1.0/scale,1.0/scale,1.0/scale);

        }
    }
}

void Robot::waitTime(float t){
    float l;
    do{
        usleep(1000);
        l = controlTimer.getLapTime();
    }while(l < t);
    controlTimer.startLap();
}

void Robot::printNearbyDoors()
{
    std::cout <<" - LEFT DOORS ";
    std::set<std::string>:: iterator i;
    for(i = this->nearbyDoorsLeft.begin(); i != this->nearbyDoorsLeft.end(); i++)
        std::cout <<*i<<" ";
    std::cout <<" - RIGHT DOORS ";
    for(i = this->nearbyDoorsRight.begin(); i != this->nearbyDoorsRight.end(); i++)
        std::cout <<*i<<" ";
    std::cout << std::endl;
}
