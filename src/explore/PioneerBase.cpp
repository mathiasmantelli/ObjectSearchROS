#include "PioneerBase.h"

#include <GL/glut.h>


PioneerBase::PioneerBase()
{
    isSimulation = true;
    srcPath = "../Phi-Exploration";

    // sensors variables
    numSonars_ = 8;
    sonars_.resize(numSonars_, 0.0);
    numLasers_ = 181;
    lasers_.resize(numLasers_, 0.0);
    maxLaserRange_ = 6.5; // 5.0;
    maxSonarRange_ = 5.0; // 5.0;


    // wheels' velocities
    vLeft_ = vRight_ = 0.0;
}

///////////////////////////
///// DRAWING METHODS /////
///////////////////////////

void PioneerBase::drawBase()
{
    glColor3f(1.0,0.0,0.0);
    glBegin( GL_POLYGON );
    {
        glVertex2f(-20, -8);
        glVertex2f(-13, -15);
        glVertex2f(8, -15);
        glVertex2f(15, -8);
        glVertex2f(15, 8);
        glVertex2f(8, 15);
        glVertex2f(-13, 15);
        glVertex2f(-20, 8);
    }
    glEnd();
    glColor3f(0.0,0.0,0.0);
    glBegin( GL_LINE_STRIP );
    {
        glVertex2f(0, 0);
        glVertex2f(30, 0);
    }
    glEnd();
}

void PioneerBase::drawSonars(bool drawCones)
{
    float angles[8] = {DEG2RAD(-90), DEG2RAD(-50), DEG2RAD(-30), DEG2RAD(-10),
                       DEG2RAD(10), DEG2RAD(30), DEG2RAD(50), DEG2RAD(90)};
    std::vector<float> s = getSonarReadings();

    glRotatef(-90,0.0,0.0,1.0);

    if(drawCones){

        float fov=DEG2RAD(15);
        float hfov=fov/2.0;

        for(int i=0; i<8; i++){
            if(i%2==0)
                glColor4f(1.0,1.0,0.0,0.5);
            else
                glColor4f(0.0,1.0,1.0,0.3);

            glBegin( GL_POLYGON);
            {
                glVertex2f(s[i]*sin(angles[i]-fov)*100, s[i]*cos(angles[i]-fov)*100);
                glVertex2f(s[i]*sin(angles[i]-hfov)*100, s[i]*cos(angles[i]-hfov)*100);
                glVertex2f(s[i]*sin(angles[i])*100, s[i]*cos(angles[i])*100);
                glVertex2f(s[i]*sin(angles[i]+hfov)*100, s[i]*cos(angles[i]+hfov)*100);
                glVertex2f(s[i]*sin(angles[i]+fov)*100, s[i]*cos(angles[i]+fov)*100);
                glVertex2f(0, 0);
            }
            glEnd();
        }

    }else{
        for(int i=0; i<8; i++){
            if(i%2==0)
                glColor3f(0.7,0.7,0.0);
            else
                glColor3f(0.0,0.7,0.7);

            glBegin( GL_LINES);
            {
                glVertex2f(s[i]*sin(angles[i])*100, s[i]*cos(angles[i])*100);
                glVertex2f(0, 0);
            }
            glEnd();
        }

    }

    glRotatef(90,0.0,0.0,1.0);
}

void PioneerBase::drawLasers(bool fill)
{
    std::vector<float> s = getLaserReadings();
    float angle = DEG2RAD(-90.0);

    int inc = 2;
    float angleInc = DEG2RAD(inc);

    glRotatef(-90,0.0,0.0,1.0);

    if(fill){
        glColor4f(0.0,1.0,0.0,0.3);
        glBegin( GL_POLYGON);
        {
            for(int i=0;i<s.size(); i+=inc)
            {
                glVertex2f(s[i]*sin(angle)*100, s[i]*cos(angle)*100);
                angle += angleInc;
            }
            glVertex2f(0, 0);
        }
        glEnd();
    }else{
        glColor3f(0.0,0.7,0.0);
        glBegin( GL_LINES);
        {
            for(int i=0;i<s.size(); i+=inc)
            {
                glVertex2f(s[i]*sin(angle)*100, s[i]*cos(angle)*100);
                glVertex2f(0, 0);
                angle += angleInc;
            }
        }
        glEnd();
    }

    glRotatef(90,0.0,0.0,1.0);
}

////////////////////////////////////////////////////////////////
////// METHODS FOR READING ODOMETRY & SENSORS MEASUREMENTS /////
////////////////////////////////////////////////////////////////

float PioneerBase::getMinSonarValueInRange(int idFirst, int idLast)
{
    float min = sonars_[idFirst];
    for(int i=idFirst+1; i<=idLast; i++)
        if(sonars_[i]<min)
            min = sonars_[i];

    return min;
}

float PioneerBase::getMinLaserValueInRange(int idFirst, int idLast, int kernelSize)
{
    float min = 1000000;
    for(int i=idFirst; i<=idLast; i++){
        float val=0;
        int c=0;
        for(int j=i-kernelSize;j<=i+kernelSize;j++)
            if(j>=0 && j<numLasers_){
                val+=lasers_[j];
                c++;
            }
        val /= c;

        if(val<min)
            min = val;
    }

    return min;
}

void PioneerBase::setGrid(Grid *g)
{
    grid_ = g;
}

float PioneerBase::getMaxLaserRange()
{
    return maxLaserRange_;
}

int PioneerBase::getNumLasers()
{
    return numLasers_;
}

float PioneerBase::getMaxSonarRange()
{
    return maxSonarRange_;
}

int PioneerBase::getNumSonars()
{
    return numSonars_;
}

const Pose& PioneerBase::getOdometry()
{
    return odometry_;
}

const Pose& PioneerBase::getSLAMPose()
{
    return slamPose_;
}

const Pose& PioneerBase::getAbsoluteTruePose()
{
    return simAbsoluteTruePose_;
}

const Pose& PioneerBase::getRelativeTruePose()
{
    return simRelativeTruePose_;
}

const std::set<std::string> &PioneerBase::getNearbyDoorsLeft()
{
    return nearbyDoorsLeft;
}

const std::set<std::string> &PioneerBase::getNearbyDoorsRight()
{
    return nearbyDoorsRight;
}

const std::vector<float> &PioneerBase::getLaserReadings()
{
    return lasers_;
}

const std::vector<float> &PioneerBase::getSonarReadings()
{
    return sonars_;
}

void PioneerBase::setOdometry(const Pose &o)
{
    odometry_ = o;
}

void PioneerBase::setSonarReadings(const std::vector<float> &s)
{
    sonars_ = s;
}

void PioneerBase::setLaserReadings(const std::vector<float> &l)
{
    lasers_ = l;
}

int PioneerBase::getNearestSonarBeam(float angle)
{
    if(angle>70.0)
        return 0;
    else if(angle>40 && angle<=70)
        return 1;
    else if(angle>20 && angle<=40)
        return 2;
    else if(angle>0 && angle<=20)
        return 3;
    else if(angle>-20 && angle<=0)
        return 4;
    else if(angle>-40 && angle<=-20)
        return 5;
    else if(angle>-70 && angle<=-40)
        return 6;
    else //if(angle<=-70.0)
        return 7;

}

float sonarAngles_[8] = {90, 50, 30, 10, -10, -30, -50, -90};

float PioneerBase::getAngleOfSonarBeam(int k)
{
    return sonarAngles_[k];
}

int PioneerBase::getNearestLaserBeam(float angle)
{
    // k = 0   -- angle  90
    // k = 90  -- angle   0
    // k = 180 -- angle -90

    if(angle>90.0)
        return 0;
    else if(angle<-90.0)
        return 180;
    else{
        return 90-(int)((angle > 0.0)?(angle + 0.5):(angle - 0.5));
    }
}

float PioneerBase::getAngleOfLaserBeam(int k)
{
    // k = 0   -- angle  90
    // k = 90  -- angle   0
    // k = 180 -- angle -90

    return 90.0-(float)k;
}

float PioneerBase::getKthSonarReading(int k)
{
    return sonars_[k];
}

float PioneerBase::getKthLaserReading(int k)
{
    return lasers_[k];
}


//////////////////////////////
///// NAVIGATION METHODS /////
//////////////////////////////

void PioneerBase::setMovementSimple(MovingDirection dir)
{
    double maxVel = 500;

    switch(dir){
        case FRONT:
            vLeft_ = 300;
            vRight_ = 300;
            break;
        case BACK:
            vLeft_ = -300;
            vRight_ = -300;
            break;
        case LEFT:
            vLeft_ = -60;
            vRight_ = 60;
            break;
        case RIGHT:
            vLeft_ = 60;
            vRight_ = -60;
            break;
        case STOP:
            vLeft_  = 0;
            vRight_ = 0;
            linVel_ = 0;
            angVel_ = 0;
            oldVLeft_  = vLeft_;
            oldVRight_ = vRight_;
            break;
        case RESTART:
            vLeft_  = oldVLeft_;
            vRight_ = oldVRight_;
            break;
    }

    if(vLeft_ > maxVel)
        vLeft_ = maxVel;
    else if(vLeft_ < -maxVel)
        vLeft_ = -maxVel;
    if(vRight_ > maxVel)
        vRight_ = maxVel;
    else if(vRight_ < -maxVel)
        vRight_ = -maxVel;

    setLinAngVelocity_fromWheelsVelocity(vLeft_,vRight_);

    std::cout << "vLeft_:" << vLeft_ << " vRight_:" << vRight_ << std::endl;
}

void PioneerBase::setMovementVel(MovingDirection dir)
{
    switch(dir){
        case DEC_LIN_VEL:
            if(linVel_ > -0.5)
                linVel_ -= 0.1;
            break;
        case INC_LIN_VEL:
            if(linVel_ < 0.5)
                linVel_ += 0.1;
            break;
        case DEC_ANG_VEL:
            if(angVel_ > -1.0)
                angVel_ -= 0.05;
            break;
        case INC_ANG_VEL:
            if(angVel_ < 1.0)
                angVel_ += 0.05;
            break;
        case STOP:
            linVel_ = 0;
            angVel_ = 0;
            break;
    }

    setWheelsVelocity_fromLinAngVelocity(linVel_,angVel_);
}

void PioneerBase::setLinAngVelocity_fromWheelsVelocity(float lv, float rv)
{
    float b=0.38;

    // the robot's velocity is given in mm/s
    lv /= 1000.0;
    rv /= 1000.0;

    linVel_ = (lv + rv)/2.0;
    angVel_ = (rv - lv)/b;

    if(linVel_ > 1.0)
        linVel_ = 1.0;
    else if(linVel_ < -1.0)
        linVel_ = -1.0;

    if(angVel_ > DEG2RAD(90.0))
        angVel_ = DEG2RAD(90.0);
    else if(angVel_ < DEG2RAD(-90.0))
        angVel_ = DEG2RAD(-90.0);

//    std::cout << "linVel: " << linV << " angVel: " << angV << std::endl;
//    std::cout << "vLeft_: " << vLeft_ << " vRight_: " << vRight_ << std::endl;

}

void PioneerBase::setWheelsVelocity_fromLinAngVelocity(float linV, float angV)
{
    float b=0.38;

    vLeft_  = linV - angV*b/(2.0);
    vRight_ = linV + angV*b/(2.0);

    // the robot's velocity is given in mm/s
    vLeft_ *= 1000;
    vRight_ *= 1000;

    double maxVel = 500;
    if(vLeft_ > maxVel)
        vLeft_ = maxVel;
    else if(vLeft_ < -maxVel)
        vLeft_ = -maxVel;
    if(vRight_ > maxVel)
        vRight_ = maxVel;
    else if(vRight_ < -maxVel)
        vRight_ = -maxVel;

//    std::cout << "linVel: " << linV << " angVel: " << angV << std::endl;
//    std::cout << "vLeft_: " << vLeft_ << " vRight_: " << vRight_ << std::endl;

}

void PioneerBase::setWheelsVelocity(float lv, float rv)
{
    vLeft_ = lv;
    vRight_ = rv;

    setLinAngVelocity_fromWheelsVelocity(vLeft_,vRight_);
}

void PioneerBase::setLinAngVelocity(float linV, float angV)
{
    linVel_ = linV;
    angVel_ = angV;

    setWheelsVelocity_fromLinAngVelocity(linVel_,angVel_);
}

/////////////////////////////////////////////////////
////// METHODS FOR READING & WRITING ON LOGFILE /////
/////////////////////////////////////////////////////

// Prints to file the data that we would normally be getting from sensors, such as the laser and the odometry.
// This allows us to later play back the exact run.
void PioneerBase::writeOnLog()
{
    logFile_->writePose("Odometry",odometry_);
    logFile_->writeSensors("Sonar",sonars_);
    logFile_->writeSensors("Laser",lasers_);
}

// Reads back into the sensor data structures the raw readings that were stored to file
// While there is still information in the file, it will return 0. When it reaches the end of the file, it will return 1.
bool PioneerBase::readFromLog() {

    if(logFile_->hasEnded())
        return true;

    setOdometry(logFile_->readPose("Odometry"));
    setSonarReadings(logFile_->readSensors("Sonar"));
    setLaserReadings(logFile_->readSensors("Laser"));

    return false;
}

//void PioneerBase::reset_pose(float x, float y, float th)
//{
//    ArRobotPacket pkt;
//    pkt.setID(ArCommands::SIM_SET_POSE);
//    pkt.uByteToBuf(0); // argument type: ignored.
//    pkt.byte4ToBuf(x);
//    pkt.byte4ToBuf(y);
//    pkt.byte4ToBuf(th);
//    pkt.finalizePacket();
//    robot_.getDeviceConnection()->write(pkt.getBuf(), pkt.getLength());

//}

