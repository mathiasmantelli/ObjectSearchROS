#ifndef PIONEERBASE_ARIA_H
#define PIONEERBASE_ARIA_H

#include <Aria.h>

#include "PioneerBase.h"
#include "Utils.h"

class PioneerBase_ARIA : public PioneerBase
{
public:
    PioneerBase_ARIA();

    // ARIA stuff
    bool initialize(ConnectionMode cmode, LogMode lmode, std::string fname);
    void closeARIAConnection();

    // Navigation stuff
    bool isMoving();
    void resumeMovement();
    void stopMovement();

    // Sensors stuff
    bool readOdometryAndSensors();
    const Pose& getAbsoluteTruePose();
    const Pose& getRelativeTruePose();

private:
    Pose simAbsoluteTruePose_;
    Pose simRelativeTruePose_;

    // ARIA stuff
    ArRobot robot_;
    ArRobotConnector *robotConnector_;
    ArArgumentParser *parser_;
    ArSonarDevice sonarDev_;
    ArSick sick_;
    ArLaserConnector *laserConnector_;
    ArGlobalRetFunctor1<bool, ArRobotPacket *>  simStatHandler_;

    bool initARIAConnection(int argc, char** argv);
    void resetSimPose();
    static bool simStatPacketHandler(ArRobotPacket* packet);

    bool resetSimPose_;

};

#endif // PIONEERBASE_ARIA_H
