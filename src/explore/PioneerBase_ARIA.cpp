#include "PioneerBase_ARIA.h"

PioneerBase_ARIA::PioneerBase_ARIA(): PioneerBase()
{
    // reset robot position in simulator
    resetSimPose_ = false;
}

//////////////////////////////////
///// INITIALIZATION METHODS /////
//////////////////////////////////

bool PioneerBase_ARIA::initialize(ConnectionMode cmode, LogMode lmode, std::string fname)
{
    // initialize logfile
    logFile_ = new LogFile(lmode,fname);

    int argc=0; char** argv;

    // initialize ARIA
    Aria::init();
    bool success=false;

    switch (cmode)
    {
        case SERIAL:
        {
            argc=6;
            argv =(char **) new char*[6];

            argv[0]= new char[4];
            argv[1]= new char[13];
            argv[2]= new char[15];
            argv[3]= new char[7];

            argv[4]= new char[4];
            argv[5]= new char[13];

            strcpy(argv[0],"-rp");
            strcpy(argv[1],"/dev/ttyUSB0");

            strcpy(argv[2],"-laserPortType");
            strcpy(argv[3],"serial");
            strcpy(argv[4],"-lp");
            strcpy(argv[5],"/dev/ttyUSB1");
            break;
        }
        case WIFI:
        {
            argc=4;
            argv =(char **) new char*[4];
            argv[0]= new char[4];
            argv[1]= new char[20];
            argv[2]= new char[20];
            argv[3]= new char[7];

            strcpy(argv[0],"-rh");
            strcpy(argv[1],"192.168.1.11");
            strcpy(argv[2],"-remoteLaserTcpPort");
            strcpy(argv[3],"10002");
            break;
        }
        case SIMULATION:
        {
            argc=2;
            argv =(char **) new char*[2];

            argv[0]= new char[4];
            argv[1]= new char[20];

            strcpy(argv[0],"-rh");
            strcpy(argv[1],"localhost");
            break;
        }
    }

    success = initARIAConnection(argc,argv);
    if(!success){
        printf("Could not connect to robot... exiting\n");
        return false;
    }

    if(cmode==SIMULATION && resetSimPose_)
        resetSimPose();

    return true;
}

bool PioneerBase_ARIA::initARIAConnection(int argc, char** argv)
{
    parser_= new ArArgumentParser(&argc, argv);
    robotConnector_ = new ArRobotConnector(parser_,&robot_);
    int success=robotConnector_->connectRobot();
    if(!success){
        Aria::shutdown();
        return false;
    }

    robot_.addRangeDevice(&sick_);

    laserConnector_ = new ArLaserConnector(parser_, &robot_, robotConnector_);
    laserConnector_->setupLaser(&sick_);

    robot_.addRangeDevice(&(sonarDev_));

    sick_.runAsync();
    robot_.setHeading(0);
    robot_.runAsync(true);
    robot_.enableMotors();
    robot_.setRotVelMax(10);
    printf("Connecting...\n");
    if (!laserConnector_->connectLaser(&(sick_))){
        printf("Could not connect to lasers... exiting\n");
        Aria::shutdown();
        return false;
    }

    simStatHandler_ = simStatPacketHandler;
    //Start gathering robot true pose every 100ms
    robot_.addPacketHandler(&simStatHandler_, ArListPos::FIRST);
    robot_.comInt(237,2);

    return true;
}

////////////////////////////////////////////////////////////

// GLOBAL VARIABLES FOR THE SIMULATOR

double theX  = 0.0;
double theY  = 0.0;
double theTh = 0.0;
double initialX = 0.0;
double initialY = 0.0;
double initialTh = 0.0;
double sinInitTh = 0.0;
double cosInitTh = 1.0;
int firstPass = true;
std::string fileName("");

bool PioneerBase_ARIA::simStatPacketHandler(ArRobotPacket* packet) {

    // if packet cannot be handled return false, else return true
    switch(packet->getID()) {
    case 0x62: {
        // Empty bytes
        char a = packet->bufToByte();  // unused byte
        char b = packet->bufToByte();  // unused byte
        ArTypes::UByte4 flags = packet->bufToUByte4();

        // Get simulation clock intervals (nominal, measured, last)
        int simint = packet->bufToUByte2();
        int realint = packet->bufToUByte2();
        int lastint = packet->bufToUByte2();

        // Get true pose
        int iTheX = packet->bufToByte4();
        int iTheY = packet->bufToByte4();
        int realZ = packet->bufToByte4();
        int iTheTh = packet->bufToByte4();
        //    cout << iTheTh << ' ';

        theX = iTheX/1000.0;
        theY = iTheY/1000.0;
        theTh = iTheTh;//*M_PI/180.0;

        if(firstPass){
            initialX = theX;
            initialY = theY;
            initialTh = theTh;

            cosInitTh = cos(DEG2RAD(-initialTh));
            sinInitTh = sin(DEG2RAD(-initialTh));

            firstPass=false;
        }

        // Get geopositioning data
        if(flags & ArUtil::BIT1)
        {
            double lat = packet->bufToByte4()/10e6;
            double lon = packet->bufToByte4()/10e6;
            double alt = packet->bufToByte4()/100;
            printf("\tLatitude = %f deg., Longitude = %f deg., Altitude = %f m\n", lat, lon, alt);
        }
        return true;
    }
    case 0x66: { // SIM_MAP_CHANGED packet
        // Did I opened the file?
        unsigned char user = packet->bufToUByte();  // unused byte
        //Was it loaded or just reopened without changes (a.k.a. selected the same map)
        unsigned char loaded = packet->bufToUByte();  // unused byte

        //get fileName
        int dataLength = packet->getDataLength();
        int remainingData = dataLength - packet->getReadLength();
        fileName.resize(remainingData);
        packet->bufToStr( &fileName[0], remainingData);

        std::cout << "\n\n\n\nFileName: " << fileName << "\n\n\n\n";

        return true;
    }
    default:
        return false;
    }
}

void PioneerBase_ARIA::resetSimPose()
{
    ArRobotPacket pkt;
    pkt.setID(ArCommands::SIM_RESET);
    pkt.uByteToBuf(0); // argument type: ignored.
    pkt.finalizePacket();
    robot_.getDeviceConnection()->write(pkt.getBuf(), pkt.getLength());
}

void PioneerBase_ARIA::closeARIAConnection()
{
    robot_.stopRunning(true);
    robot_.disconnect();
    sick_.lockDevice();
    sick_.stopRunning();
    Aria::exit(0);
    Aria::shutdown();
    if(parser_!=NULL)
        delete parser_;
    if(robotConnector_!=NULL)
        delete robotConnector_;
    if(laserConnector_!=NULL)
        delete laserConnector_;
}

////////////////////////////////////////////////////////////////
////// METHODS FOR READING ODOMETRY & SENSORS MEASUREMENTS /////
////////////////////////////////////////////////////////////////

bool PioneerBase_ARIA::readOdometryAndSensors()
{
    std::vector < ArSensorReading > *readings;
    std::vector < ArSensorReading > ::iterator it;
    ArPose p;
    sick_.lockDevice();
    readings = sick_.getRawReadingsAsVector();
    it = readings->begin();

    if(!readings->empty()){
        p = (*it).getPoseTaken();
    }
    else{
        sick_.unlockDevice();
        return false;
    }

    // coordinates are given in mm, we convert to m
    odometry_.x = p.getX()/1000.0;
    odometry_.y = p.getY()/1000.0;
    odometry_.theta = p.getTh();

    while (odometry_.theta > 180.0)
        odometry_.theta -= 360.0;
    while (odometry_.theta < -180.0)
        odometry_.theta += 360.0;

    // sensors readings are given in mm, we convert to m
    int i = 0;
    for (it = readings->begin(); it!=readings->end(); it++){
        lasers_[i++] = (float)(*it).getRange()/1000.0;
    }

    sick_.unlockDevice();

    for(int i=0;i<numSonars_;i++)
        sonars_[i]=(float)(robot_.getSonarRange(i))/1000.0;

    return true;
}

const Pose& PioneerBase_ARIA::getAbsoluteTruePose()
{
    simAbsoluteTruePose_.x = theX;
    simAbsoluteTruePose_.y = theY;
    simAbsoluteTruePose_.theta = theTh;

    while (simAbsoluteTruePose_.theta > 180.0)
        simAbsoluteTruePose_.theta -= 360.0;
    while (simAbsoluteTruePose_.theta < -180.0)
        simAbsoluteTruePose_.theta += 360.0;

    return simAbsoluteTruePose_;
}

const Pose& PioneerBase_ARIA::getRelativeTruePose()
{
    simRelativeTruePose_.x = cosInitTh*(theX-initialX) - sinInitTh*(theY-initialY);
    simRelativeTruePose_.y = sinInitTh*(theX-initialX) + cosInitTh*(theY-initialY);
    simRelativeTruePose_.theta = theTh-initialTh;

    while (simRelativeTruePose_.theta > 180.0)
        simRelativeTruePose_.theta -= 360.0;
    while (simRelativeTruePose_.theta < -180.0)
        simRelativeTruePose_.theta += 360.0;

    return simRelativeTruePose_;
}

//////////////////////////////
///// NAVIGATION METHODS /////
//////////////////////////////

void PioneerBase_ARIA::stopMovement()
{
    robot_.stop();
}

void PioneerBase_ARIA::resumeMovement()
{
    robot_.setVel2(vLeft_, vRight_);
}

bool PioneerBase_ARIA::isMoving()
{
    if(robot_.getRightVel()!=0.0)
        return true;
    if(robot_.getLeftVel()!=0.0)
        return true;
    return false;
}
