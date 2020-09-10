#include <pthread.h>
#include <iostream>
#include <string.h>

#include "Robot.h"
#include "Planning.h"
#include "GeoAssistant.h"
#include "GlutClass.h"
#include "Configuration.h"

ConnectionMode connectionMode;
LogMode logMode;

std::string filename;
pthread_mutex_t* mutex;

int *id_frame = 0;

void* startRobotThread (void* ref)
{
    Robot* robot=(Robot*) ref;

    robot->initialize(connectionMode, logMode, filename);

    while(robot->isRunning()){
        robot->run();
    }

	return NULL;
}

void* startGeoAssistantThread (void* ref)
{
    Robot* robot=(Robot*) ref;

    GeoAssistant* geoAss = new GeoAssistant(robot->configuration);
    geoAss->setRobot((Robot*) ref);
    geoAss->polling();

    return NULL;
}

void* startGlutThread (void* ref)
{
    GlutClass* glut = GlutClass::getInstance();
    glut->setRobot((Robot*) ref);

    glut->initialize();

    glut->process();


    while(true){

    }

	return NULL;
}

void* startPlanningThread (void* ref)
{
    bool writeFile = false;
    Robot* robot=(Robot*) ref;
    while(!robot->isReady()){
        //std::cout << "Planning is waiting..." << std::endl;
        usleep(100000);
    }

    if(robot->plan->exploration_kind() == SEMANTIC_exp)
        robot->startingMovement();

    robot->plan->initialize();

    if(robot->plan->exploration_kind() == BVP_exp || robot->plan->exploration_kind() == BVP_MOD_exp)
        robot->motionMode_ = POTFIELD;
    else
        robot->motionMode_ = LOCALPOTFIELD;

    while(robot->isRunning()){
        if(!robot->plan->run() || robot->limit_frames) //the limit_frames is to kill the execution in cases which the robot is spinning and it won't finish the exploration
        {
            if(robot->motionMode_ != PREENDING && robot->motionMode_ != ENDING)
                robot->motionMode_ = PREENDING;
            string address;
            if(robot->configuration->GetString("output_address", address) && !writeFile)
            {
                writeFile = true;
                ofstream myfile;
                myfile.open(address, std::ios_base::app);
                myfile << robot->ComputePathSize() << ";" << robot->plan->reached << "\n";
                myfile.close();
                std::cout << "--------------------------- THE PATH SIZE IN METERS IS: " << robot->ComputePathSize() << " ---------------------------" << std::endl;
            }
        }
        usleep(100000);
    }


    return NULL;
}

int main(int argc, char* argv[])
{
    connectionMode = SIMULATION;
    logMode = NONE;

    filename = "";

    Configuration *configuration = new Configuration();
    if(true)
    {
        if(!configuration->Load("/home/mathias/PioneerWorkspace/src/Phi-Exploration/config_sample_file.ini"))
            return 0;
    }
    else
    {
        std::cout << std::endl << "ERROR!!! Can't find configuration file!" << std::endl;
        return 0;
    }

    if(argc > 2){
        if(!strncmp(argv[2], "sim", 3))
            connectionMode=SIMULATION;
        else if(!strncmp(argv[2], "wifi", 4))
            connectionMode=WIFI;
        else if(!strncmp(argv[2], "serial", 6))
            connectionMode=SERIAL;
    }

    if(argc > 3){
        if (!strncmp(argv[3], "-R", 2)){
            logMode = RECORDING;
        }else if (!strncmp(argv[3], "-r", 2)) {
            logMode = RECORDING;
        }
        else if (!strncmp(argv[3], "-p", 2)) {
            logMode = PLAYBACK;
            filename = argv[4];
        }
        else if (!strncmp(argv[3], "-P", 2)){
            logMode = PLAYBACK;
            filename = argv[4];
        }else if(!strncmp(argv[5], "-n", 2)){
            logMode = NONE;
        }
    }

    Robot* r;
    r = new Robot(configuration);

    pthread_t robotThread, glutThread, planningThread, geoAssistantThread;
    mutex = new pthread_mutex_t;
    pthread_mutex_unlock(mutex);

    pthread_create(&(robotThread),NULL,startRobotThread,(void*)r);
    pthread_create(&(glutThread),NULL,startGlutThread,(void*)r);
    pthread_create(&(planningThread),NULL,startPlanningThread,(void*)r);
//    pthread_create(&(geoAssistantThread),NULL,startGeoAssistantThread,(void*)r);

    pthread_join(robotThread, 0);
    pthread_join(glutThread, 0);
    pthread_join(planningThread, 0);
//    pthread_join(geoAssistantThread, 0);

    return 0;
}

