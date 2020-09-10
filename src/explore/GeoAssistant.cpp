#include "GeoAssistant.h"
#include <fstream>
#include <iostream>
#include <string>
#include <cstring>
#include <boost/algorithm/string.hpp>
#include <stdlib.h>
#include <math.h>
using namespace std;


GeoAssistant::GeoAssistant(Configuration *config){
    string key, door, x, y, angle, line;
	vector<string> coord;
    ifstream arq(config->GetString("doors_file_address"));

    if (!arq.is_open())
    		perror("error while opening file");

    while(getline(arq, line)) {
        boost::split(coord, line, [](char c){return c == ' ';});
    	x = coord[0];
    	y = coord[1];
        angle = coord[3];
    	door = coord[2];
        this->checkPoints[door] = make_tuple(stof(x),stof(y),stof(angle));
    }

	if (arq.bad())
    		perror("error while reading file");
    
    arq.close();
}

void GeoAssistant::setRobot(Robot *r)
{
    this->robot = r;
}

void GeoAssistant::polling()
{
    while(this->robot->isReady() == false){
    	usleep(100000);
    }

    while(this->robot->isRunning() == true){
            this->checkNearbyLeftRightDoors();
    }
    //exit(0);
}

void GeoAssistant::checkNearbyDoors()
{
    map<string, tuple<float,float,float>>::iterator i;
	Pose absPose = this->robot->getAbsCurrentPose(); //x,y,theta
	float thetaRad = M_PI*(absPose.theta-90)/180; //bot angle (radians)
	Pose vetC = Pose(cos(thetaRad), sin(thetaRad), 0); //bot front vector (unitary)
	Pose vetD;
	for(i = this->checkPoints.begin(); i != this->checkPoints.end(); ++i){ 
		vetD = Pose(get<0>(i->second)-absPose.x,get<1>(i->second)-absPose.y,0); //distance between bot and door's point
        float angle = vetC.angleBetween(vetD); //angle between distance from bot to door and bot right vector (camera)
        if(angle < (M_PI/9) && vetD.norm() < 5) // distance vector inside camera's 120 degree FOV
			this->robot->nearbyDoors.insert(i->first);
		else
			this->robot->nearbyDoors.erase(i->first);
    }

}

void GeoAssistant::checkNearbyLeftRightDoors()
{
    map<string, tuple<float,float,float>>::iterator i;
    Pose absPose = this->robot->getAbsCurrentPose(); //x,y,theta
    float thetaRadR = M_PI*(absPose.theta-90)/180; //bot angle (radians)
    float thetaRadL = M_PI*(absPose.theta+90)/180; //bot angle (radians)
    Pose vetCR = Pose(cos(thetaRadR), sin(thetaRadR), 0); //bot front vector (unitary)
    Pose vetCL = Pose(cos(thetaRadL), sin(thetaRadL), 0); //bot front vector (unitary)
    Pose vetD;
    for(i = this->checkPoints.begin(); i != this->checkPoints.end(); ++i){
        vetD = Pose(get<0>(i->second)-absPose.x,get<1>(i->second)-absPose.y,0); //distance between bot and door's point
        float angleR = vetCR.angleBetween(vetD); //angle between distance from bot to door and bot right vector (camera)
        float angleL = vetCL.angleBetween(vetD); //angle between distance from bot to door and bot right vector (camera)

        float angleDoor = M_PI*(get<2>(i->second))/180;
        Pose vetInFront = Pose(cos(angleDoor), sin(angleDoor), 0); //angle between front of door and robot
        float angleInFront = vetInFront.angleBetween(vetD);


        if(angleInFront < (M_PI/9) && angleR < (M_PI/9) && vetD.norm() < 5) // distance vector inside camera's 120 degree FOV
            this->robot->nearbyDoorsRight.insert(i->first);
        else
            this->robot->nearbyDoorsRight.erase(i->first);

        if(angleInFront < (M_PI/9) && angleL < (M_PI/9) && vetD.norm() < 5) // distance vector inside camera's 120 degree FOV
            this->robot->nearbyDoorsLeft.insert(i->first);
        else
            this->robot->nearbyDoorsLeft.erase(i->first);

    }
}
