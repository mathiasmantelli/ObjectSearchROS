#ifndef GEOASSISTANT_H
#define GEOASSISTANT_H

#include <iostream>
#include <map>
#include "Robot.h"
#include "Utils.h"
#include "Configuration.h"


using namespace std;


class GeoAssistant
{
	public:

        GeoAssistant(Configuration* config);
		void setRobot(Robot* r);
		void polling();

	protected:
		void checkNearbyDoors();
        void checkNearbyLeftRightDoors();
        map<string, tuple<float,float,float>> checkPoints;
		Robot* robot;
};
#endif  // GEOASSISTANT_H
