#ifndef UTILS_H
#define UTILS_H

#include <sys/time.h>
#include <fstream>
#include <iostream>
#include <cmath>
#include <vector>

//using namespace std;

enum ExplorationKind {BVP_exp, BVP_MOD_exp, VORONOIBVP_exp, SEMANTIC_exp};

enum ConnectionMode {SIMULATION, SERIAL, WIFI};
enum LogMode { NONE, RECORDING, PLAYBACK};
enum MotionMode {MANUAL_SIMPLE, MANUAL_VEL, WANDER, WALLFOLLOW, POTFIELD, LOCALPOTFIELD, ENDING, PREENDING};
enum MovingDirection {STOP, FRONT, BACK, LEFT, RIGHT, RESTART, DEC_ANG_VEL, INC_ANG_VEL, INC_LIN_VEL, DEC_LIN_VEL};

#define DEG2RAD(x) x*M_PI/180.0
#define RAD2DEG(x) x*180.0/M_PI

float normalizeAngleDEG(float a);
float normalizeAngleRAD(float a);

class Pose{
    public:
        Pose();
        Pose(double a, double b, double c);
	float norm();
        float dotProduct(Pose v);
        float angleBetween(Pose v);

        friend std::ostream& operator<<(std::ostream& os, const Pose& p);

        double x, y, theta;
        bool up;
};

class LogFile
{
    public:
        LogFile(LogMode mode, std::string name);

        Pose readPose(std::string info);
        std::vector<float> readSensors(std::string info);

        void writePose(std::string info, Pose pose);
        void writeSensors(std::string s, std::vector<float> sensors);

        bool hasEnded();

    private:
        std::fstream file;
        std::string filename;
};

class Timer{
    public:
        Timer();

        void startCounting();
        void startLap();
        void stopCounting();
        void waitTime(float);

        float getTotalTime();
        float getLapTime();

    private:
        struct timeval tstart, tlapstart, tnow;
};

void drawNumbers(std::string number, double x, double y);

#endif // UTILS_H
