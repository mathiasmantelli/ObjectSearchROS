#include "Utils.h"

#include <unistd.h>
#include <sstream>
#include <iomanip>
#include <errno.h>
#include <string.h>

#include <GL/glut.h>

float normalizeAngleDEG(float a)
{
    while(a>180.0)
        a -= 360.0;
    while(a<=-180.0)
        a += 360.0;
    return a;
}

float normalizeAngleRAD(float a)
{
    while(a>M_PI)
        a -= 2*M_PI;
    while(a<=-M_PI)
        a += 2*M_PI;
    return a;
}

/////////////////////////////////
///// METHODS OF CLASS POSE /////
/////////////////////////////////

Pose::Pose(){
    x=y=theta=0.0;
    up=false;
}

Pose::Pose(double a, double b, double c){
    x=a; y=b; theta=c;
}

std::ostream& operator<<(std::ostream& os, const Pose& p)
{
    os << std::setprecision(3) << "(" << p.x << ',' << p.y << ',' << p.theta << ")";
    return os;
}

float Pose::norm()
{
    float vx = this->x;
    float vy = this->y;

    return sqrt( vx*vx + vy*vy);
}

float Pose::dotProduct(Pose v)
{
    float vx = v.x;
    float vy = v.y;

    return this->x*vx + this->y*vy;
}

float Pose::angleBetween(Pose v)
{
    float num = this->dotProduct(v);
    float den = this->norm()*v.norm();
    return acos(num/den);
}

////////////////////////////////////
///// METHODS OF CLASS LOGFILE /////
////////////////////////////////////

LogFile::LogFile(LogMode mode, std::string name)
{
    time_t t = time(0);
    struct tm *now = localtime(&t);
    std::stringstream ss;

    if(mode == RECORDING)
    {
        ss << "../phir2framework/Sensors/sensors-" << -100+now->tm_year
                        << std::setfill('0') << std::setw(2) << 1+now->tm_mon
                        << std::setfill('0') << std::setw(2) << now->tm_mday << '-'
                        << std::setfill('0') << std::setw(2) << now->tm_hour
                        << std::setfill('0') << std::setw(2) << now->tm_min
                        << std::setfill('0') << std::setw(2) << now->tm_sec << ".txt";
        filename = ss.str();

        file.open(filename.c_str(), std::fstream::out);
    }
    else if(mode == PLAYBACK)
    {
        filename = "../phir2framework/Sensors/"+name;
        std::cout << filename << std::endl;
        file.open(filename.c_str(), std::fstream::in);
        if(file.fail()){
            std::cerr << "Error: " << strerror(errno) << std::endl;
            exit(1);
        }
    }
}

Pose LogFile::readPose(std::string info)
{
    std::string tempStr;
    Pose p;

    file >> tempStr >> p.x >> p.y >> p.theta;
    getline(file,tempStr);

    return p;
}

std::vector<float> LogFile::readSensors(std::string info)
{
    int max;
    std::string tempStr;
    std::vector<float> sensors;

    file >> tempStr >> max;
    sensors.resize(max);
    for (int i = 0; i < max; i++) {
        file >> sensors[i];
    }
    getline(file,tempStr);

    return sensors;
}

void LogFile::writePose(std::string info, Pose pose)
{
    file << info << ' ' << pose.x << ' ' << pose.y << ' ' << pose.theta << std::endl;
}

void LogFile::writeSensors(std::string info, std::vector<float> sensors)
{
    file << info << ' ' << sensors.size() << ' ';
    for (int i = 0; i < sensors.size(); i++)
        file << sensors[i] << ' ';
    file << std::endl;
}

bool LogFile::hasEnded()
{
    return file.peek() == std::fstream::traits_type::eof();
}

//////////////////////////////////
///// METHODS OF CLASS TIMER /////
//////////////////////////////////

Timer::Timer()
{
    startCounting();
}

void Timer::startCounting()
{
    gettimeofday(&tstart, NULL);
    gettimeofday(&tlapstart, NULL);
}

void Timer::startLap()
{
    gettimeofday(&tlapstart, NULL);
}

void Timer::stopCounting()
{
    gettimeofday(&tnow, NULL);
}

float Timer::getTotalTime()
{
    gettimeofday(&tnow, NULL);

    if (tstart.tv_usec > tnow.tv_usec) {
        tnow.tv_usec += 1000000;
        tnow.tv_sec--;
    }

    return (float)(tnow.tv_sec - tstart.tv_sec) +
           ((float)tnow.tv_usec - (float)tstart.tv_usec)/1000000.0;
}

float Timer::getLapTime()
{
    gettimeofday(&tnow, NULL);

    if (tlapstart.tv_usec > tnow.tv_usec) {
        tnow.tv_usec += 1000000;
        tnow.tv_sec--;
    }
    return (float)(tnow.tv_sec - tlapstart.tv_sec) +
           ((float)tnow.tv_usec - (float)tlapstart.tv_usec)/1000000.0;
}

void Timer::waitTime(float t){
    float l;
    do{
        usleep(1000);
        l = getLapTime();
    }while(l < t);
    startLap();
}

////////////////////////////////////////////
///  Draw Seven-Segment Digits in OpenGL ///
////////////////////////////////////////////

static double SEGMENT_WIDTH = 1 / 4.0;
static double SEGMENT_HEIGHT = 1 / 16.0;
static double SEGMENT_SHORT_WIDTH_FACTOR = 2.25 / 3.0;

static double DIGIT_WIDTH = SEGMENT_WIDTH + SEGMENT_HEIGHT;
static double DIGIT_SPACING = DIGIT_WIDTH + DIGIT_WIDTH / 4.0;

void drawLedSegment(bool active, double rotation, double x, double y)
{
   if(active)
       glColor3f(1.0,0.0,0.0);
   else
       glColor3f(0.2,0.0,0.0);

    glRotated(rotation, 0, 0, 1);
    glTranslated(x, y, 0);

    glBegin(GL_POLYGON);
        glVertex2d((SEGMENT_WIDTH / 2) * SEGMENT_SHORT_WIDTH_FACTOR, SEGMENT_HEIGHT / 2);
        glVertex2d(SEGMENT_WIDTH / 2, 0);
        glVertex2d((SEGMENT_WIDTH / 2) * SEGMENT_SHORT_WIDTH_FACTOR, -SEGMENT_HEIGHT / 2);
        glVertex2d(-(SEGMENT_WIDTH / 2) * SEGMENT_SHORT_WIDTH_FACTOR, -SEGMENT_HEIGHT / 2);
        glVertex2d(-SEGMENT_WIDTH / 2, 0);
        glVertex2d(-(SEGMENT_WIDTH / 2) * SEGMENT_SHORT_WIDTH_FACTOR, SEGMENT_HEIGHT / 2);
    glEnd();

    glTranslated(-x, -y, 0);
    glRotated(-rotation, 0, 0, 1);
}

typedef struct
{
    bool a,b,c,d,e,f,g;
} SevenSegmentDigit;

void drawDigit(char digit, double x, double y)
{
   SevenSegmentDigit ssDigit;
   switch(digit){
    case '0': ssDigit.a=true;  ssDigit.b=true;  ssDigit.c=true;  ssDigit.d=true;
            ssDigit.e=true;  ssDigit.f=true;  ssDigit.g=false;
            break;
    case '1': ssDigit.a=false; ssDigit.b=true;  ssDigit.c=true;  ssDigit.d=false;
            ssDigit.e=false; ssDigit.f=false; ssDigit.g=false;
            break;
    case '2': ssDigit.a=true;  ssDigit.b=true;  ssDigit.c=false; ssDigit.d=true;
            ssDigit.e=true;  ssDigit.f=false; ssDigit.g=true;
            break;
    case '3': ssDigit.a=true;  ssDigit.b=true;  ssDigit.c=true;  ssDigit.d=true;
            ssDigit.e=false; ssDigit.f=false; ssDigit.g=true;
            break;
    case '4': ssDigit.a=false; ssDigit.b=true;  ssDigit.c=true;  ssDigit.d=false;
            ssDigit.e=false; ssDigit.f=true;  ssDigit.g=true;
            break;
    case '5': ssDigit.a=true; ssDigit.b=false; ssDigit.c=true; ssDigit.d=true;
            ssDigit.e=false; ssDigit.f=true; ssDigit.g=true;
            break;
    case '6': ssDigit.a=true; ssDigit.b=false; ssDigit.c=true; ssDigit.d=true;
            ssDigit.e=true; ssDigit.f=true; ssDigit.g=true;
            break;
    case '7': ssDigit.a=true; ssDigit.b=true; ssDigit.c=true; ssDigit.d=false;
            ssDigit.e=false; ssDigit.f=false; ssDigit.g=false;
            break;
    case '8': ssDigit.a=true; ssDigit.b=true; ssDigit.c=true; ssDigit.d=true;
            ssDigit.e=true; ssDigit.f=true; ssDigit.g=true;
            break;
    case '9': ssDigit.a=true; ssDigit.b=true; ssDigit.c=true; ssDigit.d=true;
            ssDigit.e=false; ssDigit.f=true; ssDigit.g=true;
            break;
   }

    glTranslated(x, y, 0);

//    drawLedSegment(ssDigit.a, 0, 0, SEGMENT_WIDTH);
//    drawLedSegment(ssDigit.b, 90, SEGMENT_WIDTH / 2, -SEGMENT_WIDTH / 2);
//    drawLedSegment(ssDigit.c, 90, -SEGMENT_WIDTH / 2, -SEGMENT_WIDTH / 2);
//    drawLedSegment(ssDigit.d, 0, 0, -SEGMENT_WIDTH);
//    drawLedSegment(ssDigit.e, 90, -SEGMENT_WIDTH / 2, SEGMENT_WIDTH / 2);
//    drawLedSegment(ssDigit.f, 90, SEGMENT_WIDTH / 2, SEGMENT_WIDTH / 2);
//    drawLedSegment(ssDigit.g, 0, 0, 0);

    if(ssDigit.a) drawLedSegment(true, 0, 0, SEGMENT_WIDTH);
    if(ssDigit.b) drawLedSegment(true, 90, SEGMENT_WIDTH / 2, -SEGMENT_WIDTH / 2);
    if(ssDigit.c) drawLedSegment(true, 90, -SEGMENT_WIDTH / 2, -SEGMENT_WIDTH / 2);
    if(ssDigit.d) drawLedSegment(true, 0, 0, -SEGMENT_WIDTH);
    if(ssDigit.e) drawLedSegment(true, 90, -SEGMENT_WIDTH / 2, SEGMENT_WIDTH / 2);
    if(ssDigit.f) drawLedSegment(true, 90, SEGMENT_WIDTH / 2, SEGMENT_WIDTH / 2);
    if(ssDigit.g) drawLedSegment(true, 0, 0, 0);

    glTranslated(-x, -y, 0);
}

double getTotalWidth(int length)
{
   return length * DIGIT_WIDTH
         + (length - 1) * (DIGIT_SPACING - DIGIT_WIDTH);
}

void drawNumbers(std::string number, double x, double y)
{
    glTranslated(x, y, 0);
    glScalef(1.5,1.5,1.5);

   double offsetX = DIGIT_WIDTH / 2 - getTotalWidth(number.size()) / 2;

      glTranslated(offsetX, 0, 0);

      for (int i = 0; i < number.size(); i++) {
         drawDigit(number[i], DIGIT_SPACING * i, 0);
      }

      glTranslated(-offsetX, 0, 0);

    glScalef(1.0/1.5,1.0/1.5,1.0/1.5);
    glTranslated(-x, -y, 0);
}

