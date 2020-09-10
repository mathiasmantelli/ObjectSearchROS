#ifndef PIONEERBASE_H
#define PIONEERBASE_H

#include "Utils.h"
#include "Grid.h"

class PioneerBase
{
public:
    PioneerBase();

    // Drawing stuff
    void drawBase();
    void drawSonars(bool drawCones=false);
    void drawLasers(bool fill=true);

    // Navigation stuff
    void setMovementSimple(MovingDirection dir);
    void setMovementVel(MovingDirection dir);
    void setLinAngVelocity_fromWheelsVelocity(float lv, float rv);
    void setWheelsVelocity_fromLinAngVelocity(float linV, float angV);
    void setWheelsVelocity(float vl, float vr);
    void setLinAngVelocity(float linV, float angV);

    // Sensors stuff
    //bool readOdometryAndSensors();
    const Pose& getOdometry();
    const Pose& getSLAMPose();
    const Pose& getAbsoluteTruePose();
    const Pose& getRelativeTruePose();
    const std::vector<float>& getLaserReadings();
    const std::vector<float>& getSonarReadings();
    const std::set<std::string>& getNearbyDoorsLeft();
    const std::set<std::string>& getNearbyDoorsRight();

    int getNumLasers();
    int getNumSonars();
    float getMaxLaserRange();
    float getMaxSonarRange();
    void setGrid(Grid* grid);
    void setOdometry(const Pose &o);
    void setSonarReadings(const std::vector<float> &s);
    void setLaserReadings(const std::vector<float> &l);

    float getMinSonarValueInRange(int idFirst, int idLast);
    float getMinLaserValueInRange(int idFirst, int idLast, int kernelSize=0);

    int getNearestSonarBeam(float angle);
    float getAngleOfSonarBeam(int k);
    float getKthSonarReading(int k);

    int getNearestLaserBeam(float angle);
    float getAngleOfLaserBeam(int k);
    float getKthLaserReading(int k);

    // Log stuff
    void writeOnLog();
    bool readFromLog();

    std::string srcPath;

    //Reset
//    void reset_pose(float x, float y, float th);

protected:
    Pose odometry_;
    Pose slamPose_;
    Pose simAbsoluteTruePose_;
    Pose simRelativeTruePose_;
    Grid* grid_;

    bool isSimulation;

    // Navigation stuff
    double vLeft_, vRight_;
    double oldVLeft_, oldVRight_;
    double linVel_, angVel_;

    // Sensors stuff
    int numSonars_;
    std::vector<float> sonars_;
    float maxSonarRange_;
    int numLasers_;
    std::vector<float> lasers_;
    float maxLaserRange_;

    std::set<std::string> nearbyDoorsLeft;
    std::set<std::string> nearbyDoorsRight;

    LogFile* logFile_;
};

#endif // PIONEERBASE_H
