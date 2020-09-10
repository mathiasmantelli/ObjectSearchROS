#ifndef PIONEERBASE_ROS_H
#define PIONEERBASE_ROS_H

#include <ros/ros.h>
#include <ros/package.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>

#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <phi_ocr/doors.h>
#include <phi_ocr/door.h>

#include "PioneerBase.h"
#include "Grid.h"
#include "Utils.h"

class PioneerBase_ROS : public PioneerBase
{
public:
    PioneerBase_ROS();

    // ROS stuff
    bool initialize();

    // Navigation stuff
    bool isMoving();
    void resumeMovement();
    void stopMovement();

    // Sensors stuff
    bool readOdometryAndSensors();

private:

    ros::NodeHandle* n_;
    ros::Rate* rate_;
    tf::TransformListener* listener;

    ros::WallTime start, last, current;

    nav_msgs::Odometry odomROS_;
    nav_msgs::Odometry relTrueROS_;
    nav_msgs::Odometry absTrueROS_;
    nav_msgs::OccupancyGrid gridROS_;
    sensor_msgs::LaserScan laserROS_;
    geometry_msgs::Twist twistROS_;
    phi_ocr::doors doorsROS_;

    // Ros topics subscribers
    ros::Publisher pub_twist_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_relTrue_;
    ros::Subscriber sub_absTrue_;
    ros::Subscriber sub_laser_;
    ros::Subscriber sub_SLAMmap_;
    ros::Subscriber sub_doors_;


    void receiveOdom(const nav_msgs::Odometry::ConstPtr &value);
    void receiveDoors(const phi_ocr::doors::ConstPtr &value);
    void receiveRelTruePose(const nav_msgs::Odometry::ConstPtr &value);
    void receiveAbsTruePose(const nav_msgs::Odometry::ConstPtr &value);
    void receiveLaser(const sensor_msgs::LaserScan::ConstPtr &value);
    void receiveSLAMmap(const nav_msgs::OccupancyGrid::ConstPtr &value);

};

#endif // PIONEERBASE_ROS_H
