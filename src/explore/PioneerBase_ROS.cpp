#include "PioneerBase_ROS.h"

PioneerBase_ROS::PioneerBase_ROS(): PioneerBase()
{
    int argc=0;
    char** argv=NULL;

    ros::init(argc, argv, "control_P3DX");

    ROS_INFO("control_P3DX!");

    n_ = new ros::NodeHandle("~");
    rate_ = new ros::Rate(20);

    start = ros::WallTime::now();
    last = start;

    listener = new tf::TransformListener;

    // Initialize publishers and subscribers
    pub_twist_ = n_->advertise<geometry_msgs::Twist>("/rosaria_phi/cmd_vel", 1);
    sub_odom_ = n_->subscribe("/rosaria_phi/pose", 100, &PioneerBase_ROS::receiveOdom, this);
    sub_relTrue_ = n_->subscribe("/rosaria_phi/relTruePose", 100, &PioneerBase_ROS::receiveRelTruePose, this);
    sub_absTrue_ = n_->subscribe("/rosaria_phi/absTruePose", 100, &PioneerBase_ROS::receiveAbsTruePose, this);
    sub_laser_ = n_->subscribe("/rosaria_phi/laser_laserscan", 100, &PioneerBase_ROS::receiveLaser, this);
    sub_SLAMmap_ = n_->subscribe("/map", 100, &PioneerBase_ROS::receiveSLAMmap, this);
    sub_doors_ = n_->subscribe("/phi_ocr/doors", 100, &PioneerBase_ROS::receiveDoors, this);


    srcPath = ros::package::getPath("phi_exploration");
    std::cout << "PATH " << srcPath << std::endl;

}

//////////////////////////////////
///// INITIALIZATION METHODS /////
//////////////////////////////////

bool PioneerBase_ROS::initialize()
{

    return true;
}


////////////////////////////////////////////////////////////////
////// METHODS FOR READING ODOMETRY & SENSORS MEASUREMENTS /////
////////////////////////////////////////////////////////////////

bool PioneerBase_ROS::readOdometryAndSensors()
{
    odometry_.x = odomROS_.pose.pose.position.x;
    odometry_.y = odomROS_.pose.pose.position.y;

    tf::Quaternion q1(odomROS_.pose.pose.orientation.x,
                     odomROS_.pose.pose.orientation.y,
                     odomROS_.pose.pose.orientation.z,
                     odomROS_.pose.pose.orientation.w);
    tf::Matrix3x3 m1(q1);
    double roll, pitch, yaw;
    m1.getRPY(roll,pitch,yaw);

    odometry_.theta = RAD2DEG(yaw);

    std::cout << "!!!ODOM: " << odometry_ << std::endl;

    if(isSimulation){
        simAbsoluteTruePose_.x = absTrueROS_.pose.pose.position.x;
        simAbsoluteTruePose_.y = absTrueROS_.pose.pose.position.y;

        tf::Quaternion q2(absTrueROS_.pose.pose.orientation.x,
                         absTrueROS_.pose.pose.orientation.y,
                         absTrueROS_.pose.pose.orientation.z,
                         absTrueROS_.pose.pose.orientation.w);
        tf::Matrix3x3 m2(q2);
//        double roll, pitch, yaw;
        m2.getRPY(roll,pitch,yaw);

        simAbsoluteTruePose_.theta = RAD2DEG(yaw);

        std::cout << "ABS : " << simAbsoluteTruePose_ << std::endl;

        simRelativeTruePose_.x = relTrueROS_.pose.pose.position.x;
        simRelativeTruePose_.y = relTrueROS_.pose.pose.position.y;

        tf::Quaternion q3(relTrueROS_.pose.pose.orientation.x,
                         relTrueROS_.pose.pose.orientation.y,
                         relTrueROS_.pose.pose.orientation.z,
                         relTrueROS_.pose.pose.orientation.w);
        tf::Matrix3x3 m3(q3);
        m3.getRPY(roll,pitch,yaw);

        simRelativeTruePose_.theta = RAD2DEG(yaw);

        std::cout << "REL : " << simRelativeTruePose_ << std::endl;

    }

    std::cout << "LASER: " << laserROS_.ranges.size() << std::endl;
    for(int i=0; i<laserROS_.ranges.size(); i++)
        lasers_[i] = laserROS_.ranges[numLasers_-i-1];

    std::cout << "DOORS: " << doorsROS_.doors.size() << std::endl;
    nearbyDoorsLeft.clear();
    nearbyDoorsRight.clear();
    for(int i=0; i<doorsROS_.doors.size(); i++){
        if(doorsROS_.doors[i].rightcamera == true)
            nearbyDoorsRight.insert(doorsROS_.doors[i].name);
        else
            nearbyDoorsLeft.insert(doorsROS_.doors[i].name);
    }
    doorsROS_.doors.clear();

    std::cout << "Map resolution " << gridROS_.info.resolution
              << " width " << gridROS_.info.width << " height " << gridROS_.info.height
              << " origin x " << gridROS_.info.origin.position.x << " y " << gridROS_.info.origin.position.y
              << std::endl;

    std::cout << "Grid limits x " << grid_->SLAMlimits.minX << " " << grid_->SLAMlimits.maxX
              << " y " << grid_->SLAMlimits.minY << " " << grid_->SLAMlimits.maxY << std::endl;


    tf::StampedTransform transform;

    bool goodSLAMPose = true;

    try{
        // faster lookup transform so far
        listener->lookupTransform("/map", "/base_link", ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
//        ROS_ERROR("%s",ex.what());
        // 30 Hz -- maybe too fast
        ros::Duration(0.05).sleep();
        goodSLAMPose=false;
    }

    if(goodSLAMPose){
        slamPose_.x = transform.getOrigin().x();
        slamPose_.y = transform.getOrigin().y();
        geometry_msgs::Pose p;
        quaternionTFToMsg(transform.getRotation(),p.orientation);


        tf::Quaternion q4(p.orientation.x,
                         p.orientation.y,
                         p.orientation.z,
                         p.orientation.w);
        tf::Matrix3x3 m4(q4);
        m4.getRPY(roll,pitch,yaw);

        slamPose_.theta = RAD2DEG(yaw);
    }

    std::cout << "SLAM : " << slamPose_ << std::endl;


    return true;
}


void PioneerBase_ROS::receiveOdom(const nav_msgs::Odometry::ConstPtr &value)
{

//  STRUCTURE OF nav_msgs::Odometry

//# This represents an estimate of a position and velocity in free space.
//# The pose in this message should be specified in the coordinate frame given by header.frame_id.
//# The twist in this message should be specified in the coordinate frame given by the child_frame_id

    //Header header
    //    # Standard metadata for higher-level stamped data types.
    //    # This is generally used to communicate timestamped data
    //    # in a particular coordinate frame.
    //    #
    //    # sequence ID: consecutively increasing ID
    //    uint32 seq
    //    #Two-integer timestamp that is expressed as:
    //    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    //    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    //    # time-handling sugar is provided by the client library
    //    time stamp
    //    #Frame this data is associated with
    //    # 0: no frame
    //    # 1: global frame
    //    string frame_id
    odomROS_.header = value->header;

    //string child_frame_id
    odomROS_.child_frame_id = value->child_frame_id;

    //geometry_msgs/PoseWithCovariance pose
        //# This represents a pose in free space with uncertainty.
        //Pose pose
            //# A representation of pose in free space, composed of position and orientation.
            //Point position
                //# This contains the position of a point in free space
                //float64 x
                //float64 y
                //float64 z
            //Quaternion orientation
                //# This represents an orientation in free space in quaternion form.
                //float64 x
                //float64 y
                //float64 z
                //float64 w
        //# Row-major representation of the 6x6 covariance matrix
        //# The orientation parameters use a fixed-axis representation.
        //# In order, the parameters are:
        //# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
        //float64[36] covariance
    odomROS_.pose = value->pose;

    //geometry_msgs/TwistWithCovariance twist
        //# This expresses velocity in free space with uncertainty.
        //Twist twist
            //# This expresses velocity in free space broken into its linear and angular parts.
            //Vector3  linear
                //float64 x
                //float64 y
                //float64 z
            //Vector3  angular
                //float64 x
                //float64 y
                //float64 z
        //# Row-major representation of the 6x6 covariance matrix
        //# The orientation parameters use a fixed-axis representation.
        //# In order, the parameters are:
        //# (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
        //float64[36] covariance
    odomROS_.twist = value->twist;
}

void PioneerBase_ROS::receiveAbsTruePose(const nav_msgs::Odometry::ConstPtr &value)
{
//  STRUCTURE OF nav_msgs::Odometry
// For details look receiveOdom
    absTrueROS_.header = value->header;
    absTrueROS_.child_frame_id = value->child_frame_id;
    absTrueROS_.pose = value->pose;
    absTrueROS_.twist = value->twist;
}

void PioneerBase_ROS::receiveRelTruePose(const nav_msgs::Odometry::ConstPtr &value)
{
//  STRUCTURE OF nav_msgs::Odometry
// For details look receiveOdom
    relTrueROS_.header = value->header;
    relTrueROS_.child_frame_id = value->child_frame_id;
    relTrueROS_.pose = value->pose;
    relTrueROS_.twist = value->twist;
}

void PioneerBase_ROS::receiveLaser(const sensor_msgs::LaserScan::ConstPtr &value)
{
//  STRUCTURE OF sensor_msgs::LaserScan

    //Header header
    //    # Standard metadata for higher-level stamped data types.
    //    # This is generally used to communicate timestamped data
    //    # in a particular coordinate frame.
    //    #
    //    # sequence ID: consecutively increasing ID
    //    uint32 seq
    //    #Two-integer timestamp that is expressed as:
    //    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    //    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    //    # time-handling sugar is provided by the client library
    //    time stamp
    //    #Frame this data is associated with
    //    # 0: no frame
    //    # 1: global frame
    //    string frame_id
    //             # timestamp in the header is the acquisition time of
    //             # the first ray in the scan.
    //             #
    //             # in frame frame_id, angles are measured around
    //             # the positive Z axis (counterclockwise, if Z is up)
    //             # with zero angle being forward along the x axis
    laserROS_.header = value->header;

    //float32 angle_min        # start angle of the scan [rad]
    //float32 angle_max        # end angle of the scan [rad]
    //float32 angle_increment  # angular distance between measurements [rad]
    laserROS_.angle_min = value->angle_min;
    laserROS_.angle_max = value->angle_max;
    laserROS_.angle_increment = value->angle_increment;

    //float32 time_increment   # time between measurements [seconds] - if your scanner
    //                         # is moving, this will be used in interpolating position
    //                         # of 3d points
    //float32 scan_time        # time between scans [seconds]
    laserROS_.time_increment = value->time_increment;
    laserROS_.scan_time = value->scan_time;

    //float32 range_min        # minimum range value [m]
    //float32 range_max        # maximum range value [m]
    laserROS_.range_min = value->range_min;
    laserROS_.range_max = value->range_max;

    //float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
    //float32[] intensities    # intensity data [device-specific units].  If your
    //                         # device does not provide intensities, please leave
    //                         # the array empty.
    laserROS_.ranges = value->ranges;
    laserROS_.intensities = value->intensities;
}

void PioneerBase_ROS::receiveDoors(const phi_ocr::doors::ConstPtr &value)
{
//  STRUCTURE OF phi_ocr::doors

    //Header header
    //    # Standard metadata for higher-level stamped data types.
    //    # This is generally used to communicate timestamped data
    //    # in a particular coordinate frame.
    //    #
    //    # sequence ID: consecutively increasing ID
    //    uint32 seq
    //    #Two-integer timestamp that is expressed as:
    //    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    //    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    //    # time-handling sugar is provided by the client library
    //    time stamp
    //    #Frame this data is associated with
    //    # 0: no frame
    //    # 1: global frame
    //    string frame_id
    //             # timestamp in the header is the acquisition time of
    //             # the first ray in the scan.
    //             #
    //             # in frame frame_id, angles are measured around
    //             # the positive Z axis (counterclockwise, if Z is up)
    //             # with zero angle being forward along the x axis
    doorsROS_.header = value->header;

    //door[] doors
    //     # door
    //          # string name
    //          # bool rightcamera
    doorsROS_.doors = value->doors;
}

void PioneerBase_ROS::receiveSLAMmap(const nav_msgs::OccupancyGrid::ConstPtr &value)
{

//  STRUCTURE OF nav_msgs::OccupancyGrid

    //# This represents a 2-D grid map, in which each cell represents the probability of
    //# occupancy.

    //Header header
    //    # Standard metadata for higher-level stamped data types.
    //    # This is generally used to communicate timestamped data
    //    # in a particular coordinate frame.
    //    #
    //    # sequence ID: consecutively increasing ID
    //    uint32 seq
    //    #Two-integer timestamp that is expressed as:
    //    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    //    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    //    # time-handling sugar is provided by the client library
    //    time stamp
    //    #Frame this data is associated with
    //    # 0: no frame
    //    # 1: global frame
    //    string frame_id
    //             # timestamp in the header is the acquisition time of
    //             # the first ray in the scan.
    //             #
    //             # in frame frame_id, angles are measured around
    //             # the positive Z axis (counterclockwise, if Z is up)
    //             # with zero angle being forward along the x axis
    gridROS_.header = value->header;

    //MapMetaData info
    //    # This hold basic information about the characterists of the OccupancyGrid

    //    # The time at which the map was loaded
    //    time map_load_time
    //    # The map resolution [m/cell]
    //    float32 resolution
    //    # Map width [cells]
    //    uint32 width
    //    # Map height [cells]
    //    uint32 height
    //    # The origin of the map [m, m, rad].  This is the real-world pose of the
    //    # cell (0,0) in the map.
    //    geometry_msgs/Pose origin
    gridROS_.info = value->info;

    //# The map data, in row-major order, starting with (0,0).  Occupancy
    //# probabilities are in the range [0,100].  Unknown is -1.
    //int8[] data
    gridROS_.data = value->data;


    // find limits
    grid_->SLAMlimits.minX = grid_->SLAMlimits.minY =  1000000;
    grid_->SLAMlimits.maxX = grid_->SLAMlimits.maxY = -1000000;

    for(int j=0; j<gridROS_.info.height; j++){
        for(int i=0; i<gridROS_.info.width; i++){
            if(gridROS_.data[i+j*gridROS_.info.height] > -1){
                if(grid_->SLAMlimits.minX > i)
                    grid_->SLAMlimits.minX = i;
                if(grid_->SLAMlimits.minY > j)
                    grid_->SLAMlimits.minY = j;
                if(grid_->SLAMlimits.maxX < i)
                    grid_->SLAMlimits.maxX = i;
                if(grid_->SLAMlimits.maxY < j)
                    grid_->SLAMlimits.maxY = j;
            }
        }
    }

    //copy data
    for(int j=grid_->SLAMlimits.minY; j<=grid_->SLAMlimits.maxY; j++){
        for(int i=grid_->SLAMlimits.minX; i<=grid_->SLAMlimits.maxX; i++){
            Cell *c = grid_->getCell(i-1000,j-1000);

            c->slamValue = (int) gridROS_.data[i+j*gridROS_.info.height];

        }
    }

}

//////////////////////////////
///// NAVIGATION METHODS /////
//////////////////////////////

void PioneerBase_ROS::stopMovement()
{
//    robot_.stop();
}

void PioneerBase_ROS::resumeMovement()
{
//    robot_.setVel2(vLeft_, vRight_);

    twistROS_.linear.x = linVel_;
    twistROS_.angular.z = angVel_;

    // Update robot motion
    pub_twist_.publish(twistROS_);

    ros::spinOnce();
    rate_->sleep();
}

bool PioneerBase_ROS::isMoving()
{
//    if(robot_.getRightVel()!=0.0)
//        return true;
//    if(robot_.getLeftVel()!=0.0)
//        return true;
//    return false;
}
