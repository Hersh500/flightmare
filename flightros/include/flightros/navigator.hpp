#ifndef NAVIGATOR_H_
#define NAVIGATOR_H_

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <flightros/QuadState.h>

class PropNavMission{

    private:

        // ROS stuff
        ros::NodeHandle _nh;
        ros::Rate _rate;

        // subscribers
        ros::Subscriber _cmd_heading_sub;
        ros::Subscriber _cmd_velocity_sub;
        ros::Subscriber _odom_sub;

        // publishers
        ros::Publisher _cmd_pub;

        // services

        // initialization functions
        void initializeSubscribers();
        void initializePublishers();
        void initializeServices();

        // callback functions
        void _cmd_heading_sub(const std_msgs::Float32::ConstPtr& msg);
        void _cmd_velocity_sub(const std_msgs::Float32::ConstPtr& msg);
        double _cmd_heading, _cmd_velocity;
        void odom_cb(const nav_msgs::Odometry::ConstPtr& msg);
        nav_msgs::Odometry _odom;

        // timing variables
        ros::Time _last_cmd;

    public:

        PropNavMission(ros::NodeHandle* nh, double freq);

        void run();

}

#endif