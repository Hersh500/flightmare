/**
 * @file navigator.cpp
 * @brief Accepts navigation commands and publishes commands to UAV
 */

#include <flightros/navigator.hpp>

#define TIME_HORIZON 2.0

Navigator::Navigator(ros::NodeHandle* nh, double freq) : _nh(*nh), 
                                                         _rate(freq){

    initializePublishers();
    initializeSubscribers();
    initializeServices();

    // command variables
    _cmd_heading = 0.0
    _cmd_velocity = 0.0

    _last_cmd = ros::Time::now();

}

/* ---------------------------------- */
/*      INITIALIZATION FUNCTIONS      */
/* ---------------------------------- */

void Navigator::initializeSubscribers(){
    _cmd_heading_sub = _nh.subscribe
        ("learner/cmd_heading", 10, &Navigator::_cmd_heading_cb, this);
    _cmd_velocity_sub = _nh.subscribe
        ("learner/cmd_velocity", 10, &Navigator::_cmd_velocity_cb, this);
    _odom_sub = _nh.subscribe
        ("hummingbird/ground_truth/odometry", 10, &Navigator::_odom_cb, this);
}

void Navigator::initializePublishers(){
    _cmd_pub = _nh.advertise<geometry_msgs::TwistStamped>
        ("hummingbird/autopilot/velocity_command", 10);
}

void Navigator::initializeServices(){

}

/* ---------------------------------- */
/*         CALLBACK FUNCTIONS         */
/* ---------------------------------- */

void Navigator::_cmd_heading_cb(const std_msgs::Float32::ConstPtr& msg){
    if (_cmd_heading > 3.14/4. || _cmd_heading < -3.14/4.)
        ROS_WARNING("Heading command not within [-PI/4, PI/4]")
    _cmd_heading = msg->data;
}

void Navigator::_cmd_velocity_cb(const std_msgs::Float32::ConstPtr& msg){
    _cmd_velocity = msg->data;
}

void Navigator::odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
    _odom = *msg;
}

/* ---------------------------------- */
/*            RUN FUNCTION            */
/* ---------------------------------- */

void Navigator::run(){

    while (ros::ok()){

        // if last message was recent enough
        if (ros::Time::now() - _last_cmd < ros::Duration(TIME_HORIZON)){
            geometry_msgs::TwistStamped vel_msg;
            vel_msg.header.stamp = ros::Time::now();

            // TODO vel msg
            // heading control? could do simple P controller but have to make sure
            // that the yaw angle from _odom is in base frame
            
            vel_msg.twist.linear.x = 0
            vel_msg.twist.linear.y = 0
            vel_msg.twist.linear.z = 0
            vel_msg.twist.angular.x = 0
            vel_msg.twist.angular.y = 0
            vel_msg.twist.angular.z = 0
            _cmd_pub.publish(vel_msg)
        }

        // spin
        ros::spinOnce();
        _rate.sleep();
    }

}
