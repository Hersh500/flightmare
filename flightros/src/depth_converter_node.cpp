#include <ros/ros.h>
#include "flightros/depth_converter.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "depth_converter");
  ROS_INFO("INIT DEPTH_CONVERTER");
  flightros::DepthConverter pilot(ros::NodeHandle(), ros::NodeHandle("~"));

  // spin the ros
  ros::spin();
  
  return 0;
}
