#include <ros/ros.h>
#include "flightros/depth_converter.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "depth_converter");
  ros::NodeHandle nh;
  /*
  double noise_std;
  double max_depth_in_pcl;
  if (!nh.getParam("noise_std", noise_std)) {
    ROS_ERROR("Could not initialize noise_std in depth conveter!");
    return -1;
  }
  if (!nh.getParam("max_depth_in_pointcloud", max_depth_in_pcl)) {
    ROS_ERROR("Could not initialize noise_std in depth conveter!");
    return -1;
  }
  */
  flightros::DepthConverter pilot(nh, ros::NodeHandle("~"));
                                  

  // spin the ros
  ros::spin();
  
  return 0;
}
