// Node that consumes Octomaps and the current robot pose and
// outputs an uncertainty frame that matches the shape of the depth image.

#include <ros/ros.h>
#include "flightros/flight_pilot.hpp"
#include "flightros/uncertainty_converter.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "uncertainty_converter");
  flightros::UncertaintyConverter converter(ros::NodeHandle(), ros::NodeHandle("~"));

  // spin the ros
  ros::spin();
  
  return 0;
}
