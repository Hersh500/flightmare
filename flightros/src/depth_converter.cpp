#include "flightros/flight_pilot.hpp"
#include "flightros/depth_converter.hpp"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace flightros { 

DepthConverter::DepthConverter(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
  : nh_(nh),
    pnh_(pnh) {
    
  sub_depth = nh_.subscribe("flight_pilot/depth", 1, &DepthConverter::depthCallback, this); 
    
  float fl = (FlightPilot::im_height/2.0)/tan((M_PI * FlightPilot::fov/180.0)/2.0);
  K(0, 0) = fl; 
  K(0, 1) = 0; 
  K(0, 2) = FlightPilot::im_width/2;
  K(1, 0) = 0;
  K(1, 1) = FlightPilot::im_height/2;
  K(2, 0) = 0;
  K(2, 1) = 0;
  K(2, 2) = 1;

  invK = K.inverse();
  cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("point_cloud", 1);
}

DepthConverter::~DepthConverter() {}

void DepthConverter::depthCallback(const sensor_msgs::ImageConstPtr& depth_img) {
    cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(depth_img, "mono8");
    std::vector<float> points;

    for (int i = 0; i < img->image.rows; i++) {
        for (int j = 0; j < img->image.cols; j++) {
            uint8_t id = img->image.at<uint8_t>(i, j);
            if (id != 0) {
                float d = FlightPilot::depth_scale/10.0 * id;
                Eigen::Vector3f image_point(j * d, i * d, d);
                Eigen::Vector3f camera_point = invK * image_point;
                points.push_back(camera_point.x());
                // switching between z and y here.
                points.push_back(camera_point.z());  // need to add a negative here?
                points.push_back(-camera_point.y());
            }
        }
    }
    
    int n_points = points.size();
    sensor_msgs::PointCloud2 cloud_msg;
    sensor_msgs::PointCloud2Modifier modifier(cloud_msg);

    modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::PointField::FLOAT32,
                                  "y", 1, sensor_msgs::PointField::FLOAT32,
                                  "z", 1, sensor_msgs::PointField::FLOAT32);

    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(n_points);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

    cloud_msg.height = 1;
    cloud_msg.width = n_points;
    cloud_msg.header = std_msgs::Header();
    cloud_msg.header.frame_id = "camera_pose";

    for(size_t i=0; i<(unsigned long)n_points/3; ++i, ++iter_x, ++iter_y, ++iter_z){
        *iter_x = points[3*i+0];
        *iter_y = points[3*i+1];
        *iter_z = points[3*i+2];

        // std::cerr << *iter_x << " " << *iter_y << " " << *iter_z << std::endl;
    }
    cloud_pub.publish(cloud_msg);
}

}
