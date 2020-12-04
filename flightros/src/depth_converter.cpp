#include "flightros/flight_pilot.hpp"
#include "flightros/depth_converter.hpp"

#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


/*
// Noisy params 1 - medium lookahead
#define NOISE_STD_ 10.0
#define MAX_DEPTH_IN_PCL_ 7.5

// Noisy params 2 - short lookahead
#define NOISE_STD_ 10.0
#define MAX_DEPTH_IN_PCL_ 6.5
*/

// Noise-free params
#define NOISE_STD_ 0.0
#define MAX_DEPTH_IN_PCL_ 20

namespace flightros { 

DepthConverter::DepthConverter(const ros::NodeHandle &nh, 
                               const ros::NodeHandle &pnh)
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

  image_transport::ImageTransport it(nh_);
  noisy_pub = it.advertise("camera/noisy_depth", 1);
}

DepthConverter::~DepthConverter() {}

void DepthConverter::depthCallback(const sensor_msgs::ImageConstPtr& depth_img) {
    cv_bridge::CvImagePtr img = cv_bridge::toCvCopy(depth_img, "mono8");


    // noise-ify ground truth depth image before converting to point cloud
    cv::Mat noise_(img->image.rows, img->image.cols, CV_64FC1);
    float sigma = NOISE_STD_;
    // cv::Mat stddev = cv::Mat::ones(1, img->image.rows, CV_64FC1) * NOISE_STD_;
    // cv::Mat mean = cv::Mat::zeros(1, img->image.rows, CV_64FC1); 
    float mean = 0.0;
    cv::randn(noise_, mean, sigma);
    noise_.convertTo(noise_, CV_8UC1);
    img->image = img->image + noise_;

    // Publish the noisy depth image
//     noisy_pub.publish(img->toImageMsg());

    std::vector<float> points;
    float width_factor = 0; // -img->image.cols/2;
    float height_factor = img->image.rows/2;
    uint8_t min_d = 255;
    for (int i = 0; i < img->image.rows; i=i + 1) {
        for (int j = 0; j < img->image.cols; j=j + 1) {
            uint8_t id = img->image.at<uint8_t>(i, j);
            if (id < min_d) {
                min_d = id;
            }
            if (id != 0) {
                float d = FlightPilot::depth_scale/100.0 * id;
                if (d < MAX_DEPTH_IN_PCL_) {
                    Eigen::Vector3f image_point((j + width_factor) * d, (-i + height_factor) * d, d);
                    Eigen::Vector3f camera_point = invK * image_point;
                    points.push_back(camera_point.x());
                    // switching between z and y here, since image rows = world frame z.
                    points.push_back(camera_point.z());
                    points.push_back(camera_point.y());
                } else {
                    img->image.at<uint8_t>(i, j) = 200.0; // set it to some far away depth, ie. unknown
                }
            }
        }
    }
    // ROS_INFO("minimum depth_value = %d\n", min_d);
    noisy_pub.publish(img->toImageMsg());
    
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
