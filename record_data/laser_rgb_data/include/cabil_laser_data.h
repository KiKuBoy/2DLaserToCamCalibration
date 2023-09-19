#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>   //相机和雷达对齐
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl_ros/transforms.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ros/package.h>
#include "yaml-cpp/yaml.h"
#include <sensor_msgs/Image.h>
#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace cv;
sensor_msgs::PointCloud2 msg;  //接收到的点云消息
sensor_msgs::PointCloud2 fusion_msg;  //等待发送的点云消息
laser_geometry::LaserProjection projector_;
std::string filePath = ros::package::getPath("laser_rgb_data");
std::string image_save_path =filePath+"/image/";
std::string scan_save_path =filePath+"/scan/";
const int ACTION_ESC = 27;
const int ACTION_SPACE = 32;
int counts = 1;
Size patternsize(11,8);
cv::Mat image,gray;
