#include "cabil_laser_data.h"

void ImageScanCallback(const sensor_msgs::Image::ConstPtr imageColor, const sensor_msgs::LaserScan::ConstPtr& scan_in)
{

    try{ //对图像进行处理，转换成opencv格式的图像
        image  = cv_bridge::toCvShare(imageColor, "bgr8")->image; //image_raw就是我们得到的图像了
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", imageColor->encoding.c_str());
    }
    string name,scan_name;
    Mat rgb;
    image.copyTo(rgb);
    // 在rgb彩色图里查找角点
    cvtColor(rgb, gray, COLOR_BGR2GRAY);
    vector<Point2f> corners;
    int key = cv::waitKey(30) & 0xFF;
    if (key == ACTION_ESC)
    {
        cv::destroyAllWindows();
        return;
    }
    else if (key == ACTION_SPACE) {
        // 确保图像中包含角点
        bool patternfound = findChessboardCorners(gray, patternsize, corners,
                                                  CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE
                                                  + CALIB_CB_FAST_CHECK);
        if (patternfound) {
            //绘制角点
            drawChessboardCorners(rgb, patternsize, Mat(corners), patternfound);
            //显示拍照效果
            bitwise_not(rgb, rgb);
            name = image_save_path + to_string(counts)+".jpg";
            bool rst =  imwrite(name, image);
            if (!rst) {
                std::cerr << "文件保存失败，请确保指定目录存在并可写：" << image_save_path << std::endl;
                return;
            }
            sensor_msgs::PointCloud2 cloud1;
            projector_.projectLaser(*scan_in, cloud1);
            pcl::PointCloud<pcl::PointXYZ> cloud;
            pcl::fromROSMsg(cloud1, cloud);
            scan_name = scan_save_path + to_string(counts)+".pcd";
            pcl::io::savePCDFileASCII (scan_name, cloud);
            counts++;
            printf("保存角点图片第%d张成功，共需要保存15张图片 \n", counts);
        }
        else {
            std::cerr << "未发现标定板！" << std::endl;

        }

    }

    cv::imshow("rgb", rgb);


}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "GET_DATA");
  ros::NodeHandle nh;
  message_filters::Subscriber<sensor_msgs::Image> sub_image(nh,"/camera_01/rgb/image_raw",1);
  message_filters::Subscriber<sensor_msgs::LaserScan> sub_cloud(nh,"/scan",1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::LaserScan> sync_policy;
  message_filters::Synchronizer<sync_policy> syncpolicy(sync_policy(20),sub_image,sub_cloud);
  syncpolicy.registerCallback(boost::bind(&ImageScanCallback,_1,_2));
  ros::Rate loop_rate(100);
  while(nh.ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }  
  return 0;
}

