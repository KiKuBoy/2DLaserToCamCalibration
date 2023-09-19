/*
* Time: 2023/09/14 19:16:06
* Author: ZHANG WEN HAO
* Contact: 821298794@qq.com
* Version: 0.1
* Language: c++
* Description:  相机与2D激光雷达标定结果验证可视化代码，将激光雷达点云数据投影到二维图像上
*/
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include "yaml-cpp/yaml.h"


//获取各图像的路径，并且有序取出图像
void ReciveImagePath(std::string image_dir,std::vector<std::string>& images_path)
{
    DIR* dir;
    struct dirent* entry;
    if ((dir = opendir(image_dir.c_str())) != nullptr)
    {
        while ((entry = readdir(dir)) != nullptr)
        {
            std::string file_name = entry->d_name;
            if (file_name.size() >= 4 && file_name.substr(file_name.size() - 4) == ".jpg")
            {
                std::string file_path = image_dir + "/"+ file_name;
                images_path.push_back(file_path);
            }
        }
        closedir(dir);
    } else {
        perror("Error opening directory");
    }
}


//获取各图像的路径，并且有序取出图像
void ReciveScanPath(std::string scan_dir,std::vector<std::string>& scans_path)
{
    DIR* dir;
    struct dirent* entry;
    if ((dir = opendir(scan_dir.c_str())) != nullptr) {
        while ((entry = readdir(dir)) != nullptr)
        {
            std::string file_name = entry->d_name;
            if (file_name.size() >= 4 && file_name.substr(file_name.size() - 4) == ".pcd")
            {
                std::string file_path = scan_dir + "/"+ file_name;
                scans_path.push_back(file_path);
            }
        }
        closedir(dir);
    } else {
        perror("Error opening directory");
    }
}


//标定代码可视化
void VisualizeCalibration(std::string& image_dir,std::string& scan_dir,Eigen::Matrix4d& T_laser2cam,cv::Mat& camera_matrix,cv::Mat& dist_coeffs)
{
    std::vector<std::string> images_path,scans_path;
    //获取图像路径
    ReciveImagePath(image_dir,images_path);
    ReciveScanPath(scan_dir,scans_path);

    for(int i=0;i<images_path.size();i++)
    {
        std::string current_image_path = images_path[i];
        //获取激光雷达文件名
        size_t pos_start = current_image_path.find_last_of("/");
        size_t pos_end = current_image_path.find_last_of(".");
        size_t num = pos_end - pos_start - 1;
        std::string file_name = current_image_path.substr(pos_start + 1, num); //文件名
        cv::Mat image = cv::imread(current_image_path);
        std::string scan_path = scan_dir + "/" + file_name + ".pcd";

        // 读取点云数据
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile<pcl::PointXYZ>(scan_path, *cloud);


        // 遍历点云中的每个点
        for (const pcl::PointXYZ& point : cloud->points) {
            // 创建一个Eigen::Vector3d表示点云中的点
            Eigen::Vector3d pclPoint(point.x, point.y, point.z);
            Eigen::Matrix4d  T_cam2laser =T_laser2cam.inverse();
            // 使用变换矩阵将点从点云坐标系转换到相机坐标系
            Eigen::Vector3d camPoint = T_cam2laser.block<3, 3>(0, 0) * pclPoint + T_cam2laser.block<3, 1>(0, 3);


            float fx = camera_matrix.at<float>(0,0);
            float fy = camera_matrix.at<float>(1,1);
            float cx = camera_matrix.at<float>(0,2);
            float cy = camera_matrix.at<float>(1,2);

            // 将3D点投影到像素坐标系
            cv::Mat point_3d = (cv::Mat_<double>(3, 1) << camPoint[0], camPoint[1], camPoint[2]);

            double u_distorted = (point_3d.at<double>(0) / point_3d.at<double>(2)) * fx + cx; // 归一化坐标映射到像素坐标
            double v_distorted = (point_3d.at<double>(1) / point_3d.at<double>(2)) * fy + cy;

            //相机去畸变，由于图像未经过去畸变操作，此时，点云数据也不进行去畸变，判断两者之间的关系
            //            // 畸变参数
            //            double k1 = dist_coeffs.at<float>(0); // 径向畸变参数
            //            double k2 = dist_coeffs.at<float>(1); // 径向畸变参数
            //            double k3 = dist_coeffs.at<float>(2); // 径向畸变参数
            //            double p1 = dist_coeffs.at<float>(3); // 切向畸变参数
            //            double p2 = dist_coeffs.at<float>(4); // 切向畸变参数
            //            // 转换为归一化坐标
            //            double x_normalized = (u_distorted - cx) / fx;
            //            double y_normalized = (v_distorted - cy) / fy;
            //            // 计算到光学中心的距离
            //            double r = sqrt(x_normalized * x_normalized + y_normalized * y_normalized);
            //            // 应用径向畸变公式
            //            double x_corrected = x_normalized * (1 + k1 * r * r + k2 * r * r * r * r + k3 * r * r * r * r * r * r);
            //            // 应用切向畸变公式
            //            double y_corrected = y_normalized + (2 * p1 * x_normalized * y_normalized + p2 * (r * r + 2 * x_normalized * x_normalized));
            //            // 转换回像素坐标
            //            double u_undistorted = x_corrected * fx + cx;
            //            double v_undistorted = y_corrected * fy + cy;

            // 在图像上绘制点
            cv::circle(image, cv::Point2d(u_distorted, v_distorted), 2, cv::Scalar(0, 0, 255), -1); // 红色点
        }
        // 显示图像
        cv::imshow("Projected Image", image);
        cv::waitKey(0); // 等待用户按下键盘任意键来关闭窗口

    }
}



int main(int argc, char **argv)
{
    // 从YAML文件中加载标定参数。
    YAML::Node config = YAML::LoadFile("/home/iimt/data/Sensor_Calibration/Camera _Lidar_Calibration/Camera _2DLidar_Calibration/2DLidarCamCalibration/result/calibrate_result.yaml");
    // 检查是否成功加载了YAML文件
    if (!config["T_laser2cam"] || !config["camera_matrix"] || !config["dist_coeffs"]) {
        std::cerr << "Failed to load transform matrix from YAML file." << std::endl;
        return 1;
    }
    //获取坐标系变换矩阵，相机内参矩阵，畸变系数
    std::vector<float> T_laser2cam_vector = config["T_laser2cam"]["data"].as<std::vector<float>>();
    std::vector<float> camera_matrix_vector = config["camera_matrix"]["data"].as<std::vector<float>>();
    std::vector<float> dist_coeffs_vector = config["dist_coeffs"]["data"].as<std::vector<float>>();

    Eigen::Matrix4d T_laser2cam;
    for (int i = 0; i < 4; ++i)
    {
        for (int j = 0; j < 4; ++j)
        {
            T_laser2cam(i, j) = T_laser2cam_vector[i * 4 + j];
        }
    }

    std::cout<<"T_laser2cam："<<T_laser2cam<<std::endl;
    cv::Mat camera_matrix = cv::Mat(3, 3, CV_32F, camera_matrix_vector.data());
    std::cout<<"camera_matrix："<<camera_matrix<<std::endl;
    cv::Mat dist_coeffs = cv::Mat(dist_coeffs_vector);

    std::string image_dir = "/home/iimt/data/Sensor_Calibration/Camera _Lidar_Calibration/Camera _2DLidar_Calibration/2DLidarCamCalibration/data/image";
    std::string scan_dir = "/home/iimt/data/Sensor_Calibration/Camera _Lidar_Calibration/Camera _2DLidar_Calibration/2DLidarCamCalibration/data/scan";
    VisualizeCalibration(image_dir,scan_dir,T_laser2cam,camera_matrix,dist_coeffs);
}
