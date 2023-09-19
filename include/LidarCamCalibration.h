/*
* Time: 2023/09/12 10:56:06
* Author: ZHANG WEN HAO
* Contact: 821298794@qq.com
* Version: 0.1
* Language: c++
* Description:  相机和激光雷达标定
*/

#ifndef LIDARCAMCALIBRATION_2DLIDARCAMCALIBRATION_H
#define LIDARCAMCALIBRATION_2DLIDARCAMCALIBRATION_H
#include <iostream>
#include <fstream>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <dirent.h>
#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include "PoseLocalParameterization.h"
#include "yaml-cpp/yaml.h"


struct Oberserve
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Oberserve()
    {
        chess_pose_qca = Eigen::Quaterniond(1,0,0,0);
        chess_pose_tca = Eigen::Vector3d::Zero();
    }

    Eigen::Quaterniond chess_pose_qca;    // 该时刻chessboard在相机中的姿态
    Eigen::Vector3d chess_pose_tca;       // 该时刻chessboard在相机中的偏移
    std::vector<Eigen::Vector3d> points;  // 该时刻打在标定板上的激光点云数据
};



class LidarCamCalibration {
public:
    LidarCamCalibration(std::string input_image_dir,std::string input_scan_dir,std::string visual_chess_dir,cv::Size pattern_size,double chess_grid_size,std::string out_path);
    ~LidarCamCalibration();
    void CamInstrinsicCalibrate(cv::Mat& camera_matrix,cv::Mat& dist_coeffs,std::vector<std::pair<std::string,Eigen::Matrix4d>>& T_cam2chesses);
    void LidarCamExtCalibrate(std::vector<std::pair<std::string ,Eigen::Matrix4d>>& T_cam2chesses,Eigen::Matrix4d& T_laser2cam);
private:
    void CamLaserCalClosedSolution(const std::vector<Oberserve> obs, Eigen::Matrix4d& T_laser2cam);
    void ReciveImagePath(std::vector<std::string>& images_path);
    void ReciveScanPath(std::vector<std::string>& scans_path);
    void GenerateObservation(std::vector<std::pair<std::string ,Eigen::Matrix4d>>& T_cam2chesses,std::vector<Oberserve>& obs);
    void CamLaserCalibrateCeres(const std::vector<Oberserve> obs, Eigen::Matrix4d &T_cam2laser);
public:
    std::string input_image_dir_,input_scan_dir_,visual_chess_dir_;
    cv::Size pattern_size_;
    double chess_grid_size_;
    std::string out_path_;
private:

};


#endif //LIDARCAMCALIBRATION_2DLIDARCAMCALIBRATION_H
