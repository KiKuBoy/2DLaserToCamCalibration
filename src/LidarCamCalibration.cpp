/*
* Time: 2023/09/12 10:56:06
* Author: ZHANG WEN HAO
* Contact: 821298794@qq.com
* Version: 0.1
* Language: c++
* Description:  相机与2D激光雷达标定
*/

#include "LidarCamCalibration.h"

//ceres优化过程中的代价函数
class PointInPlaneFactor: public ceres::SizedCostFunction<1,7>
{
private:
    Eigen::Vector4d planar_;
    Eigen::Vector3d point_;
    double scale_ = 1.0;

public:
    PointInPlaneFactor(Eigen::Vector4d planar,    // planar
                       Eigen::Vector3d point, double scale = 1.)   // point
            : planar_(planar),point_(point)
    {
        scale_ = scale;
        // std::cout << scale_ << std::endl;
    }

    virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const;
};


Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &q)
{
    Eigen::Matrix3d ans;
    ans << 0.0, -q(2), q(1),
            q(2), 0.0, -q(0),
            -q(1), q(0), 0.0;
    return ans;
}


bool PointInPlaneFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    Eigen::Vector3d tcl(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond qcl(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d pt_c = qcl.toRotationMatrix() * point_ + tcl;
    residuals[0] = scale_ * ( planar_.head(3).transpose() * pt_c + planar_[3] );
    // std::cout << residuals[0] <<std::endl;
    if (jacobians)
    {
        if (jacobians[0]) {
            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
            Eigen::Matrix<double, 1, 6> jaco_i;
            jaco_i.leftCols<3>() = planar_.head(3);
            jaco_i.rightCols<3>() = planar_.head(3).transpose() * (-qcl.toRotationMatrix() * skewSymmetric(point_));

            jacobian_pose_i.leftCols<6>() = scale_ * jaco_i;
            jacobian_pose_i.rightCols<1>().setZero();
        }
    }

    return true;
}

//实例化整个标定过程
LidarCamCalibration::LidarCamCalibration(std::string input_image_dir,std::string input_scan_dir,std::string visual_chess_dir,cv::Size pattern_size,double chess_grid_size,std::string out_path):input_image_dir_(input_image_dir),
input_scan_dir_(input_scan_dir),visual_chess_dir_(visual_chess_dir),pattern_size_(pattern_size),chess_grid_size_(chess_grid_size),out_path_(out_path)
{

}

LidarCamCalibration::~LidarCamCalibration()
{

}

//获取各图像的路径，并且有序取出图像
void LidarCamCalibration::ReciveImagePath(std::vector<std::string>& images_path)
{
    DIR* dir;
    struct dirent* entry;
    if ((dir = opendir(input_image_dir_.c_str())) != nullptr)
    {
        while ((entry = readdir(dir)) != nullptr)
        {
            std::string file_name = entry->d_name;
            if (file_name.size() >= 4 && file_name.substr(file_name.size() - 4) == ".jpg")
            {
                std::string file_path = input_image_dir_ + "/"+ file_name;
                images_path.push_back(file_path);
            }
        }
        closedir(dir);
    } else {
        perror("Error opening directory");
    }
}


//获取各激光的路径，并且有序取出图像
void LidarCamCalibration::ReciveScanPath(std::vector<std::string>& scans_path)
{
    DIR* dir;
    struct dirent* entry;
    if ((dir = opendir(input_scan_dir_.c_str())) != nullptr) {
        while ((entry = readdir(dir)) != nullptr)
        {
            std::string file_name = entry->d_name;
            if (file_name.size() >= 4 && file_name.substr(file_name.size() - 4) == ".pcd")
            {
                std::string file_path = input_scan_dir_ + "/"+ file_name;
                scans_path.push_back(file_path);
            }
        }
        closedir(dir);
    } else {
        perror("Error opening directory");
    }
}


//相机内参标定
void LidarCamCalibration::CamInstrinsicCalibrate(cv::Mat& camera_matrix,cv::Mat& dist_coeffs,std::vector<std::pair<std::string ,Eigen::Matrix4d>>& T_cam2chesses)
{
    std::vector<std::string> calibrate_images_path;
    ReciveImagePath(calibrate_images_path);

    std::vector<std::vector<cv::Point3f>> object_points;  // 棋盘格角点
    std::vector<std::vector<cv::Point2f>> image_points;   // 二维图像像素点
    std::vector<std::string> image_base_names;

    // 棋盘格世界坐标系的坐标
    std::vector<cv::Point3f> obj;
    for (int i = 0; i < pattern_size_.height; ++i) {
        for (int j = 0; j < pattern_size_.width; ++j) {
            obj.push_back(cv::Point3f(chess_grid_size_*j, chess_grid_size_*i, 0.0f));
        }
    }

    cv::Mat gray_frame,image;
    for(int i = 0;i<calibrate_images_path.size();i++)
    {
        size_t pos_start = calibrate_images_path[i].find_last_of("/");
        size_t pos_end = calibrate_images_path[i].find_last_of(".");
        size_t num = pos_end - pos_start - 1;
        std::string file_name = calibrate_images_path[i].substr(pos_start+1, num); //文件名
        image =  cv::imread(calibrate_images_path[i]);
        cv::cvtColor(image, gray_frame, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> corners;
        bool ret = cv::findChessboardCorners(gray_frame, pattern_size_, corners,
                                             cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);
        //按顺序进行对应添加
        if(ret)
        {
            //角点亚像素提取
            cv::cornerSubPix(gray_frame, corners, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));
            //将对应的图像id的棋盘格以及图像点赋值，记录掩膜
            object_points.push_back(obj);
            image_points.push_back(corners);
            image_base_names.push_back(file_name);
            cv::drawChessboardCorners(image, pattern_size_, corners, ret);
            std::string image_out_path = visual_chess_dir_+ "/"+ file_name +".png";
            cv::imwrite(image_out_path,image);
        }
    }
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;
    // 执行相机内参标定
    cv::calibrateCamera(object_points, image_points,gray_frame.size(), camera_matrix, dist_coeffs, rvecs, tvecs);
    // 打印内参矩阵和畸变系数
    std::cout << "Camera Matrix:\n" << camera_matrix << "\n\n";
    std::cout << "Distortion Coefficients:\n" << dist_coeffs << "\n";

    //将信息进行组合
    //将旋转向量和平移量转为变换矩阵
    for (int i=0; i < rvecs.size(); i++)
    {
        cv::Mat current_rvec = rvecs[i];
        cv::Mat current_tvec = tvecs[i];
        std::string name = image_base_names[i];
        cv::Mat rotation_matrix;
        cv::Rodrigues(current_rvec, rotation_matrix);
        Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
        for(int row=0;row<3;row++)
        {
            for(int col=0;col<3;col++)
            {
                transformation_matrix(row,col) = rotation_matrix.at<double>(row, col);
            }
        }
        transformation_matrix(0,3)=current_tvec.at<double>(0);
        transformation_matrix(1,3)=current_tvec.at<double>(1);
        transformation_matrix(2,3)=current_tvec.at<double>(2);
        T_cam2chesses.push_back(std::make_pair(name,transformation_matrix));
    }
}

//生成多帧观测数据
void LidarCamCalibration::GenerateObservation(std::vector<std::pair<std::string ,Eigen::Matrix4d>>& T_cam2chesses,std::vector<Oberserve>& obs)
{
    std::vector<std::string> calibrate_scans_path;
    ReciveScanPath(calibrate_scans_path);

    //将点云数据与文件名组合
    for(int i=0;i<calibrate_scans_path.size();i++)
    {
        //获取激光雷达文件名
        size_t pos_start = calibrate_scans_path[i].find_last_of("/");
        size_t pos_end = calibrate_scans_path[i].find_last_of(".");
        size_t num = pos_end - pos_start - 1;
        std::string file_name = calibrate_scans_path[i].substr(pos_start+1, num); //文件名

        Eigen::Matrix4d T_cam2chess;
        for(int j=0;j<T_cam2chesses.size();j++)
        {
            if(T_cam2chesses[j].first == file_name)
            {
                T_cam2chess = T_cam2chesses[j].second;
            }
        }
        // 提取旋转部分
        Eigen::Matrix3d rotation_matrix = T_cam2chess.block<3, 3>(0, 0);
        // 提取平移向量
        Eigen::Vector3d translation_vector = T_cam2chess.block<3, 1>(0, 3);
        // 将旋转矩阵转换为四元数
        Eigen::Quaterniond quaternion(rotation_matrix);

        //定义observe对象，保存每个chess在相机坐标系下的变换，以及每个chess对应的激光点数据
        Oberserve ob;
        ob.chess_pose_qca = quaternion;
        ob.chess_pose_tca = translation_vector;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // 从PCD文件中加载点云数据
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(calibrate_scans_path[i], *cloud) == -1) {
            PCL_ERROR("Couldn't read file 'point_cloud.pcd'\n");
            return;
        }

        for(int j=0;j<cloud->points.size();j++)
        {
            Eigen::Vector3d current_point;
            current_point(0) = cloud->points[j].x;
            current_point(1) = cloud->points[j].y;
            current_point(2) = cloud->points[j].z;
            ob.points.push_back(current_point);
        }
        obs.push_back(ob);
    }
}

//求解激光雷达与相机变换矩阵的初始封闭解
void LidarCamCalibration::CamLaserCalClosedSolution(const std::vector<Oberserve> obs, Eigen::Matrix4d& T_laser2cam)
{
    //记录整个观测数据中的点数
    int whole_points_num = 0;
    for (size_t i = 0; i < obs.size(); i++)
    {
        whole_points_num += obs[i].points.size();
    }

    // nHp = -d --> AH = b   //其中n为相机坐标系下的chessboard的法向量，H为假设矩阵，p为雷达坐标系下的点，d为chessboard平面到相机坐标系原点的距离
    /*     [ h1, h4, h7]
       H = [ h2, h5, h8]
           [ h3, h6, h9]
    */
    Eigen::MatrixXd A(whole_points_num,9);
    Eigen::VectorXd b(whole_points_num);

    int index = 0;
    for (size_t i = 0; i < obs.size(); i++)
    {
        Oberserve obi = obs[i];
        Eigen::Vector4d planar_chess_board(0,0,1,0);  //棋盘格坐标系平面设置为z轴与棋盘格表面垂直，原点在棋盘格表面
        Eigen::Matrix4d T_cam2chess = Eigen::Matrix4d::Identity();
        T_cam2chess.block(0,0,3,3) = obi.chess_pose_qca.toRotationMatrix();
        T_cam2chess.block(0,3,3,1) = obi.chess_pose_tca;
        Eigen::Vector4d planar_cam = (T_cam2chess.inverse()).transpose() * planar_chess_board; //得到在相机坐标系下的棋盘格位姿
        Eigen::Vector3d nc = planar_cam.head<3>();
        double dc = planar_cam[3];
        //对数据进行展开然后进行一系列的操作，判断标定点是否符合
        std::vector<Eigen::Vector3d> calibra_pts;
        calibra_pts =  obi.points;
        for (size_t j = 0; j < calibra_pts.size(); ++j) //遍历每个标定用的激光点
        {
            Eigen::Vector3d pt =  calibra_pts[j];    //标定用的激光点
            Eigen::Vector3d bar_p(pt.x(),pt.y(),1);  //使用x和y的激光雷达上的坐标

            Eigen::Matrix<double , 1, 9> Ai;  //展开后前面H各个元素的系数，按照h1，h2...的顺序进行输入
            Ai<<nc.x() * bar_p.x(), nc.y() * bar_p.x(), nc.z()*bar_p.x(),
                    nc.x() * bar_p.y(), nc.y() * bar_p.y(), nc.z()*bar_p.y(),
                    nc.x() * bar_p.z(), nc.y() * bar_p.z(), nc.z()*bar_p.z();

            A.row(index) = Ai; //将计算后的展开成1行
            b(index) = -dc;
            index ++;
        }
    }
    //通过上述过程得到Ax=b的最小二乘方程，使用svd判断是否可以求解
    Eigen::MatrixXd AtA = A.transpose() * A;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(AtA, Eigen::ComputeThinU | Eigen::ComputeThinV);
    bool unobservable = false;
    for (size_t i = 0; i < svd.singularValues().size(); i++)
    {
        if(svd.singularValues()[i] < 1e-10)
        {
            unobservable = true;
        }
    }
    if(unobservable)
    {
        std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
        std::cout << " Notice Notice Notice: system unobservable !!!!!!!" << std::endl;
        std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl<<std::endl;
    }

    Eigen::VectorXd H(9);
    H = (AtA).ldlt().solve(A.transpose() * b);  //求解出最后的矩阵

    //将求得的H矩阵进行分解，拆为各方向向量
    Eigen::Vector3d h1 = H.segment<3>(0);
    Eigen::Vector3d h2 = H.segment<3>(3);
    Eigen::Vector3d h3 = H.segment<3>(6);

    Eigen::Matrix3d R_cam2laser;
    R_cam2laser.col(0) = h1;
    R_cam2laser.col(1) = h2;
    R_cam2laser.col(2) = h1.cross(h2); //z轴方向向量通过x和y轴叉乘获得
    Eigen::Matrix3d R_laser2cam = R_cam2laser.transpose();
    Eigen::Vector3d t_laser2cam = -R_laser2cam * h3;  //将上述旋转矩阵回代进行偏移量的求解

    // On Closed-Form Formulas for the 3D Nearest Rotation Matrix
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_tmp(R_laser2cam, Eigen::ComputeThinU | Eigen::ComputeThinV);
    R_laser2cam = svd_tmp.matrixU() * svd_tmp.matrixV().transpose(); //将旋转矩阵进行SVD分解，使其满足最终的正交阵性质，原先求解的矩阵可能不满足旋转矩阵正交阵的性质

    T_laser2cam.setIdentity();
    T_laser2cam.block(0,0,3,3) = R_laser2cam;
    T_laser2cam.block(0,3,3,1) = t_laser2cam;
    std::cout <<"------- Closed-form solution Tlc: -------\n" << T_laser2cam <<std::endl;
}

//使用ceres对标定结果进行优化
void LidarCamCalibration::CamLaserCalibrateCeres(const std::vector<Oberserve> obs, Eigen::Matrix4d &T_cam2laser)
{
    //相机雷达标定的四元数
    Eigen::Quaterniond q(T_cam2laser.block<3,3>(0,0));
    ceres::Problem problem;
    //为减少优化时旋转矩阵的参数量，将旋转矩阵转为4元数进行位姿求解
    Eigen::VectorXd pose(7);
    pose << T_cam2laser(0,3),T_cam2laser(1,3),T_cam2laser(2,3),q.x(),q.y(),q.z(), q.w();
    //获取每组标定数据
    for(size_t i = 0; i< obs.size(); ++i) {
        Oberserve obi = obs[i];
        //获取在chessboard在相机坐标系下的位姿
        Eigen::Vector4d planar_tag(0, 0, 1, 0);  // tag 坐标系下的平面方程
        Eigen::Matrix4d T_cam2chess = Eigen::Matrix4d::Identity();
        T_cam2chess.block(0, 0, 3, 3) = obi.chess_pose_qca.toRotationMatrix();
        T_cam2chess.block(0, 3, 3, 1) = obi.chess_pose_tca;
        Eigen::Vector4d planar_cam = (T_cam2chess.inverse()).transpose() * planar_tag; //

        std::vector<Eigen::Vector3d> calibra_pts;
        calibra_pts = obi.points;

        //每个点的影响权重求解
        double scale = calibra_pts.size();
        scale = 1. / sqrt(scale);
        for (size_t j = 0; j < calibra_pts.size(); ++j)
        {
            Eigen::Vector3d pt = calibra_pts[j];

            PointInPlaneFactor *costfunction = new PointInPlaneFactor(planar_cam, pt, scale);
            //ceres::LossFunctionWrapper* loss_function(new ceres::HuberLoss(1.0), ceres::TAKE_OWNERSHIP);
            ceres::LossFunction *loss_function = new ceres::CauchyLoss(0.05 * scale);
            problem.AddResidualBlock(costfunction, loss_function, pose.data());
        }
    }
    ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
    problem.AddParameterBlock(pose.data(), 7, local_parameterization);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 100;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.FullReport() << std::endl;

    q = Eigen::Quaterniond(pose[6], pose[3], pose[4], pose[5]);

    T_cam2laser.block<3, 3>(0, 0) = q.toRotationMatrix();
    T_cam2laser.block<3, 1>(0, 3) << pose[0], pose[1], pose[2];

    /// =============================  analysis code ==============================
    /// Get Information matrix from ceres, used to analysis the Gauge of the system
    Eigen::MatrixXd H(6, 6);
    Eigen::MatrixXd b(6, 1);
    H.setZero();
    b.setZero();
    double chi = 0;
    for (size_t i = 0; i < obs.size(); ++i) {
        Oberserve obi = obs[i];
        // transform planar in tag frame to cam frame
//        https://stackoverflow.com/questions/7685495/transforming-a-3d-plane-using-a-4x4-matrix
        Eigen::Vector4d planar_tag(0, 0, 1, 0);  // tag 坐标系下的平面方程
        Eigen::Matrix4d Tctag = Eigen::Matrix4d::Identity();
        Tctag.block(0, 0, 3, 3) = obi.chess_pose_qca.toRotationMatrix();
        Tctag.block(0, 3, 3, 1) = obi.chess_pose_tca;
        Eigen::Vector4d planar_cam = (Tctag.inverse()).transpose() * planar_tag;

        std::vector<Eigen::Vector3d> calibra_pts;
        calibra_pts = obi.points;

        double scale = calibra_pts.size(); // scale = 1/sqrt(n)
        scale = 1. / sqrt(scale);
        for (size_t j = 0; j < calibra_pts.size(); ++j) {
            Eigen::Vector3d pt = calibra_pts[j];
            double *res = new double[1];
            double **jaco = new double *[1];
            jaco[0] = new double[1 * 7];
            PointInPlaneFactor *costfunction = new PointInPlaneFactor(planar_cam, pt, scale);
            costfunction->Evaluate(std::vector<double *>{pose.data()}.data(), res, jaco);
            Eigen::Map<Eigen::Matrix<double, 1, 7, Eigen::RowMajor>> jacobian_pose_i(jaco[0]);
            Eigen::Map<Eigen::Matrix<double, 1, 1>> resd(res);

//            std::cout << jacobian_pose_i << std::endl;
            H += jacobian_pose_i.leftCols<6>().transpose() * jacobian_pose_i.leftCols<6>();
            b -= jacobian_pose_i.leftCols<6>().transpose() * resd;

            chi += resd * resd;

        }
    }

    std::cout << "----- H singular values--------:\n";
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    std::cout << svd.singularValues() << std::endl;
    int n = 0;
    for (size_t i = 0; i < svd.singularValues().size(); i++) {
        if (svd.singularValues()[i] < 1e-8)
            n++;
    }
    if (n > 0) {
        std::cout << "====== null space basis, it's means the unobservable direction for Tcl ======" << std::endl;
        std::cout << "       please note the unobservable direction is for Tcl, not for Tlc        " << std::endl;
        std::cout << svd.matrixV().rightCols(n) << std::endl;
    }
    std::cout << "\nrecover chi2: " << chi / 2. << std::endl;
}


//进行相机雷达外参标定
void LidarCamCalibration::LidarCamExtCalibrate(std::vector<std::pair<std::string ,Eigen::Matrix4d>>& T_cam2chesses,Eigen::Matrix4d& T_laser2cam)
{
    std::vector<Oberserve> obs;
    GenerateObservation(T_cam2chesses,obs);//产生观测数据
    if(obs.size() < 5)
    {
        std::cout << "Valid Calibra Data Less"<<std::endl;
        return ;
    }
    //通过建立Ax=b，使用SVD求解，变换矩阵封闭解，将其作为初值
    Eigen::Matrix4d T_laser2cam_initial = Eigen::Matrix4d::Identity();
    CamLaserCalClosedSolution(obs,T_laser2cam_initial);

    //代入初值后，使用ceres进行优化
    Eigen::Matrix4d T_cam2laser = T_laser2cam_initial.inverse();
    //进行ceres优化
    CamLaserCalibrateCeres(obs,T_cam2laser);
    T_laser2cam = T_cam2laser.inverse();
    std::cout<< T_laser2cam <<std::endl;
}


int main(int argc, char **argv)
{
    // 读取YAML文件
    YAML::Node config = YAML::LoadFile("../config/setting_config.yaml");
    std::string out_path = config["outfile_path"].as<std::string>();
    cv::Size pattern_size;
    pattern_size.width = config["chess_width"].as<int>(); //棋盘格的宽
    pattern_size.height = config["chess_height"].as<int>(); //棋盘格的高
    double chess_grid_size = config["chess_grid_size"].as<double>(); //棋盘格格子尺寸
    std::string input_image_dir = config["input_image_dir"].as<std::string>(); //输入图像的目录
    std::string input_scan_dir = config["input_scan_dir"].as<std::string>(); //输入激光点的目录
    std::string visual_chess_dir = config["visual_chess_dir"].as<std::string>(); //可视化标定板角点
    //初始化内参标定类
    LidarCamCalibration calibrater(input_image_dir,input_scan_dir,visual_chess_dir,pattern_size,chess_grid_size,out_path);
    cv::Mat camera_matrix,dist_coeffs;
    std::vector<std::pair<std::string ,Eigen::Matrix4d>> T_cam2chesses;
    calibrater.CamInstrinsicCalibrate(camera_matrix,dist_coeffs,T_cam2chesses); //相机内参标定
    Eigen::Matrix4d T_laser2cam;
    calibrater.LidarCamExtCalibrate(T_cam2chesses,T_laser2cam); //相机雷达外参标定

    cv::Mat T_laser2cam_mat(4, 4, CV_64F); // 创建一个CV_64F类型的4x4矩阵
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            T_laser2cam_mat.at<double>(i, j) = T_laser2cam(i, j);
        }
    }

    cv::FileStorage fs(out_path, cv::FileStorage::WRITE);
    // 将Eigen::Matrix4d写入到YAML文件
    fs << "T_laser2cam" << T_laser2cam_mat;
    // 将cv::Mat写入到YAML文件
    fs << "camera_matrix" << camera_matrix;
    fs << "dist_coeffs" << dist_coeffs;
    // 关闭文件
    fs.release();

    std::cout << "标定数据写入完成，写入至："<< out_path<< std::endl;

}