
# 单线激光雷达与相机联合标定项目

此项目包括使用ros进行激光雷达与相机标定数据采集、激光雷达和相机代码标定等。项目使用棋盘格标定板进行相机和雷达标定，对现有基于apriltag开源代码进行重整。

## 作者

- [@zhangwenhao](https://github.com/xiaoxina12?tab=repositories)


## 安装

1. 安装 ros系统 
参考链接鱼香ros直接安装
```bash
https://fishros.org.cn/forum/topic/20/%E5%B0%8F%E9%B1%BC%E7%9A%84%E4%B8%80%E9%94%AE%E5%AE%89%E8%A3%85%E7%B3%BB%E5%88%97?lang=zh-CN
```
2. 安装ceres库(依赖版本为1.14)，参考官方链接
```bash
https://github.com/ceres-solver/ceres-solver
```
3. 依赖的Eigen库、opencv、pcl库可以通过步骤1（ros安装）直接完成。(opencv4.0)
## 运行
1. 进行标定数据采集，创建ros工作空间，将record_data目录下的laser_rgb_data功能包拷贝至工作空间。

```bash
  mkdir -p catkin_ws/src
  sudo cp -r laser_rgb_data  catkin_ws/src
  cd catkin_ws
  catkin_make 
```
2. 修改代码中的激光雷达话题名以及相机图像话题名称，按空格键进行标定数据保存，后续标定代码要求图像保存为.jpg文件，激光数据保存为.pcd文件，文件名均相同。保存标定文件至少5对，一般20对左右，存放在image和scan文件夹中。
```bash
rosrun  laser_rgb_data laser_rgb_data_node
```
3. 对保存下来的激光数据进行标定板分割，该项目使用手动分割的方式（手动分割准确度高），不过暂未进行该部分代码编写，此处使用cloudcompare软件进行标定板激光数据分割，保存到相应的文件夹。

4. 进行相机内参以及激光雷达与相机联合标定。进入项目目录
```bash
mkdir build && cd build
cmake ..
make 
```
4.1 修改config/setting_config.yaml中的路径参数以及标定板的尺寸参数。完成相机雷达标定,得到相机相对雷达坐标系的变换矩阵
```bash
./LidarCamCalibration
```
4.2 对最终标定结果进行准确度可视化测试，将点云数据投影至相机图像上
```bash
./CalibrationTest
```

## 参考
1. 知乎https://zhuanlan.zhihu.com/p/137501892
2. apriltag码相机雷达标定代码https://github.com/MegviiRobot/CamLaserCalibraTool


