# 1. 3DGS_LIVO_reconstruction

### 1.1 Dependencies



The following has been verified to be compatible, although other configurations may work too:

- Ubuntu 20.04
- ROS Noetic (`roscpp`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `pcl_ros`)
- C++ 14
- CMake >= `3.12.4`
- OpenMP >= `4.5`
- Point Cloud Library >= `1.10.0`
- Eigen >= `3.3.7`
- GTSAM >= `4.0.0`
- Sophus

### 1.2 Compiling



Livox

```
git clone https://github.com/Livox-SDK/Livox-SDK.git
cd Livox-SDK
cd build && cmake ..
make -j16
sudo make install
```



~~直接自带，无需再安装livox_ros_driver~~

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver.git
cd ~/catkin_ws && catkin_make
```



Compile using the [`catkin_tools`](https://catkin-tools.readthedocs.io/en/latest/) package via:

```bash
cd ~/catkin_ws 
git clone https://github.com/lovelyyoshino/3DGS_LIVO_reconstruction.git src
catkin_make
```



### 1.3 Execution

After compiling, source the workspace and execute via:

```bash
roslaunch fast_livo mapping_avia.launch
rosbag play YOUR_DOWNLOADED.bag
```

Fast-LIVO流程图
```mermaid
graph TD;
    A[laserMapping.cpp] --> B0[订阅点云信息sub_pcl]
    A --> B1[订阅IMU信息sub_imu]
    A --> B2[订阅图像信息sub_img]
    A --> C[数据包对齐同步sync_packages]
    C --> C1[判断是否有图像信息，并将所有数据存入到LidarMeasureGroup中]
    A --> D[利用IMU数据对状态变量进行积分递推p_imu->Process2]
    D --> D1[进入IMU_processing.cpp中]
    D1 --> D1_1[IMU初始化IMU_init]
    D1 --> D1_2[对点云去畸变UndistortPcl]
    D1_2 --> D1_2_1[根据时间戳完成点云去畸变操作]
    A --> E[当前是图像帧，则需要做视觉VIO操作lidar_selector->detect]
    E --> E1[进入lidar_selection.cpp中]
    E1 --> E1_1[使用相机模型和当前帧图像，构造一个图像帧]
    E1 --> E1_2[用IMU预积分更新一下状态]
    E1 --> E1_3[根据FOV去除遮挡点、深度不连续点，然后在于当前img作光度误差addFromSparseMap]
    E1_3 --> E1_3_1[计算上一帧的 LiDAR 点云投影到当前帧图像下，给图像的点赋值深度]
    E1_3 --> E1_3_2[遍历上面找到的所有体素，把体素中的所有地图点都拿出来投影到图像上]
    E1_3 --> E1_3_3[遍历上面的所有网格，如果里面有3D点，那么进一步处理看是否要把这个3D点作为最后的观测]
    E1_3_3 --> E1_3_3_1[判断点深度连续性]
    E1_3_3 --> E1_3_3_2[寻找这个地图点的所有patch中，和当前的图像的观测角度最相近的那个patch]
    E1_3_3 --> E1_3_3_3[计算这个patch所在的图像帧和当前帧的图像像素之间的affine变换]
    E1_3_3 --> E1_3_3_4[利用affine变换，计算ref帧的patch变换到当前帧的图像之后的像素值]
    E1_3_3 --> E1_3_3_5[对当前帧的图像取patch，得到patch中的像素值]
    E1_3_3 --> E1_3_3_6[计算ref帧的patch和cur帧的patch之间的像素误差]
    E1_3_3 --> E1_3_3_7[如果这个误差过大，说明可能是误匹配]
    E1_3_3 --> E1_3_3_8[如果当前点可以用，把所有的信息收集起来]
    E1_3 --> E1_3_4[优化之后添加新的地图点到视觉地图中]
    E1 --> E1_4[使用视觉直接对齐，对状态进行优化]
    E1 --> E1_5[对视觉地图点添加当前帧图像的新的patch观测]
    A --> F[如果是雷达帧，则运行LIO操作，这部分和FAST_LIO2一致。代码在lidar_en里面。]
```


