# 1. DLIO-PGO

### 1.1 Dependencies



The following has been verified to be compatible, although other configurations may work too:

- Ubuntu 20.04
- ROS Noetic (`roscpp`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `pcl_ros`)
- C++ 14
- CMake >= `3.12.4`
- OpenMP >= `4.5`
- Point Cloud Library >= `1.10.0`
- Eigen >= `3.3.7`
- GTSAM >= 4.0.0

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

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/Livox-SDK/livox_ros_driver.git
cd ~/catkin_ws && catkin_make
```



Compile using the [`catkin_tools`](https://catkin-tools.readthedocs.io/en/latest/) package via:

```
cd ~/catkin_ws 
git clone https://github.com/lovelyyoshino/3DGS_LIVO_reconstruction.git src
catkin_make
```



### 1.3 Execution



After compiling, source the workspace and execute via:
需要修改的就是：`livox_topic`、`imu_topic`这两个参数

```
roslaunch direct_lidar_inertial_odometry dlio_mid70_hk.launch 
```

然后运行PGO这部分只需要修改对应的参数即可，输入的topic保持不变
```
roslaunch aloam_velodyne dlio_mid70_hk.launch 
```


for Livox sensors (`xfer_format: 1`).

Be sure to change the topic names to your corresponding topics. Alternatively, edit the launch file directly if desired. If successful, you should see the following output in your terminal:

[![drawing](terminal.png)]()

### 1.3Services



To save DLIO's generated map into `.pcd` format, call the following service:

```
rosservice call /robot/dlio_map/save_pcd LEAF_SIZE SAVE_PATH
```
如果要保存PGO的地图可以使用
```
rosservice call  opt/save_map  LEAF_SIZE SAVE_PATH
```

定位部分则是
```
roslaunch direct_lidar_inertial_odometry vgicp.launch 

```

topic 发送
```
rostopic pub /initialpose geometry_msgs/PoseWithCovarianceStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'robot/odom'
pose:
  pose:
    position: {x: 0.0, y: 0.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
  covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" 

```

### 1.4 Test Data

For your convenience, we provide test data [here](https://drive.proton.me/urls/Z83QCWKZWW#bMIqDh02AJZZ) (1.2GB, 1m 13s, Ouster OS1-32) of an aggressive motion to test our motion correction scheme, and [here](https://drive.proton.me/urls/7NQSK9DXJ0#gZ9yjGNrDBgG) (16.5GB, 4m 21s, Ouster OSDome) of a longer trajectory outside with lots of trees. Try these two datasets with both deskewing on and off!

