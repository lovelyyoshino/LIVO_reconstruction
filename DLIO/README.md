# README

## 层楼（multi_floor）

```bash
$ roslaunch direct_lidar_inertial_odometry multi_floor.launch
$ transform_node _model:=VLP16 _calibration:=/opt/ros/noetic/share/velodyne_pointcloud/params/VLP16db.yaml

$ rosbag record /robot/dlio/odom_node/odom -O multi_floor_dlio

$ rosbag play multi_floor.bag /cmu_sp1/camera_1/image_raw:=/rgb/image /cmu_sp1/imu/data:=/imu/data /cmu_sp1/velodyne_packets:=/velodyne_packets

$ evo_traj bag multi_floor_dlio.bag /robot/dlio/odom_node/odom --save_as_tum -p
```

## 长廊（long_corridor）

```bash
$ roslaunch direct_lidar_inertial_odometry long_corridor.launch
$ transform_node _model:=VLP16 _calibration:=/opt/ros/noetic/share/velodyne_pointcloud/params/VLP16db.yaml

$ rosbag record /robot/dlio/odom_node/odom -O long_corridor_dlio

$ rosbag play long_corridor.bag /camera_1/image_raw:=/rgb/image /cmu_sp1/imu/data:=/imu/data

$ evo_traj bag long_corridor_dlio.bag /robot/dlio/odom_node/odom --save_as_tum -p
```
