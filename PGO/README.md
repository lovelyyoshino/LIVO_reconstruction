# DLIO-PGO 

## 1.baseline 

移植FAST_LIO_LC的[PGO](https://github.com/yanliang-wang/FAST_LIO_LC/tree/master/PGO)部分，回环部分支持 SC-PGO (ScanContext回环)  RS-PGO(Radius回环)

## 2.Quick Start

```shell
roslaunch direct_lidar_inertial_odometry long_corridor.launch

roslaunch aloam_velodyne dlio_velodyne_VLP_16.launch
```

## 3.闭环得分

```shell
SubT_MRS_Hawkins_Long_Corridor_RC.txt, ATE: 2.115740, RPE: 0.064375.
```

​																																													edited by kaho 2023.9.22