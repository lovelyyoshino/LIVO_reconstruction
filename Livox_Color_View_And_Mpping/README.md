# Livox_Color_View-And-Mpping

A module to realtime color the livox frame and color for FASTLIO2 map, inorder to checkout our extrinsicT


<p align='center'>
    <img src="./pic/FASTLIO-COLOR3.gif " alt="drawing" width="500" height ="280"/>
</p>
<p align='center'>
    <img src="./pic/mapping2.gif " alt="drawing" width="400" height ="300"/>
    <img src="./pic/view2.gif" alt="drawing" width="400" height =300/>
</p>

## Update Lists

```shell
# Verson 2023.3
	提交 livox_view   livox_mapping 模块，用于  快速验证camera-livox外参  &  基于FASTLIO2的上色
# Verson 2023.8
    1.修正 livox_mapping 模块  2. 幷提交livox_trigger 模块  3.提供测试数据集
    具体更改： livox_mapping  & livox_trigger 模块增加了时间软同步，避免之前因为camera与lidar没有数据同步，有较大时间差距，导致无法上色的问题。	两模块的时间软同步区别在于，livox_mapping使用的是ros自带的时间对齐缓冲器，livox_trigger 使用的自己写的逻辑时间软同步器。
    // livox_mapping  ROS 自带缓冲器
    sync_.reset(new Sync(MySyncPolicy(100), path_sub_, points_sub_, image_sub_));			//  时间软同步最大容忍时间为100ms
    sync_->registerCallback(boost::bind(&DataCallback,  _1, _2, _3));

    //  livox_trigger  时间软同步器
    if(lidar_buff_.size() == 0 || image_buff_.size() == 0 || odom_buff_.size() == 0) return false ;
```

## Related worked

1.感谢[luckyluckydadada](https://github.com/luckyluckydadada) 的 [LIVOX_COLOR](https://github.com/luckyluckydadada/LIVOX_COLOR) 工作 ,Livox_Color_View&Mapping 是基于LIVOX_COLOR进行修改的 。

2.视频参考来源[【自制】尝试Livox Mid40与D435i离线跑fastlio2彩色地图](https://www.bilibili.com/video/BV1MG411n7zd/?spm_id_from=333.999.0.0&vd_source=ed6bf57ee5a8e930b7a857e261dac86d)

## Our PlatForm

| Lidar       | Camera | Computer    | IMU     |
| ----------- | ------ | ----------- | ------- |
| Livox mid70 | D435i  | Intel NUC11 | LPMSIG1 |

<p align='center'>
    <img src="./pic/handle.gif " alt="drawing" width="450" height ="250"/>
</p>

## Contributions

**Livox_Color_View**：调用 camera lidar 的外参结果，进行实时单帧点云上色显示。

**FastLio2_Color_Mapping**：订阅FastLio2 输出的/Odometry信息,进行地图上色。

## Prepare work

**外参标定**：使用 [livox_camera_calib](https://github.com/hku-mars/livox_camera_calib)工具进行 lidar to camera 外参标定

**相机内参**：有条件可使用棋盘格进行相机内参标定，本次实验使用的是D435i原厂的相机内参

**SLAM里程计**：里程计订阅FastLio2的里程计信息，进行点云实时渲染上色。

## Prerequisites

- Ubuntu 18.04 and ROS Melodic
- PCL >= 1.8 (default for Ubuntu 18.04)
- Eigen >= 3.3.4 (default for Ubuntu 18.04)
- OpenCV >= 3.3

## Build

```shell
cd YOUR_WORKSPACE/src
git clone https://github.com/kahowang/Livox_Color_View_And_Mpping
cd ..
catkin_make
```

## Quick test by dataset

<p align='center'>
    <img src="./pic/view1.gif " alt="drawing" width="400" height ="250"/>
    <img src="./pic/mapping1.gif" alt="drawing" width="400" height =250/>
</p>

数据集由两部分组成，分别为 **Huck_SCAU_Mapping** 和 **Huck_SCAU_view**，由**SCAU**  [**Huke Wei**](https://gist.github.com/tangyubbb)同学提供，非常感谢~

### 1.Dataset Download

Huck_SCAU_view : 为静止测试单帧累积数据集，数据集下载地址 ：https://drive.google.com/file/d/1LY9TqONkWVVogt_S9epl6IqjNINgufAz/view?usp=sharing

Huck_SCAU_Mapping：为建图实时上色数据集，本数据集移动范围较少，提供已进行FASTLIO2后获得的点云和里程计，数据集下载地址 ：https://drive.google.com/file/d/1GLYfwe1vizN5BxuS6xPymwlOwf8dYy9s/view?usp=sharing

| Lidar       | Camera | Computer    | IMU         |
| ----------- | ------ | ----------- | ----------- |
| Livox mid40 | D455   | Intel NUC11 | D455内置IMU |

### 2.Livox_Color_View

```shell
#运行color view launch文件
roslaunch livox_color livox_color_view.launch
#运行数据集 Huck_SCAU_view
rosbag play  Huck_SCAU_view.bag
```

### 3.FastLio2_Color_Mapping  

使用ros时间软同步器		

```shell
#step1 : 运行color mappping launch文件
roslaunch livox_color livox_color_mapping.launch
#step2 : 运行数据集 Huck_SCAU_view
rosbag play  Huck_SCAU_Mapping.bag
#step3 : 保存地图,地图文件会保存于对应的文件夹中
rosservice call /save_map "{}"
```

### 4.FastLio2_Color_Mapping_Trigger  

使用自写时间软同步器		

```shell
#step1 : 运行color mappping launch文件
roslaunch livox_color livox_color_mapping_trigger.launch
#step2 : 运行数据集 Huck_SCAU_view
rosbag play  Huck_SCAU_Mapping.bag
#step3 : 保存地图,地图文件会保存于对应的文件夹中
rosservice call /save_map "{}"
```



## Quick start

<p align='center'>
    <img src="./pic/FASTLIO-COLOR.gif " alt="drawing" width="400" height ="250"/>
    <img src="./pic/FASTLIO-COLOR2.gif" alt="drawing" width="400" height =250/>
</p>

### 1.Livox_Color_View

读取标定后的结果，进行实时效果查看

```shell
#step0
cd /livox_color_ws
source devel/setup.bash

# step 1 启动livox雷达 注意使用livox数据类型为Pointcloud2
roslaunch livox_ros_driver livox_lidar.launch    

#step2 启动相机获取图像，运行 D435i节点  1280 x 720
roslaunch realsense2_camera rs_camera.launch

#step3 启动
roslaunch livox_color livox_color_view.launch
```

### 2.FastLio2_Color_Mapping		

```shell
#step0
cd /livox_color_ws
source devel/setup.bash

#step1 运行FASTLIO2 , 适配mid70 在FASTLIO2下建图的yaml文件已在本文件夹下
roslaunch fast_lio mapping_mid70_manual.launch

#step2 运行 livox_color_mapping 节点，订阅FASTLIO2里程计
roslaunch livox_color livox_color_mapping.launch

#step3 播放数据集
rosbag play *.bag

#step4 保存地图,地图文件会保存于对应的文件夹中
rosservice call /save_map "{}"
```

### 3.some config

```yaml
#FILE : livox_color_view.yaml && livox_color_mapping.yaml
common:
    lidar_topic:  "/cloud_registered"                  # 订阅FASTLIO2输出点云
    lidar_color_topic:  "/livox/color_lidar"      # 发布彩色点云话题
    odom_topic:  "/Odometry"                             # 订阅FASTLIO2输出里程计
    camera_topic:  "/camera/color/image_raw"   		# 订阅相机话题
    frame_id: "camera_init"                                   # FASTLIO2   Frame_ID

mapping:
    extrinsicT: [ 0.0144457,  -0.999828,  0.0116559,  -0.115962,
                            -0.0142691,  -0.0118621,  -0.999828, -0.0215207,
                            0.999794, 0.0142769,  -0.014438,  -0.0131816,
                            0.0,  0.0,  0.0,  1]        #  lidar2camera 外参  From livox_camera_calib

    intrisicT: [ 913.197692871094, 0, 648.626220703125,
                         0, 913.52783203125, 358.518096923828,
                         0, 0, 1]           # camera 内参

    ditortion: [0, 0, 0, 0, 0]      # camera 畸变系数

savepcd:
    savePCDDirectory: "/livox_color_ws/src/livox_color/PCD/"
    
#FILE:	livox_color_view.cpp && livox_color_mapping.cpp
define Hmax 720						#camera Height
define Wmax 1280				 #camera width 
```

## Acknowledgements

 In this project, the baseline is from  [LIVOX_COLOR](https://github.com/luckyluckydadada/LIVOX_COLOR) .

 Also thanks [Tomato1107](https://github.com/Tomato1107)    [lovelyyoshino](https://github.com/lovelyyoshino)   [Huang Hongqian](https://github.com/Natsu-Akatsuki)   [Huke Wei](https://gist.github.com/tangyubbb)  LeiHe 's great help .

​																																																							edited by kaho 2023.8.9
