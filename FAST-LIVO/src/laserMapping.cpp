// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
// #include <common_lib.h>
#include <image_transport/image_transport.h>
#include "IMU_Processing.h"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>


#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <fast_livo/States.h>
#include <geometry_msgs/Vector3.h>
#include <livox_ros_driver/CustomMsg.h>
#include "preprocess.h"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vikit/camera_loader.h>
#include"lidar_selection.h"


// add gtsam
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "GNSS_processing.h"

#ifdef USE_ikdtree
    #ifdef USE_ikdforest
    #include <ikd-Forest/ikd_Forest.h>
    #else
    #include <ikd-Tree/ikd_Tree.h>
    #endif
#else
#include <pcl/kdtree/kdtree_flann.h>
#endif

#define INIT_TIME           (0.5)
#define MAXN                (360000)
#define PUBFRAME_PERIOD     (20)

float DET_RANGE = 300.0f;
#ifdef USE_ikdforest
    const int laserCloudWidth  = 200;
    const int laserCloudHeight = 200;
    const int laserCloudDepth  = 200;
    const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;
#else
    const float MOV_THRESHOLD = 1.5f;
#endif

mutex mtx_buffer;
std::mutex mtx;
condition_variable sig_buffer;

// mutex mtx_buffer_pointcloud;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic, img_topic, config_file;;
M3D Eye3d(M3D::Identity());
M3F Eye3f(M3F::Identity());
V3D Zero3d(0, 0, 0);
V3F Zero3f(0, 0, 0);
// Vector3d Lidar_offset_to_IMU(0.04165, 0.02326, -0.0284); // Avia
Vector3d Lidar_offset_to_IMU;
int iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0,\
    effct_feat_num = 0, time_log_counter = 0, publish_count = 0;
int MIN_IMG_COUNT = 0;

double res_mean_last = 0.05;
//double gyr_cov_scale, acc_cov_scale;
double gyr_cov_scale = 0, acc_cov_scale = 0;
//double last_timestamp_lidar, last_timestamp_imu = -1.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0, last_timestamp_img = -1.0;
//double filter_size_corner_min, filter_size_surf_min, filter_size_map_min, fov_deg;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
//double cube_len, HALF_FOV_COS, FOV_DEG, total_distance, lidar_end_time, first_lidar_time = 0.0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
double first_img_time=-1.0;
//double kdtree_incremental_time, kdtree_search_time;
double kdtree_incremental_time = 0, kdtree_search_time = 0, kdtree_delete_time = 0.0;
int kdtree_search_counter = 0, kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;;
//double copy_time, readd_time, fov_check_time, readd_box_time, delete_box_time;
double copy_time = 0, readd_time = 0, fov_check_time = 0, readd_box_time = 0, delete_box_time = 0;
double T1[MAXN], T2[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN];

double match_time = 0, solve_time = 0, solve_const_H_time = 0;

bool lidar_pushed, flg_reset, flg_exit = false;
bool ncc_en;
int dense_map_en = 1;
int img_en = 1;
int lidar_en = 1;
int debug = 0;
int frame_num = 0;
bool fast_lio_is_ready = false;
int grid_size, patch_size;
double outlier_threshold, ncc_thre;
/**
 * @brief add by crz
 *
 */
bool onlyUpdateBias, useVio, onlyUpdateBg, useKalmanSmooth, zero_point_one;
int eigenValueThreshold;
double img_time_offset;
PointCloudXYZI::Ptr pcl_wait_test(new PointCloudXYZI());

vector<BoxPointType> cub_needrm;
vector<BoxPointType> cub_needad;
// deque<sensor_msgs::PointCloud2::ConstPtr> lidar_buffer;
deque<PointCloudXYZI::Ptr>  lidar_buffer;
deque<double>          time_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
deque<cv::Mat> img_buffer;
deque<double>          img_time_buffer;
vector<bool> point_selected_surf; 
vector<vector<int>> pointSearchInd_surf; 
vector<PointVector> Nearest_Points; 
vector<double> res_last;
vector<double> extrinT(3, 0.0);
vector<double> extrinR(9, 0.0);
vector<double> cameraextrinT(3, 0.0);
vector<double> cameraextrinR(9, 0.0);
double total_residual;
double LASER_POINT_COV, IMG_POINT_COV, cam_fx, cam_fy, cam_cx, cam_cy; 
bool flg_EKF_inited, flg_EKF_converged, EKF_stop_flg = 0;
bool lio_first = false;
//surf feature in map
PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr cube_points_add(new PointCloudXYZI());
PointCloudXYZI::Ptr map_cur_frame_point(new PointCloudXYZI());
PointCloudXYZI::Ptr sub_map_cur_frame_point(new PointCloudXYZI()); //当前图像用的视觉地图点，很稀疏的点云

PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI());
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI());
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI());

// add gtsam
float transformTobeMapped[6] = {0, 0, 0, 0, 0, 0};//  当前帧的位姿(world系下)
// Surrounding map
float surroundingkeyframeAddingDistThreshold;  //  判断是否为关键帧的距离阈值
float surroundingkeyframeAddingAngleThreshold; //  判断是否为关键帧的角度阈值
float surroundingKeyframeDensity;
float surroundingKeyframeSearchRadius;
long int recontruct_map_index = 0;

// Loop close
bool aLoopIsClosed = false;
map<int, int> loopIndexContainer; // from new to old
vector<pair<int, int>> loopIndexQueue;
vector<gtsam::Pose3> loopPoseQueue;
vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
deque<std_msgs::Float64MultiArray> loopInfoVec;

bool startFlag = true;
bool loopClosureEnableFlag;
float loopClosureFrequency; //   回环检测频率
int surroundingKeyframeSize;
float historyKeyframeSearchRadius;   // 回环检测 radius kdtree搜索半径
float historyKeyframeSearchTimeDiff; //  帧间时间阈值
int historyKeyframeSearchNum;        //   回环时多少个keyframe拼成submap
float historyKeyframeFitnessScore;   // icp 匹配阈值
bool potentialLoopFlag = false;

// ros::Publisher pubHistoryKeyFrames; //  发布 loop history keyframe submap
// ros::Publisher pubIcpKeyFrames;
ros::Publisher pubHistoryRGBMap;
ros::Publisher pubLoopConstraintEdge;

// 局部关键帧构建的map点云，对应kdtree，用于scan-to-map找相邻点
pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses(new pcl::KdTreeFLANN<PointType>());


/*back end*/
PointCloudXYZRGB::Ptr laserCloudLidarRGB(new PointCloudXYZRGB());
vector<pcl::PointCloud<PointType>::Ptr>surfCloudKeyFrames;
vector<PointCloudXYZRGB::Ptr> surfCloudKeyFramesRGB;   // 历史所有关键帧的平面点集合（降采样）
PointCloudXYZRGB::Ptr reconstructed_map(new PointCloudXYZRGB());


pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D(new pcl::PointCloud<PointType>());         // 历史关键帧位姿（位置）
pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D(new pcl::PointCloud<PointTypePose>()); // 历史关键帧位姿
pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D(new pcl::PointCloud<PointTypePose>());

pcl::PointCloud<PointTypePose>::Ptr fastlio_unoptimized_cloudKeyPoses6D(new pcl::PointCloud<PointTypePose>()); //  存储fastlio 未优化的位姿
pcl::PointCloud<PointTypePose>::Ptr gnss_cloudKeyPoses6D(new pcl::PointCloud<PointTypePose>()); //  gnss 轨迹

gtsam::NonlinearFactorGraph gtSAMgraph;
gtsam::Values initialEstimate;
gtsam::Values optimizedEstimate;
gtsam::ISAM2 *isam;
gtsam::Values isamCurrentEstimate;
Eigen::MatrixXd poseCovariance;
int numberOfCores =4;


// gnss
double last_timestamp_gnss = -1.0 ;
deque<nav_msgs::Odometry> gnss_buffer;
geometry_msgs::PoseStamped msg_gnss_pose;
string gnss_topic ;
bool useImuHeadingInitialization;   
bool useGpsElevation;             //  是否使用gps高层优化
float gpsCovThreshold;          //   gps方向角和高度差的协方差阈值
float poseCovThreshold;       //  位姿协方差阈值  from isam2

M3D Gnss_R_wrt_Lidar(Eye3d) ;         // gnss  与 imu 的外参
V3D Gnss_T_wrt_Lidar(Zero3d);
bool gnss_inited = false ;                        //  是否完成gnss初始化
shared_ptr<GnssProcess> p_gnss(new GnssProcess());
GnssProcess gnss_data;
ros::Publisher pubGnssPath ;
nav_msgs::Path gps_path ;
vector<double>       extrinT_Gnss2Lidar(3, 0.0);
vector<double>       extrinR_Gnss2Lidar(9, 0.0);
nav_msgs::Path globalPath;


pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;
pcl::VoxelGrid<PointType> downSizeFilterICP;
float mappingSurfLeafSize;

#ifdef USE_ikdtree
    #ifdef USE_ikdforest
    KD_FOREST ikdforest;
    #else
    KD_TREE ikdtree;
    #endif
#else
pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<PointType>());
#endif

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
Eigen::Matrix3d Rcl;
Eigen::Vector3d Pcl;

//estimator inputs and output;
LidarMeasureGroup LidarMeasures; // 同步之后的消息
// SparseMap sparse_map;
#ifdef USE_IKFOM
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;
#else
StatesGroup  state;//全局变量 18维状态量
#endif

nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::Quaternion geoQuat;
geometry_msgs::PoseStamped msg_body_pose;

//这个应该是lidar的前端特征点云的预处理类，主要是对lidar的点云进行稍微的处理（但是和提取平面特征又不完全一样）
shared_ptr<Preprocess> p_pre(new Preprocess());

PointCloudXYZRGB::Ptr pcl_wait_save(new PointCloudXYZRGB());  //add save rbg map
PointCloudXYZI::Ptr pcl_wait_save_lidar(new PointCloudXYZI());  //add save xyzi map

bool pcd_save_en = false;
int pcd_save_interval = 20, pcd_index = 0;


void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

inline void dump_lio_state_to_log(FILE *fp)  
{
    #ifdef USE_IKFOM
    //state_ikfom write_state = kf.get_x();
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", LidarMeasures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g  
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a  
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a  
    fprintf(fp, "\r\n");  
    fflush(fp);  
    #else
    V3D rot_ang(Log(state.rot_end));
    fprintf(fp, "%lf ", LidarMeasures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state.pos_end(0), state.pos_end(1), state.pos_end(2)); // Pos  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
    fprintf(fp, "%lf %lf %lf ", state.vel_end(0), state.vel_end(1), state.vel_end(2)); // Vel  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
    fprintf(fp, "%lf %lf %lf ", state.bias_g(0), state.bias_g(1), state.bias_g(2));    // Bias_g  
    fprintf(fp, "%lf %lf %lf ", state.bias_a(0), state.bias_a(1), state.bias_a(2));    // Bias_a  
    fprintf(fp, "%lf %lf %lf ", state.gravity(0), state.gravity(1), state.gravity(2)); // Bias_a  
    fprintf(fp, "\r\n");  
    fflush(fp);
    #endif  
}


/**
 * 对点云cloudIn进行变换transformIn，返回结果点云， 修改liosam, 考虑到外参的表示
 */
pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);
    
   // 注意：lio_sam 中的姿态用的euler表示，而fastlio存的姿态角是旋转矢量。而 pcl::getTransformation是将euler_angle 转换到rotation_matrix 不合适，注释
  // Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
    #ifdef USE_IKFOM
    Eigen::Isometry3d T_b_lidar(state_point.offset_R_L_I);
    T_b_lidar.pretranslate(state_point.offset_T_L_I);         //  获取  body2lidar  外参
    #else
    Eigen::Isometry3d T_b_lidar = Eigen::Isometry3d::Identity();
    T_b_lidar.translation() =  Lidar_offset_to_IMU;       //  获取  body2lidar  外参
    #endif
    
    
          

    Eigen::Affine3f T_w_b_ = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
    Eigen::Isometry3d T_w_b ;          //   world2body  
    T_w_b.matrix() = T_w_b_.matrix().cast<double>();

    Eigen::Isometry3d  T_w_lidar  =  T_w_b * T_b_lidar  ;           //  T_w_lidar  转换矩阵

    Eigen::Isometry3d transCur = T_w_lidar;        

#pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
        cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
        cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }
    return cloudOut;
}


pcl::PointCloud<PointType>::Ptr transformPointCloudRGB_None(PointCloudXYZRGB::Ptr cloudIn, PointTypePose *transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);
    
   // 注意：lio_sam 中的姿态用的euler表示，而fastlio存的姿态角是旋转矢量。而 pcl::getTransformation是将euler_angle 转换到rotation_matrix 不合适，注释
  // Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
    #ifdef USE_IKFOM
    Eigen::Isometry3d T_b_lidar(state_point.offset_R_L_I);
    T_b_lidar.pretranslate(state_point.offset_T_L_I);         //  获取  body2lidar  外参
    #else
    Eigen::Isometry3d T_b_lidar = Eigen::Isometry3d::Identity();
    T_b_lidar.translation() =  Lidar_offset_to_IMU;       //  获取  body2lidar  外参
    #endif
    
    
          

    Eigen::Affine3f T_w_b_ = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
    Eigen::Isometry3d T_w_b ;          //   world2body  
    T_w_b.matrix() = T_w_b_.matrix().cast<double>();

    Eigen::Isometry3d  T_w_lidar  =  T_w_b * T_b_lidar  ;           //  T_w_lidar  转换矩阵

    Eigen::Isometry3d transCur = T_w_lidar;        

#pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
        cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
        cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
        cloudOut->points[i].intensity = 0;
    }
    return cloudOut;
}



/**
 * 对点云cloudIn进行变换transformIn，返回结果点云， 修改liosam, 考虑到外参的表示
 */
PointCloudXYZRGB::Ptr transformPointCloud(PointCloudXYZRGB::Ptr cloudIn, PointTypePose *transformIn)
{
    PointCloudXYZRGB::Ptr cloudOut(new PointCloudXYZRGB());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);
    
   // 注意：lio_sam 中的姿态用的euler表示，而fastlio存的姿态角是旋转矢量。而 pcl::getTransformation是将euler_angle 转换到rotation_matrix 不合适，注释
  // Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
    #ifdef USE_IKFOM
    Eigen::Isometry3d T_b_lidar(state_point.offset_R_L_I);
    T_b_lidar.pretranslate(state_point.offset_T_L_I);         //  获取  body2lidar  外参
    #else
    Eigen::Isometry3d T_b_lidar = Eigen::Isometry3d::Identity();
    T_b_lidar.translation() =  Lidar_offset_to_IMU;       //  获取  body2lidar  外参
    #endif
    
    
          

    Eigen::Affine3f T_w_b_ = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);
    Eigen::Isometry3d T_w_b ;          //   world2body  
    T_w_b.matrix() = T_w_b_.matrix().cast<double>();

    Eigen::Isometry3d  T_w_lidar  =  T_w_b * T_b_lidar  ;           //  T_w_lidar  转换矩阵

    Eigen::Isometry3d transCur = T_w_lidar;        

#pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
        cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
        cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
        cloudOut->points[i].r = pointFrom.r;
        cloudOut->points[i].g = pointFrom.g;
        cloudOut->points[i].b = pointFrom.b;
    }
    return cloudOut;
}

#ifdef USE_IKFOM
//project the lidar scan to world frame
void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}
#endif

/// @brief 把局部坐标系下的点，通过全局维护的位姿变量投影到世界坐标系下
/// @param pi 输入点
/// @param po 输出点
void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    #ifdef USE_IKFOM
    //state_ikfom transfer_state = kf.get_x();
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);
    #else
    V3D p_global(state.rot_end * (p_body + Lidar_offset_to_IMU) + state.pos_end);
    #endif

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    #ifdef USE_IKFOM
    //state_ikfom transfer_state = kf.get_x();
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);
    #else
    V3D p_global(state.rot_end * (p_body + Lidar_offset_to_IMU) + state.pos_end);
    #endif
    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    #ifdef USE_IKFOM
    //state_ikfom transfer_state = kf.get_x();
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);
    #else
    V3D p_global(state.rot_end * (p_body + Lidar_offset_to_IMU) + state.pos_end);
    #endif
    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;

    float intensity = pi->intensity;
    intensity = intensity - floor(intensity);

    int reflection_map = intensity*10000;
}

void RGBpointWorldToBody(PointCloudXYZRGB::Ptr pi, PointCloudXYZRGB::Ptr po)
{
    for (int i = 0; i < pi->points.size(); i++)
    {
        V3D p_global(pi->points[i].x, pi->points[i].y, pi->points[i].z);
        #ifdef USE_IKFOM
        //state_ikfom transfer_state = kf.get_x();
        V3D p_body(state_point.rot.transpose() * (p_global - state_point.pos) - state_point.offset_T_L_I);
        #else
        V3D p_body(state.rot_end.transpose() * (p_global - state.pos_end) - Lidar_offset_to_IMU);
        #endif
        PointTypeRGB temp_point;
        temp_point.x = p_body(0);
        temp_point.y = p_body(1);
        temp_point.z = p_body(2);
        temp_point.r = pi->points[i].r;
        temp_point.g = pi->points[i].g;
        temp_point.b = pi->points[i].b;
        po->points.emplace_back(temp_point);
    }
}


#ifndef USE_ikdforest
int points_cache_size = 0;
void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    points_cache_size = points_history.size();
}
#endif

BoxPointType get_cube_point(float center_x, float center_y, float center_z)
{
    BoxPointType cube_points;
    V3F center_p(center_x, center_y, center_z);
    // cout<<"center_p: "<<center_p.transpose()<<endl;

    for(int i = 0; i < 3; i++)
    {
        cube_points.vertex_max[i] = center_p[i] + 0.5 * cube_len;
        cube_points.vertex_min[i] = center_p[i] - 0.5 * cube_len;
    }

    return cube_points;
}

BoxPointType get_cube_point(float xmin, float ymin, float zmin, float xmax, float ymax, float zmax)
{
    BoxPointType cube_points;
    cube_points.vertex_max[0] = xmax;
    cube_points.vertex_max[1] = ymax;
    cube_points.vertex_max[2] = zmax;
    cube_points.vertex_min[0] = xmin;
    cube_points.vertex_min[1] = ymin;
    cube_points.vertex_min[2] = zmin;
    return cube_points;
}

#ifndef USE_ikdforest
BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
/// @brief 过滤在当前LiDAR的FOV内的点云
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    #ifdef USE_IKFOM
    //state_ikfom fov_state = kf.get_x();
    //V3D pos_LiD = fov_state.pos + fov_state.rot * fov_state.offset_T_L_I;
    V3D pos_LiD = pos_lid;
    #else
    V3D pos_LiD = state.pos_end;
    #endif
    if (!Localmap_Initialized){
        //if (cube_len <= 2.0 * MOV_THRESHOLD * DET_RANGE) throw std::invalid_argument("[Error]: Local Map Size is too small! Please change parameter \"cube_side_length\" to larger than %d in the launch file.\n");
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    // printf("Local Map is (%0.2f,%0.2f) (%0.2f,%0.2f) (%0.2f,%0.2f)\n", LocalMap_Points.vertex_min[0],LocalMap_Points.vertex_max[0],LocalMap_Points.vertex_min[1],LocalMap_Points.vertex_max[1],LocalMap_Points.vertex_min[2],LocalMap_Points.vertex_max[2]);
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
            // printf("Delete Box is (%0.2f,%0.2f) (%0.2f,%0.2f) (%0.2f,%0.2f)\n", tmp_boxpoints.vertex_min[0],tmp_boxpoints.vertex_max[0],tmp_boxpoints.vertex_min[1],tmp_boxpoints.vertex_max[1],tmp_boxpoints.vertex_min[2],tmp_boxpoints.vertex_max[2]);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
            // printf("Delete Box is (%0.2f,%0.2f) (%0.2f,%0.2f) (%0.2f,%0.2f)\n", tmp_boxpoints.vertex_min[0],tmp_boxpoints.vertex_max[0],tmp_boxpoints.vertex_min[1],tmp_boxpoints.vertex_max[1],tmp_boxpoints.vertex_min[2],tmp_boxpoints.vertex_max[2]);                     
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
    // printf("Delete time: %0.6f, delete size: %d\n",kdtree_delete_time,kdtree_delete_counter);
    // printf("Delete Box: %d\n",int(cub_needrm.size()));
}
#endif

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    mtx_buffer.lock();
    sensor_msgs::PointCloud2::Ptr msg_in(new sensor_msgs::PointCloud2(*msg));
    if (zero_point_one) {
        msg_in->header.stamp = ros::Time().fromSec(msg->header.stamp.toSec() - 0.1);
    }
    // cout<<"got feature"<<endl;
    if (msg_in->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    
    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg_in, ptr);
    // ROS_INFO("get point cloud at time: %.6f and size: %d", msg_in->header.stamp.toSec() - 0.1, ptr->points.size());
    printf("[ INFO ]: get point cloud at time: %.6f and size: %d.\n", msg_in->header.stamp.toSec(), int(ptr->points.size()));
    lidar_buffer.push_back(ptr);
    // time_buffer.push_back(msg_in->header.stamp.toSec() - 0.1);
    // last_timestamp_lidar = msg_in->header.stamp.toSec() - 0.1;
    time_buffer.push_back(msg_in->header.stamp.toSec());
    last_timestamp_lidar = msg_in->header.stamp.toSec();
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

/// @brief  livox激光雷达的消息回调函数
/// @param msg 激光雷达消息
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg) 
{
    //数据加锁，说明IMU Lidar camera同时只有一个数据能放buffer
    mtx_buffer.lock();
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    printf("[ INFO ]: get point cloud at time: %.6f.\n", msg->header.stamp.toSec());
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    // 对收到的原始点云进行一些预处理，得到后面要使用的面点
    p_pre->process(msg, ptr);// 把livox数据转成pcl点云
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(msg->header.stamp.toSec());//把时间也放入time_buffer
    last_timestamp_lidar = msg->header.stamp.toSec();

    mtx_buffer.unlock();//把时间也放入time_buffer
    sig_buffer.notify_all();
}

/// @brief  IMU消息的回调函数：存储IMU消息到buf中
/// @param msg_in IMU消息
void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) 
{
    publish_count ++;
    //cout<<"msg_in:"<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
    
    double timestamp = msg->header.stamp.toSec();
    mtx_buffer.lock();//数据加锁

    if (timestamp < last_timestamp_imu)
    {
        ROS_ERROR("imu loop back, clear buffer");
        imu_buffer.clear();
        flg_reset = true;
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    // cout<<"got imu: "<<timestamp<<" imu size "<<imu_buffer.size()<<endl;
    mtx_buffer.unlock();
    sig_buffer.notify_all();//唤醒wait线程
}

/**
 * @brief 从ros消息中把图像数据转成cv::Mat类型
 *  参考博客：https://blog.csdn.net/bigdog_1027/article/details/79090571
 *  https://zhuanlan.zhihu.com/p/392285419
 */
cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr& img_msg) {
  cv::Mat img;
    // cv_bridge::toCvShare(img_msg, "bgr8") 是一个cv_bridge::CvImagePtr类型的指针，
    // 这里用的是匿名对象的写法，然后直接调用这个指针的成员变量 image，就得到了对应的cv::Mat类型的数据
    img = cv_bridge::toCvShare(img_msg, "bgr8")->image;
    // 下面这个修改是github上一个人提的issue说的，说图片比较大的时候使用toCvShare容易出问题，
    // 但是作者说测试过最大500w像素的图片没有出问题，因此没有采纳这个issue的建议
    // img = cv_bridge::toCvCopy(img_msg, "bgr8")->image;
  return img;
}

/// @brief 图像消息时间戳
/// @param msg 图像消息
void img_cbk(const sensor_msgs::ImageConstPtr& msg)
{
    // cout<<"In Img_cbk"<<endl;
    // if (first_img_time<0 && time_buffer.size()>0) {
    //     first_img_time = msg->header.stamp.toSec() - time_buffer.front();
    // }
    if (!img_en) 
    {
        return;
    }
    printf("[ INFO ]: get img at time: %.6f.\n", msg->header.stamp.toSec());
    if (msg->header.stamp.toSec() - img_time_offset < last_timestamp_img)
    {
        ROS_ERROR("img loop back, clear buffer");
        img_buffer.clear();
        img_time_buffer.clear();
    }
    mtx_buffer.lock();
    // cout<<"Lidar_buff.size()"<<lidar_buffer.size()<<endl;
    // cout<<"Imu_buffer.size()"<<imu_buffer.size()<<endl;
    img_buffer.push_back(getImageFromMsg(msg));//转换为mat格式
    img_time_buffer.push_back(msg->header.stamp.toSec() - img_time_offset);
    last_timestamp_img = msg->header.stamp.toSec() - img_time_offset;
    // cv::imshow("img", img);
    // cv::waitKey(1);
    // cout<<"last_timestamp_img:::"<<last_timestamp_img<<endl;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

//  发布gnss 轨迹
void publish_gnss_path(const ros::Publisher pubPath)
{
    gps_path.header.stamp = ros::Time().fromSec(lidar_end_time);
    gps_path.header.frame_id = "camera_init";

    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) 
    {
        pubPath.publish(gps_path);
    }
}


void gnss_cbk(const sensor_msgs::NavSatFixConstPtr& msg_in)
{
    //  ROS_INFO("GNSS DATA IN ");
    double timestamp = msg_in->header.stamp.toSec();

    mtx_buffer.lock();

    // 没有进行时间纠正
    if (timestamp < last_timestamp_gnss)
    {
        ROS_WARN("gnss loop back, clear buffer");
        gnss_buffer.clear();
    }

    last_timestamp_gnss = timestamp;

    // convert ROS NavSatFix to GeographicLib compatible GNSS message:
    gnss_data.time = msg_in->header.stamp.toSec();
    gnss_data.status = msg_in->status.status;
    gnss_data.service = msg_in->status.service;
    gnss_data.pose_cov[0] = msg_in->position_covariance[0];
    gnss_data.pose_cov[1] = msg_in->position_covariance[4];
    gnss_data.pose_cov[2] = msg_in->position_covariance[8];

    mtx_buffer.unlock();
   
    if(!gnss_inited){           //  初始化位置
        gnss_data.InitOriginPosition(msg_in->latitude, msg_in->longitude, msg_in->altitude) ; 
        gnss_inited = true ;
    }else{                               //   初始化完成
        gnss_data.UpdateXYZ(msg_in->latitude, msg_in->longitude, msg_in->altitude) ;             //  WGS84 -> ENU  ???  调试结果好像是 NED 北东地

        Eigen::Matrix4d gnss_pose = Eigen::Matrix4d::Identity();
        gnss_pose(0,3) = gnss_data.local_N ;                 //    北
        gnss_pose(1,3) = gnss_data.local_E ;                 //     东
        gnss_pose(2,3) = -gnss_data.local_U ;                 //    地

        Eigen::Isometry3d gnss_to_lidar(Gnss_R_wrt_Lidar) ;
        gnss_to_lidar.pretranslate(Gnss_T_wrt_Lidar);
        gnss_pose  =  gnss_to_lidar  *  gnss_pose ;                    //  gnss 转到 lidar 系下, （当前Gnss_T_wrt_Lidar，只是一个大致的初值）

        nav_msgs::Odometry gnss_data_enu ;
        // add new message to buffer:
        gnss_data_enu.header.stamp = ros::Time().fromSec(gnss_data.time);
        gnss_data_enu.pose.pose.position.x =  gnss_pose(0,3) ;  //gnss_data.local_E ;   北
        gnss_data_enu.pose.pose.position.y =  gnss_pose(1,3) ;  //gnss_data.local_N;    东
        gnss_data_enu.pose.pose.position.z =  gnss_pose(2,3) ;  //  地

        gnss_data_enu.pose.pose.orientation.x =  geoQuat.x ;                //  gnss 的姿态不可观，所以姿态只用于可视化，取自imu
        gnss_data_enu.pose.pose.orientation.y =  geoQuat.y;
        gnss_data_enu.pose.pose.orientation.z =  geoQuat.z;
        gnss_data_enu.pose.pose.orientation.w =  geoQuat.w;

        gnss_data_enu.pose.covariance[0] = gnss_data.pose_cov[0] ;
        gnss_data_enu.pose.covariance[7] = gnss_data.pose_cov[1] ;
        gnss_data_enu.pose.covariance[14] = gnss_data.pose_cov[2] ;

        gnss_buffer.push_back(gnss_data_enu);

        // visial gnss path in rviz:
        msg_gnss_pose.header.frame_id = "camera_init";
        msg_gnss_pose.header.stamp = ros::Time().fromSec(gnss_data.time);

        msg_gnss_pose.pose.position.x = gnss_pose(0,3) ;  
        msg_gnss_pose.pose.position.y = gnss_pose(1,3) ;
        msg_gnss_pose.pose.position.z = gnss_pose(2,3) ;

        gps_path.poses.push_back(msg_gnss_pose);

        //  save_gnss path
        PointTypePose thisPose6D;  
        thisPose6D.x = msg_gnss_pose.pose.position.x ;
        thisPose6D.y = msg_gnss_pose.pose.position.y ;
        thisPose6D.z = msg_gnss_pose.pose.position.z ;
        thisPose6D.intensity = 0;
        thisPose6D.roll =0;
        thisPose6D.pitch = 0;
        thisPose6D.yaw = 0;
        thisPose6D.time = lidar_end_time;
        gnss_cloudKeyPoses6D->push_back(thisPose6D);   
    }


}


/**
 * @brief 以LiDAR时间戳为一次处理前提的数据包对齐。
 *          LiDAR时间戳：   |            |           |   （注意以一帧点云结尾时间戳为准）
 *                         1            2           3
 *           相机时间戳 ：      |   |   |   |   |   |   |
 *                            1   2   3   4   5   6   7
 * 两种情况：1. 当前最新帧就是LiDAR，没有更老的图像，比如上面第1帧lidar，那么同步的方法和LIO一样，非常简单
 *         2. 当前必须有一个LIDAR，比如上面第2帧lidar。但是LiDAR前面有一些更老的图像没有处理，比如
 *            第1、2、3帧图像，那么此时就要先同步这三帧图像。但是显然这种方法对图像的处理是有一些滞后的， 
 *            因为它要求必须有了一帧LiDAR才能对 比这帧LiDAR更老的图像进行处理。不过lidar是10hz，也不会
 *            滞后太多，因此问题也不大。
 * @param[in] meas  一次处理的数据包
 * @return true 
 * @return false 
 */
bool sync_packages(LidarMeasureGroup &meas)
{
    if ((lidar_buffer.empty() && img_buffer.empty())) { // has lidar topic or img topic?
        return false;
    }
    // If meas.is_lidar_end==true, means it just after scan end, clear all buffer in meas. 一次扫描结束
    //; 如果上次同步的消息是以lidar为结尾的，说明上次处理了一帧以lidar为结尾的数据，那么这次统计
    //; 图像数据的时候就要先清空了，因为measures里面存储的是以上一帧LiDAR为结尾的多帧的图像数据和IMU数据
    if (meas.is_lidar_end) // If meas.is_lidar_end==true, means it just after scan end, clear all buffer in meas.
    {
        meas.measures.clear();
        meas.is_lidar_end = false;
    }
    
    if (!lidar_pushed) { // If not in lidar scan, need to generate new meas
        //! 疑问：这里这种时间戳对齐要求当前对齐一帧图像或者LiDAR数据的时候，必须有LiDAR数据存在。
        // 也就是视觉的数据会被延迟最大0.1s处理。比如图像是30hz, LiDAR是10hz，他们都经过硬件时间同步，
        // 也就是时间戳是完全准确的，没有任何时间偏移。如下图所示：
        //                                    1           2           3
        //    LiDAR时间戳：   |           |           | 
        //    相机时间戳：       |   |   |   |   |   |   |
        //                                 1   2   3   4   5   6   7
        // (1)假设LiDAR先来，那么后面会先把第1帧LiDAR处理掉。
        // (2)第二次同步的时候进入当前if分支，发现LiDAR是空的，那么直接返回。此时即使有几帧相机数据，
        //    但是由于没有第2帧的LiDAR数据，所以这里仍然不会处理相机数据，而是要等到第2帧LiDAR数据
        //    来了之后才会往下进行。往下进行发现有一些比第2帧LiDAR数据更老的图像数据，也就是相机的
        //    前3帧都没有被处理，所以此时才会处理这3帧图像数据。处理完这3帧图像数据之后，然后继续
        //    处理第2帧的LiDAR数据。
        //; 另外注意：这里的lidar时间戳是以一帧的结束为标准的，因为后面去畸变是把一帧点云对齐到结尾
        if (lidar_buffer.empty()) {
            // ROS_ERROR("out sync");
            return false;
        }
        //这里的LiDAR是单独存成了PCL点云格式，所以后面用一个单独的time_buffer来存储它的时间戳了
        meas.lidar = lidar_buffer.front(); // push the firsrt lidar topic       
        //; 如果这帧lidar点云无效，则要弹出图像数据。但是这个地方正常来说应该不会发生？
        if(meas.lidar->points.size() <= 1)
        {
            mtx_buffer.lock();
            if (img_buffer.size()>0) // temp method, ignore img topic when no lidar points, keep sync
            {
                lidar_buffer.pop_front();
                img_buffer.pop_front();
            }
            mtx_buffer.unlock();
            sig_buffer.notify_all();
            // ROS_ERROR("out sync");
            return false;
        }
        sort(meas.lidar->points.begin(), meas.lidar->points.end(), time_list);  //对lidar中的点云根据时间戳进行排序
        // generate lidar_beg_time // 雷达开始时间
        //! 疑问：这个地方感觉和前面 lidar_buffer.pop_front(); 不同步？但是正常来说前面的问题应该不会发生
        meas.lidar_beg_time = time_buffer.front(); // 把点云开始时间也赋上，用于后续的时间戳对齐
        //最后点的时间戳加上曲率，得到点云结束时间
        lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000); //  计算雷达扫描结束时间
        lidar_pushed = true;  //lidar_pushed 表示meas中的lidar点云插入了，但是还没有从buf中弹出
    }

    //没图像，只有雷达数据，收集IMU信息
    if (img_buffer.empty()) { // no img topic, means only has lidar topic
        if (last_timestamp_imu < lidar_end_time+0.02) {  // lidar_pushed 表示meas中的lidar点云插入了，但是还没有从buf中弹出
            // ROS_ERROR("out sync");
            return false;
        }
        struct MeasureGroup m; //standard method to keep imu message.
        double imu_time = imu_buffer.front()->header.stamp.toSec();
        m.imu.clear();
        mtx_buffer.lock();
        //把小于最后一个雷达点云的IMU都放进去m中
        while ((!imu_buffer.empty() && (imu_time<lidar_end_time))) {
            imu_time = imu_buffer.front()->header.stamp.toSec();
            if(imu_time > lidar_end_time) break;
            m.imu.push_back(imu_buffer.front());
            imu_buffer.pop_front();
        }
        //现在真正统计完一次以LiDAR为结尾的数据了，所以要把LiDAR消息和对应的时间戳弹出
        lidar_buffer.pop_front();
        time_buffer.pop_front();
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        // lidar_pushed=true，说明 meas 插入了LiDAR消息，但是还没有同步完成，也就是在buffer中还有这个lidar消息
        // 而如果=fasle，说明 meas 插入了LIDAR消息并且同步完成了，也就是buffer中已经弹出这个消息了
        lidar_pushed = false; //这个标志用来控制每次收集好了IMU信息后，下一次就是放雷达点云了
        meas.is_lidar_end = true; //说明已经把雷达点云前的IMU放入了，当前的meas是以雷达结束的
        meas.measures.push_back(m);
        // ROS_ERROR("out sync");
        return true;
    }
    struct MeasureGroup m;//; 运行到这里，说明图像不为空，则要同时统计图像和LiDAR的时间戳
    // cout<<"lidar_buffer.size(): "<<lidar_buffer.size()<<" img_buffer.size(): "<<img_buffer.size()<<endl;
    // cout<<"time_buffer.size(): "<<time_buffer.size()<<" img_time_buffer.size(): "<<img_time_buffer.size()<<endl;
    // cout<<"img_time_buffer.front(): "<<img_time_buffer.front()<<"lidar_end_time: "<<lidar_end_time<<"last_timestamp_imu: "<<last_timestamp_imu<<endl;
    
    //有图像topic，但是图像队列头的时间戳已经大于雷达队列头时间戳，说明还是处理雷达数据，和上面一样。
    if ((img_time_buffer.front()>lidar_end_time) )
    { // has img topic, but img topic timestamp larger than lidar end time, process lidar topic.
        if (last_timestamp_imu < lidar_end_time+0.02) 
        {
            // ROS_ERROR("out sync");
            return false;
        }
        double imu_time = imu_buffer.front()->header.stamp.toSec();
        m.imu.clear();
        mtx_buffer.lock();
        while ((!imu_buffer.empty() && (imu_time<lidar_end_time))) 
        {
            imu_time = imu_buffer.front()->header.stamp.toSec();
            if(imu_time > lidar_end_time) break;
            m.imu.push_back(imu_buffer.front());
            imu_buffer.pop_front();
        }
        lidar_buffer.pop_front();
        time_buffer.pop_front();
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        lidar_pushed = false;
        meas.is_lidar_end = true;
        meas.measures.push_back(m);
    }//相当于处理目前雷达时间前，每个相机间隔间的IMU信息
    else 
    {
        double img_start_time = img_time_buffer.front(); // process img topic, record timestamp
        if (last_timestamp_imu < img_start_time) 
        {
            // ROS_ERROR("out sync");
            return false;
        }
        double imu_time = imu_buffer.front()->header.stamp.toSec();
        m.imu.clear();
        m.img_offset_time = img_start_time - meas.lidar_beg_time; // record img offset time, it shoule be the Kalman update timestamp.
        m.img = img_buffer.front();
        mtx_buffer.lock();
        while ((!imu_buffer.empty() && (imu_time<img_start_time))) 
        {
            imu_time = imu_buffer.front()->header.stamp.toSec();
            if(imu_time > img_start_time) break;
            m.imu.push_back(imu_buffer.front());
            imu_buffer.pop_front();
        }
        img_buffer.pop_front();
        img_time_buffer.pop_front();
        mtx_buffer.unlock();
        sig_buffer.notify_all();
        meas.is_lidar_end = false; // has img topic in lidar scan, so flag "is_lidar_end=false" 
        //这里可以发现没有对measures之前的图像数据清空，也就是当前帧LiDAR之前的多帧图像每次同步
        //都会被放到measures里面。后面看一下处理的时候怎么做的，肯定不会重复处理
        meas.measures.push_back(m);
    }
    return true;
}

void map_incremental()
{
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
    }
#ifdef USE_ikdtree
    #ifdef USE_ikdforest
    ikdforest.Add_Points(feats_down_world->points, lidar_end_time);
    #else
    ikdtree.Add_Points(feats_down_world->points, true);
    #endif
#endif
}

// PointCloudXYZRGB::Ptr pcl_wait_pub_RGB(new PointCloudXYZRGB(500000, 1));
PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI());//在一次LIO优化后，当前LiDAR帧的点云转到世界坐标系下的点云
void publish_frame_world_rgb(const ros::Publisher &pubLaserCloudFullRes, lidar_selection::LidarSelectorPtr lidar_selector)
{
    uint size = pcl_wait_pub->points.size();
    PointCloudXYZRGB::Ptr laserCloudWorldRGB(new PointCloudXYZRGB(size, 1));
    laserCloudLidarRGB = PointCloudXYZRGB::Ptr(new PointCloudXYZRGB());
    // laserCloudLidarRGB = PointCloudXYZRGB::Ptr(new PointCloudXYZRGB(size, 1));
    if (img_en)
    {
        laserCloudWorldRGB->clear();
        for (int i = 0; i < size; i++)
        {
            PointTypeRGB pointRGB;
            pointRGB.x = pcl_wait_pub->points[i].x;
            pointRGB.y = pcl_wait_pub->points[i].y;
            pointRGB.z = pcl_wait_pub->points[i].z;
            V3D p_w(pcl_wait_pub->points[i].x, pcl_wait_pub->points[i].y, pcl_wait_pub->points[i].z);
            V3D pf(lidar_selector->new_frame_->w2f(p_w));
            if (pf[2] < 0)  continue;
            V2D pc(lidar_selector->new_frame_->w2c(p_w));
            if (lidar_selector->new_frame_->cam_->isInFrame(pc.cast<int>(), 0))
            {
                cv::Mat img_rgb = lidar_selector->img_rgb;
                V3F pixel = lidar_selector->getpixel(img_rgb, pc);
                pointRGB.r = pixel[2];
                pointRGB.g = pixel[1];
                pointRGB.b = pixel[0];
                laserCloudWorldRGB->push_back(pointRGB);
            }
        }
    }
    // if (1) // if(publish_count >= PUBFRAME_PERIOD)
    // {
    //     sensor_msgs::PointCloud2 laserCloudmsg;
    //     if (img_en)
    //     {
    //         // cout<<"RGB pointcloud size: "<<laserCloudWorldRGB->size()<<endl;
    //         pcl::toROSMsg(*laserCloudWorldRGB, laserCloudmsg);
    //     }
    //     else
    //     {
    //         pcl::toROSMsg(*pcl_wait_pub, laserCloudmsg);
    //     }
    //     laserCloudmsg.header.stamp = ros::Time::now(); //.fromSec(last_timestamp_lidar);
    //     laserCloudmsg.header.frame_id = "camera_init";
    //     pubLaserCloudFullRes.publish(laserCloudmsg);
    //     publish_count -= PUBFRAME_PERIOD;
    // }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (1)
    {
        // cout<<"save map"<<laserCloudWorldRGB->size()<<","<<laserCloudLidarRGB->size()<<endl;
        //转回到lidar坐标系下
        RGBpointWorldToBody(laserCloudWorldRGB, laserCloudLidarRGB);
        *pcl_wait_save += *laserCloudWorldRGB;
    }
}

void publish_frame_world(const ros::Publisher & pubLaserCloudFullRes)
{
    // PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);
    // int size = laserCloudFullRes->points.size();
    // if(size==0) return;
    // PointCloudXYZI::Ptr laserCloudWorld( new PointCloudXYZI(size, 1));

    // for (int i = 0; i < size; i++)
    // {
    //     RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
    //                         &laserCloudWorld->points[i]);
    // }
    uint size = pcl_wait_pub->points.size();
    // PointCloudXYZ::Ptr laserCloudWorld(new PointCloudXYZ(size, 1));
    // else
    // {
    //*pcl_wait_pub = *laserCloudWorld;
    // }
    // mtx_buffer_pointcloud.lock();
    if (1)//if(publish_count >= PUBFRAME_PERIOD)
    {
        sensor_msgs::PointCloud2 laserCloudmsg;

        pcl::toROSMsg(*pcl_wait_pub, laserCloudmsg);
        
        laserCloudmsg.header.stamp = ros::Time::now();//.fromSec(last_timestamp_lidar);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFullRes.publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
        // pcl_wait_pub->clear();
    }
    // mtx_buffer_pointcloud.unlock();
    if (pcd_save_en) *pcl_wait_save_lidar += *pcl_wait_pub;
}

void publish_visual_world_map(const ros::Publisher & pubVisualCloud)
{
    PointCloudXYZI::Ptr laserCloudFullRes(map_cur_frame_point);
    int size = laserCloudFullRes->points.size();
    if (size==0) return;
    // PointCloudXYZI::Ptr laserCloudWorld( new PointCloudXYZI(size, 1));

    // for (int i = 0; i < size; i++)
    // {
    //     RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
    //                         &laserCloudWorld->points[i]);
    // }
    // mtx_buffer_pointcloud.lock();
    PointCloudXYZI::Ptr pcl_visual_wait_pub(new PointCloudXYZI());
    *pcl_visual_wait_pub = *laserCloudFullRes;
    if (1)//if(publish_count >= PUBFRAME_PERIOD)
    {
        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*pcl_visual_wait_pub, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time::now();//.fromSec(last_timestamp_lidar);
        laserCloudmsg.header.frame_id = "camera_init";
        pubVisualCloud.publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
        // pcl_wait_pub->clear();
    }
    // mtx_buffer_pointcloud.unlock();
}

void publish_visual_world_sub_map(const ros::Publisher & pubSubVisualCloud)
{
    PointCloudXYZI::Ptr laserCloudFullRes(sub_map_cur_frame_point);//当前帧图像用的很稀疏的视觉地图点云
    int size = laserCloudFullRes->points.size();
    if (size==0) return;
    // PointCloudXYZI::Ptr laserCloudWorld( new PointCloudXYZI(size, 1));

    // for (int i = 0; i < size; i++)
    // {
    //     RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
    //                         &laserCloudWorld->points[i]);
    // }
    // mtx_buffer_pointcloud.lock();
    PointCloudXYZI::Ptr sub_pcl_visual_wait_pub(new PointCloudXYZI());
    *sub_pcl_visual_wait_pub = *laserCloudFullRes;
    if (1)//if(publish_count >= PUBFRAME_PERIOD)
    {
        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*sub_pcl_visual_wait_pub, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time::now();//.fromSec(last_timestamp_lidar);
        laserCloudmsg.header.frame_id = "camera_init";
        pubSubVisualCloud.publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
        // pcl_wait_pub->clear();
    }
    // mtx_buffer_pointcloud.unlock();
}

void publish_effect_world(const ros::Publisher & pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time::now();//.fromSec(last_timestamp_lidar);
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect.publish(laserCloudFullRes3);
}

void publish_map(const ros::Publisher & pubLaserCloudMap)
{
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = ros::Time::now();
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap.publish(laserCloudMap);
}

template<typename T>
void set_posestamp(T & out)
{
    #ifdef USE_IKFOM
    //state_ikfom stamp_state = kf.get_x();
    out.position.x = state_point.pos(0);
    out.position.y = state_point.pos(1);
    out.position.z = state_point.pos(2);
    #else
    out.position.x = state.pos_end(0);
    out.position.y = state.pos_end(1);
    out.position.z = state.pos_end(2);
    #endif
    out.orientation.x = geoQuat.x;
    out.orientation.y = geoQuat.y;
    out.orientation.z = geoQuat.z;
    out.orientation.w = geoQuat.w;
}

void publish_odometry(const ros::Publisher & pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "aft_mapped";
    odomAftMapped.header.stamp = ros::Time::now();//.ros::Time()fromSec(last_timestamp_lidar);
    set_posestamp(odomAftMapped.pose.pose);
    // odomAftMapped.twist.twist.linear.x = state_point.vel(0);
// odomAftMapped.twist.twist.linear.y = state_point.vel(1);
// odomAftMapped.twist.twist.linear.z = state_point.vel(2);
// if (Measures.imu.size()>0) {
//     Vector3d tmp(Measures.imu.back()->angular_velocity.x, Measures.imu.back()->angular_velocity.y,Measures.imu.back()->angular_velocity.z);
//     odomAftMapped.twist.twist.angular.x = tmp[0] - state_point.bg(0);
//     odomAftMapped.twist.twist.angular.y = tmp[1] - state_point.bg(1);
//     odomAftMapped.twist.twist.angular.z = tmp[2] - state_point.bg(2);
// }
    // static tf::TransformBroadcaster br;
    // tf::Transform                   transform;
    // tf::Quaternion                  q;
    // transform.setOrigin(tf::Vector3(state.pos_end(0), state.pos_end(1), state.pos_end(2)));
    // q.setW(geoQuat.w);
    // q.setX(geoQuat.x);
    // q.setY(geoQuat.y);
    // q.setZ(geoQuat.z);
    // transform.setRotation( q );
    // br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "camera_init", "aft_mapped" ) );
    pubOdomAftMapped.publish(odomAftMapped);
}

void publish_mavros(const ros::Publisher & mavros_pose_publisher)
{
    msg_body_pose.header.stamp = ros::Time::now();
    msg_body_pose.header.frame_id = "camera_odom_frame";
    set_posestamp(msg_body_pose.pose);
    mavros_pose_publisher.publish(msg_body_pose);
}

void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose.pose);
    msg_body_pose.header.stamp = ros::Time::now();
    msg_body_pose.header.frame_id = "camera_init";
    path.poses.push_back(msg_body_pose);
    pubPath.publish(path);
}

#ifdef USE_IKFOM
void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear(); 
    corr_normvect->clear(); 
    total_residual = 0.0; 

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i]; 
        PointType &point_world = feats_down_world->points[i]; 
        //double search_start = omp_get_wtime();
        /* transform to world frame */
        //pointBodyToWorld_ikfom(&point_body, &point_world, s);
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
    #ifdef USE_ikdtree
        auto &points_near = Nearest_Points[i];
    #else
        auto &points_near = pointSearchInd_surf[i];
    #endif
        
        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
        #ifdef USE_ikdtree
            #ifdef USE_ikdforest
                uint8_t search_flag = 0;                        
                search_flag = ikdforest.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis, first_lidar_time, 5);                            
            #else
                ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            #endif
        #else
            kdtreeSurfFromMap->nearestKSearch(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
        #endif

            point_selected_surf[i] = pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;

        #ifdef USE_ikdforest
            point_selected_surf[i] = point_selected_surf[i] && (search_flag == 0);
        #endif
        }

        //kdtree_search_time += omp_get_wtime() - search_start;

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f)) //(planeValid)
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }
    // cout<<"pca time test: "<<pca_time1<<" "<<pca_time2<<endl;
    
    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i] && (res_last[i] <= 2.0))
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    res_mean_last = total_residual / effct_feat_num;
    // cout << "[ mapping ]: Effective feature num: "<<effct_feat_num<<" res_mean_last "<<res_mean_last<<endl;
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    //MatrixXd H(effct_feat_num, 23);
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num); // = VectorXd::Zero(effct_feat_num);
    //VectorXd meas_vec(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be +s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C); // s.rot.conjugate() * norm_vec);
        V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
        //H.row(i) = Eigen::Matrix<double, 1, 23>::Zero();
        ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        //ekfom_data.h_x.block<1, 3>(i, 6) << VEC_FROM_ARRAY(A);
        //ekfom_data.h_x.block<1, 6>(i, 17) << VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);

        /*** Measuremnt: distance to the closest surface/corner ***/
        //meas_vec(i) = - norm_p.intensity;
        ekfom_data.h(i) = -norm_p.intensity;
    }
    //ekfom_data.h_x =H;
    solve_time += omp_get_wtime() - solve_start_;
    //return meas_vec;
}
#endif    

/**
 * Eigen格式的位姿变换
 */
Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
{
    return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
}

/**
 * Eigen格式的位姿变换
 */
Eigen::Affine3f trans2Affine3f(float transformIn[])
{
    return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
}

/**
 * 位姿格式变换
 */
PointTypePose trans2PointTypePose(float transformIn[])
{
    PointTypePose thisPose6D;
    thisPose6D.x = transformIn[3];
    thisPose6D.y = transformIn[4];
    thisPose6D.z = transformIn[5];
    thisPose6D.roll = transformIn[0];
    thisPose6D.pitch = transformIn[1];
    thisPose6D.yaw = transformIn[2];
    return thisPose6D;
}

/**
 * 位姿格式变换
 */
gtsam::Pose3 trans2gtsamPose(float transformIn[])
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]),
                        gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
}

/**
 * 位姿格式变换
 */
gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
{
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                        gtsam::Point3(double(thisPoint.x), double(thisPoint.y), double(thisPoint.z)));
}

/**
 * 点到坐标系原点距离
 */
float pointDistance(PointType p)
{
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

/**
 * 两点之间距离
 */
float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}


/**
 * 计算当前帧与前一帧位姿变换，如果变化太小，不设为关键帧，反之设为关键帧
 */
bool saveFrame()
{
    if (cloudKeyPoses3D->points.empty())
        return true;

    // 前一帧位姿
    Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
    // 当前帧位姿
    Eigen::Affine3f transFinal = trans2Affine3f(transformTobeMapped);
    // Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
    //                                                     transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
                    
    // 位姿变换增量
    Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
    float x, y, z, roll, pitch, yaw;
    pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw); //  获取上一帧 相对 当前帧的 位姿

    // 旋转和平移量都较小，当前帧不设为关键帧
    if (abs(roll) < surroundingkeyframeAddingAngleThreshold &&
        abs(pitch) < surroundingkeyframeAddingAngleThreshold &&
        abs(yaw) < surroundingkeyframeAddingAngleThreshold &&
        sqrt(x * x + y * y + z * z) < surroundingkeyframeAddingDistThreshold)
        return false;
    return true;
}

/**
 * 添加激光里程计因子
 */
void addOdomFactor()
{
    if (cloudKeyPoses3D->points.empty())
    {
        // 第一帧初始化先验因子
        gtsam::noiseModel::Diagonal::shared_ptr priorNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) <<1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12).finished()); // rad*rad, meter*meter   // indoor 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12    //  1e-2, 1e-2, M_PI*M_PI, 1e8, 1e8, 1e8
        gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));
        // 变量节点设置初始值
        initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
    }
    else
    {
        // 添加激光里程计因子
        gtsam::noiseModel::Diagonal::shared_ptr odometryNoise = gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
        gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back()); /// 根据上一帧的位姿
        gtsam::Pose3 poseTo = trans2gtsamPose(transformTobeMapped);                   // 当前帧的位姿
        // 参数：前一帧id，当前帧id，前一帧与当前帧的位姿变换（作为观测值），噪声协方差
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(cloudKeyPoses3D->size() - 1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));
        // 变量节点设置初始值
        initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
    }
}

/**
 * 添加闭环因子
 */
void addLoopFactor()
{
    if (loopIndexQueue.empty())
        return;

    // 闭环队列
    for (int i = 0; i < (int)loopIndexQueue.size(); ++i)
    {
        // 闭环边对应两帧的索引
        int indexFrom = loopIndexQueue[i].first; //   cur
        int indexTo = loopIndexQueue[i].second;  //    pre
        // 闭环边的位姿变换
        gtsam::Pose3 poseBetween = loopPoseQueue[i];
        gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
    }

    loopIndexQueue.clear();
    loopPoseQueue.clear();
    loopNoiseQueue.clear();
    aLoopIsClosed = true;
}

/**
 * 添加GPS因子
*/
void addGPSFactor()
{
    if (gnss_buffer.empty())
        return;
    // 如果没有关键帧，或者首尾关键帧距离小于5m，不添加gps因子
    if (cloudKeyPoses3D->points.empty())
        return;
    else
    {
        if (pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) < 5.0)
            return;
    }
    // 位姿协方差很小，没必要加入GPS数据进行校正
    if (poseCovariance(3,3) < poseCovThreshold && poseCovariance(4,4) < poseCovThreshold)
        return;
    static PointType lastGPSPoint;      // 最新的gps数据
    while (!gnss_buffer.empty())
    {
        // 删除当前帧0.2s之前的里程计
        if (gnss_buffer.front().header.stamp.toSec() < lidar_end_time - 0.05)
        {
            gnss_buffer.pop_front();
        }
        // 超过当前帧0.2s之后，退出
        else if (gnss_buffer.front().header.stamp.toSec() > lidar_end_time + 0.05)
        {
            break;
        }
        else
        {
            nav_msgs::Odometry thisGPS = gnss_buffer.front();
            gnss_buffer.pop_front();
            // GPS噪声协方差太大，不能用
            float noise_x = thisGPS.pose.covariance[0];         //  x 方向的协方差
            float noise_y = thisGPS.pose.covariance[7];
            float noise_z = thisGPS.pose.covariance[14];      //   z(高层)方向的协方差
            if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold)
                continue;
            // GPS里程计位置
            float gps_x = thisGPS.pose.pose.position.x;
            float gps_y = thisGPS.pose.pose.position.y;
            float gps_z = thisGPS.pose.pose.position.z;
            if (!useGpsElevation)           //  是否使用gps的高度
            {
                gps_z = transformTobeMapped[5];
                noise_z = 0.01;
            }

            // (0,0,0)无效数据
            if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
                continue;
            // 每隔5m添加一个GPS里程计
            PointType curGPSPoint;
            curGPSPoint.x = gps_x;
            curGPSPoint.y = gps_y;
            curGPSPoint.z = gps_z;
            if (pointDistance(curGPSPoint, lastGPSPoint) < 5.0)
                continue;
            else
                lastGPSPoint = curGPSPoint;
            // 添加GPS因子
            gtsam::Vector Vector3(3);
            Vector3 << max(noise_x, 1.0f), max(noise_y, 1.0f), max(noise_z, 1.0f);
            gtsam::noiseModel::Diagonal::shared_ptr gps_noise = gtsam::noiseModel::Diagonal::Variances(Vector3);
            gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(), gtsam::Point3(gps_x, gps_y, gps_z), gps_noise);//  添加GPS因子
            gtSAMgraph.add(gps_factor);
            aLoopIsClosed = true;
            ROS_INFO("GPS Factor Added");
            break;
        }
    }
}



// add gtsam
/// @brief 将更新的pose赋值到 transformTobeMapped
/// @param cur_state 当前帧状态
void getCurPose(StatesGroup cur_state)
{
    //  欧拉角是没有群的性质，所以从SO3还是一般的rotation matrix 转换过来的结果一样
    Eigen::Vector3d eulerAngle = cur_state.rot_end.matrix().eulerAngles(2,1,0);        //  yaw pitch roll  单位：弧度
    // V3D eulerAngle  =  SO3ToEuler(cur_state.rot)/57.3 ;     //   fastlio 自带  roll pitch yaw  单位: 度，旋转顺序 zyx

    // transformTobeMapped[0] = eulerAngle(0);                //  roll     使用 SO3ToEuler 方法时，顺序是 rpy
    // transformTobeMapped[1] = eulerAngle(1);                //  pitch
    // transformTobeMapped[2] = eulerAngle(2);                //  yaw
    
    transformTobeMapped[0] = eulerAngle(2);                //  roll  使用 eulerAngles(2,1,0) 方法时，顺序是 ypr
    transformTobeMapped[1] = eulerAngle(1);                //  pitch
    transformTobeMapped[2] = eulerAngle(0);                //  yaw
    transformTobeMapped[3] = cur_state.pos_end(0);          //  x
    transformTobeMapped[4] = cur_state.pos_end(1);          //   y
    transformTobeMapped[5] = cur_state.pos_end(2);          // z
}
//  eulerAngle 2 Quaterniond
Eigen::Quaterniond  EulerToQuat(float roll_, float pitch_, float yaw_)
{
    Eigen::Quaterniond q ;            //   四元数 q 和 -q 是相等的
    Eigen::AngleAxisd roll(double(roll_), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitch(double(pitch_), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yaw(double(yaw_), Eigen::Vector3d::UnitZ());
    q = yaw * pitch * roll ;
    q.normalize();
    return q ;
}
/// @brief 主要核心代码，保存关键帧，添加因子，执行优化
void saveKeyFramesAndFactor()
{
    //  计算当前帧与前一帧位姿变换，如果变化太小，不设为关键帧，反之设为关键帧
    if (saveFrame() == false)
        return;
    // 激光里程计因子(from fast-lio),  输入的是frame_relative pose  帧间位姿(body 系下)
    addOdomFactor();
    // GPS因子 (UTM -> WGS84)
    addGPSFactor();
    // 闭环因子 (rs-loop-detect)  基于欧氏距离的检测
    addLoopFactor();
    // 执行优化
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();
    if (aLoopIsClosed == true) // 有回环因子，多update几次
    {
        isam->update();
        isam->update();
        isam->update();
        isam->update();
        isam->update();
    }
    // update之后要清空一下保存的因子图，注：历史数据不会清掉，ISAM保存起来了
    gtSAMgraph.resize(0);
    initialEstimate.clear();

    PointType thisPose3D;
    PointTypePose thisPose6D;
    gtsam::Pose3 latestEstimate;

    // 优化结果
    isamCurrentEstimate = isam->calculateBestEstimate();
    // 当前帧位姿结果
    latestEstimate = isamCurrentEstimate.at<gtsam::Pose3>(isamCurrentEstimate.size() - 1);

    // cloudKeyPoses3D加入当前帧位置
    thisPose3D.x = latestEstimate.translation().x();
    thisPose3D.y = latestEstimate.translation().y();
    thisPose3D.z = latestEstimate.translation().z();
    // 索引
    thisPose3D.intensity = cloudKeyPoses3D->size(); //  使用intensity作为该帧点云的index
    cloudKeyPoses3D->push_back(thisPose3D);         //  新关键帧帧放入队列中

    // cloudKeyPoses6D加入当前帧位姿
    thisPose6D.x = thisPose3D.x;
    thisPose6D.y = thisPose3D.y;
    thisPose6D.z = thisPose3D.z;
    thisPose6D.intensity = thisPose3D.intensity;
    thisPose6D.roll = latestEstimate.rotation().roll();
    thisPose6D.pitch = latestEstimate.rotation().pitch();
    thisPose6D.yaw = latestEstimate.rotation().yaw();
    thisPose6D.time = lidar_end_time;
    cloudKeyPoses6D->push_back(thisPose6D);
    // 位姿协方差
    // poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size() - 1);

    // FAST_LIVO 更新
    state.pos_end << thisPose6D.x, thisPose6D.y, thisPose6D.z;
    state.rot_end = EulerToQuat(thisPose6D.roll, thisPose6D.pitch, thisPose6D.yaw);
    // state.cov = poseCovariance;
    // ESKF状态和方差  更新
    // state_ikfom state_updated = kf.get_x(); //  获取cur_pose (还没修正)
    // Eigen::Vector3d pos(latestEstimate.translation().x(), latestEstimate.translation().y(), latestEstimate.translation().z());
    // Eigen::Quaterniond q = EulerToQuat(latestEstimate.rotation().roll(), latestEstimate.rotation().pitch(), latestEstimate.rotation().yaw());

    // //  更新状态量
    
    // state_updated.pos = pos;
    // state_updated.rot =  q;
    // state_point = state_updated; // 对state_point进行更新，state_point可视化用到
    // // if(aLoopIsClosed == true )
    // kf.change_x(state_updated);  //  对cur_pose 进行isam2优化后的修正

    // // TODO:  P的修正有待考察，按照yanliangwang的做法，修改了p，会跑飞
    // // esekfom::esekf<state_ikfom, 12, input_ikfom>::cov P_updated = kf.get_P(); // 获取当前的状态估计的协方差矩阵
    // // P_updated.setIdentity();
    // // P_updated(6, 6) = P_updated(7, 7) = P_updated(8, 8) = 0.00001;
    // // P_updated(9, 9) = P_updated(10, 10) = P_updated(11, 11) = 0.00001;
    // // P_updated(15, 15) = P_updated(16, 16) = P_updated(17, 17) = 0.0001;
    // // P_updated(18, 18) = P_updated(19, 19) = P_updated(20, 20) = 0.001;
    // // P_updated(21, 21) = P_updated(22, 22) = 0.00001;
    // // kf.change_P(P_updated);

    // // 当前帧激光角点、平面点，降采样集合
    // // pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
    // // pcl::copyPointCloud(*feats_undistort,  *thisCornerKeyFrame);
    pcl::copyPointCloud(*feats_undistort, *thisSurfKeyFrame); // 存储关键帧,没有降采样的点云

    // surfCloudKeyFrames.push_back(thisSurfKeyFrame);
    surfCloudKeyFramesRGB.push_back(laserCloudLidarRGB); // todo：这个转到局部坐标。然后优化的是path位置
    cout<< "------------- rgb keyframe size: "<< surfCloudKeyFramesRGB.size()<<endl;
    // updatePath(thisPose6D); //  可视化update后的path
}

/**
 * 更新里程计轨迹
 */
void updatePath(const PointTypePose &pose_in)
{
    string odometryFrame = "camera_init";
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);

    pose_stamped.header.frame_id = odometryFrame;
    pose_stamped.pose.position.x =  pose_in.x;
    pose_stamped.pose.position.y = pose_in.y;
    pose_stamped.pose.position.z =  pose_in.z;
    tf::Quaternion q = tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.orientation.w = q.w();

    globalPath.poses.push_back(pose_stamped);
}

void recontructMap(){
    assert(surfCloudKeyFramesRGB.size() == cloudKeyPoses6D->points.size());
    reconstructed_map.reset(new PointCloudXYZRGB());
     // 创建一个点云对象来存储重建的地图
    for (int i = 0; i < surfCloudKeyFramesRGB.size(); ++i){
        *reconstructed_map += *transformPointCloud(surfCloudKeyFramesRGB[i], &cloudKeyPoses6D->points[i]);
    }
    // 发布地图
    sensor_msgs::PointCloud2 map_msg;
    pcl::toROSMsg(*reconstructed_map, map_msg);
    map_msg.header.stamp = ros::Time().now();
    map_msg.header.frame_id = "camera_init";
    pubHistoryRGBMap.publish(map_msg);
}

/**
 * 更新因子图中所有变量节点的位姿，也就是所有历史关键帧的位姿，更新里程计轨迹
 */
void correctPoses()
{
    if (cloudKeyPoses3D->points.empty())
        return;

    if (aLoopIsClosed == true)
    {
        // 清空里程计轨迹
        globalPath.poses.clear();
        // 更新因子图中所有变量节点的位姿，也就是所有历史关键帧的位姿
        int numPoses = isamCurrentEstimate.size();
        for (int i = 0; i < numPoses; ++i)
        {
            cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().x();
            cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().y();
            cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<gtsam::Pose3>(i).translation().z();

            cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
            cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
            cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
            cloudKeyPoses6D->points[i].roll = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().roll();
            cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().pitch();
            cloudKeyPoses6D->points[i].yaw = isamCurrentEstimate.at<gtsam::Pose3>(i).rotation().yaw();

            // 更新里程计轨迹
            updatePath(cloudKeyPoses6D->points[i]);
        }
        recontruct_map_index = 0;
        // 清空局部map， reconstruct  ikdtree submap
        recontructMap();
        ROS_INFO("ISMA2 Update");
        aLoopIsClosed = false;
    }else{
        assert(surfCloudKeyFramesRGB.size() == cloudKeyPoses6D->points.size());
        int end_size = surfCloudKeyFramesRGB.size()-1;
        // 创建一个点云对象来存储重建的地图
        *reconstructed_map += *transformPointCloud(surfCloudKeyFramesRGB[end_size], &cloudKeyPoses6D->points[end_size]);
        // 发布地图
        sensor_msgs::PointCloud2 map_msg;
        pcl::toROSMsg(*reconstructed_map, map_msg);
        map_msg.header.stamp = ros::Time().now();
        map_msg.header.frame_id = "camera_init";
        pubHistoryRGBMap.publish(map_msg);
        cout<< "not closed loop, pub map"<<surfCloudKeyFramesRGB.size()<<endl;
        }
}

//回环检测三大要素
// 1.设置最小时间差，太近没必要
// 2.控制回环的频率，避免频繁检测，每检测一次，就做一次等待
// 3.根据当前最小距离重新计算等待时间
bool detectLoopClosureDistance(int *latestID, int *closestID)
{
    // 当前关键帧帧
    int loopKeyCur = copy_cloudKeyPoses3D->size() - 1; //  当前关键帧索引
    int loopKeyPre = -1;

    // 当前帧已经添加过闭环对应关系，不再继续添加
    auto it = loopIndexContainer.find(loopKeyCur);
    if (it != loopIndexContainer.end())
        return false;
    // 在历史关键帧中查找与当前关键帧距离最近的关键帧集合
    std::vector<int> pointSearchIndLoop;                        //  候选关键帧索引
    std::vector<float> pointSearchSqDisLoop;                    //  候选关键帧距离
    kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D); //  历史帧构建kdtree
    kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
    // 在候选关键帧集合中，找到与当前帧时间相隔较远的帧，设为候选匹配帧
    for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
    {
        int id = pointSearchIndLoop[i];
        if (abs(copy_cloudKeyPoses6D->points[id].time - lidar_end_time) > historyKeyframeSearchTimeDiff)
        {
            loopKeyPre = id;
            break;
        }
    }
    if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
        return false;
    *latestID = loopKeyCur;
    *closestID = loopKeyPre;

    ROS_INFO("Find loop clousre frame ");
    return true;
}



/**
 * 提取key索引的关键帧前后相邻若干帧的关键帧特征点集合，降采样
 */
void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr &nearKeyframes, const int &key, const int &searchNum)
{
    // 提取key索引的关键帧前后相邻若干帧的关键帧特征点集合
    nearKeyframes->clear();
    int cloudSize = copy_cloudKeyPoses6D->size();
    auto surfcloud_keyframes_size = surfCloudKeyFramesRGB.size() ;
    for (int i = -searchNum; i <= searchNum; ++i)
    {
        int keyNear = key + i;
        if (keyNear < 0 || keyNear >= cloudSize)
            continue;

        if (keyNear < 0 || keyNear >= surfcloud_keyframes_size)
            continue;

        // 注意：cloudKeyPoses6D 存储的是 T_w_b , 而点云是lidar系下的，构建icp的submap时，需要通过外参数T_b_lidar 转换 , 参考pointBodyToWorld 的转换
        *nearKeyframes += *transformPointCloudRGB_None(surfCloudKeyFramesRGB[keyNear], &copy_cloudKeyPoses6D->points[keyNear]); //  fast-lio 没有进行特征提取，默认点云就是surf
    }

    if (nearKeyframes->empty())
        return;

    // 降采样
    pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    downSizeFilterICP.setInputCloud(nearKeyframes);
    downSizeFilterICP.filter(*cloud_temp);
    *nearKeyframes = *cloud_temp;
}

// /**
//  * 发布thisCloud，返回thisCloud对应msg格式
//  */
// sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
// {
//     sensor_msgs::PointCloud2 tempCloud;
//     pcl::toROSMsg(*thisCloud, tempCloud);
//     tempCloud.header.stamp = thisStamp;
//     tempCloud.header.frame_id = thisFrame;
//     if (thisPub->getNumSubscribers() != 0)
//         thisPub->publish(tempCloud);
//     return tempCloud;
// }


void performLoopClosure()
{
    ros::Time timeLaserInfoStamp = ros::Time().fromSec(lidar_end_time); //  时间戳
    string odometryFrame = "camera_init";

    if (cloudKeyPoses3D->points.empty() == true)
    {
        return;
    }

    mtx.lock();
    *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
    *copy_cloudKeyPoses6D = *cloudKeyPoses6D;
    mtx.unlock();

    // 当前关键帧索引，候选闭环匹配帧索引
    int loopKeyCur;
    int loopKeyPre;
    // 在历史关键帧中查找与当前关键帧距离最近的关键帧集合，选择时间相隔较远的一帧作为候选闭环帧
    if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false)
    {
        return;
    }

    // 提取
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>()); //  cue keyframe
    pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>()); //   history keyframe submap
    {
        // 提取当前关键帧特征点集合，降采样
        loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0); //  将cur keyframe 转换到world系下
        // 提取闭环匹配关键帧前后相邻若干帧的关键帧特征点集合，降采样
        loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum); //  选取historyKeyframeSearchNum个keyframe拼成submap
        // 如果特征点较少，返回
        // if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
        //     return;
        // // 发布闭环匹配关键帧局部map
        // if (pubHistoryKeyFrames.getNumSubscribers() != 0)
        //     publishCloud(&pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
    }

    // ICP Settings
    pcl::IterativeClosestPoint<PointType, PointType> icp;
    icp.setMaxCorrespondenceDistance(150); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);

    // scan-to-map，调用icp匹配
    icp.setInputSource(cureKeyframeCloud);
    icp.setInputTarget(prevKeyframeCloud);
    pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
    icp.align(*unused_result);

    // 未收敛，或者匹配不够好
    if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
        return;

    std::cout << "icp  success  " << std::endl;

    // // 发布当前关键帧经过闭环优化后的位姿变换之后的特征点云
    // if (pubIcpKeyFrames.getNumSubscribers() != 0)
    // {
    //     pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
    //     pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
    //     publishCloud(&pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
    // }

    // 闭环优化得到的当前关键帧与闭环关键帧之间的位姿变换
    float x, y, z, roll, pitch, yaw;
    Eigen::Affine3f correctionLidarFrame;
    correctionLidarFrame = icp.getFinalTransformation();

    // 闭环优化前当前帧位姿
    Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
    // 闭环优化后当前帧位姿
    Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;
    pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw); //  获取上一帧 相对 当前帧的 位姿
    gtsam::Pose3 poseFrom = gtsam::Pose3(gtsam::Rot3::RzRyRx(roll, pitch, yaw), gtsam::Point3(x, y, z));
    // 闭环匹配帧的位姿
    gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
    gtsam::Vector Vector6(6);
    float noiseScore = icp.getFitnessScore() ; //  loop_clousre  noise from icp
    Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
    gtsam::noiseModel::Diagonal::shared_ptr constraintNoise = gtsam::noiseModel::Diagonal::Variances(Vector6);
    std::cout << "loopNoiseQueue   =   " << noiseScore << std::endl;

    // 添加闭环因子需要的数据
    mtx.lock();
    loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
    loopPoseQueue.push_back(poseFrom.between(poseTo));//  闭环帧之间的位姿变换
    loopNoiseQueue.push_back(constraintNoise);
    mtx.unlock();

    loopIndexContainer[loopKeyCur] = loopKeyPre; //   使用hash map 存储回环对
}

/**
 * rviz展示闭环边
 */
void visualizeLoopClosure()
{
    ros::Time timeLaserInfoStamp = ros::Time().fromSec(lidar_end_time); //  时间戳
    string odometryFrame = "camera_init";

    if (loopIndexContainer.empty())
        return;

    visualization_msgs::MarkerArray markerArray;
    // 闭环顶点
    visualization_msgs::Marker markerNode;
    markerNode.header.frame_id = odometryFrame;
    markerNode.header.stamp = timeLaserInfoStamp;
    markerNode.action = visualization_msgs::Marker::ADD;
    markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
    markerNode.ns = "loop_nodes";
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 0.3;
    markerNode.scale.y = 0.3;
    markerNode.scale.z = 0.3;
    markerNode.color.r = 0;
    markerNode.color.g = 0.8;
    markerNode.color.b = 1;
    markerNode.color.a = 1;
    // 闭环边
    visualization_msgs::Marker markerEdge;
    markerEdge.header.frame_id = odometryFrame;
    markerEdge.header.stamp = timeLaserInfoStamp;
    markerEdge.action = visualization_msgs::Marker::ADD;
    markerEdge.type = visualization_msgs::Marker::LINE_LIST;
    markerEdge.ns = "loop_edges";
    markerEdge.id = 1;
    markerEdge.pose.orientation.w = 1;
    markerEdge.scale.x = 0.1;
    markerEdge.color.r = 0.9;
    markerEdge.color.g = 0.9;
    markerEdge.color.b = 0;
    markerEdge.color.a = 1;

    // 遍历闭环
    for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it)
    {
        int key_cur = it->first;
        int key_pre = it->second;
        geometry_msgs::Point p;
        p.x = copy_cloudKeyPoses6D->points[key_cur].x;
        p.y = copy_cloudKeyPoses6D->points[key_cur].y;
        p.z = copy_cloudKeyPoses6D->points[key_cur].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
        p.x = copy_cloudKeyPoses6D->points[key_pre].x;
        p.y = copy_cloudKeyPoses6D->points[key_pre].y;
        p.z = copy_cloudKeyPoses6D->points[key_pre].z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
    }

    markerArray.markers.push_back(markerNode);
    markerArray.markers.push_back(markerEdge);
    pubLoopConstraintEdge.publish(markerArray);
}


//回环检测线程
void loopClosureThread()
{
    if (loopClosureEnableFlag == false)
    {
        std::cout << "loopClosureEnableFlag   ==  false " << endl;
        return;
    }

    ros::Rate rate(loopClosureFrequency); //   回环频率
    while (ros::ok() && startFlag)
    {
        rate.sleep();
        performLoopClosure();   //  回环检测
        visualizeLoopClosure(); // rviz展示闭环边
    }
}

void readParameters(ros::NodeHandle &nh)
{
    nh.param<int>("dense_map_enable",dense_map_en,1);
    nh.param<int>("img_enable",img_en,1);
    nh.param<int>("lidar_enable",lidar_en,1);
    nh.param<int>("debug", debug, 0);
    nh.param<int>("max_iteration",NUM_MAX_ITERATIONS,4);
    nh.param<bool>("ncc_en",ncc_en,false);
    nh.param<int>("min_img_count",MIN_IMG_COUNT,1000);    
    nh.param<double>("cam_fx",cam_fx,453.483063);// 相机内参
    nh.param<double>("cam_fy",cam_fy,453.254913);
    nh.param<double>("cam_cx",cam_cx,318.908851);
    nh.param<double>("cam_cy",cam_cy,234.238189);
    nh.param<double>("laser_point_cov",LASER_POINT_COV,0.001);
    nh.param<double>("img_point_cov",IMG_POINT_COV,10);
    nh.param<string>("map_file_path",map_file_path,"");
    nh.param<string>("common/lid_topic",lid_topic,"/livox/lidar");
    nh.param<string>("common/imu_topic", imu_topic,"/livox/imu");
    nh.param<string>("camera/img_topic", img_topic,"/usb_cam/image_raw");
    nh.param<double>("filter_size_corner",filter_size_corner_min,0.5);
    nh.param<double>("filter_size_surf",filter_size_surf_min,0.5);
    nh.param<double>("filter_size_map",filter_size_map_min,0.5);
    nh.param<double>("cube_side_length",cube_len,200);
    nh.param<double>("mapping/fov_degree",fov_deg,180);// FOV
    nh.param<double>("mapping/gyr_cov_scale",gyr_cov_scale,1.0);
    nh.param<double>("mapping/acc_cov_scale",acc_cov_scale,1.0);
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
    nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);
    nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, 0);
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>()); // 雷达imu外参
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());
    nh.param<vector<double>>("camera/Pcl", cameraextrinT, vector<double>()); // 相机雷达外参
    nh.param<vector<double>>("camera/Rcl", cameraextrinR, vector<double>());
    nh.param<int>("grid_size", grid_size, 40);    // 每个网格的像素宽高，配置为 40
    nh.param<int>("patch_size", patch_size, 4);   // 选择的 patch 的宽高，配置为 8
    nh.param<double>("outlier_threshold",outlier_threshold,100);
    nh.param<double>("ncc_thre", ncc_thre, 100);

    /**
    * @brief add by crz
    *
    */
    nh.param<bool>("onlyUpdateBias", onlyUpdateBias, false);
    nh.param<bool>("onlyUpdateBg", onlyUpdateBg, false);
    nh.param<bool>("useKalmanSmooth", useKalmanSmooth, true);
    nh.param<bool>("zero_point_one", zero_point_one, false);
    nh.param<double>("img_time_offset", img_time_offset, 0.0);
    nh.param<bool>("useVio", useVio, true);
    nh.param<int>("eigenValueThreshold", eigenValueThreshold, 0);

    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);


    // add gtsam

    // gnss
    nh.param<string>("common/gnss_topic", gnss_topic,"/gps/fix");
    nh.param<vector<double>>("mapping/extrinR_Gnss2Lidar", extrinR_Gnss2Lidar, vector<double>());
    nh.param<vector<double>>("mapping/extrinT_Gnss2Lidar", extrinT_Gnss2Lidar, vector<double>());
    nh.param<bool>("useImuHeadingInitialization", useImuHeadingInitialization, false);
    nh.param<bool>("useGpsElevation", useGpsElevation, false);
    nh.param<float>("gpsCovThreshold", gpsCovThreshold, 2.0);
    nh.param<float>("poseCovThreshold", poseCovThreshold, 25.0);

    // save keyframes
    nh.param<float>("surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 0.1);
    nh.param<float>("surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);//稠密点云，这里我们空间内存比较小，就只能设置0.2
    nh.param<float>("surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
    nh.param<float>("surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);
    nh.param<float>("mappingSurfLeafSize", mappingSurfLeafSize, 0.2);

    // loop clousre
    nh.param<bool>("loopClosureEnableFlag", loopClosureEnableFlag, false);
    nh.param<float>("loopClosureFrequency", loopClosureFrequency, 1.0);
    nh.param<int>("surroundingKeyframeSize", surroundingKeyframeSize, 50);
    nh.param<float>("historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
    nh.param<float>("historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
    nh.param<int>("historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
    nh.param<float>("historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);

    //ISAM2参数
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new gtsam::ISAM2(parameters);

}

/**
 * 初始化
 */
void allocateMemory()
{
    cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
    copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
    copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

    kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

    laserCloudOri.reset(new pcl::PointCloud<PointType>());

    for (int i = 0; i < 6; ++i)
    {
        transformTobeMapped[i] = 0;
    }
}

int main(int argc, char** argv)
{
    allocateMemory();
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    readParameters(nh);
    cout<<"debug:"<<debug<<" MIN_IMG_COUNT: "<<MIN_IMG_COUNT<<endl;
    pcl_wait_pub->clear();//等待发布的点云
    // 订阅LiDAR、IMU、img消息
    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? \
        nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : \
        nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
    ros::Subscriber sub_img = nh.subscribe(img_topic, 200000, img_cbk);
    image_transport::Publisher img_pub = it.advertise("/rgb_img", 1);
    ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered", 100);
    ros::Publisher pubVisualCloud = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_visual_map", 100);
    ros::Publisher pubSubVisualCloud = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_visual_sub_map", 100);
    ros::Publisher pubLaserCloudEffect  = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_effected", 100);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>
            ("/Laser_map", 100);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> 
            ("/aft_mapped_to_init", 10);
    ros::Publisher pubPath          = nh.advertise<nav_msgs::Path> 
            ("/path", 10);

    pubGnssPath = nh.advertise<nav_msgs::Path>("/gnss_path", 100000);
            

    // loop clousre
    // 发布闭环匹配关键帧局部map
    // pubHistoryKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("fast_lio_sam/mapping/icp_loop_closure_history_cloud", 1);
    // // 发布当前关键帧经过闭环优化后的位姿变换之后的特征点云
    // pubIcpKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("fast_lio_sam/mapping/icp_loop_closure_corrected_cloud", 1);
    // 发布闭环边，rviz中表现为闭环帧之间的连线
    pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>("/fast_lio_sam/mapping/loop_closure_constraints", 1);
    // 发布整体的RGB点云map
    pubHistoryRGBMap = nh.advertise<sensor_msgs::PointCloud2>("/fast_lio_sam/mapping/history_cloud_map", 1);
    
    /* add by crz*/
    ros::Publisher pubLaserCloudFrame =
      nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100);

    // gnss
    ros::Subscriber sub_gnss = nh.subscribe(gnss_topic, 200000, gnss_cbk);

    // 回环检测线程
    std::thread loopthread(&loopClosureThread);

#ifdef DEPLOY
    ros::Publisher mavros_pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10);
#endif
    
    path.header.stamp    = ros::Time::now();
    path.header.frame_id ="camera_init";

    /*** variables definition ***/
    #ifndef USE_IKFOM
    VD(DIM_STATE) solution;// 18*1
    MD(DIM_STATE, DIM_STATE) G, H_T_H, I_STATE,G_k;// 18*18
    V3D rot_add, t_add;
    StatesGroup state_propagat;
    StatesGroup state_last_lidar;
    PointType pointOri, pointSel, coeff;
    #endif
    //PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    
    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    // 降采样系数
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
    #ifdef USE_ikdforest
        ikdforest.Set_balance_criterion_param(0.6);
        ikdforest.Set_delete_criterion_param(0.5);
        ikdforest.Set_environment(laserCloudDepth,laserCloudWidth,laserCloudHeight,cube_len);
        ikdforest.Set_downsample_param(filter_size_map_min);    
    #endif
    // IMU处理的函数
    shared_ptr<ImuProcess> p_imu(new ImuProcess());
    // p_imu->set_extrinsic(V3D(0.04165, 0.02326, -0.0284));   //avia
    // p_imu->set_extrinsic(V3D(0.05512, 0.02226, -0.0297));   //horizon
    V3D extT;//Lidar to IMU 外参
    M3D extR;//Lidar to IMU 外参
    extT<<VEC_FROM_ARRAY(extrinT);
    extR<<MAT_FROM_ARRAY(extrinR);




    //! 重要：处理VIO部分的类
    lidar_selection::LidarSelectorPtr lidar_selector(
        new lidar_selection::LidarSelector(grid_size, new SparseMap));
    //; 从命名空间中读取参数，生成一个虚拟的相机类
    if (!vk::camera_loader::loadFromRosNs("laserMapping", lidar_selector->cam))
        throw std::runtime_error("Camera model not correctly specified.");

    // TODO：初始化lidar_selection的一些参数
    //; 这个没用到
    lidar_selector->MIN_IMG_COUNT = MIN_IMG_COUNT;   // 1000
    lidar_selector->debug = debug;  // 0 是否显示debug信息
    lidar_selector->patch_size = patch_size;   // 8
    lidar_selector->outlier_threshold = outlier_threshold;  // 300
    lidar_selector->ncc_thre = ncc_thre;   // 0 ncc 的阈值
    //; 进去内部看，应该是 T_camera_lidar?
    lidar_selector->sparse_map->set_camera2lidar(cameraextrinR, cameraextrinT); // hr: from camera to lidar
    //; 传入的是 T_imu_lidar，内部赋值做了转换，变成 T_lidar_imu
    lidar_selector->set_extrinsic(extT, extR);  // hr: TODO:return T from imu to lidar
    //; 绑定状态变量，这样会在VIO里面直接更改LIO的结果
    lidar_selector->state = &state;  
    //; IMU预测的状态，这和IEKF有关，因为IEKF会一直计算当前状态和预测状态之间的差值
    lidar_selector->state_propagat = &state_propagat;  
    lidar_selector->NUM_MAX_ITERATIONS = NUM_MAX_ITERATIONS; // 4，IEKF迭代的最大阈值
    //; 和优化有关，视觉点的协方差
    lidar_selector->img_point_cov = IMG_POINT_COV; // 100
    //; 给成员变量中的相机内参赋值
    lidar_selector->fx = cam_fx;
    lidar_selector->fy = cam_fy;
    lidar_selector->cx = cam_cx;
    lidar_selector->cy = cam_cy;
    //; NCC是归一化相关性，是相比使用patch对齐的更复杂的差异度量方式，见十四讲P230
    lidar_selector->ncc_en = ncc_en; // 0
    /* add by crz */
    lidar_selector->eigenValueThreshold = eigenValueThreshold;
    lidar_selector->init();
    //------------------------------- vio 部分变量初始化完毕 --------------------------
    
   //; 对IMU类设置噪声等消息
    p_imu->set_extrinsic(extT, extR); // TODO:lidar to imu??
    p_imu->set_gyr_cov_scale(V3D(gyr_cov_scale, gyr_cov_scale, gyr_cov_scale));
    p_imu->set_acc_cov_scale(V3D(acc_cov_scale, acc_cov_scale, acc_cov_scale));
    //    p_imu->set_gyr_bias_cov(V3D(0.00001, 0.00001, 0.00001));
    //    p_imu->set_acc_bias_cov(V3D(0.00001, 0.00001, 0.00001));
    p_imu->set_gyr_bias_cov(V3D(0.00003, 0.00003, 0.00003));
    p_imu->set_acc_bias_cov(V3D(0.01, 0.01, 0.01));
    p_imu->set_state_last_lidar(state_last_lidar);
    p_imu->set_G_k(G_k);

    //设置gnss外参数
    Gnss_T_wrt_Lidar<<VEC_FROM_ARRAY(extrinT_Gnss2Lidar);
    Gnss_R_wrt_Lidar<<MAT_FROM_ARRAY(extrinR_Gnss2Lidar);


    #ifndef USE_IKFOM
    G.setZero();// 18 * 18 的矩阵，三个都是
    H_T_H.setZero();
    I_STATE.setIdentity();
    G_k.setZero();
    #endif

    #ifdef USE_IKFOM
    double epsi[23] = {0.001};
    fill(epsi, epsi+23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);
    #endif
    /*** debug record ***/
    FILE *fp;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(),"w");

    ofstream fout_pre, fout_out, fout_dbg;
    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
    fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
    // if (fout_pre && fout_out)
    //     cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
    // else
    //     cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;

    #ifdef USE_ikdforest
        ikdforest.Set_balance_criterion_param(0.6);
        ikdforest.Set_delete_criterion_param(0.5);
    #endif
//------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();
    while (status)
    {
        if (flg_exit) break;
        ros::spinOnce();
        if(!sync_packages(LidarMeasures))// Step 1: 同步LiDAR、IMU、Image信息，如果没有同步成功，则一直在这里等待
        {
            status = ros::ok();
            cv::waitKey(1);
            rate.sleep();
            continue;
        }

        /*** Packaged got ***/
        if (flg_reset)
        {
            ROS_WARN("reset when rosbag play back");
            p_imu->Reset();
            flg_reset = false;
            continue;
        }

        // double t0,t1,t2,t3,t4,t5,match_start, match_time, solve_start, solve_time, svd_time;
        double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

        match_time = kdtree_search_time = kdtree_search_counter = solve_time = solve_const_H_time = svd_time   = 0;
        t0 = omp_get_wtime();
        #ifdef USE_IKFOM
        p_imu->Process(LidarMeasures, kf, feats_undistort);
        state_point = kf.get_x();
        pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
        #else
        // Step 2: 利用IMU数据对状态变量进行积分递推，同时得到去畸变之后的LIDAR点云
        //! 疑问：里面的代码太乱，没有看懂如果当前帧是图像，到底有没有对点云进行去畸变
        //! 暂时解答：感觉应该是没有去畸变处理的，以为里面的操作如果是图像则点的时间都不满足要求，都不会去畸变
        p_imu->Process2(LidarMeasures, state, feats_undistort);
        state_propagat = state;

        #endif

        if (lidar_selector->debug)
        {
            LidarMeasures.debug_show();
        }

        if (feats_undistort->empty() || (feats_undistort == nullptr))
        {
            // cout<<" No point!!!"<<endl;
            if (!fast_lio_is_ready)
            {
                first_lidar_time = LidarMeasures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                LidarMeasures.measures.clear();
                cout<<"FAST-LIO not ready"<<endl;
                continue;
            }
        }
        else
        {
            int size = feats_undistort->points.size();
        }
        fast_lio_is_ready = true;
        flg_EKF_inited = (LidarMeasures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                        false : true;//是否完成初始化

        // Step 3: 当前是图像帧，则进行视觉VIO处理
        if (! LidarMeasures.is_lidar_end&& useVio && fast_lio_is_ready &&
        lio_first) //相机与IMU数据
        {
            cout<<"[ VIO ]: Raw feature num: "<<pcl_wait_pub->points.size() << "." << endl;//; 打印本次VIO之前，上一次LIO的LiDAR点云(转到world系下)
            if (first_lidar_time<10)
            {
                continue;
            }
            //如果开启VIO模块，则才往下处理
            if (img_en) {
                euler_cur = RotMtoEuler(state.rot_end);
                fout_pre << setw(20) << LidarMeasures.last_update_time - first_lidar_time << " " << euler_cur.transpose()*57.3 << " " << state.pos_end.transpose() << " " << state.vel_end.transpose() \
                                <<" "<<state.bias_g.transpose()<<" "<<state.bias_a.transpose()<<" "<<state.gravity.transpose()<< endl;
                
                /* visual main */
                //! 重要：视觉VIO的主函数！
                //; 传入: 当前帧的图像 和 上一帧的LiDAR在世界坐标系下的点云
                //先用IMU预积分更新一下状态，pcl_wait_pub是点云数据，
                //视觉是从地图点里抽FOV里面的点，当然会作检查，取出遮挡点、深度不连续点，然后在于当前img作光度误差
                //1 先是addFromSparseMap choose %d points from sub_sparse_map，在sub_sparse_map中，利用最近雷达扫描从地图中选取子图。
                //2 是addSparseMap，Add %d 3D points 这里不是很懂
                //3 ComputeJ里面：UpdateState迭代更新，最后一次更新updateFrameState
                //4 addObservation
                lidar_selector->detect(LidarMeasures.measures.back().img, pcl_wait_pub);//这里取最后一个          
                
                // int size = lidar_selector->map_cur_frame_.size();
                int size_sub = lidar_selector->sub_map_cur_frame_.size();
                
                // map_cur_frame_point->clear();
                sub_map_cur_frame_point->clear();
                for(int i=0; i<size_sub; i++)
                {
                    PointType temp_map;
                    temp_map.x = lidar_selector->sub_map_cur_frame_[i]->pos_[0];
                    temp_map.y = lidar_selector->sub_map_cur_frame_[i]->pos_[1];
                    temp_map.z = lidar_selector->sub_map_cur_frame_[i]->pos_[2];
                    temp_map.intensity = 0.;
                    sub_map_cur_frame_point->push_back(temp_map);
                }
                cv::Mat img_rgb = lidar_selector->img_cp;
                cv_bridge::CvImage out_msg;
                out_msg.header.stamp = ros::Time::now();
                // out_msg.header.frame_id = "camera_init";
                out_msg.encoding = sensor_msgs::image_encodings::BGR8;
                out_msg.image = img_rgb;
                img_pub.publish(out_msg.toImageMsg());

                if(img_en) publish_frame_world_rgb(pubLaserCloudFullRes, lidar_selector);// 发布带有rgb信息的点云信息
                publish_visual_world_sub_map(pubSubVisualCloud);

                
                geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));
                publish_odometry(pubOdomAftMapped);
                euler_cur = RotMtoEuler(state.rot_end);
                fout_out << setw(20) << LidarMeasures.last_update_time - first_lidar_time << " " << euler_cur.transpose()*57.3 << " " << state.pos_end.transpose() << " " << state.vel_end.transpose() \
                <<" "<<state.bias_g.transpose()<<" "<<state.bias_a.transpose()<<" "<<state.gravity.transpose()<<" "<<feats_undistort->points.size()<<endl;
                /**
                * @brief 如果img和lidar时间不完全一致，VIO只更新Bias
                *        理论有待验证，反正能跑.....
                *
                */
                if (onlyUpdateBias) {
                    state_last_lidar += G_k * (state - state_propagat);
                    // state_last_lidar += G_k * (state_propagat - state);
                    state_last_lidar.cov +=
                        G_k * (state.cov - state_propagat.cov) * G_k.transpose();
                    // state_last_lidar.cov += G_k * (state_propagat.cov - state.cov) *
                    // G_k.transpose();
                    state = state_last_lidar;
                    /* debug */
                    // cout << "[DEBUG]------------->VIO" << state.pos_end << endl;
                    // cout << "[DEBUG]------------->VIO" << state_last_lidar.pos_end <<
                    // endl;
                }
                if (useKalmanSmooth) {
                    // compute F_k from x_lidar to x_camera_hat
                    MD(DIM_STATE, 1)
                    F_lc_18x1 = state_propagat - state_last_lidar;

                    MD(DIM_STATE, DIM_STATE)
                    F_lc = Matrix<double, 18, 18>::Zero();
                    F_lc(0, 0) = F_lc_18x1(0, 0);
                    F_lc(1, 1) = F_lc_18x1(1, 0);
                    F_lc(2, 2) = F_lc_18x1(2, 0);
                    F_lc(3, 3) = F_lc_18x1(3, 0);
                    F_lc(4, 4) = F_lc_18x1(4, 0);
                    F_lc(5, 5) = F_lc_18x1(5, 0);
                    F_lc(6, 6) = F_lc_18x1(6, 0);
                    F_lc(7, 7) = F_lc_18x1(7, 0);
                    F_lc(8, 8) = F_lc_18x1(8, 0);
                    F_lc(9, 9) = F_lc_18x1(9, 0);
                    F_lc(10, 10) = F_lc_18x1(10, 0);
                    F_lc(11, 11) = F_lc_18x1(11, 0);
                    F_lc(12, 12) = F_lc_18x1(12, 0);
                    F_lc(13, 13) = F_lc_18x1(13, 0);
                    F_lc(14, 14) = F_lc_18x1(14, 0);
                    F_lc(15, 15) = F_lc_18x1(15, 0);
                    F_lc(16, 16) = F_lc_18x1(16, 0);
                    F_lc(17, 17) = F_lc_18x1(17, 0);

                    MD(DIM_STATE, DIM_STATE)
                    G_lc = state_last_lidar.cov * F_lc.transpose() *
                            state_propagat.cov.inverse();
                    state_last_lidar = state_last_lidar + G_lc * (state - state_propagat);
                    state_last_lidar.cov =
                        state_last_lidar.cov +
                        G_lc * (state.cov - state_propagat.cov) * G_lc.transpose();

                    // save data
                    // frame_num += 1;
                    // string img_name = std::to_string(frame_num) + ".png";
                    // cv::imwrite(root_dir + "/image/" + img_name, img_copy);
                    // Eigen::Quaterniond rot_(state.rot_end);

                    // fout_pose << frame_num << " " << rot_.w() << " " << rot_.x() << " "
                    //           << rot_.y() << " " << rot_.z() << " "
                    //           << state.pos_end.transpose() << " " << int(1) << " "
                    //           << img_name << " " << endl;
                    /**
                    * @brief 关掉了视觉里程计
                    *
                    */
                    // publish_odometry(pubOdomAftMapped);
                }
            }
            continue;//相机处理好了，下一轮。不用继续往下处理了，因为当前只是处理视觉的部分，而不包括激光的信息
        }

        // Step 4: 运行到这里，说明当前是LiDAR帧，则运行LIO
        /*** Segment the map in lidar FOV ***/
        #ifndef USE_ikdforest            
            lasermap_fov_segment();
        #endif
        /*** downsample the feature points in a scan ***/
        downSizeFilterSurf.setInputCloud(feats_undistort);
        downSizeFilterSurf.filter(*feats_down_body);
    #ifdef USE_ikdtree
        /*** initialize the map kdtree ***/
        #ifdef USE_ikdforest
        if (!ikdforest.initialized){
            if(feats_down_body->points.size() > 5){
                ikdforest.Build(feats_down_body->points, true, lidar_end_time);
            }
            continue;                
        }
        int featsFromMapNum = ikdforest.total_size;
        #else
        if(ikdtree.Root_Node == nullptr)
        {
            if(feats_down_body->points.size() > 5)
            {
                ikdtree.set_downsample_param(filter_size_map_min);
                ikdtree.Build(feats_down_body->points);
            }
            continue;
        }
        int featsFromMapNum = ikdtree.size();
        #endif
    #else
        if(featsFromMap->points.empty())
        {
            downSizeFilterMap.setInputCloud(feats_down_body);
        }
        else
        {
            downSizeFilterMap.setInputCloud(featsFromMap);
        }
        downSizeFilterMap.filter(*featsFromMap);
        int featsFromMapNum = featsFromMap->points.size();
    #endif
        feats_down_size = feats_down_body->points.size();
        cout<<"[ LIO ]: Raw feature num: "<<feats_undistort->points.size()<<" downsamp num "<<feats_down_size<<" Map num: "<<featsFromMapNum<< "." << endl;

        /*** ICP and iterated Kalman filter update ***/
        normvec->resize(feats_down_size);
        feats_down_world->resize(feats_down_size);
        //vector<double> res_last(feats_down_size, 1000.0); // initial //
        res_last.resize(feats_down_size, 1000.0);
        
        t1 = omp_get_wtime();
        if (lidar_en)//要用雷达数据
        {
            euler_cur = RotMtoEuler(state.rot_end);//欧拉角
            #ifdef USE_IKFOM
            //state_ikfom fout_state = kf.get_x();
            fout_pre << setw(20) << LidarMeasures.last_update_time - first_lidar_time << " " << euler_cur.transpose()*57.3 << " " << state_point.pos.transpose() << " " << state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;
            #else
            fout_pre << setw(20) << LidarMeasures.last_update_time  - first_lidar_time << " " << euler_cur.transpose()*57.3 << " " << state.pos_end.transpose() << " " << state.vel_end.transpose() \
            <<" "<<state.bias_g.transpose()<<" "<<state.bias_a.transpose()<<" "<<state.gravity.transpose()<< endl;
            #endif
        }

    #ifdef USE_ikdtree
        if(0)
        {
            PointVector ().swap(ikdtree.PCL_Storage);
            ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
            featsFromMap->clear();
            featsFromMap->points = ikdtree.PCL_Storage;
        }
    #else
        kdtreeSurfFromMap->setInputCloud(featsFromMap);
    #endif

        point_selected_surf.resize(feats_down_size, true);
        pointSearchInd_surf.resize(feats_down_size);
        Nearest_Points.resize(feats_down_size);
        int  rematch_num = 0;
        bool nearest_search_en = true; //

        t2 = omp_get_wtime();
        
        /*** iterated state estimation ***/
        #ifdef MP_EN
        printf("[ LIO ]: Using multi-processor, used core number: %d.\n", MP_PROC_NUM);
        #endif
        double t_update_start = omp_get_wtime();
        #ifdef USE_IKFOM
        double solve_H_time = 0;
        kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
        //state_ikfom updated_state = kf.get_x();
        state_point = kf.get_x();
        //euler_cur = RotMtoEuler(state_point.rot.toRotationMatrix());
        euler_cur = SO3ToEuler(state_point.rot);
        pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
        // cout<<"position: "<<pos_lid.transpose()<<endl;
        geoQuat.x = state_point.rot.coeffs()[0];
        geoQuat.y = state_point.rot.coeffs()[1];
        geoQuat.z = state_point.rot.coeffs()[2];
        geoQuat.w = state_point.rot.coeffs()[3];
        #else

        if(img_en)
        {
            omp_set_num_threads(MP_PROC_NUM);// 设置线程的默认周期数为MP_PROC_NUM 4
            #pragma omp parallel for// 并行计算
            for(int i=0;i<1;i++) {}
        }

        if(lidar_en)//用雷达点云做状态更新
        {
            for (iterCount = -1; iterCount < NUM_MAX_ITERATIONS && flg_EKF_inited; iterCount++) 
            {
                match_start = omp_get_wtime();
                PointCloudXYZI ().swap(*laserCloudOri);
                PointCloudXYZI ().swap(*corr_normvect);
                // laserCloudOri->clear(); 
                // corr_normvect->clear(); 
                total_residual = 0.0; 

                /** closest surface search and residual computation **/
                #ifdef MP_EN
                    omp_set_num_threads(MP_PROC_NUM);
                    #pragma omp parallel for
                #endif
                // normvec->resize(feats_down_size);
                for (int i = 0; i < feats_down_size; i++)
                {
                    PointType &point_body  = feats_down_body->points[i];
                    PointType &point_world = feats_down_world->points[i];
                    V3D p_body(point_body.x, point_body.y, point_body.z);
                    /* transform to world frame */
                    pointBodyToWorld(&point_body, &point_world);
                    vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
                    #ifdef USE_ikdtree
                        auto &points_near = Nearest_Points[i];
                    #else
                        auto &points_near = pointSearchInd_surf[i];
                    #endif
                    uint8_t search_flag = 0;  
                    double search_start = omp_get_wtime();
                    if (nearest_search_en)
                    {
                        /** Find the closest surfaces in the map **/
                        #ifdef USE_ikdtree
                            #ifdef USE_ikdforest
                                search_flag = ikdforest.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis, first_lidar_time, 5);
                            #else
                                ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
                            #endif
                        #else
                            kdtreeSurfFromMap->nearestKSearch(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
                        #endif

                        point_selected_surf[i] = pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;

                        #ifdef USE_ikdforest
                            point_selected_surf[i] = point_selected_surf[i] && (search_flag == 0);
                        #endif
                        kdtree_search_time += omp_get_wtime() - search_start;
                        kdtree_search_counter ++;                        
                    }


                    // if (!point_selected_surf[i]) continue;


                    // Debug
                    // if (points_near.size()<5) {
                    //     printf("\nERROR: Return Points is less than 5\n\n");
                    //     printf("Target Point is: (%0.3f,%0.3f,%0.3f)\n",point_world.x,point_world.y,point_world.z);
                    // }
                    if (!point_selected_surf[i] || points_near.size() < NUM_MATCH_POINTS) continue;

                    VF(4) pabcd;
                    point_selected_surf[i] = false;
                    if (esti_plane(pabcd, points_near, 0.1f)) //(planeValid)
                    {
                        float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
                        float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());//点到平面距离公式

                        if (s > 0.9)//threshold 点到平面距离 < 1/9
                        {
                            point_selected_surf[i] = true;
                            normvec->points[i].x = pabcd(0);
                            normvec->points[i].y = pabcd(1);
                            normvec->points[i].z = pabcd(2);
                            normvec->points[i].intensity = pd2;
                            res_last[i] = abs(pd2);
                        }
                    }
                }
                // cout<<"pca time test: "<<pca_time1<<" "<<pca_time2<<endl;
                effct_feat_num = 0;
                laserCloudOri->resize(feats_down_size);
                corr_normvect->reserve(feats_down_size);
                for (int i = 0; i < feats_down_size; i++)
                {
                    if (point_selected_surf[i] && (res_last[i] <= 2.0))
                    {
                        laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
                        corr_normvect->points[effct_feat_num] = normvec->points[i];
                        total_residual += res_last[i];
                        effct_feat_num ++;
                    }
                }

                res_mean_last = total_residual / effct_feat_num;
                // cout << "[ mapping ]: Effective feature num: "<<effct_feat_num<<" res_mean_last "<<res_mean_last<<endl;
                match_time  += omp_get_wtime() - match_start;// 匹配结束
                solve_start  = omp_get_wtime(); // 迭代求解开始
                
                /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
                MatrixXd Hsub(effct_feat_num, 6);
                VectorXd meas_vec(effct_feat_num);

                for (int i = 0; i < effct_feat_num; i++)
                {
                    const PointType &laser_p  = laserCloudOri->points[i];
                    V3D point_this(laser_p.x, laser_p.y, laser_p.z);
                    point_this += Lidar_offset_to_IMU;
                    M3D point_crossmat;
                    point_crossmat<<SKEW_SYM_MATRX(point_this);// 斜对称矩阵

                    /*** get the normal vector of closest surface/corner ***/
                    const PointType &norm_p = corr_normvect->points[i];
                    V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

                    /*** calculate the Measuremnt Jacobian matrix H ***/
                    V3D A(point_crossmat * state.rot_end.transpose() * norm_vec);
                    Hsub.row(i) << VEC_FROM_ARRAY(A), norm_p.x, norm_p.y, norm_p.z;

                    /*** Measuremnt: distance to the closest surface/corner ***/
                    meas_vec(i) = - norm_p.intensity;
                }
                solve_const_H_time += omp_get_wtime() - solve_start;

                MatrixXd K(DIM_STATE, effct_feat_num);

                EKF_stop_flg = false;
                flg_EKF_converged = false;
                
                /*** Iterative Kalman Filter Update ***/
                if (!flg_EKF_inited)
                {
                    cout<<"||||||||||Initiallizing LiDar||||||||||"<<endl;
                    /*** only run in initialization period ***/
                    MatrixXd H_init(MD(9, DIM_STATE)::Zero());
                    MatrixXd z_init(VD(9)::Zero());
                    H_init.block<3,3>(0,0)  = M3D::Identity();
                    H_init.block<3,3>(3,3)  = M3D::Identity();
                    H_init.block<3,3>(6,15) = M3D::Identity();
                    z_init.block<3,1>(0,0)  = - Log(state.rot_end);
                    z_init.block<3,1>(0,0)  = - state.pos_end;

                    auto H_init_T = H_init.transpose();
                    auto &&K_init = state.cov * H_init_T * (H_init * state.cov * H_init_T + \
                                    0.0001 * MD(9, 9)::Identity()).inverse();
                    solution      = K_init * z_init;

                    // solution.block<9,1>(0,0).setZero();
                    // state += solution;
                    // state.cov = (MatrixXd::Identity(DIM_STATE, DIM_STATE) - K_init * H_init) * state.cov;

                    state.resetpose();// R：单位阵；p,v:Zero3d
                    EKF_stop_flg = true;
                }
                else
                {
                    auto &&Hsub_T = Hsub.transpose();
                    auto &&HTz = Hsub_T * meas_vec;
                    H_T_H.block<6,6>(0,0) = Hsub_T * Hsub;
                    // EigenSolver<Matrix<double, 6, 6>> es(H_T_H.block<6,6>(0,0));
                    MD(DIM_STATE, DIM_STATE) &&K_1 = \
                            (H_T_H + (state.cov / LASER_POINT_COV).inverse()).inverse();//雷达协方差
                    G.block<DIM_STATE,6>(0,0) = K_1.block<DIM_STATE,6>(0,0) * H_T_H.block<6,6>(0,0);
                    auto vec = state_propagat - state;
                    solution = K_1.block<DIM_STATE,6>(0,0) * HTz + vec - G.block<DIM_STATE,6>(0,0) * vec.block<6,1>(0,0);

                    int minRow, minCol;
                    if(0)//if(V.minCoeff(&minRow, &minCol) < 1.0f)
                    {
                        VD(6) V = H_T_H.block<6,6>(0,0).eigenvalues().real();
                        cout<<"!!!!!! Degeneration Happend, eigen values: "<<V.transpose()<<endl;
                        EKF_stop_flg = true;
                        solution.block<6,1>(9,0).setZero();
                    }

                    state += solution;

                    rot_add = solution.block<3,1>(0,0);
                    t_add   = solution.block<3,1>(3,0);

                    if ((rot_add.norm() * 57.3 < 0.01) && (t_add.norm() * 100 < 0.015))// threshold EKF收敛
                    {
                        flg_EKF_converged = true;
                    }

                    deltaR = rot_add.norm() * 57.3;// 弧度 to 角度
                    deltaT = t_add.norm() * 100; // m to cm
                }
                euler_cur = RotMtoEuler(state.rot_end);
                

                /*** Rematch Judgement ***/
                nearest_search_en = false;
                if (flg_EKF_converged || ((rematch_num == 0) && (iterCount == (NUM_MAX_ITERATIONS - 2))))
                {
                    nearest_search_en = true;
                    rematch_num ++;
                }

                /*** Convergence Judgements and Covariance Update ***/
                if (!EKF_stop_flg && (rematch_num >= 2 || (iterCount == NUM_MAX_ITERATIONS - 1)))
                {
                    if (flg_EKF_inited)
                    {
                        /*** Covariance Update ***/
                        // G.setZero();
                        // G.block<DIM_STATE,6>(0,0) = K * Hsub;
                        state.cov = (I_STATE - G) * state.cov;
                        total_distance += (state.pos_end - position_last).norm();
                        position_last = state.pos_end;
                        geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                                    (euler_cur(0), euler_cur(1), euler_cur(2));

                        VD(DIM_STATE) K_sum  = K.rowwise().sum();
                        VD(DIM_STATE) P_diag = state.cov.diagonal();
                        // cout<<"K: "<<K_sum.transpose()<<endl;
                        // cout<<"P: "<<P_diag.transpose()<<endl;
                        // cout<<"position: "<<state.pos_end.transpose()<<" total distance: "<<total_distance<<endl;
                    }
                    EKF_stop_flg = true;
                }
                solve_time += omp_get_wtime() - solve_start;
                /* debug */
                // cout << "[DEBUG]------------->LIO" << state_last_lidar.pos_end <<
                // endl; cout << "[DEBUG]------------->LIO" << state.pos_end << endl;
                /**
                * @brief state_last_lidar 保存LIO结束时的状态
                *
                */
                getCurPose(state); //   更新transformTobeMapped
                /*back end*/
                // 1.计算当前帧与前一帧位姿变换，如果变化太小，不设为关键帧，反之设为关键帧
                // 2.添加激光里程计因子、GPS因子、闭环因子
                // 3.执行因子图优化
                // 4.得到当前帧优化后的位姿，位姿协方差
                // 5.添加cloudKeyPoses3D，cloudKeyPoses6D，更新transformTobeMapped，添加当前关键帧的角点、平面点集合
                saveKeyFramesAndFactor();
                // 更新因子图中所有变量节点的位姿，也就是所有历史关键帧的位姿，更新里程计轨迹， 重构ikdtree
                correctPoses();

                state_last_lidar = state;
                if (EKF_stop_flg)   break;
            }
        }
        
        // cout<<"[ mapping ]: iteration count: "<<iterCount+1<<endl;
        #endif
        // SaveTrajTUM(LidarMeasures.lidar_beg_time, state.rot_end, state.pos_end);
        double t_update_end = omp_get_wtime();
        /******* Publish odometry *******/
        euler_cur = RotMtoEuler(state.rot_end);
        geoQuat = tf::createQuaternionMsgFromRollPitchYaw(euler_cur(0), euler_cur(1), euler_cur(2));
        publish_odometry(pubOdomAftMapped);
        lio_first = true;
        /*** add the feature points to map kdtree ***/
        t3 = omp_get_wtime();
        map_incremental();
        t5 = omp_get_wtime();

        publish_gnss_path(pubGnssPath);                        //   发布gnss轨迹

        kdtree_incremental_time = t5 - t3 + readd_time;
        /******* Publish points *******/

        PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort : feats_down_body);          
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_pub = *laserCloudWorld; // 保存当前帧的点云数据

        if(!img_en) {
            publish_frame_world(pubLaserCloudFullRes);
        }
        // publish_visual_world_map(pubVisualCloud);
        publish_effect_world(pubLaserCloudEffect);
        // publish_map(pubLaserCloudMap);
        publish_path(pubPath);
        #ifdef DEPLOY
        publish_mavros(mavros_pose_publisher);
        #endif

        /*** Debug variables ***/
        frame_num ++;
        aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
        aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;
        aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
        #ifdef USE_IKFOM
        aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;
        aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
        #else
        aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time)/frame_num;
        aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_const_H_time / frame_num;
        //cout << "construct H:" << aver_time_const_H_time << std::endl;
        #endif
        // aver_time_consu = aver_time_consu * 0.9 + (t5 - t0) * 0.1;
        T1[time_log_counter] = LidarMeasures.lidar_beg_time;
        s_plot[time_log_counter] = aver_time_consu;
        s_plot2[time_log_counter] = kdtree_incremental_time;
        s_plot3[time_log_counter] = kdtree_search_time/kdtree_search_counter;
        s_plot4[time_log_counter] = featsFromMapNum;
        s_plot5[time_log_counter] = t5 - t0;
        time_log_counter ++;
        // cout<<"[ mapping ]: time: fov_check "<< fov_check_time <<" fov_check and readd: "<<t1-t0<<" match "<<aver_time_match<<" solve "<<aver_time_solve<<" ICP "<<t3-t1<<" map incre "<<t5-t3<<" total "<<aver_time_consu << "icp:" << aver_time_icp << "construct H:" << aver_time_const_H_time <<endl;
        printf("[ LIO ]: time: fov_check: %0.6f fov_check and readd: %0.6f match: %0.6f solve: %0.6f  ICP: %0.6f  map incre: %0.6f total: %0.6f icp: %0.6f construct H: %0.6f.\n",fov_check_time,t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu,aver_time_icp, aver_time_const_H_time);
        if (lidar_en)
        {
            euler_cur = RotMtoEuler(state.rot_end);
            #ifdef USE_IKFOM
            fout_out << setw(20) << LidarMeasures.last_update_time - first_lidar_time << " " << euler_cur.transpose()*57.3 << " " << state_point.pos.transpose() << " " << state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;
            #else
            fout_out << setw(20) << LidarMeasures.last_update_time - first_lidar_time << " " << euler_cur.transpose()*57.3 << " " << state.pos_end.transpose() << " " << state.vel_end.transpose() \
            <<" "<<state.bias_g.transpose()<<" "<<state.bias_a.transpose()<<" "<<state.gravity.transpose()<<" "<<feats_undistort->points.size()<<endl;
            #endif
        }
        // dump_lio_state_to_log(fp);
    }
    //--------------------------save map---------------
    // string surf_filename(map_file_path + "/surf.pcd");
    // string corner_filename(map_file_path + "/corner.pcd");
    // string all_points_filename(map_file_path + "/all_points.pcd");

    // PointCloudXYZI surf_points, corner_points;
    // surf_points = *featsFromMap;
    // fout_out.close();
    // fout_pre.close();
    // if (surf_points.size() > 0 && corner_points.size() > 0) 
    // {
    // pcl::PCDWriter pcd_writer;
    // cout << "saving...";
    // pcd_writer.writeBinary(surf_filename, surf_points);
    // pcd_writer.writeBinary(corner_filename, corner_points);
    // }

         /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("rgb_scan_all.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current rgb scan saved" << endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    if (pcl_wait_save_lidar->size() > 0 && pcd_save_en)
    {
        string file_name = string("intensity_sacn_all.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current intensity scan saved" << endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save_lidar);
    }

    fout_out.close();
    fout_pre.close();

    #ifndef DEPLOY
    vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;    
    FILE *fp2;
    string log_dir = root_dir + "/Log/fast_livo_time_log.csv";
    fp2 = fopen(log_dir.c_str(),"w");
    fprintf(fp2,"time_stamp, average time, incremental time, search time,fov check time, total time, alpha_bal, alpha_del\n");
    for (int i = 0;i<time_log_counter; i++){
        fprintf(fp2,"%0.8f,%0.8f,%0.8f,%0.8f,%0.8f,%0.8f,%f,%f\n",T1[i],s_plot[i],s_plot2[i],s_plot3[i],s_plot4[i],s_plot5[i],s_plot6[i],s_plot7[i]);
        t.push_back(T1[i]);
        s_vec.push_back(s_plot[i]);
        s_vec2.push_back(s_plot2[i]);
        s_vec3.push_back(s_plot3[i]);
        s_vec4.push_back(s_plot4[i]);
        s_vec5.push_back(s_plot5[i]);
        s_vec6.push_back(s_plot6[i]);        
        s_vec7.push_back(s_plot7[i]);                             
    }
    fclose(fp2);
    if (!t.empty())
    {
        // plt::named_plot("incremental time",t,s_vec2);
        // plt::named_plot("search_time",t,s_vec3);
        // plt::named_plot("total time",t,s_vec5);
        // plt::named_plot("average time",t,s_vec);
        // plt::legend();
        // plt::show();
        // plt::pause(0.5);
        // plt::close();
    }
    #endif
    return 0;
}
