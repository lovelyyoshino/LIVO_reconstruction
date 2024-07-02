#include <ros/ros.h>
#include <livox_ros_driver/CustomMsg.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/thread/mutex.hpp>
#include <iostream>

struct PointXYZINormalLineTagCurvature {
    PCL_ADD_POINT4D;                // 4D点
    float intensity;                // Intensity
    float normal_x, normal_y, normal_z; // Normal vector
    int line;                       // Line number
    unsigned char tag[16];          // Fixed size tag
    float curvature;                // Curvature

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // 确保内存对齐

    // 默认构造函数
    PointXYZINormalLineTagCurvature()
        : intensity(0), normal_x(0), normal_y(0), normal_z(0), 
          line(0), curvature(0) {
        std::memset(tag, 0, sizeof(tag));
    }

    // 带参数的构造函数
    PointXYZINormalLineTagCurvature(float x, float y, float z, float intensity, 
                                    float normal_x, float normal_y, float normal_z,
                                    int line, const std::string& tag, float curvature)
        : intensity(intensity), normal_x(normal_x), normal_y(normal_y), normal_z(normal_z), 
          line(line), curvature(curvature) {
        this->x = x;
        this->y = y;
        this->z = z;
        std::strncpy(reinterpret_cast<char*>(this->tag), tag.c_str(), sizeof(this->tag) - 1);
        this->tag[sizeof(this->tag) - 1] = '\0';
    }

    // 重载加法运算符，将两个点云合并
    PointXYZINormalLineTagCurvature operator+(const PointXYZINormalLineTagCurvature& other) const {
        PointXYZINormalLineTagCurvature result;
        result.x = this->x + other.x;
        result.y = this->y + other.y;
        result.z = this->z + other.z;
        result.intensity = this->intensity + other.intensity;
        result.normal_x = this->normal_x + other.normal_x;
        result.normal_y = this->normal_y + other.normal_y;
        result.normal_z = this->normal_z + other.normal_z;
        result.line = this->line + other.line; // This is just an example, adjust as needed
        std::strncpy(reinterpret_cast<char*>(result.tag), reinterpret_cast<const char*>(this->tag), sizeof(result.tag) - 1);
        result.tag[sizeof(result.tag) - 1] = '\0';
        result.curvature = this->curvature + other.curvature;
        return result;
    }
};

// 必须在宏定义外定义这些元素
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZINormalLineTagCurvature,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, normal_x, normal_x)
    (float, normal_y, normal_y)
    (float, normal_z, normal_z)
    (int, line, line)
    (unsigned char[16], tag, tag)
    (float, curvature, curvature)
)

typedef PointXYZINormalLineTagCurvature PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

class PointsConcatFilter {
public:
    PointsConcatFilter() : node_handle_(), private_node_handle_("~") {
        sub_avia = new message_filters::Subscriber<livox_ros_driver::CustomMsg>(node_handle_, "/avia/livox/lidar", 1);
        sub_mid360 = new message_filters::Subscriber<livox_ros_driver::CustomMsg>(node_handle_, "/mid360/livox/lidar", 1);

        cloud_synchronizer_ = new message_filters::Synchronizer<SyncPolicyT>(
                SyncPolicyT(100), *sub_avia, *sub_mid360);
        cloud_synchronizer_->registerCallback(boost::bind(&PointsConcatFilter::pointcloud_callback, this, _1, _2));

        cloud_publisher_ = node_handle_.advertise<livox_ros_driver::CustomMsg>("/livox/lidar", 1);
    }

    void pointcloud_callback(const livox_ros_driver::CustomMsgConstPtr &msg1, const livox_ros_driver::CustomMsgConstPtr &msg2) {
        boost::mutex::scoped_lock lock(mutex_);

        std::cout << "[DEBUG] Received messages with point counts: " << msg1->point_num << " and " << msg2->point_num << std::endl;

        PointCloudXYZI::Ptr pointCloud_avia(new PointCloudXYZI);
        PointCloudXYZI::Ptr pointCloud_mid360(new PointCloudXYZI);
        PointCloudXYZI::Ptr pointCloud_mid360_transformed(new PointCloudXYZI);
        PointCloudXYZI::Ptr finalPointCloud(new PointCloudXYZI);

        convert2PointCloud2(msg1, pointCloud_avia);
        convert2PointCloud2(msg2, pointCloud_mid360);

        std::cout << "[DEBUG] PointCloud sizes after conversion: " << pointCloud_avia->points.size() << " and " << pointCloud_mid360->points.size() << std::endl;

        // 旋转平移矩阵
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform << 1.000, -0.022,  0.005, -0.015,
                    0.022,  1.000, -0.019,  0.009,
                    -0.005,  0.019,  1.000,  0.067,
                    0.000,  0.000,  0.000,  1.000;

        pcl::transformPointCloud(*pointCloud_mid360, *pointCloud_mid360_transformed, transform);

        *finalPointCloud = *pointCloud_avia + *pointCloud_mid360_transformed;

        std::cout << "[DEBUG] Final PointCloud size: " << finalPointCloud->points.size() << std::endl;

        livox_ros_driver::CustomMsg finalMsg;
        finalMsg.header = msg1->header;
        finalMsg.timebase = msg1->timebase;
        finalMsg.point_num = finalPointCloud->size();
        finalMsg.lidar_id = msg1->lidar_id;

        for (unsigned int i = 0; i < finalMsg.point_num; i++) {
            livox_ros_driver::CustomPoint p;
            p.x = finalPointCloud->points[i].x;
            p.y = finalPointCloud->points[i].y;
            p.z = finalPointCloud->points[i].z;
            p.reflectivity = finalPointCloud->points[i].intensity;
            p.line = finalPointCloud->points[i].line;
            p.tag = finalPointCloud->points[i].tag[0];
            p.offset_time = finalPointCloud->points[i].curvature * float(1000000);
            finalMsg.points.push_back(p);
        }

        cloud_publisher_.publish(finalMsg);
        std::cout << "[DEBUG] Published final message with " << finalMsg.point_num << " points." << std::endl;
    }

    void convert2PointCloud2(const livox_ros_driver::CustomMsgConstPtr &lidarMsg, PointCloudXYZI::Ptr &pclPointCloud) {
        std::cout << "[DEBUG] Converting CustomMsg to PointCloud2 with " << lidarMsg->point_num << " points." << std::endl;
        for (unsigned int i = 0; i < lidarMsg->point_num; i++) {
            PointType point;
            point.x = lidarMsg->points[i].x;
            point.y = lidarMsg->points[i].y;
            point.z = lidarMsg->points[i].z;
            point.intensity = lidarMsg->points[i].reflectivity;
            point.curvature = lidarMsg->points[i].offset_time / float(1000000);
            point.line = lidarMsg->points[i].line;
            std::memcpy(point.tag, &lidarMsg->points[i].tag, sizeof(point.tag));
            pclPointCloud->points.push_back(point);
        }
        pclPointCloud->width = pclPointCloud->points.size();
        pclPointCloud->height = 1;
        pclPointCloud->is_dense = true;
        std::cout << "[DEBUG] Converted PointCloud2 size: " << pclPointCloud->points.size() << std::endl;
    }

private:
    ros::NodeHandle node_handle_, private_node_handle_;
    ros::Publisher cloud_publisher_;
    message_filters::Subscriber<livox_ros_driver::CustomMsg> *sub_avia;
    message_filters::Subscriber<livox_ros_driver::CustomMsg> *sub_mid360;
    typedef message_filters::sync_policies::ApproximateTime<livox_ros_driver::CustomMsg, livox_ros_driver::CustomMsg> SyncPolicyT;
    message_filters::Synchronizer<SyncPolicyT> *cloud_synchronizer_;
    boost::mutex mutex_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "points_concat_filter");
    PointsConcatFilter node;
    ros::spin();
    return 0;
}
