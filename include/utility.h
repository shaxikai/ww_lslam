
#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

/**
 * 变量名XYZIRT是每个变量的首字母
*/
struct PointXYZIRT
{
    PCL_ADD_POINT4D     // 位置
    PCL_ADD_INTENSITY;  // 激光点反射强度，也可以存点的索引
    uint16_t ring;      // 扫描线
    float time;         // 时间戳，记录相对于当前帧第一个激光点的时差，第一个点time=0
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;        // 内存16字节对齐，EIGEN SSE优化要求
// 注册为PCL点云格式
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (std::uint16_t, ring, ring) (float, time, time)
)

//CustomMsg 并转换成 PointCloud2
void convertLivox2PCD(
    const livox_ros_driver::CustomMsg::ConstPtr& livox_cld_msg,
          pcl::PointCloud<PointXYZIRT>::Ptr    & pcd
        ) 
{
    // 确保点云是空的，并预分配空间
    pcd->clear();
    pcd->reserve(livox_cld_msg->point_num);

    // 遍历 Livox 自定义消息中的每一个点
    for (size_t i = 0; i < livox_cld_msg->point_num; ++i) {
        PointXYZIRT point;

        // 提取点的 XYZ 坐标
        point.x = livox_cld_msg->points[i].x;
        point.y = livox_cld_msg->points[i].y;
        point.z = livox_cld_msg->points[i].z;

        // 提取反射强度
        point.intensity = static_cast<float>(livox_cld_msg->points[i].reflectivity);

        // 提取环号（Livox 数据通常没有环号，可能需要手动分配）
        point.ring = livox_cld_msg->lidar_id;

        if (point.ring) std::cout << point.ring << std::endl;

        // 计算点的时间戳
        point.time = static_cast<float>(livox_cld_msg->points[i].offset_time);

        // 将点添加到点云中
        pcd->push_back(point);
    }

    // 设置点云的 header，确保与原始数据对齐
    pcd->width = pcd->size();
    pcd->height = 1;
    pcd->is_dense = false;
}

// 函数: 将 pcl::PointCloud<PointXYZIRT>::Ptr 转换为 sensor_msgs::PointCloud2ConstPtr
void convertPCD2ROS(
    const std::string&                          frame_id,
    const ros::Time&                            stamp,
    const pcl::PointCloud<PointXYZIRT>::Ptr&    pcl_cloud, 
          sensor_msgs::PointCloud2Ptr&          ros_cloud
    ) 
{
    // 将 PCL 点云转换为 ROS 消息
    pcl::toROSMsg(*pcl_cloud, *ros_cloud);

    // 设置 header 信息，包括时间戳和帧 ID
    ros_cloud->header.stamp = stamp;
    ros_cloud->header.frame_id = frame_id;
}