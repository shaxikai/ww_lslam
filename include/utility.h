#include <iomanip> 
#include <vector>
#include <deque>
#include <thread>

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Eigen>

#include <sophus/so3.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <livox_ros_driver/CustomMsg.h>

#include <ww_lslam/Pose6D.h>


geometry_msgs::Pose convert2ROSPose(Sophus::SO3& rot, Eigen::Vector3d& pos)
{
    geometry_msgs::Pose out;
    out.position.x = pos(0);
    out.position.y = pos(1);
    out.position.z = pos(2);

    auto q = Eigen::Quaterniond(rot.matrix());
    out.orientation.x = q.coeffs()[0];
    out.orientation.y = q.coeffs()[1];
    out.orientation.z = q.coeffs()[2];
    out.orientation.w = q.coeffs()[3];

    return out;
}

