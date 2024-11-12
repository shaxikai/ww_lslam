#include <iomanip> 
#include <vector>
#include <array>
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

#include "ikd-Tree/ikd_Tree.h"

#define VEC_FROM_ARRAY(v)   v[0],v[1],v[2]
#define MAT_FROM_ARRAY(v)   v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8]
#define SKEW_SYM_MATRX(v)   0.0,-v[2],v[1],v[2],0.0,-v[0],-v[1],v[0],0.0

typedef Eigen::Vector3d V3D;
typedef Eigen::Matrix3d M3D;
typedef Eigen::Vector3f V3F;
typedef Eigen::Matrix3f M3F;
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
typedef vector<PointType, Eigen::aligned_allocator<PointType>>  PointVector;
typedef Eigen::Matrix<double, 24, 24> COV;				// 24X24的协方差矩阵
typedef Eigen::Matrix<double, 24, 1>  StateV24;        // 24X1的向量

KD_TREE<PointType> ikdtree;

enum LidType
{
  AVIA      = 1,
  MID360,
  VELO16,
  OUST64,
  RS32
};

enum TIME_UNIT
{
  SEC = 0,
  MS = 1,
  US = 2,
  NS = 3
};

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

