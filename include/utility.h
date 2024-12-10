#ifndef UTILITY_H
#define UTILITY_H

#include <iomanip> 
#include <vector>
#include <array>
#include <deque>
#include <thread>
#include <csignal>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Eigen>

#include <sophus/se3.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <livox_ros_driver/CustomMsg.h>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

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

#include "ivox3d/ivox3d.h"

using namespace std;

#define VEC_FROM_ARRAY(v)   v[0],v[1],v[2]
#define MAT_FROM_ARRAY(v)   v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8]
#define SKEW_SYM_MATRX(v)   0.0,-v[2],v[1],v[2],0.0,-v[0],-v[1],v[0],0.0

#define ASSERT(condition, describe) \
    if (!(condition)) { \
        std::cerr << describe <<" file " << __FILE__ << ", line " << __LINE__ << std::endl; \
        return false; \
    }

typedef Eigen::Vector3d V3D;
typedef Eigen::Matrix3d M3D;
typedef Eigen::Vector3f V3F;
typedef Eigen::Matrix3f M3F;
typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;
typedef vector<PointType, Eigen::aligned_allocator<PointType>>  PointVector;
typedef Eigen::Matrix<double, 24, 24> COV;				// 24X24的协方差矩阵
typedef Eigen::Matrix<double, 24, 1>  StateV24;        // 24X1的向量

namespace rslidar_ros
{
  struct EIGEN_ALIGN16 Point
  {
    PCL_ADD_POINT4D;
    std::uint8_t intensity;
    std::uint16_t ring = 0;
    double timestamp = 0;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace rslidar_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(rslidar_ros::Point,
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (std::uint8_t, intensity, intensity)
                                 (std::uint16_t, ring, ring)
                                 (double, timestamp, timestamp))

namespace velodyne_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      float time;
      std::uint16_t ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (float, intensity, intensity)
                                 (float, time, time)
                                 (std::uint16_t, ring, ring))



namespace ouster_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      std::uint32_t t;
      std::uint16_t reflectivity;
      std::uint8_t ring;
      std::uint16_t noise;
      std::uint32_t range;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (std::uint32_t, t, t) (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring) (std::uint16_t, noise, noise) (std::uint32_t, range, range)
)

namespace mulren_ros {
    struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D
      float intensity;
      std::uint32_t t;
      int ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace velodyne_ros
 POINT_CLOUD_REGISTER_POINT_STRUCT (mulren_ros::Point,
     (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
     (std::uint32_t, t, t) (int, ring, ring)
 )


enum LidType
{
  AVIA      = 1,
  MID360,
  VELO16,
  OUST64,
  OUST128,
  RS32,
  MULRAN
};

enum TIME_UNIT
{
  SEC = 0,
  MS = 1,
  US = 2,
  NS = 3
};

struct Param
{
    string  lidTopic;
    string  imuTopic;
    string  gpsTopic;

    bool    enPathPub;
    bool    enCurPclPub;
    bool    enDenPclPub;
    bool    enLocMapPub;

    string  locPthTopic;
    string  curPclTopic;
    string  denPclTopic;
    string  locMapTopic;
    string  gloPthTopic;
    string  gpsPthTopic;

    string  worldCoord;

    int     lidType;                        // 激光雷达的类型
    int     N_SCANS;                        // 激光雷达扫描的线数（livox avia为6线）
    double  detRange;                       // 激光雷达的最大探测范围
    double  blind;                          // 最小距离阈值，即过滤掉0～blind范围内的点云
    int     SCAN_RATE;
    double  fovDeg;

    double  gyrCov;                         // IMU陀螺仪的协方差
    double  accCov;                         // IMU加速度计的协方差
    double  bGyrCov;                        // IMU陀螺仪偏置的协方差
    double  bAccCov;                        // IMU加速度计偏置的协方差
    double  lidPtCov;   

    int     nMaxIter;                       // 卡尔曼滤波的最大迭代次数
    double  minMapFilterSize;
    int     ivoxNearbyType;   
    int     ivoxCapacity;
    double  mapMovThr;

    int     mapSaveType;                      // 是否将点云地图保存到PCD文件
    string  mapSavePath;                    // 地图保存路径

    int     nPointFilter;                   // 采样间隔，即每隔point_filter_num个点取1个点
    bool    enEstExtrinsic;
    int     pcdSaveInterval;
    vector<double>  extrinT;                // 雷达相对于IMU的外参T（即雷达在IMU坐标系中的坐标）
    vector<double>  extrinR;                // 雷达相对于IMU的外参R

    double  keyAngThr;
    double  keyTraThr;

    //imu preint
    double  gravity;
    double  epsi;

    bool    enGPS;
    double  gpsCovThr;

    bool    lioOutFlg;
    bool    bkdOutFlg;

};


// 设置终端为非阻塞模式
void setNonBlocking() {
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);  // 获取当前文件描述符的标志
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);  // 设置为非阻塞模式
}

// 获取非阻塞的字符输入
char getKey()
{
    char ch;
    if (read(STDIN_FILENO, &ch, 1) > 0)  // 非阻塞读取
        return ch;
    return 0;  // 没有输入时返回 0
}

void convert2GtsamPose(Sophus::SE3 &se3, gtsam::Pose3 &pos)
{
  gtsam::Rot3 rotationGTSAM(se3.rotation_matrix());
  gtsam::Point3 translationGTSAM(se3.translation());
  pos = gtsam::Pose3(rotationGTSAM, translationGTSAM);
}

void convert2SE3(gtsam::Pose3 &pos, Sophus::SE3 &se3)
{
  Eigen::Matrix<double, 3, 3> R = pos.rotation().matrix();
  Eigen::Vector3d t = pos.translation();
  se3 = Sophus::SE3(R, t);
}

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

#endif // UTILITY_H