
#include <utility.h>


void savePointCloudToFile(pcl::PointCloud<PointXYZIRT>::Ptr cloud, const std::string& filename) {
    if (cloud->points.empty()) {
        std::cout << "Point cloud is empty!" << std::endl;
        return;
    }

    // 打开一个输出文件流
    std::ofstream file;
    file.open(filename);

    if (!file.is_open()) {
        std::cerr << "Failed to open file " << filename << std::endl;
        return;
    }

    // 遍历点云中的每一个点，并将数据写入文件
    for (const auto& point : cloud->points) {
        file << point.x << " " 
             << point.y << " " 
             << point.z << " " 
             << point.intensity << " "
             << point.ring << " "
             << point.time << "\n";
    }

    // 关闭文件
    file.close();
    std::cout << "Point cloud data saved to " << filename << std::endl;
}


class LaserCloudIn
{
private:

    ros::NodeHandle nh;

    // 订阅原始激光点云
    ros::Subscriber subLaserCloud;
    ros::Publisher  pubLaserCloud;

    std::string lidTopic;  
    std::string lidFrame;

    std_msgs::Header cloudHeader;                               // 当前帧header，包含时间戳信息
    pcl::PointCloud<PointXYZIRT>::Ptr pcdIn;

    int cnt = 0;

public:
    LaserCloudIn():
    lidTopic("/livox/lidar"),
    lidFrame("base_link")
    {
        // 订阅原始lidar数据
        subLaserCloud = nh.subscribe<livox_ros_driver::CustomMsg>(lidTopic, 5, &LaserCloudIn::lidHandler, this, ros::TransportHints().tcpNoDelay());

        // 发布点云
        pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2> ("orig_cloud", 1);

        allocateMemory();
    }

    void allocateMemory()
    {
        pcdIn.reset(new pcl::PointCloud<PointXYZIRT>());
        resetParameters();
    }

    void resetParameters()
    {
        pcdIn->clear();
    }

    void lidHandler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
    {
        // 当前帧头部
        cloudHeader = msg->header;
        convertLivox2PCD(msg, pcdIn);
        if (cnt++ == 100)
            savePointCloudToFile(pcdIn, "pointcloud.txt");

        sensor_msgs::PointCloud2Ptr rosCloud(new sensor_msgs::PointCloud2);
        convertPCD2ROS(lidFrame, cloudHeader.stamp, pcdIn, rosCloud);
        pubLaserCloud.publish(rosCloud);
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "avia_in");

    LaserCloudIn LCI;

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();
    
    return 0;
}
