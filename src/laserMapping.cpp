#include "lio.h"
#include "backend.h"
 

class LSLAM
{
private:

    ros::NodeHandle nh;
    std::shared_ptr<Param>   pa;
    std::shared_ptr<LIO>     lio;
    std::shared_ptr<BACKEND> bkd;

    bool rcvLioFlg;
    double lioCurTs;
    Sophus::SE3 lioBdyPos;
    PointCloudXYZI::Ptr lioCurLid;


    ros::Subscriber subLid;             // 订阅原始激光点云
    ros::Subscriber subImu;             // imu数据队列（原始数据，转lidar系下）
    ros::Subscriber subGPS;             

    nav_msgs::Path  locPth;
    ros::Publisher  locPthPub;
    ros::Publisher  curPclPub;
    ros::Publisher  denPclPub;
    ros::Publisher  locMapPub;
    ros::Publisher  gloMapPub;

    ros::Publisher  gloPthPub;
    ros::Publisher  gpsPthPub;

    thread getLioOutThr;
    thread pubStateThr;

public:

    LSLAM()
    {
        pa = std::make_shared<Param>();
        readParm();

        lio = std::make_shared<LIO>(pa);
        bkd = std::make_shared<BACKEND>(pa);

        locPth.header.stamp = ros::Time::now();
        locPth.header.frame_id = pa->worldCoord;

        // 订阅原始imu数据
        subImu = nh.subscribe(pa->imuTopic, 2000, &LSLAM::imuHandler, this, ros::TransportHints().tcpNoDelay());

        // 订阅原始lidar数据
        switch (pa->lidType)
        {
        case MID360:
        case AVIA:
            subLid = nh.subscribe<livox_ros_driver::CustomMsg>(
                     pa->lidTopic, 5, &LSLAM::livoxLidHandler, this, ros::TransportHints().tcpNoDelay());
            break;
        case VELO16:
            subLid = nh.subscribe<sensor_msgs::PointCloud2>(
                     pa->lidTopic, 5, &LSLAM::velLidHandler, this, ros::TransportHints().tcpNoDelay());
            break;
        case RS32:
            subLid = nh.subscribe<sensor_msgs::PointCloud2>(
                     pa->lidTopic, 5, &LSLAM::rsLidHandler, this, ros::TransportHints().tcpNoDelay());
            break;
        case MULRAN:
            subLid = nh.subscribe<sensor_msgs::PointCloud2>(
                     pa->lidTopic, 5, &LSLAM::mulLidHandler, this, ros::TransportHints().tcpNoDelay());
            break;
        default:
            break;
        }

        subGPS = nh.subscribe<sensor_msgs::NavSatFix> 
                (pa->gpsTopic, 200, &LSLAM::gpsHandler, this, ros::TransportHints().tcpNoDelay());

        locPthPub = nh.advertise<nav_msgs::Path>(pa->locPthTopic, 100000);
        gloPthPub = nh.advertise<nav_msgs::Path>(pa->gloPthTopic, 100000);
        gpsPthPub = nh.advertise<nav_msgs::Path>(pa->gpsPthTopic, 100000);
        curPclPub = nh.advertise<sensor_msgs::PointCloud2>(pa->curPclTopic, 100000);
        denPclPub = nh.advertise<sensor_msgs::PointCloud2>(pa->denPclTopic, 100000);
        locMapPub = nh.advertise<sensor_msgs::PointCloud2>(pa->locMapTopic, 100000);
        gloMapPub = nh.advertise<sensor_msgs::PointCloud2>(pa->gloMapTopic, 100000);

        getLioOutThr = thread(&LSLAM::getLioOutput, this);
        pubStateThr  = thread(&LSLAM::pubState, this);
    }

    void readParm()
    {
        pa->lioOutFlg = false;
        pa->bkdOutFlg = false;

        nh.param<string>("common/coordinate", pa->worldCoord, "camera_init");         // 雷达点云topic名称

        nh.param<string>("common/lid_topic", pa->lidTopic, "/livox/lidar");         // 雷达点云topic名称
        nh.param<string>("common/imu_topic", pa->imuTopic, "/livox/imu");           // IMU的topic名称
        nh.param<string>("common/gps_topic", pa->gpsTopic, "/gps");           // IMU的topic名称

        nh.param<bool>("publish/path_pub_en", pa->enPathPub, true);
        nh.param<bool>("publish/cur_pcl_pub_en", pa->enCurPclPub, true);
        nh.param<bool>("publish/den_pcl_pub_en", pa->enDenPclPub, true);
        nh.param<bool>("publish/map_pub_en", pa->enLocMapPub, true);

        nh.param<string>("publish/loc_path_topic", pa->locPthTopic, "/loc_path");
        nh.param<string>("publish/cur_pcl_topic", pa->curPclTopic, "/cur_pcl");
        nh.param<string>("publish/den_pcl_topic", pa->denPclTopic, "/den_pcl");
        nh.param<string>("publish/loc_map_topic", pa->locMapTopic, "/loc_map");
        nh.param<string>("publish/glo_map_topic", pa->gloMapTopic, "/glo_map");
        nh.param<string>("publish/glo_path_topic", pa->gloPthTopic, "/glo_path");
        nh.param<string>("publish/gps_path_topic", pa->gpsPthTopic, "/gps_path");

        nh.param<int>("common/lidar_type", pa->lidType, AVIA); // 激光雷达的类型
        nh.param<double>("common/blind", pa->blind, 0.01);        // 最小距离阈值，即过滤掉0～blind范围内的点云
        nh.param<int>("common/scan_line", pa->N_SCANS, 16);       // 激光雷达扫描的线数（livox avia为6线）
        nh.param<double>("common/det_range", pa->detRange, 300.f); // 激光雷达的最大探测范围
        nh.param<double>("mapping/fov_degree", pa->fovDeg, 180);
        nh.param<int>("common/scan_rate", pa->SCAN_RATE, 10);
        nh.param<int>("common/point_filter_num", pa->nPointFilter, 2);           // 采样间隔，即每隔point_filter_num个点取1个点

        nh.param<double>("mapping/epsi", pa->epsi, 0.001);
        nh.param<double>("mapping/g_world", pa->gravity, 9.81);

        nh.param<double>("mapping/gyr_cov", pa->gyrCov, 0.1);               // IMU陀螺仪的协方差
        nh.param<double>("mapping/acc_cov", pa->accCov, 0.1);               // IMU加速度计的协方差
        nh.param<double>("mapping/b_gyr_cov", pa->bGyrCov, 0.0001);        // IMU陀螺仪偏置的协方差
        nh.param<double>("mapping/b_acc_cov", pa->bAccCov, 0.0001);        // IMU加速度计偏置的协方差
        nh.param<double>("mapping/lid_point_cov", pa->lidPtCov, 0.001);        // IMU加速度计偏置的协方差

        nh.param<int>("mapping/max_iteration", pa->nMaxIter, 3);                   // 卡尔曼滤波的最大迭代次数
        nh.param<double>("mapping/filter_size_map", pa->minMapFilterSize, 0.5);
        nh.param<int>("mapping/ivox_nearby_type", pa->ivoxNearbyType, 18);
        nh.param<int>("mapping/ivox_capacity", pa->ivoxCapacity, 1000000);
        nh.param<double>("mapping/map_move_thr", pa->mapMovThr, 1.5);

        nh.param<double>("mapping/key_ang_thr", pa->keyAngThr, 0.2);
        nh.param<double>("mapping/key_tra_thr", pa->keyTraThr, 1.0);

        vector<double> extrinR;
        vector<double> extrinT;
        nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>(3, 0.0)); // 雷达相对于IMU的外参T（即雷达在IMU坐标系中的坐标）
        nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>(9, 0.0)); // 雷达相对于IMU的外参R

        nh.param<bool>("gps/en_gps", pa->enGPS, false);   
        nh.param<double>("gps/gps_cov_thr", pa->gpsCovThr, 10.0);       

        nh.param<int>("save/map_save_type", pa->mapSaveType, 0); // 是否将点云地图保存到PCD文件
        nh.param<string>("save/map_save_path", pa->mapSavePath, "./PCD/");                    // 地图保存路径

        pa->extrinT << VEC_FROM_ARRAY(extrinT);
        pa->extrinR << MAT_FROM_ARRAY(extrinR);

        cerr << "lid topic: " << pa->lidTopic << endl;
        cerr << "imu topic: " << pa->imuTopic << endl;
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr &msg)
    {
        lio->imuInput(msg);
    }

    void livoxLidHandler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
    {
        PointCloudXYZI::Ptr pclPtr(new PointCloudXYZI());
        PointType pt, lastPt;
        int nPts = msg->point_num;
        uint gap = pa->nPointFilter;
        for (uint i = 1; i < nPts; i+=gap)
        {
            auto &msgPt = msg->points[i];
            if ((msgPt.line < pa->N_SCANS) 
            && ((msgPt.tag & 0x30) == 0x10 || (msgPt.tag & 0x30) == 0x00)
            && ((abs(msgPt.x) > 1e-7) || (abs(msgPt.y) > 1e-7) || (abs(msgPt.z) > 1e-7)))
            {
                pt.x = msgPt.x;
                pt.y = msgPt.y;
                pt.z = msgPt.z;
                pt.intensity = msgPt.reflectivity;
                pt.curvature = msgPt.offset_time * 1e-9; 

                if ((abs(pt.x - lastPt.x) > 1e-7) || (abs(pt.y - lastPt.y) > 1e-7) || (abs(pt.z - lastPt.z) > 1e-7) 
                && (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z > (pa->blind * pa->blind)))
                {
                    pclPtr->push_back(pt);
                    lastPt = pt;
                }
            }
        }

        double ts = msg->header.stamp.toSec();
        lio->lidInput(ts, pclPtr);
    }

    void rsLidHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        pcl::PointCloud<rslidar_ros::Point> rsPts;
        pcl::fromROSMsg(*msg, rsPts);
        int nPts = rsPts.points.size();

        PointCloudXYZI::Ptr pclPtr(new PointCloudXYZI());
        PointType pt, lastPt;
        uint gap = pa->nPointFilter;
        for (uint i = 0; i < nPts; i+=gap)
        {
            if (rsPts.points[i].ring < pa->N_SCANS)
            {
                pt.x = rsPts.points[i].x;
                pt.y = rsPts.points[i].y;
                pt.z = rsPts.points[i].z;
                pt.intensity = rsPts.points[i].intensity;
                pt.curvature = rsPts.points[i].timestamp - rsPts.points[0].timestamp; 

                if ((abs(pt.x - lastPt.x) > 1e-7) || (abs(pt.y - lastPt.y) > 1e-7) || (abs(pt.z - lastPt.z) > 1e-7) 
                && (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z > (pa->blind * pa->blind)))
                {
                    pclPtr->push_back(pt);
                    lastPt = pt;
                }
            }
        }

        double ts = msg->header.stamp.toSec();
        lio->lidInput(ts, pclPtr);
    }

    void velLidHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        pcl::PointCloud<velodyne_ros::Point> rsPts;
        pcl::fromROSMsg(*msg, rsPts);
        int nPts = rsPts.points.size();
        uint gap = pa->nPointFilter;

        PointType pt, lastPt;
        PointCloudXYZI::Ptr pclPtr(new PointCloudXYZI());
        for (uint i = 1; i < nPts; i+=gap)
        {
            auto &msgPt = rsPts.points[i];
            if ((msgPt.ring < pa->N_SCANS) 
            && ((abs(msgPt.x) > 1e-7) || (abs(msgPt.y) > 1e-7) || (abs(msgPt.z) > 1e-7)))
            {
                pt.x = msgPt.x;
                pt.y = msgPt.y;
                pt.z = msgPt.z;
                pt.intensity = msgPt.intensity;
                pt.curvature = (msgPt.time - rsPts.points[0].time); 

                if ((abs(pt.x - lastPt.x) > 1e-7) || (abs(pt.y - lastPt.y) > 1e-7) || (abs(pt.z - lastPt.z) > 1e-7) 
                && (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z > (pa->blind * pa->blind)))
                {
                    pclPtr->push_back(pt);
                    lastPt = pt;
                }
            }
        }
        double ts = msg->header.stamp.toSec();
        lio->lidInput(ts, pclPtr);
    }

    void mulLidHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        pcl::PointCloud<mulren_ros::Point> rsPts;
        pcl::fromROSMsg(*msg, rsPts);
        int nPts = rsPts.points.size();
        uint gap = pa->nPointFilter;
        PointType pt, lastPt;
        PointCloudXYZI::Ptr pclPtr(new PointCloudXYZI());
        for (uint i = 1; i < nPts; i+=gap)
        {
            auto &msgPt = rsPts.points[i];
            if ((msgPt.ring < pa->N_SCANS) 
            && ((abs(msgPt.x) > 1e-7) || (abs(msgPt.y) > 1e-7) || (abs(msgPt.z) > 1e-7)))
            {
                pt.x = msgPt.x;
                pt.y = msgPt.y;
                pt.z = msgPt.z;
                pt.intensity = msgPt.intensity;
                pt.curvature = (rsPts.points[0].t) * 1e-9; 

                if ((abs(pt.x - lastPt.x) > 1e-7) || (abs(pt.y - lastPt.y) > 1e-7) || (abs(pt.z - lastPt.z) > 1e-7) 
                && (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z > (pa->blind * pa->blind)))
                {
                    pclPtr->push_back(pt);
                    lastPt = pt;
                }
            }
        }

        double ts = msg->header.stamp.toSec();
        lio->lidInput(ts, pclPtr);
    }

    void gpsHandler(const sensor_msgs::NavSatFixConstPtr& msg)
    {
        if (!pa->enGPS)
            return;

        BACKEND::GpsData data;
        data.ts  = msg->header.stamp.toSec();
        data.pos = Vec3d(msg->latitude, msg->longitude, msg->altitude);
        data.cov = Vec3d(msg->position_covariance[0], msg->position_covariance[4], msg->position_covariance[8]);
        bkd->gpsInput(data);
    }

    void getLioOutput()
    {
        rcvLioFlg = false;
        LIO::LioOutData lioOutData;
        BACKEND::LioData bkdInData;

        ros::Rate rate(50000);
        while (ros::ok())
        {     
            if (rcvLioFlg != pa->lioOutFlg)
            {
                lio->getOutData(lioOutData);
                lioCurTs  = lioOutData.ts;
                lioBdyPos = lioOutData.pos;
                lioCurLid = lioOutData.lid;

                bkdInData.ts  = lioOutData.ts;
                bkdInData.pos = lioOutData.pos;
                bkdInData.cov = lioOutData.cov;
                bkdInData.lid = lioOutData.lid;
                bkd->lidInput(bkdInData);

                rcvLioFlg = pa->lioOutFlg;
            }
            rate.sleep();
        }
    }

    void pubState()
    {
        bool tmpRcvLioFlg = false;
        bool tmpRcvBkdFlg = false;

        ros::Rate rate(50000);
        double tmpTs = 0;
        while (ros::ok())
        {     
            if (tmpRcvLioFlg != rcvLioFlg)
            {
                pubLocPath();

                PointCloudXYZI::Ptr pts(new PointCloudXYZI);
                Sophus::SE3 tmpLidPos = lioBdyPos * Sophus::SE3(pa->extrinR, pa->extrinT);
                ptsLid2World(lioCurLid, tmpLidPos, pts);
                pubPcl(curPclPub, pts);

                pts->clear();
                lio->getLocMap(pts);
                pubPcl(locMapPub, pts);

                tmpRcvLioFlg = rcvLioFlg;
            }

            if (tmpRcvBkdFlg != pa->bkdOutFlg)
            {
                BACKEND::BkdOutData data;

                bkd->getOutData(data);
                pubPath(gloPthPub, data.pth);
                pubPath(gpsPthPub, data.gpsPth);

                if (tmpTs + 10. < lioCurTs)
                {
                    PointCloudXYZI::Ptr map(new PointCloudXYZI);
                    bkd->getGloMap(map);
                    pubPcl(gloMapPub, map); 
                    tmpTs = lioCurTs;
                }

                tmpRcvBkdFlg = pa->bkdOutFlg;
            }

            rate.sleep();
        }
    }

    void pubPose()
    {
        
    }

    void pubLocPath()
    {
        geometry_msgs::PoseStamped poseMsg;
        poseMsg.pose = convert2ROSPose(lioBdyPos.so3(), lioBdyPos.translation());
        poseMsg.header.stamp = ros::Time().fromSec(0.0);
        poseMsg.header.frame_id = pa->worldCoord;
        locPth.poses.push_back(poseMsg);
        locPthPub.publish(locPth);
    }

    void pubPath(ros::Publisher &pub, vector<Sophus::SE3> &pth)
    {
        nav_msgs::Path  msgPth;
        msgPth.header.stamp = ros::Time::now();
        msgPth.header.frame_id = pa->worldCoord;
        geometry_msgs::PoseStamped poseMsg;
        for (auto &p : pth)
        {
            poseMsg.pose = convert2ROSPose(p.so3(), p.translation());
            poseMsg.header.stamp = ros::Time().fromSec(0.0);
            poseMsg.header.frame_id = pa->worldCoord;
            msgPth.poses.push_back(poseMsg);
        }

        pub.publish(msgPth);
    }

    void pubPcl(ros::Publisher &pub, PointCloudXYZI::Ptr &pts)
    {
        if (pts->points.empty())
            return;

        sensor_msgs::PointCloud2 lidMsg;
        pcl::toROSMsg(*pts, lidMsg);
        lidMsg.header.stamp = ros::Time().fromSec(0.0);
        lidMsg.header.frame_id = pa->worldCoord;
        pub.publish(lidMsg);
    }


};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ww_lio_map");

    LSLAM slam;

    ros::spin();
    
    return 0;
}
