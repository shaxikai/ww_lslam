#include "utility.h"
 
#define NUM_MATCH_POINT (5)
#define INI_IMU_COUNT (10)  //最大迭代次数
#define MAX_NUM_SIG_LID (100000)

using namespace std;

//判断点的时间先后顺序(注意curvature中存储的是时间戳)
const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

class LaserMapping
{

private:

    struct Param
    {
        string  lidTopic;
        string  imuTopic;

        bool    enPathPub;
        bool    enCurPclPub;
        bool    enDenPclPub;
        bool    enMapPub;

        string  pathTopic;
        string  curPclTopic;
        string  denPclTopic;
        string  mapTopic;

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
        double  minCornerFilterSize;            // VoxelGrid降采样时的体素大小
        double  minSurfFilterSize;
        double  minMapFilterSize;
        double  cubeLen;                        // 地图的局部区域的长度（FastLio2论文中有解释）

        int     mapSaveType;                      // 是否将点云地图保存到PCD文件
        string  mapSavePath;                    // 地图保存路径

        int     nPointFilter;                   // 采样间隔，即每隔point_filter_num个点取1个点
        bool    enEstExtrinsic;
        int     pcdSaveInterval;
        vector<double>  extrinT;                // 雷达相对于IMU的外参T（即雷达在IMU坐标系中的坐标）
        vector<double>  extrinR;                // 雷达相对于IMU的外参R

        //imu preint
        double  gravity;

        // eskf
        double  epsi;

        // map
        double  mapMovThr;

    };

    struct MeasureGroup     // Lidar data and imu dates for the current process
    {
        MeasureGroup()
        {
            lidBigTime = 0.0;
            this->lid.reset(new PointCloudXYZI());
        };
        double lidBigTime;
        double lidEndTime;
        PointCloudXYZI::Ptr lid;
        deque<sensor_msgs::Imu::ConstPtr> imu;
    };

    struct ImuData
    {
        V3D acc = Eigen::Vector3d(0,0,0);
        V3D gyr = Eigen::Vector3d(0,0,0);
    };

    struct PjtData
    {
        double offt;
        V3D vel = Eigen::Vector3d(0,0,0);
        Eigen::Vector3d pos = Eigen::Vector3d(0,0,0);
        Sophus::SO3 rot = Sophus::SO3(Eigen::Matrix3d::Identity());
    };

    struct StateVal
    {
        Eigen::Vector3d pos = Eigen::Vector3d(0,0,0);
        Sophus::SO3 rot = Sophus::SO3(Eigen::Matrix3d::Identity());
        Sophus::SO3 extrinR = Sophus::SO3(Eigen::Matrix3d::Identity());
        Eigen::Vector3d extrinT = Eigen::Vector3d(0,0,0);
        Eigen::Vector3d vel = Eigen::Vector3d(0,0,0);
        Eigen::Vector3d bg = Eigen::Vector3d(0,0,0);
        Eigen::Vector3d ba = Eigen::Vector3d(0,0,0);
        Eigen::Vector3d grav = Eigen::Vector3d(0,0,0);
    };
    
    struct EskfDynData
    {
        bool valid;												   //有效特征点数量是否满足要求
        bool converge;											   //迭代时，是否已经收敛     
        array<bool, MAX_NUM_SIG_LID> validPtsFlag;	       
        Eigen::Matrix<double, Eigen::Dynamic, 1> h;				   //残差	(公式(14)中的z)
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> h_x; //雅可比矩阵H (公式(14)中的H)
    };

    ros::NodeHandle nh;
    Param pa;

    ros::Subscriber subLid;             // 订阅原始激光点云
    ros::Subscriber subImu;             // imu数据队列（原始数据，转lidar系下）

    ros::Publisher  pubPath;
    ros::Publisher  pubCurPcl;
    ros::Publisher  pubDenPcl;
    ros::Publisher  pubMap;

    double curTime;

    std::deque<double> timeBuffer;     // 单位 s
    std::deque<PointCloudXYZI::Ptr> lidBuffer;
    std::deque<sensor_msgs::Imu::ConstPtr> imuBuffer;

    sensor_msgs::ImuConstPtr lastImu;
    double lastLidEndTime;

    bool imuInited;
    int imuCnt;
    V3D mean_acc;                           //加速度均值,用于计算方差
    V3D mean_gyr;                           //角速度均值，用于计算方差
    Eigen::Matrix<double, 12, 12> Q;

    StateVal X;
    COV P = COV::Identity();

    PointCloudXYZI::Ptr pclPjt;
    PointCloudXYZI::Ptr pclPjtDown;
    PointCloudXYZI::Ptr pclPjtDownWorld;
    PointCloudXYZI::Ptr pclPjtWorld;
    PointCloudXYZI::Ptr pclGlobal;

    bool Localmap_Initialized = false; // 局部地图是否初始化
    vector<PointVector> nearPclPts;
    pcl::VoxelGrid<PointType> downSizeFilterMap;

    nav_msgs::Path path;

    std::mutex imuLock;
    std::mutex lidLock;

public:
    LaserMapping()
    {
        readParm();

        reset();

        readIkdTreeMap();

        // 订阅原始imu数据
        subImu = nh.subscribe(pa.imuTopic, 2000, &LaserMapping::imuHandler, this, ros::TransportHints().tcpNoDelay());

        // 订阅原始lidar数据
        switch (pa.lidType)
        {
        case MID360:
        case AVIA:
            subLid = nh.subscribe<livox_ros_driver::CustomMsg>(
                     pa.lidTopic, 5, &LaserMapping::livoxLidHandler, this, ros::TransportHints().tcpNoDelay());
            break;
        case RS32:
            subLid = nh.subscribe<sensor_msgs::PointCloud2>(
                     pa.lidTopic, 5, &LaserMapping::rsLidHandler, this, ros::TransportHints().tcpNoDelay());
        default:
            break;
        }

        pubPath     = nh.advertise<nav_msgs::Path>(pa.pathTopic, 100000);
        pubCurPcl   = nh.advertise<sensor_msgs::PointCloud2>(pa.curPclTopic, 100000);
        pubDenPcl   = nh.advertise<sensor_msgs::PointCloud2>(pa.denPclTopic, 100000);
        pubMap      = nh.advertise<sensor_msgs::PointCloud2>(pa.mapTopic, 100000);

    }

    void reset()
    {
        imuInited = false;
        imuCnt    = 1;

        Q = Eigen::MatrixXd::Zero(12, 12);            
        Q.block<3, 3>(0, 0).diagonal() = V3D(pa.gyrCov, pa.gyrCov, pa.gyrCov);
        Q.block<3, 3>(3, 3).diagonal() = V3D(pa.accCov, pa.accCov, pa.accCov);
        Q.block<3, 3>(6, 6).diagonal() = V3D(pa.bGyrCov, pa.bGyrCov, pa.bGyrCov);
        Q.block<3, 3>(9, 9).diagonal() = V3D(pa.bAccCov, pa.bAccCov, pa.bAccCov);

        P = Eigen::MatrixXd::Identity(24,24);      //在esekfom.hpp获得P_的协方差矩阵
        P(6,6) = P(7,7) = P(8,8) = 0.00001;
        P(9,9) = P(10,10) = P(11,11) = 0.00001;
        P(15,15) = P(16,16) = P(17,17) = 0.0001;
        P(18,18) = P(19,19) = P(20,20) = 0.001;
        P(21,21) = P(22,22) = P(23,23) = 0.00001; 

        X.grav = Eigen::Vector3d(0,0,-pa.gravity);
        X.extrinT << VEC_FROM_ARRAY(pa.extrinT);
        Eigen::Matrix3d extrinR = Eigen::Matrix3d::Identity();
        extrinR << MAT_FROM_ARRAY(pa.extrinR);
        X.extrinR = Sophus::SO3(extrinR);

        pclPjt.reset(new PointCloudXYZI()); 
        pclPjtDown.reset(new PointCloudXYZI());
        pclPjtDownWorld.reset(new PointCloudXYZI()); 
        pclPjtWorld.reset(new PointCloudXYZI()); 
        pclGlobal.reset(new PointCloudXYZI()); 

        ikdtree.set_downsample_param(pa.minMapFilterSize);
        downSizeFilterMap.setLeafSize(pa.minMapFilterSize, pa.minMapFilterSize, pa.minMapFilterSize);

        path.header.stamp = ros::Time::now();
        path.header.frame_id = pa.worldCoord;
    }

    void readParm()
    {
        nh.param<string>("common/coordinate", pa.worldCoord, "camera_init");         // 雷达点云topic名称

        nh.param<string>("common/lid_topic", pa.lidTopic, "/livox/lidar");         // 雷达点云topic名称
        nh.param<string>("common/imu_topic", pa.imuTopic, "/livox/imu");           // IMU的topic名称

        nh.param<bool>("publish/path_pub_en", pa.enPathPub, true);
        nh.param<bool>("publish/cur_pcl_pub_en", pa.enCurPclPub, true);
        nh.param<bool>("publish/den_pcl_pub_en", pa.enDenPclPub, true);
        nh.param<bool>("publish/map_pub_en", pa.enMapPub, true);

        nh.param<string>("publish/path_topic", pa.pathTopic, "/path");
        nh.param<string>("publish/cur_pcl_topic", pa.curPclTopic, "/cur_pcl");
        nh.param<string>("publish/den_pcl_topic", pa.denPclTopic, "/den_pcl");
        nh.param<string>("publish/map_topic", pa.mapTopic, "/map");

        nh.param<int>("common/lidar_type", pa.lidType, AVIA); // 激光雷达的类型
        nh.param<double>("common/blind", pa.blind, 0.01);        // 最小距离阈值，即过滤掉0～blind范围内的点云
        nh.param<int>("common/scan_line", pa.N_SCANS, 16);       // 激光雷达扫描的线数（livox avia为6线）
        nh.param<double>("common/det_range", pa.detRange, 300.f); // 激光雷达的最大探测范围
        nh.param<double>("mapping/fov_degree", pa.fovDeg, 180);
        nh.param<int>("common/scan_rate", pa.SCAN_RATE, 10);
        nh.param<int>("common/point_filter_num", pa.nPointFilter, 2);           // 采样间隔，即每隔point_filter_num个点取1个点

        nh.param<double>("mapping/epsi", pa.epsi, 0.001);
        nh.param<double>("mapping/g_world", pa.gravity, 9.81);

        nh.param<double>("mapping/gyr_cov", pa.gyrCov, 0.1);               // IMU陀螺仪的协方差
        nh.param<double>("mapping/acc_cov", pa.accCov, 0.1);               // IMU加速度计的协方差
        nh.param<double>("mapping/b_gyr_cov", pa.bGyrCov, 0.0001);        // IMU陀螺仪偏置的协方差
        nh.param<double>("mapping/b_acc_cov", pa.bAccCov, 0.0001);        // IMU加速度计偏置的协方差
        nh.param<double>("mapping/lid_point_cov", pa.lidPtCov, 0.001);        // IMU加速度计偏置的协方差

        nh.param<int>("mapping/max_iteration", pa.nMaxIter, 3);                   // 卡尔曼滤波的最大迭代次数
        nh.param<double>("mapping/filter_size_map", pa.minMapFilterSize, 0.5);
        nh.param<double>("mapping/map_move_thr", pa.mapMovThr, 1.5);
        nh.param<double>("mapping/cube_side_length", pa.cubeLen, 200);    // 地图的局部区域的长度（FastLio2论文中有解释）

        nh.param<vector<double>>("mapping/extrinsic_T", pa.extrinT, vector<double>(3, 0.0)); // 雷达相对于IMU的外参T（即雷达在IMU坐标系中的坐标）
        nh.param<vector<double>>("mapping/extrinsic_R", pa.extrinR, vector<double>(9, 0.0)); // 雷达相对于IMU的外参R

        nh.param<int>("save/map_save_type", pa.mapSaveType, 0); // 是否将点云地图保存到PCD文件
        nh.param<string>("save/map_save_path", pa.mapSavePath, "./PCD/");                    // 地图保存路径
    }

    void livoxLidHandler(const livox_ros_driver::CustomMsg::ConstPtr &msg)
    {
        PointCloudXYZI::Ptr pclPtr(new PointCloudXYZI());
        PointType pt, lastPt;
        int nPts = msg->point_num;
        uint gap = pa.nPointFilter;
        for (uint i = 1; i < nPts; i+=gap)
        {
            auto &msgPt = msg->points[i];
            if ((msgPt.line < pa.N_SCANS) 
            && ((msgPt.tag & 0x30) == 0x10 || (msgPt.tag & 0x30) == 0x00)
            && ((abs(msgPt.x) > 1e-7) || (abs(msgPt.y) > 1e-7) || (abs(msgPt.z) > 1e-7)))
            {
                pt.x = msgPt.x;
                pt.y = msgPt.y;
                pt.z = msgPt.z;
                pt.intensity = msgPt.reflectivity;
                pt.curvature = msgPt.offset_time * 1e-9; 

                if ((abs(pt.x - lastPt.x) > 1e-7) || (abs(pt.y - lastPt.y) > 1e-7) || (abs(pt.z - lastPt.z) > 1e-7) 
                && (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z > (pa.blind * pa.blind)))
                {
                    pclPtr->push_back(pt);
                    lastPt = pt;
                }
            }
        }

        std::lock_guard<std::mutex> lock1(lidLock);
        lidBuffer.push_back(pclPtr);
        timeBuffer.push_back(msg->header.stamp.toSec());
    }

    void rsLidHandler(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        pcl::PointCloud<rslidar_ros::Point> rsPts;
        pcl::fromROSMsg(*msg, rsPts);
        int nPts = rsPts.points.size();

        PointCloudXYZI::Ptr pclPtr(new PointCloudXYZI());
        PointType pt, lastPt;
        uint gap = pa.nPointFilter;
        for (uint i = 0; i < nPts; i+=gap)
        {
            if (rsPts.points[i].ring < pa.N_SCANS)
            {
                pt.x = rsPts.points[i].x;
                pt.y = rsPts.points[i].y;
                pt.z = rsPts.points[i].z;
                pt.intensity = rsPts.points[i].intensity;
                pt.curvature = rsPts.points[i].timestamp - rsPts.points[0].timestamp; 

                if ((abs(pt.x - lastPt.x) > 1e-7) || (abs(pt.y - lastPt.y) > 1e-7) || (abs(pt.z - lastPt.z) > 1e-7) 
                && (pt.x * pt.x + pt.y * pt.y + pt.z * pt.z > (pa.blind * pa.blind)))
                {
                    pclPtr->push_back(pt);
                    lastPt = pt;
                }
            }
        }

        std::lock_guard<std::mutex> lock1(lidLock);
        lidBuffer.push_back(pclPtr);
        timeBuffer.push_back(msg->header.stamp.toSec());
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        imuBuffer.push_back(msg);
    }

    void processThread()
    {
        ros::Rate rate(5000);

        bool state = false;
        while (ros::ok())
        {
            state = process();
            if (!state) break;
            rate.sleep();
        }
    }

    bool process()
    {
        bool state = true;

        ASSERT(ikdtree.Root_Node != nullptr, "no map!")
        
        MeasureGroup measDate;
        state = syncPackages(measDate);
        if (!state) return true;

        if (!imuInited)
        {
            imuInited = imuInit(measDate);
            return true;
        }

        vector<PjtData> pjtData;
        imuPreint(measDate, pjtData);
        projectPcl(measDate, pjtData, pclPjt);

        V3D lidPos = X.pos + X.rot.matrix() * X.extrinT;
        cerr << "prei pose : " << X.pos(0) << " " << X.pos(1) << " " << X.pos(2) << endl;
        lasermap_fov_segment(lidPos);

        //点云下采样
        downSizeFilterMap.setInputCloud(pclPjt);
        downSizeFilterMap.filter(*pclPjtDown);
        int npts  = pclPjtDown->points.size();

        pclPjtDownWorld->resize(npts);
        ptsBody2World(pclPjtDown, pclPjtDownWorld);

        nearPclPts.resize(npts);
        eskfUpdte(pclPjtDown);
        cerr << "eskf pose : " << X.pos(0) << " " << X.pos(1) << " " << X.pos(2) << endl;

        ptsBody2World(pclPjt, pclPjtWorld);
        publishStatus();

        return true;
    }

    bool readIkdTreeMap()
    {
        string mapName;
        PointCloudXYZI::Ptr mapPcl(new PointCloudXYZI());
        if (1 == pa.mapSaveType)
        {
            mapName = "ikdTreeMap.pcd";
        }
        else
        {
            mapName = "denseMap.pcd";
        }

        pcl::PCDReader pcdReader;
        string saveMapPN = pa.mapSavePath + mapName;
        ASSERT(pcdReader.read(saveMapPN, *mapPcl) != -1, "no map!");
        ikdtree.Build(mapPcl->points);

        return true;
    }

    bool syncPackages(MeasureGroup &meas)
    {
        if (lidBuffer.size() <= 2) return false;

        PointCloudXYZI::Ptr lid = lidBuffer.front();
        double lidBigTime = timeBuffer.front();
        double lidEndTime = lidBigTime + lid->points.back().curvature;

        // 要求imu数据包含激光数据，否则不往下处理了
        if (imuBuffer.front()->header.stamp.toSec() > lidBigTime)
        {
            lidBuffer.pop_front();
            timeBuffer.pop_front();
            return false;
        }

        if (imuBuffer.empty() || imuBuffer.back()->header.stamp.toSec() < lidEndTime)
        {
            ROS_DEBUG("Waiting for IMU data ...");
            return false;
        }

        // 从imu队列中删除当前激光帧前面时刻的imu数据
        while (!imuBuffer.empty())
        {
            if (imuBuffer.front()->header.stamp.toSec() < lidBigTime)
                imuBuffer.pop_front();
            else
                break;
        }

        if (imuBuffer.empty())
            return false;

        // 遍历当前激光帧起止时刻之间的imu数据
        meas.imu.clear();
        for (int i = 0; i < (int)imuBuffer.size(); ++i)
        {
            sensor_msgs::Imu::ConstPtr thisImu = imuBuffer[i];
            double imuTime = thisImu->header.stamp.toSec();

            // 超过当前激光帧结束时刻，结束
            if (imuTime > lidEndTime)
                break;

            meas.imu.push_back(thisImu);
        }

        curTime = lidEndTime;
        meas.lidBigTime = lidBigTime;
        meas.lidEndTime = lidEndTime;
        meas.lid = lid;
        lidBuffer.pop_front();
        timeBuffer.pop_front();

        std::cerr << std::setprecision(15) 
        << lidBuffer.size() << " "
        << lidBigTime << " " << lidEndTime << " " 
        << meas.imu.front()->header.stamp.toSec() << " "
        << meas.imu.back()->header.stamp.toSec() << std::endl;

        return true;
    }

    StateVal StatePlus(StateVal x, Eigen::Matrix<double, 24, 1> dx)
    {
        StateVal x_r;
        x_r.pos = x.pos + dx.block<3, 1>(0, 0);

        x_r.rot = x.rot * Sophus::SO3::exp(dx.block<3, 1>(3, 0));
        x_r.extrinR = x.extrinR * Sophus::SO3::exp(dx.block<3, 1>(6, 0));

        x_r.extrinT = x.extrinT + dx.block<3, 1>(9, 0);
        x_r.vel = x.vel + dx.block<3, 1>(12, 0);
        x_r.bg = x.bg + dx.block<3, 1>(15, 0);
        x_r.ba = x.ba + dx.block<3, 1>(18, 0);
        x_r.grav = x.grav + dx.block<3, 1>(21, 0);

        return x_r;
    }

    bool imuInit(const MeasureGroup &meas)
    {
        if(meas.imu.empty()) return false;

        //MeasureGroup这个struct表示当前过程中正在处理的所有数据，包含IMU队列和一帧lidar的点云 以及lidar的起始和结束时间
        //初始化重力、陀螺仪偏差、acc和陀螺仪协方差  将加速度测量值归一化为单位重力   **/
        const auto &imu_acc = meas.imu.front()->linear_acceleration;    //IMU初始时刻的加速度
        const auto &gyr_acc = meas.imu.front()->angular_velocity;       //IMU初始时刻的角速度

        if (1 == imuCnt)
        {
            mean_acc << imu_acc.x, imu_acc.y, imu_acc.z;              //第一帧加速度值作为初始化均值
            mean_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;              //第一帧角速度值作为初始化均值
        }

        V3D cur_acc, cur_gyr;
        for (const auto &imu : meas.imu)    //根据所有IMU数据，计算平均值和方差
        {
            const auto &imu_acc = imu->linear_acceleration;
            const auto &gyr_acc = imu->angular_velocity;
            cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
            cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

            mean_acc  += (cur_acc - mean_acc) / imuCnt;    //根据当前帧和均值差作为均值的更新
            mean_gyr  += (cur_gyr - mean_gyr) / imuCnt;
            imuCnt++;
        }


        if (imuCnt > INI_IMU_COUNT)
        {
            X.grav = - mean_acc / mean_acc.norm() * pa.gravity;    //得平均测量的单位方向向量 * 重力加速度预设值
            X.bg   = mean_gyr;      //角速度测量作为陀螺仪偏差
            return true;
        }

        return false;
    }

    void imuUnitPreint(double &dt, Eigen::Matrix<double, 12, 12> &Q, const ImuData &in)
    {
        // 对应顺序为速度(3)，角速度(3),外参T(3),外参旋转R(3)，加速度(3),角速度偏置(3),加速度偏置(3),位置(3)，与论文公式顺序不一致
        Eigen::Matrix<double, 24, 1> f = Eigen::Matrix<double, 24, 1>::Zero();	  //公式(3)的f
        V3D omega = in.gyr - X.bg;		// 输入的imu的角速度(也就是实际测量值) - 估计的bias值(对应公式的第1行)
        V3D a_inertial = X.rot.matrix() * (in.acc - X.ba);		//  输入的imu的加速度，先转到世界坐标系（对应公式的第3行）
        for (int i = 0; i < 3; i++)
        {
            f(i) = X.vel[i];		//速度（对应公式第2行）
            f(i + 3) = omega[i];	//角速度（对应公式第1行）
            f(i + 12) = a_inertial[i] + X.grav[i];		//加速度（对应公式第3行）
        }

        //对应公式(7)的Fx  注意该矩阵没乘dt，没加单位阵
        Eigen::Matrix<double, 24, 24> fx = Eigen::Matrix<double, 24, 24>::Zero();
        fx.block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();	//对应公式(7)第2行第3列   I
        V3D acc_ = in.acc - X.ba;   	//测量加速度 = a_m - bias	

        fx.block<3, 3>(12, 3) = -X.rot.matrix() * Sophus::SO3::hat(acc_);		//对应公式(7)第3行第1列
        fx.block<3, 3>(12, 18) = -X.rot.matrix(); 				//对应公式(7)第3行第5列 

        fx.template block<3, 3>(12, 21) = Eigen::Matrix3d::Identity();		//对应公式(7)第3行第6列   I
        fx.template block<3, 3>(3, 15) = -Eigen::Matrix3d::Identity();		//对应公式(7)第1行第4列 (简化为-I)

        //对应公式(7)的Fw  注意该矩阵没乘dt
        Eigen::Matrix<double, 24, 12> fw = Eigen::Matrix<double, 24, 12>::Zero();
        fw.block<3, 3>(12, 3) = -X.rot.matrix();					//对应公式(7)第3行第2列  -R 
        fw.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();		//对应公式(7)第1行第1列  -A(w dt)简化为-I
        fw.block<3, 3>(15, 6) = Eigen::Matrix3d::Identity();		//对应公式(7)第4行第3列  I
        fw.block<3, 3>(18, 9) = Eigen::Matrix3d::Identity();		//对应公式(7)第5行第4列  I

        X = StatePlus(X, f * dt); //前向传播 公式(4)
        // std::cout << "P: " << P << std::endl;
        // std::cout << "Q: " << Q << std::endl;
        fx = Eigen::Matrix<double, 24, 24>::Identity() + fx * dt; //之前Fx矩阵里的项没加单位阵，没乘dt   这里补上
        P = (fx) * P * (fx).transpose() + (dt * fw) * Q * (dt * fw).transpose(); //传播协方差矩阵，即公式(8)
        // std::cout << "dt: " << dt << std::endl;
        // std::cout << "fx: " << fx << std::endl;
        // std::cout << "fw: " << fx << std::endl;
        // getchar();
    }

    void imuPreint(const MeasureGroup &meas, vector<PjtData> &pjtData)
    {
        deque<sensor_msgs::Imu::ConstPtr> v_imu = meas.imu;
        const double &imu_end_time = v_imu.back()->header.stamp.toSec();
        const double &pcl_beg_time = meas.lidBigTime;

        pjtData.clear();
        pjtData.push_back({0.0, X.vel, X.pos, X.rot});

        double dt = 0;
        ImuData in;
        V3D angvel_avr, acc_avr;
        for  (size_t i = 0; i < v_imu.size(); i++)
        {

            sensor_msgs::Imu::ConstPtr tail = v_imu[i];
            if (i == 0)
            {
                angvel_avr << tail->angular_velocity.x, tail->angular_velocity.y, tail->angular_velocity.z;
                acc_avr << tail->linear_acceleration.x, tail->linear_acceleration.y, tail->linear_acceleration.z;
                dt = tail->header.stamp.toSec() - pcl_beg_time;
            }
            else
            {
                sensor_msgs::Imu::ConstPtr head = v_imu[i - 1];
                angvel_avr<<0.5 * (head->angular_velocity.x + tail->angular_velocity.x),      // 中值积分
                            0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                            0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
                acc_avr   <<0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
                            0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                            0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);
                dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
            }
            acc_avr  = acc_avr * pa.gravity / mean_acc.norm();

            in.acc = acc_avr;
            in.gyr = angvel_avr;

            imuUnitPreint(dt, Q, in);

            double &&offt = tail->header.stamp.toSec() - pcl_beg_time;    //后一个IMU时刻距离此次雷达开始的时间间隔
            pjtData.push_back({offt, X.vel, X.pos, X.rot});
        }

        dt = abs(meas.lidEndTime - v_imu.back()->header.stamp.toSec());
        imuUnitPreint(dt, Q, in);

    }

    void projectPcl(const MeasureGroup &meas, vector<PjtData> &pjtData, PointCloudXYZI::Ptr &pclPjt)
    {
        // 根据点云中每个点的时间戳对点云进行重排序
        PointCloudXYZI::Ptr lid= meas.lid;
        sort(lid->points.begin(), lid->points.end(), time_list);  //这里curvature中存放了时间戳（在preprocess.cpp中）

        //遍历每个IMU帧
        auto it_pcl = lid->points.end() - 1;
        for (auto it_kp = pjtData.end() - 1; it_kp != pjtData.begin(); it_kp--)
        {
            auto head = it_kp - 1;
            auto tail = it_kp;
            M3D rot_imu = head->rot.matrix();   //拿到前一帧的IMU旋转矩阵
            // cout<<"head imu acc: "<<acc_imu.transpose()<<endl;
            V3D vel_imu = head->vel;     //拿到前一帧的IMU速度
            V3D pos_imu = head->pos;     //拿到前一帧的IMU位置          

            //之前点云按照时间从小到大排序过，IMUpose也同样是按照时间从小到大push进入的
            //此时从IMUpose的末尾开始循环，也就是从时间最大处开始，因此只需要判断 点云时间需>IMU head时刻  即可   不需要判断 点云时间<IMU tail
            for(; it_pcl->curvature > head->offt; it_pcl --)
            {
                double dt = it_pcl->curvature - head->offt;    //点到IMU开始时刻的时间间隔 
                V3D P_i(it_pcl->x, it_pcl->y, it_pcl->z);   //点所在时刻的位置(雷达坐标系下)
                V3D T_ei(pos_imu + vel_imu * dt - X.pos);   //从点所在的世界位置-雷达末尾世界位置
                V3D P_compensate = X.extrinR.matrix().transpose() * 
                (X.rot.matrix().transpose() * (rot_imu * (X.extrinR.matrix() * P_i + X.extrinT) + T_ei) - X.extrinT);

                it_pcl->x = P_compensate(0);
                it_pcl->y = P_compensate(1);
                it_pcl->z = P_compensate(2);

                if (it_pcl == lid->points.begin()) break;
            }
        }

        pclPjt = meas.lid;
    }

    void lasermap_fov_segment(V3D pos_lid)
    {
        vector<BoxPointType> cub_needrm;
        BoxPointType LocalMap_Points;      // ikd-tree地图立方体的2个角点
        cub_needrm.clear(); // 清空需要移除的区域

        //初始化局部地图范围，以pos_LiD为中心,长宽高均为pa.cubeLen
        if (!Localmap_Initialized)
        {
            for (int i = 0; i < 3; i++)
            {
                LocalMap_Points.vertex_min[i] = pos_lid(i) - pa.cubeLen / 2.0;
                LocalMap_Points.vertex_max[i] = pos_lid(i) + pa.cubeLen / 2.0;
            }
            Localmap_Initialized = true;
            return;
        }

        //各个方向上pos_LiD与局部地图边界的距离
        float dist_to_map_edge[3][2];
        bool need_move = false;
        for (int i = 0; i < 3; i++)
        {
            dist_to_map_edge[i][0] = fabs(pos_lid(i) - LocalMap_Points.vertex_min[i]);
            dist_to_map_edge[i][1] = fabs(pos_lid(i) - LocalMap_Points.vertex_max[i]);
            // 与某个方向上的边界距离（1.5*300m）太小，标记需要移除need_move(FAST-LIO2论文Fig.3
            if (dist_to_map_edge[i][0] <= pa.mapMovThr * pa.detRange || dist_to_map_edge[i][1] <= pa.mapMovThr * pa.detRange)
                need_move = true;
        }
        if (!need_move)
            return; //如果不需要，直接返回，不更改局部地图

        BoxPointType New_LocalMap_Points, tmp_boxpoints;
        New_LocalMap_Points = LocalMap_Points;
        //需要移动的距离
        float mov_dist = max((pa.cubeLen - 2.0 * pa.mapMovThr * pa.detRange) * 0.5 * 0.9, double(pa.detRange * (pa.mapMovThr - 1)));
        for (int i = 0; i < 3; i++)
        {
            tmp_boxpoints = LocalMap_Points;
            if (dist_to_map_edge[i][0] <= pa.mapMovThr * pa.detRange)
            {
                New_LocalMap_Points.vertex_max[i] -= mov_dist;
                New_LocalMap_Points.vertex_min[i] -= mov_dist;
                tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
                cub_needrm.push_back(tmp_boxpoints);
            }
            else if (dist_to_map_edge[i][1] <= pa.mapMovThr * pa.detRange)
            {
                New_LocalMap_Points.vertex_max[i] += mov_dist;
                New_LocalMap_Points.vertex_min[i] += mov_dist;
                tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
                cub_needrm.push_back(tmp_boxpoints);
            }
        }
        LocalMap_Points = New_LocalMap_Points;

        PointVector points_history;
        ikdtree.acquire_removed_points(points_history);

        if (cub_needrm.size() > 0)
            int count = ikdtree.Delete_Point_Boxes(cub_needrm); //删除指定范围内的点
    }

    void ptBody2World(PointType const *const pi, PointType *const po)
    {
        V3D p_body(pi->x, pi->y, pi->z);
        V3D p_global(X.rot.matrix() * (X.extrinR.matrix() * p_body + X.extrinT) + X.pos);

        po->x = p_global(0);
        po->y = p_global(1);
        po->z = p_global(2);
        po->intensity = pi->intensity;
    }

    void ptsBody2World(PointCloudXYZI::Ptr &pclBody, PointCloudXYZI::Ptr &pclWorld)
    {
        int npts = pclBody->points.size();
        pclWorld->resize(npts);
        for (int i = 0; i < npts; i++)
        {
            ptBody2World(&(pclBody->points[i]), &(pclWorld->points[i])); // lidar坐标系转到世界坐标系
        }
    }

    void eskfUpdte(PointCloudXYZI::Ptr &lid)
    {
        EskfDynData eskfData;
        eskfData.valid = true;
        eskfData.converge = true;

        int convergeCnt = 0;
        StateVal x_propagated = X; //这里的x_和P_分别是经过正向传播后的状态量和协方差矩阵，因为会先调用predict函数再调用这个函数
        COV P_propagated = P;
        StateV24 dx_new = StateV24::Zero(); // 24X1的向量
        for (int i = -1; i < pa.nMaxIter; i++) // maximum_iter是卡尔曼滤波的最大迭代次数
        {
            // 计算雅克比，也就是点面残差的导数 H(代码里是h_x)
            eskfData.valid = true;
            h_share_model(lid, eskfData);

            // 没有有效的近邻点
            if (!eskfData.valid) return;

            StateV24 dx;
            dx_new = boxminus(X, x_propagated); //公式(18)中的 x^k - x^

            //由于H矩阵是稀疏的，只有前12列有非零元素，后12列是零 因此这里采用分块矩阵的形式计算 减少计算量
            auto H = eskfData.h_x;												// m X 12 的矩阵
            Eigen::Matrix<double, 24, 24> HTH = Eigen::Matrix<double, 24, 24>::Zero(); //矩阵 H^T * H
            HTH.block<12, 12>(0, 0) = H.transpose() * H;
            auto K_front = (HTH / pa.lidPtCov + P.inverse()).inverse();
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> K;
            K = K_front.block<24, 12>(0, 0) * H.transpose() / pa.lidPtCov; //卡尔曼增益  这里R视为常数

            Eigen::Matrix<double, 24, 24> KH = Eigen::Matrix<double, 24, 24>::Zero(); //矩阵 K * H
            KH.block<24, 12>(0, 0) = K * H;
            Eigen::Matrix<double, 24, 1> dx_ = K * eskfData.h + (KH - Eigen::Matrix<double, 24, 24>::Identity()) * dx_new; //公式(18)
            X = StatePlus(X, dx_); //公式(18)

            //std::cout << "dx_: " << dx_[0] << " " << dx_[1] << " " << dx_[2] << std::endl;
            //getchar();

            eskfData.converge = true;
            for (int j = 0; j < 24; j++)
            {
                if (std::fabs(dx_[j]) > pa.epsi) //如果dx>epsi 认为没有收敛
                {
                    eskfData.converge = false;
                    break;
                }
            }

            if (eskfData.converge)
                convergeCnt++;

            if (0 == convergeCnt && i == pa.nMaxIter - 2) //如果迭代了3次还没收敛 强制令成true，h_share_model函数中会重新寻找近邻点
            {
                eskfData.converge = true;
            }

            if (convergeCnt > 1 || i == pa.nMaxIter - 1)
            {
                P = (Eigen::Matrix<double, 24, 24>::Identity() - KH) * P; //公式(19)
                return;
            }
        }
    }

    //计算每个特征点的残差及H矩阵
    void h_share_model(PointCloudXYZI::Ptr &lid, EskfDynData &eskfData)
    {
        auto &validPtsFlag = eskfData.validPtsFlag;
        int npts = lid->points.size();

        PointCloudXYZI::Ptr validPts(new PointCloudXYZI(MAX_NUM_SIG_LID, 1));  //有效特征点
	    PointCloudXYZI::Ptr norms(new PointCloudXYZI(MAX_NUM_SIG_LID, 1));	   //特征点在地图中对应的平面参数(平面的单位法向量,以及当前点到平面距离)
        validPts->resize(npts);
        norms->resize(npts);

// #ifdef MP_EN
//         omp_set_num_threads(MP_PROC_NUM);
// #pragma omp parallel for
// #endif
        
        int nValid = 0; //有效特征点的数量
        for (int i = 0; i < npts; i++) //遍历所有的特征点
        {
            PointType &point_body = lid->points[i];
            PointType point_world;

            V3D p_body(point_body.x, point_body.y, point_body.z);

            //把Lidar坐标系的点先转到IMU坐标系，再根据前向传播估计的位姿x，转到世界坐标系
            V3D p_global(X.rot * (X.extrinR * p_body + X.extrinT) + X.pos);
            point_world.x = p_global(0);
            point_world.y = p_global(1);
            point_world.z = p_global(2);
            point_world.intensity = point_body.intensity;

            vector<float> pointSearchSqDis(NUM_MATCH_POINT);
            auto &points_near = nearPclPts[i]; // nearPclPts[i]打印出来发现是按照离point_world距离，从小到大的顺序的vector

            if (eskfData.converge)
            {
                //寻找point_world的最近邻的平面点
                ikdtree.Nearest_Search(point_world, NUM_MATCH_POINT, points_near, pointSearchSqDis);
                //判断是否是有效匹配点，与loam系列类似，要求特征点最近邻的地图点数量>阈值，距离<阈值  满足条件的才置为true
                validPtsFlag[i] = points_near.size() < NUM_MATCH_POINT ? 
                false : pointSearchSqDis[NUM_MATCH_POINT - 1] > 5 ? false : true;
            }
            
            if (!validPtsFlag[i])
                continue;

            //拟合平面方程ax+by+cz+d=0并求解点到平面距离
            array<float, 4> pabcd;		//平面点信息
            if (estiPlane(pabcd, points_near, 0.1f))
            {
                float pd2 = pabcd[0] * point_world.x + pabcd[1] * point_world.y + pabcd[2] * point_world.z + pabcd[3]; //当前点到平面的距离
                float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());												   //如果残差大于经验阈值，则认为该点是有效点  简言之，距离原点越近的lidar点  要求点到平面的距离越苛刻

                if (s > 0.9) //如果残差大于阈值，则认为该点是有效点
                {   
                    validPtsFlag[i] = true;
                    validPts->points[nValid] = lid->points[i];
                    norms->points[nValid].x = pabcd[0]; //存储平面的单位法向量  以及当前点到平面距离
                    norms->points[nValid].y = pabcd[1];
                    norms->points[nValid].z = pabcd[2];
                    norms->points[nValid].intensity = pd2;
                    nValid++;
                }
                else
                {
                    validPtsFlag[i] = false;
                }
            }
        }

        if (nValid < 10)
        {
            eskfData.valid = false;
            ROS_WARN("No Effective Points! \n");
            return;
        }

        // 雅可比矩阵H和残差向量的计算
        eskfData.h_x = Eigen::MatrixXd::Zero(nValid, 12);
        eskfData.h.resize(nValid);
        for (int i = 0; i < nValid; i++)
        {
            V3D pt(validPts->points[i].x, validPts->points[i].y, validPts->points[i].z);
            M3D point_crossmat;
            point_crossmat << SKEW_SYM_MATRX(pt);
            V3D point_I_ = X.extrinR * pt + X.extrinT;
            M3D point_I_crossmat;
            point_I_crossmat << SKEW_SYM_MATRX(point_I_);

            // 得到对应的平面的法向量
            const PointType &norm_p = norms->points[i];
            V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

            // 计算雅可比矩阵H
            V3D C(X.rot.matrix().transpose() * norm_vec);
            V3D A(point_I_crossmat * C);
            eskfData.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

            //残差：点面距离
            eskfData.h(i) = -norm_p.intensity;
        }
    }

    //广义减法
    StateV24 boxminus(StateVal x1, StateVal x2)
    {
        StateV24 x_r = StateV24::Zero();

        x_r.block<3, 1>(0, 0) = x1.pos - x2.pos;

        x_r.block<3, 1>(3, 0) = Sophus::SO3(x2.rot.matrix().transpose() * x1.rot.matrix()).log();
        x_r.block<3, 1>(6, 0) = Sophus::SO3(x2.extrinR.matrix().transpose() * x1.extrinR.matrix()).log();

        x_r.block<3, 1>(9, 0) = x1.extrinT - x2.extrinT;
        x_r.block<3, 1>(12, 0) = x1.vel - x2.vel;
        x_r.block<3, 1>(15, 0) = x1.bg - x2.bg;
        x_r.block<3, 1>(18, 0) = x1.ba - x2.ba;
        x_r.block<3, 1>(21, 0) = x1.grav - x2.grav;

        return x_r;
    }

    void map_incremental(PointCloudXYZI::Ptr &lid)
    {
        PointVector PointToAdd;
        PointVector PointNoNeedDownsample;
        int npts = lid->points.size();
        PointToAdd.reserve(npts);
        PointNoNeedDownsample.reserve(npts);
        for (int i = 0; i < npts; i++)
        {
            if (!nearPclPts[i].empty())
            {
                //cerr << 0;
                const PointVector &points_near = nearPclPts[i];
                bool need_add = true;
                BoxPointType Box_of_Point;
                PointType mid_point; //点所在体素的中心
                mid_point.x = floor(lid->points[i].x / pa.minMapFilterSize) * pa.minMapFilterSize + 0.5 * pa.minMapFilterSize;
                mid_point.y = floor(lid->points[i].y / pa.minMapFilterSize) * pa.minMapFilterSize + 0.5 * pa.minMapFilterSize;
                mid_point.z = floor(lid->points[i].z / pa.minMapFilterSize) * pa.minMapFilterSize + 0.5 * pa.minMapFilterSize;
                float dist = calc_dist(lid->points[i], mid_point);
                if (fabs(points_near[0].x - mid_point.x) > 0.5 * pa.minMapFilterSize 
                && fabs(points_near[0].y - mid_point.y) > 0.5 * pa.minMapFilterSize 
                && fabs(points_near[0].z - mid_point.z) > 0.5 * pa.minMapFilterSize)
                {
                    PointNoNeedDownsample.push_back(lid->points[i]); //如果距离最近的点都在体素外，则该点不需要Downsample
                    continue;
                }
                for (int j = 0; j < NUM_MATCH_POINT; j++)
                {
                    if (points_near.size() < NUM_MATCH_POINT)
                        break;
                    if (calc_dist(points_near[j], mid_point) < dist) //如果近邻点距离 < 当前点距离，不添加该点
                    {
                        need_add = false;
                        break;
                    }
                }
                if (need_add)
                {
                    //cerr << 1;
                    PointToAdd.push_back(lid->points[i]);
                }
                else
                {
                    //cerr << 2;
                }
            }
            else
            {
                PointToAdd.push_back(lid->points[i]);
            }
        }

        double st_time = omp_get_wtime();
        int add_point_size = ikdtree.Add_Points(PointToAdd, true);
        ikdtree.Add_Points(PointNoNeedDownsample, false);
        add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    }   

    bool estiPlane(array<float, 4> &pca_result, const PointVector &point, const double &threshold)
    {
        Eigen::Matrix<float, NUM_MATCH_POINT, 3> A;
        Eigen::Matrix<float, NUM_MATCH_POINT, 1> b;
        A.setZero();
        b.setOnes();
        b *= -1.0f;

        //求A/Dx + B/Dy + C/Dz + 1 = 0 的参数 
        for (int j = 0; j < NUM_MATCH_POINT; j++)
        {
            A(j,0) = point[j].x;
            A(j,1) = point[j].y;
            A(j,2) = point[j].z;
        }

        Eigen::Matrix<float, 3, 1> normvec = A.colPivHouseholderQr().solve(b);

        float n = normvec.norm();
        //pca_result是平面方程的4个参数  /n是为了归一化
        pca_result[0] = normvec(0) / n;
        pca_result[1] = normvec(1) / n;
        pca_result[2] = normvec(2) / n;
        pca_result[3] = 1.0 / n;

        //如果几个点中有距离该平面>threshold的点 认为是不好的平面 返回false
        for (int j = 0; j < NUM_MATCH_POINT; j++)
        {
            if (fabs(pca_result[0] * point[j].x + pca_result[1] * point[j].y + pca_result[2] * point[j].z + pca_result[3]) > threshold)
            {
                return false;
            }
        }
        return true;
    }

    float calc_dist(PointType p1, PointType p2)
    {
        float d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
        return d;
    }

    void publishPath()
    {
        geometry_msgs::PoseStamped poseMsg;
        poseMsg.pose = convert2ROSPose(X.rot, X.pos);
        poseMsg.header.stamp = ros::Time().fromSec(curTime);
        poseMsg.header.frame_id = pa.worldCoord;

        /*** if path is too large, the rvis will crash ***/
        static int jjj = 0;
        jjj++;
        if (jjj % 3 == 0)
        {
            path.poses.push_back(poseMsg);
            pubPath.publish(path);
        }
    }

    void publishCurPcl()
    {
        sensor_msgs::PointCloud2 lidMsg;
        pcl::toROSMsg(*pclPjtWorld, lidMsg);
        lidMsg.header.stamp = ros::Time().fromSec(curTime);
        lidMsg.header.frame_id = pa.worldCoord;
        pubCurPcl.publish(lidMsg);
    }

    void publishDenPcl()
    {

    }

    void publishMap()
    {
        PointCloudXYZI::Ptr mapPcl(new PointCloudXYZI());
        PointVector().swap(ikdtree.PCL_Storage);
        ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
        mapPcl->clear();
        mapPcl->points = ikdtree.PCL_Storage;

        sensor_msgs::PointCloud2 mapPclMsg;
        pcl::toROSMsg(*mapPcl, mapPclMsg);
        mapPclMsg.header.stamp = ros::Time().fromSec(curTime);
        mapPclMsg.header.frame_id = pa.worldCoord;
        pubMap.publish(mapPclMsg);
    }   

    void saveMap()
    {
        string mapName;
        PointCloudXYZI::Ptr mapPcl(new PointCloudXYZI());
        if (1 == pa.mapSaveType)
        {
            mapName = "ikdTreeMap.pcd";
            PointVector().swap(ikdtree.PCL_Storage);
            ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
            mapPcl->clear();
            mapPcl->points = ikdtree.PCL_Storage;
        }
        else
        {
            mapName = "denseMap.pcd";
            mapPcl  = pclGlobal;
        }
        
        pcl::PCDWriter pcdWriter;
        string saveMapPN = pa.mapSavePath + mapName;
        pcdWriter.writeBinary(saveMapPN, *mapPcl);
        cerr << "save map: " +  saveMapPN<< endl;
    }

    void publishStatus()
    {
        if (pa.enPathPub)
            publishPath();

        if (pa.enCurPclPub)
            publishCurPcl();

        if (pa.enMapPub)
            publishMap();
    }

};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ww_lio_loc");

    LaserMapping LP;
    
    thread loopthread(&LaserMapping::processThread, &LP);

    ros::spin();
    
    return 0;
}
