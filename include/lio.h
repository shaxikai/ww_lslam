#include "utility.h"

#define NUM_MATCH_POINT (5)
#define INI_IMU_COUNT (10)  //最大迭代次数
#define MAX_NUM_SIG_LID (100000)

using namespace std;
using namespace ww_lslam;

#ifdef IVOX_NODE_TYPE_PHC
    using IVoxType = IVox<3, IVoxNodeType::PHC, PointType>;
#else
    using IVoxType = IVox<3, IVoxNodeType::DEFAULT, PointType>;
#endif

//判断点的时间先后顺序(注意curvature中存储的是时间戳)
const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

class LIO
{

public:
    struct LioOutData
    {
        double ts;
        Sophus::SE3 pos;
        Vec6d  cov;
        PointCloudXYZI::Ptr lid;
    };

private:

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

    shared_ptr<Param> pa;

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

    IVoxType::Options ivox_options_;
    std::shared_ptr<IVoxType> ivox_ = nullptr;

    thread processThr;

    std::mutex imuLock;
    std::mutex lidLock;

public:
    LIO(shared_ptr<Param> _pa)
    {
        pa = _pa;

        reset();

        processThr = thread(&LIO::process, this);
    }

    void reset()
    {
        imuInited = false;
        imuCnt    = 1;

        Q = Eigen::MatrixXd::Zero(12, 12);            
        Q.block<3, 3>(0, 0).diagonal() = V3D(pa->gyrCov, pa->gyrCov, pa->gyrCov);
        Q.block<3, 3>(3, 3).diagonal() = V3D(pa->accCov, pa->accCov, pa->accCov);
        Q.block<3, 3>(6, 6).diagonal() = V3D(pa->bGyrCov, pa->bGyrCov, pa->bGyrCov);
        Q.block<3, 3>(9, 9).diagonal() = V3D(pa->bAccCov, pa->bAccCov, pa->bAccCov);

        P = Eigen::MatrixXd::Identity(24,24);      //在esekfom.hpp获得P_的协方差矩阵
        P(6,6) = P(7,7) = P(8,8) = 0.00001;
        P(9,9) = P(10,10) = P(11,11) = 0.00001;
        P(15,15) = P(16,16) = P(17,17) = 0.0001;
        P(18,18) = P(19,19) = P(20,20) = 0.001;
        P(21,21) = P(22,22) = P(23,23) = 0.00001; 

        X.grav = Eigen::Vector3d(0,0,-pa->gravity);
        X.extrinT = pa->extrinT;
        X.extrinR = Sophus::SO3(pa->extrinR);

        pclPjt.reset(new PointCloudXYZI()); 
        pclPjtDown.reset(new PointCloudXYZI());
        pclPjtDownWorld.reset(new PointCloudXYZI()); 
        pclPjtWorld.reset(new PointCloudXYZI()); 
        pclGlobal.reset(new PointCloudXYZI()); 

        downSizeFilterMap.setLeafSize(pa->minMapFilterSize, pa->minMapFilterSize, pa->minMapFilterSize);

        path.header.stamp = ros::Time::now();
        path.header.frame_id = pa->worldCoord;

        IVoxType::Options ivox_options_;
        ivox_options_.resolution_ = pa->minMapFilterSize;
        ivox_options_.capacity_   = pa->ivoxCapacity;
        switch (pa->ivoxNearbyType)
        {
        case 0:
            ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
            break;
        case 16:
            ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
            break;
        case 26:
            ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
            break;
        default:
            break;
        }

        ivox_ = std::make_shared<IVoxType>(ivox_options_);
    }

    void process()
    {
        ros::Rate rate(1000);

        setNonBlocking();  // 设置终端为非阻塞模式
        while (ros::ok())
        {
            if (getKey() == 'q') break;
            processUnit();
            rate.sleep();
        }

        if (pa->mapSaveType)
        {
            saveMap();
        }
    }

    void processUnit()
    {
        bool state = false;

        MeasureGroup measDate;
        state = syncPackages(measDate);
        if (!state) return;

        if (!imuInited)
        {
            imuInited = imuInit(measDate);
            return;
        }

        vector<PjtData> pjtData;
        imuPreint(measDate, pjtData);
        projectPcl(measDate, pjtData, pclPjt);

        V3D lidPos = X.pos + X.rot.matrix() * X.extrinT;
        // cerr << "prei pose : " << X.pos(0) << " " << X.pos(1) << " " << X.pos(2) << endl;

        //点云下采样
        downSizeFilterMap.setInputCloud(pclPjt);
        downSizeFilterMap.filter(*pclPjtDown);
        int npts  = pclPjtDown->points.size();

        pclPjtDownWorld->resize(npts);
        ptsBody2World(pclPjtDown, pclPjtDownWorld);

        if (0 == ivox_->NumValidGrids())
        {
            ivox_->AddPoints(pclPjtDownWorld->points);
            return;
        }

        nearPclPts.resize(npts);
        eskfUpdte(pclPjtDown);
        // cerr << "eskf pose : " << X.pos(0) << " " << X.pos(1) << " " << X.pos(2) << endl;

        pclPjtDownWorld->resize(npts);
        ptsBody2World(pclPjtDown, pclPjtDownWorld);
        map_incremental(pclPjtDownWorld);

        // cerr << "add " << pclPjtDownWorld->points.size() << "pts.\t" 
        //      << "local map size " << ivox_->NumValidGrids() << endl;

        ptsBody2World(pclPjt, pclPjtWorld);

        pa->lioOutFlg = !pa->lioOutFlg;
    }

    void imuInput(const sensor_msgs::Imu::ConstPtr &imu)
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        imuBuffer.push_back(imu);
    }

    void lidInput(double ts, PointCloudXYZI::Ptr &lid)
    {
        std::lock_guard<std::mutex> lock1(lidLock);
        timeBuffer.push_back(ts);
        lidBuffer.push_back(lid);
    }

    void getLidPos(Sophus::SE3 &lidPos)
    {
        lidPos = Sophus::SE3(X.rot, X.pos);
    }

    void getCurLid(PointCloudXYZI::Ptr &curLid)
    {
        curLid = pclPjtWorld;
    }

    void getLocMap(PointCloudXYZI::Ptr &locMap)
    {
        ivox_->GetAllPoints(locMap->points);
    }

    void getOutData(LioOutData &data)
    {
        data.ts  = curTime;
        data.pos = Sophus::SE3(X.rot, X.pos);
        data.lid = pclPjt;

        data.cov(0) = P(3,3), data.cov(1) = P(4,4), data.cov(2) = P(5,5);
        data.cov(3) = P(0,0), data.cov(4) = P(1,1), data.cov(5) = P(2,2);
    }

    bool syncPackages(MeasureGroup &meas)
    {
        if (lidBuffer.size() <= 2) return false;

        PointCloudXYZI::Ptr lid = lidBuffer.front();
        double lidBigTime = timeBuffer.front();
        double lidEndTime = lidBigTime + lid->points.back().curvature;
        
        if (abs(lid->points.back().curvature) < 1e-5)
            lidEndTime = timeBuffer[1];

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

        // std::cerr << std::setprecision(15) 
        // << lidBuffer.size() << " "
        // << lidBigTime << " " << lidEndTime << " " 
        // << meas.imu.front()->header.stamp.toSec() << " "
        // << meas.imu.back()->header.stamp.toSec() << std::endl;

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
            X.grav = - mean_acc / mean_acc.norm() * pa->gravity;    //得平均测量的单位方向向量 * 重力加速度预设值
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
            acc_avr  = acc_avr * pa->gravity / mean_acc.norm();

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
        for (int i = -1; i < pa->nMaxIter; i++) // maximum_iter是卡尔曼滤波的最大迭代次数
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
            auto K_front = (HTH / pa->lidPtCov + P.inverse()).inverse();
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> K;
            K = K_front.block<24, 12>(0, 0) * H.transpose() / pa->lidPtCov; //卡尔曼增益  这里R视为常数

            Eigen::Matrix<double, 24, 24> KH = Eigen::Matrix<double, 24, 24>::Zero(); //矩阵 K * H
            KH.block<24, 12>(0, 0) = K * H;
            Eigen::Matrix<double, 24, 1> dx_ = K * eskfData.h + (KH - Eigen::Matrix<double, 24, 24>::Identity()) * dx_new; //公式(18)
            X = StatePlus(X, dx_); //公式(18)

            //std::cout << "dx_: " << dx_[0] << " " << dx_[1] << " " << dx_[2] << std::endl;
            //getchar();

            eskfData.converge = true;
            for (int j = 0; j < 24; j++)
            {
                if (std::fabs(dx_[j]) > pa->epsi) //如果dx>epsi 认为没有收敛
                {
                    eskfData.converge = false;
                    break;
                }
            }

            if (eskfData.converge)
                convergeCnt++;

            if (0 == convergeCnt && i == pa->nMaxIter - 2) //如果迭代了3次还没收敛 强制令成true，h_share_model函数中会重新寻找近邻点
            {
                eskfData.converge = true;
            }

            //cerr << i << " " <<  eskfData.converge << " " << convergeCnt <<endl;
            if (convergeCnt > 1 || i == pa->nMaxIter - 1)
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
                //ikdtree.Nearest_Search(point_world, NUM_MATCH_POINT, points_near, pointSearchSqDis);
                ivox_->GetClosestPoint(point_world, points_near, NUM_MATCH_POINT);
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
        //cerr << "nValid: " << nValid << endl;
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
                const PointVector &points_near = nearPclPts[i];
                bool need_add = true;
                PointType mid_point; //点所在体素的中心
                mid_point.x = floor(lid->points[i].x / pa->minMapFilterSize) * pa->minMapFilterSize + 0.5 * pa->minMapFilterSize;
                mid_point.y = floor(lid->points[i].y / pa->minMapFilterSize) * pa->minMapFilterSize + 0.5 * pa->minMapFilterSize;
                mid_point.z = floor(lid->points[i].z / pa->minMapFilterSize) * pa->minMapFilterSize + 0.5 * pa->minMapFilterSize;
                float dist = calc_dist(lid->points[i], mid_point);
                if (fabs(points_near[0].x - mid_point.x) > 0.5 * pa->minMapFilterSize 
                && fabs(points_near[0].y - mid_point.y) > 0.5 * pa->minMapFilterSize 
                && fabs(points_near[0].z - mid_point.z) > 0.5 * pa->minMapFilterSize)
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
                    PointToAdd.push_back(lid->points[i]);
                }
            }
            else
            {
                PointToAdd.push_back(lid->points[i]);
            }
        }

        //FIXME
        ivox_->AddPoints(PointToAdd);
        ivox_->AddPoints(PointNoNeedDownsample);
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

    void saveMap()
    {
        string mapName;
        PointCloudXYZI::Ptr mapPcl(new PointCloudXYZI());
        if (1 == pa->mapSaveType)
        {
            // FIXME
            //mapName = "ikdTreeMap.pcd";
            //mapPcl->points = ikdtree.PCL_Storage;
        }
        else
        {
            mapName = "denseMap.pcd";
            mapPcl  = pclGlobal;
        }
        
        pcl::PCDWriter pcdWriter;
        string saveMapPN = pa->mapSavePath + mapName;
        pcdWriter.writeBinary(saveMapPN, *mapPcl);
        cerr << "save map: " +  saveMapPN<< endl;
    }


};
