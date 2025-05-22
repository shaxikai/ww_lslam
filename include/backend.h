#include "utility.h"

using namespace gtsam;

class BACKEND
{
public:
    struct LioData
    {
        double ts;
        Sophus::SE3 pos;
        Vec6d cov;
        PointCloudXYZI::Ptr lid;
    };


    struct GpsData
    {
        double ts;
        Vec3d pos;
        Vec3d cov;
    };

    struct LoopData
    {
        int newId;
        int hisId;
        Sophus::SE3 relPosN2H;
        Vec6d cov;
    }

    struct BkdOutData
    {
        vector<Sophus::SE3> pth;
        vector<Sophus::SE3> gpsPth;
    };

private:

    std::shared_ptr<Param> pa;

    std::deque<GpsData>  gpsBuffer;
    std::deque<LioData>  lidBuffer;
    std::deque<LoopData> loopBuffer;

    int curId;
    LioData curLid;
    Sophus::SE3 lastLidPos;

    bool firstGps;
    Vec3d lastGpsPos;
    GeographicLib::LocalCartesian gpsTrans;

    SCManager scManager;

    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate;

    // map
    vector<Sophus::SE3> gpsPos;
    vector<Sophus::SE3> bdysPos;
    vector<PointCloudXYZI::Ptr> lids;

    thread processThr;
    thread loopDetThr;

    std::mutex gpsLock;
    std::mutex lidLock;

public:
    BACKEND(std::shared_ptr<Param> _pa)
    {
        pa = _pa;
        firstGps = true;

        curId = -1;

        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);

        processThr = thread(&BACKEND::process, this);
        loopDetThr = thread(&BACKEND::loopDet, this);
    }

    void gpsInput(GpsData &data)
    {
        Vec3d pos = data.pos;

        if (firstGps) 
        {
            firstGps  = false;
            gpsTrans.Reset(pos(0), pos(1), pos(2));
        }

        gpsTrans.Forward(pos(0), pos(1), pos(2), data.pos(0), data.pos(1), data.pos(2));

        std::lock_guard<std::mutex> lock1(gpsLock);
        gpsBuffer.push_back(data);
    }

    void lidInput(LioData &data)
    {
        std::lock_guard<std::mutex> lock2(lidLock);
        lidBuffer.push_back(data);
    }

    void process()
    {
        ros::Rate rate(1000);

        while (ros::ok())
        {
            processUnit();
            rate.sleep();
        }
    }

    void processUnit()
    {
        bool sts = isKeyFrame();
        if (!sts) return;
        
        addLioFactor();

        addGpsFactor();

        addLoopFactor();

        // update iSAM
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();
        gtSAMgraph.resize(0);
        initialEstimate.clear();

        getOptRes();

        pa->bkdOutFlg = !pa->bkdOutFlg;
    }

    void loopDet()
    {
        int tmpId = 0;
        ros::Rate rate(100);
        while (ros::ok())
        {
            if (tmpId != curId)
            {
                loopDetUnit();  
                tmpId = curId;
            }
            rate.sleep();
        }
    }

    void loopDetUnit()
    {
        scManager.makeAndSaveScancontextAndKeys(*(curLid.lid));

        auto detRes = scManager.detectLoopClosureID(); 
        int hisId   = detectResult.first;
        if( hisId == -1)
            return;

        PointCloudXYZI curPts(new pcl::PointCloud<PointType>());
        PointCloudXYZI hisPts(new pcl::PointCloud<PointType>());
        {
            int base_key = 0;
            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0, base_key); // giseop 
            loopFindNearKeyframes(prevKeyframeCloud, hisId, historyKeyframeSearchNum, base_key); // giseop 

            if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
                return;
            if (pubHistoryKeyFrames.getNumSubscribers() != 0)
                publishCloud(pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
        }

        // ICP Settings
        pcl::IterativeClosestPoint<PointType, PointType> icp;
        icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius*2);
        icp.setMaximumIterations(100);
        icp.setTransformationEpsilon(1e-6);
        icp.setEuclideanFitnessEpsilon(1e-6);
        icp.setRANSACIterations(0);

        // Align clouds
        icp.setInputSource(cureKeyframeCloud);
        icp.setInputTarget(prevKeyframeCloud);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        icp.align(*unused_result);

        if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
            return;

        // publish corrected cloud
        if (pubIcpKeyFrames.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
            pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
            publishCloud(pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
        }

        // Get pose transformation
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = icp.getFinalTransformation();

        // // transform from world origin to wrong pose
        // Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        // // transform from world origin to corrected pose
        // Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong;// pre-multiplying -> successive rotation about a fixed frame
        // pcl::getTranslationAndEulerAngles (tCorrect, x, y, z, roll, pitch, yaw);
        // gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        // gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[hisId]);

        // gtsam::Vector Vector6(6);
        // float noiseScore = icp.getFitnessScore();
        // Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        // noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

        // giseop 
        pcl::getTranslationAndEulerAngles (correctionLidarFrame, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));

        // giseop, robust kernel for a SC loop
        float robustNoiseScore = 0.5; // constant is ok...
        gtsam::Vector robustNoiseVector6(6); 
        robustNoiseVector6 << robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore, robustNoiseScore;
        noiseModel::Base::shared_ptr robustConstraintNoise; 
        robustConstraintNoise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure, but with a good front-end loop detector, Cauchy is empirically enough.
            gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6)
        ); // - checked it works. but with robust kernel, map modification may be delayed (i.e,. requires more true-positive loop factors)

        // Add pose constraint
        mtx.lock();
        loopIndexQueue.push_back(make_pair(loopKeyCur, hisId));
        loopPoseQueue.push_back(poseFrom.between(poseTo));
        loopNoiseQueue.push_back(robustConstraintNoise);
        mtx.unlock();

        // add loop constriant
        loopIndexContainer[loopKeyCur] = hisId;
    }

    bool isKeyFrame()
    {
        if (lidBuffer.size() < 2)
            return false;

        LioData data = lidBuffer.front();
        lidBuffer.pop_front();

        Sophus::SE3 relPos;
        if (!bdysPos.empty())
        {
            Sophus::SE3 curPos  = data.pos;
            relPos = lastLidPos.inverse() * curPos;
            Vec3d t = relPos.translation();
            Vec3d r = relPos.rotation_matrix().eulerAngles(2, 1, 0);

            if (abs(r(0)) < pa->keyAngThr && abs(r(1)) < pa->keyAngThr && 
                abs(r(2)) < pa->keyAngThr && t.squaredNorm() < pa->keyTraThr)
            {
                return false;
            }
        }

        curLid = data;
        lastLidPos = data.pos;
        curLid.pos = relPos;
        lids.push_back(data.lid);
        curId++;

        return true;
    }

    void addLioFactor()
    {
        if (0 == bdysPos.size())
        {
            noiseModel::Diagonal::shared_ptr priorNoise = 
            noiseModel::Diagonal::Variances((Vector(6) << 1e6, 1e6, 1e6, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
            gtSAMgraph.add(PriorFactor<Pose3>(0, gtsam::Pose3(), priorNoise));
            initialEstimate.insert(0, gtsam::Pose3());
        }else{
            gtsam::Pose3 pos;
            convert2GtsamPose(curLid.pos, pos);
            noiseModel::Diagonal::shared_ptr odometryNoise = 
            noiseModel::Diagonal::Variances((Vector(6) << curLid.cov).finished());
            //noiseModel::Diagonal::Variances((Vector(6) << 1e-3, 1e-3, 1e-3, 1, 1, 1).finished());
            gtSAMgraph.add(BetweenFactor<Pose3>(bdysPos.size()-1, bdysPos.size(), pos));

            Sophus::SE3 lastPos = bdysPos.back();
            Sophus::SE3 tmpCurPos = lastPos * curLid.pos;
            convert2GtsamPose(tmpCurPos, pos);
            initialEstimate.insert(bdysPos.size(), pos);
        }
    }

    void addGpsFactor()
    {
        if (gpsBuffer.empty())
            return;

        // wait for system initialized and settles down
        if (bdysPos.empty() || (bdysPos.front().inverse() * bdysPos.back()).translation().norm() < 5.0)
            return;

        while (!gpsBuffer.empty())
        {
            GpsData data = gpsBuffer.front();
            
            if (data.ts < curLid.ts - 0.1)
            {
                gpsBuffer.pop_front();
                continue;
            }
            
            if (data.ts > curLid.ts + 0.1)
            {
                break;
            }

            gpsBuffer.pop_front();

            if (data.cov(0) > pa->gpsCovThr || 
                data.cov(1) > pa->gpsCovThr ||
                data.cov(2) > pa->gpsCovThr)
                {
                    continue;
                }

            // GPS not properly initialized (0,0,0)
            if (abs(data.pos(0)) < 1e-6 && abs(data.pos(1)) < 1e-6)
            {
                continue;
            }

            if ((data.pos - lastGpsPos).norm() < 5.0)
            {
                continue;

            }
            else
                lastGpsPos = data.pos;

            gpsPos.push_back(Sophus::SE3(Mat3d::Identity(), data.pos));
            gtsam::Vector cov(3);
            cov << max(data.cov(0), 1.0), max(data.cov(1), 1.0), max(data.cov(2), 1.0);
            noiseModel::Diagonal::shared_ptr gps_noise = noiseModel::Diagonal::Variances(cov);
            gtsam::GPSFactor gps_factor(bdysPos.size(), data.pos, gps_noise);
            gtSAMgraph.add(gps_factor);

            break;
        }
    }

    void addLoopFactor()
    {

    }

    void getOptRes()
    {
        Values vals = isam->calculateEstimate();
        int npos = vals.size();
        bdysPos.resize(npos);
        for (int i = 0; i < npos; ++i)
        {
            gtsam::Pose3 pos = vals.at<Pose3>(i);
            convert2SE3(pos, bdysPos[i]);
        }
    }

    void nearKeyFrmPts(PointCloudXYZI::Ptr& pts, int& key, int& searchNum, const int& loop_index)
    {
        // extract near keyframes
        pts->clear();
        int cloudSize = copy_cloudKeyPoses6D->size();
        for (int i = -searchNum; i <= searchNum; ++i)
        {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= cloudSize )
                continue;

            int select_loop_index = (loop_index != -1) ? loop_index : key + i;
            *pts += *transformPointCloud(surfCloudKeyFrames[keyNear],   &copy_cloudKeyPoses6D->points[select_loop_index]);
        }

        if (pts->empty())
            return;

        // downsample near keyframes
        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
        *nearKeyframes = *cloud_temp;
    }

    void getOutData(BkdOutData &data)
    {
        data.pth = bdysPos;
        data.gpsPth = gpsPos;
    }

    void getGloMap(PointCloudXYZI::Ptr &map)
    {
        PointCloudXYZI::Ptr allLid(new PointCloudXYZI);
        PointCloudXYZI::Ptr pts(new PointCloudXYZI);
        for (int i = 0; i < bdysPos.size() - 1; ++i)
        {
            auto &lid = lids[i];
            Sophus::SE3 tmpLidPos = bdysPos[i] * Sophus::SE3(pa->extrinR, pa->extrinT);
            ptsLid2World(lid, tmpLidPos, pts);
            *allLid += *pts;
        }

        pcl::VoxelGrid<PointType> voxelFilter;
        voxelFilter.setLeafSize(pa->minMapFilterSize, pa->minMapFilterSize, pa->minMapFilterSize);
        voxelFilter.setInputCloud(allLid);
        voxelFilter.filter(*map);
    }
};
