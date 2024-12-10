#include "utility.h"

using namespace gtsam;

class BACKEND
{
public:
    struct GpsData
    {
        double ts;
        Vec3d pos;
        Vec3d cov;
    };

    struct LioData
    {
        double ts;
        Sophus::SE3 pos;
        Vec6d cov;
    };

    struct BkdOutData
    {
        vector<Sophus::SE3> pth;
        vector<Sophus::SE3> gpsPth;
    };

private:

    std::shared_ptr<Param> pa;

    std::deque<GpsData> gpsBuffer;
    std::deque<LioData> lidBuffer;

    bool firstGps;
    Vec3d lastGpsPos;
    GeographicLib::LocalCartesian gpsTrans;

    LioData curLid;
    Sophus::SE3 lastLidPos;

    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate;

    // map
    vector<Sophus::SE3> gpsPos;
    vector<Sophus::SE3> lidsPos;
    vector<pcl::PointCloud<PointType>::Ptr> lidsPts;

    thread processThr;

    std::mutex gpsLock;
    std::mutex lidLock;

public:
    BACKEND(std::shared_ptr<Param> _pa)
    {
        pa = _pa;
        firstGps = true;

        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);

        processThr = thread(&BACKEND::process, this);
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

        // update iSAM
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();
        gtSAMgraph.resize(0);
        initialEstimate.clear();

        getOptRes();

        pa->bkdOutFlg = !pa->bkdOutFlg;
    }

    bool isKeyFrame()
    {
        if (lidBuffer.size() < 2)
            return false;

        LioData data = lidBuffer.front();
        lidBuffer.pop_front();

        Sophus::SE3 relPos;
        if (!lidsPos.empty())
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
        return true;
    }

    void addLioFactor()
    {
        if (0 == lidsPos.size())
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
            gtSAMgraph.add(BetweenFactor<Pose3>(lidsPos.size()-1, lidsPos.size(), pos));

            Sophus::SE3 lastPos = lidsPos.back();
            Sophus::SE3 tmpCurPos = lastPos * curLid.pos;
            convert2GtsamPose(tmpCurPos, pos);
            initialEstimate.insert(lidsPos.size(), pos);
        }
    }

    void addGpsFactor()
    {

        if (gpsBuffer.empty())
            return;

        // wait for system initialized and settles down
        if (lidsPos.empty() || (lidsPos.front().inverse() * lidsPos.back()).translation().norm() < 5.0)
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
            gtsam::GPSFactor gps_factor(lidsPos.size(), data.pos, gps_noise);
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
        lidsPos.resize(npos);
        for (int i = 0; i < npos; ++i)
        {
            gtsam::Pose3 pos = vals.at<Pose3>(i);
            convert2SE3(pos, lidsPos[i]);
        }
    }

    void getOutData(BkdOutData &data)
    {
        data.pth = lidsPos;
        data.gpsPth = gpsPos;
    }

};
