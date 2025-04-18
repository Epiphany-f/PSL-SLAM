#ifndef INSECTLINE_H
#define INSECTLINE_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"
#include "Converter.h"
#include "Config.h"

#include<opencv2/core/core.hpp>
#include<mutex>
#include <map>

#include <Eigen/Core>
#include <Eigen/Dense>

#include<tuple>
#include<utility>
#include "Converter.h"
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/ModelCoefficients.h>
using namespace Eigen;
using namespace std;
using namespace cv;



namespace ORB_SLAM2
{

class KeyFrame;
class Map;
class Frame;
    typedef Matrix<double,6,1> Vector6d;
    // typedef std::tuple<ORB_SLAM3::MapLine*,ORB_SLAM3::MapLine*,int,int> insectline;
    // typedef std::tuple<ORB_SLAM3::MapLine*,ORB_SLAM3::MapLine*,int,int,Vector6d> insectline_cross;
    // typedef std::tuple<ORB_SLAM3::MapLine*,Vector6d,int> insectline_single;
    class InsectLine
    {
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud <PointT> PointCloud;
    public:

    InsectLine(const Matrix3d& Pos,const cv::Mat& plane, const Vector3d& intersection, const Vector3d& startpoint,const Vector3d& endtpoint, KeyFrame* pRefKF, int idx, Map* pMap,const Vector6d& Line1,const Vector6d& Line2,cv::Vec4f plane_normal,pair<int, int> id);
    void SetWorldPos(const Eigen::Vector3d &Pos_sP,const Eigen::Vector3d &Pos_eP, const Eigen::Vector3d insec,const cv::Mat &Pos);
    void SetWorldPos(const Eigen::Vector3d &Pos_sP1,const Eigen::Vector3d &Pos_eP1, const Eigen::Vector3d insec,const Eigen::Vector3d &Pos_sP2,const Eigen::Vector3d &Pos_eP2);
    void SetWorldPos(const Eigen::Matrix<double,15,1> &LIL);
    Matrix3d  GetWorldPos();
    Mat GetWorldPos_plane();

    void AddObservation(KeyFrame* pKF, int idx);
    void EraseObservation(KeyFrame* pKF);
    void SetBadFlag();
    map<KeyFrame*, size_t> GetObservations();
    int Observations();
    KeyFrame* GetReferenceKeyFrame();
    int GetIndexInKeyFrame(KeyFrame *pKF);
    void UpdateBoundary(const Frame& pF, int id);
    bool IsInKeyFrame(KeyFrame* pKF);
    bool isBad();
    void Replace(InsectLine* pMP);
    InsectLine* GetReplaced();
    void IncreaseVisible(int n=1);
    void IncreaseFound(int n=1);
    float GetFoundRatio();
    Map* GetMap();
    cv::Mat GetPoseInFrame(ORB_SLAM2::Frame &pF);
    void AddFrameObservation(ORB_SLAM2::Frame& pF);
    void UpdatePlaneNormal();
    void UpdatePlaneNormal(ORB_SLAM2::Frame &pF, int id);
    public:
        long unsigned int mnId; ///< Global ID for MapPlane;
        static long unsigned int nNextId;
        static std::mutex mGlobalMutex;
        long int mnFirstKFid;
        long int mnFirstFrame;
        int nObs;
        int mRed;
        int mGreen;
        int mBlue;

        long unsigned int mnBALocalForKF; //used in local BA
        long unsigned int mnCorrectedByKF; //used by loop closing
        long unsigned int mnCorrectedReference; //used by loop closing
        long unsigned int mnLoopPointForKF; //used by loop closing
        long unsigned int mnBAGlobalForKF;
        bool mbSeen;
        int mnVisible;
        int mnFound;
        // MapLine* line1;
        // MapLine* line2;
        Matrix3d mWorldPos;
        Vector3d struct_s;
        Vector3d struct_e;       
        Vector3d crosspoint;
        Vector6d line1;
        Vector6d line2;
        std::set<int> mObserveFrames;

        Vec4f* mvPlanes;
        std::vector<Vector3d> mvPlaneNormal;
        std::vector<pair<int, int>> mvPlaneLineNo;
        pair<int, int>* ids;
        cv::Vec4f plane;
        bool mbBad;
        bool mbBadPre;
        int mnObserveTh;
    protected:
        // cv::Mat mWorldPos; ///< Position in absolute coordinates
        cv::Mat mPosGBA;
        cv::Mat plane_pos;
        // bool mbBad;
        InsectLine* mpReplaced;
        std::map<KeyFrame*, size_t> mObservations;

        std::mutex mMutexMap;
        std::mutex mMutexPos;
        std::mutex mMutexFeatures;

        Map* mpMap;
        // Bad flag (we do not currently erase MapPoint from memory)
        // bool mbBad;

        // Reference KeyFrame
        KeyFrame* mpRefKF;
    };

}  //namespace ORB_SLAM

#endif //INSECTLINE_H