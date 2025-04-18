#include "insectline.h"

#include<mutex>
#include<time.h>

namespace ORB_SLAM2{
    long unsigned int InsectLine::nNextId = 0;
    mutex InsectLine::mGlobalMutex;

    InsectLine::InsectLine(const Matrix3d& Pos,const cv::Mat& plane, const Vector3d& intersection, const Vector3d& startpoint,const Vector3d& endtpoint, KeyFrame* pRefKF, int idx, Map* pMap,const Vector6d& Line1,const Vector6d& Line2,cv::Vec4f plane_normal,pair<int, int> id):
    mnBALocalForKF(0), mpMap(pMap), mpRefKF(pRefKF),mbBad(false),crosspoint(intersection),struct_s(startpoint),struct_e(endtpoint),
    plane_pos(plane),mnFirstKFid(pRefKF->mnId), mnFirstFrame(pRefKF->mnFrameId), mnVisible(1), mnFound(1),nObs(0),line1(Line1),line2(Line2),mvPlanes(&plane_normal),ids(&id)
    {

        mnId = nNextId++;

        mRed = rand() % 256;
        mBlue = rand() % 256;
        mGreen = rand() % 256;
        mbBad = true;
        mbBadPre = true;
        mnObserveTh = Config::Get<int>("MapPlane.ObserveTimes");
    // MapLines can be created from Tracking and Local Mapping. This mutex avoid conflicts with id.
    // unique_lock<mutex> lock(mpMap->mMutexLineCreation);    
    }

    void InsectLine::AddObservation(ORB_SLAM2::KeyFrame *pKF, int idx)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if(mObservations.count(pKF))
            return;
        mObservations[pKF] = idx;
        nObs++;
        if(mObservations.size() > 1)
            if(!mbBadPre)
                mbBad = false;
    }

    void InsectLine::AddFrameObservation(ORB_SLAM2::Frame& pF)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        mObserveFrames.insert(pF.mnId);
        if(mObserveFrames.size() > mnObserveTh) {
            mbBadPre = false;
        }
    }

    void InsectLine::EraseObservation(ORB_SLAM2::KeyFrame *pKF)
    {
        bool bBad = false;
        {
            unique_lock<mutex> lock(mMutexFeatures);
            if (mObservations.count(pKF)) {
                mObservations.erase(pKF);
                nObs--;

                if (mpRefKF == pKF)
                    mpRefKF = mObservations.begin()->first;

                if (nObs <= 2)
                    bBad = true;
            }
        }

        if (bBad) {
            SetBadFlag();
        }
    }

    void InsectLine::SetBadFlag()
    {
        map<KeyFrame*,size_t> obs, verObs, parObs;
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            mbBad=true;
            obs = mObservations;
            mObservations.clear();
        }
        for(auto & ob : obs)
        {
            KeyFrame* pKF = ob.first;
            pKF->EraseMapinsecMatch(ob.second);
        }

        mpMap->EraseMapInsec(this);
    }

    map<ORB_SLAM2::KeyFrame*, size_t> InsectLine::GetObservations()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mObservations;
    }

    int InsectLine::Observations()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return nObs;
    }

    KeyFrame* InsectLine::GetReferenceKeyFrame()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return mpRefKF;
    }

    int InsectLine::GetIndexInKeyFrame(ORB_SLAM2::KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        if (mObservations.count(pKF))
            return mObservations[pKF];
        else
            return -1;        
    }

    Map* InsectLine::GetMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mpMap;
    }

    void InsectLine::SetWorldPos(const Eigen::Vector3d &Pos_sP,const Eigen::Vector3d &Pos_eP, const Eigen::Vector3d insec,const cv::Mat &Pos)
    {
        unique_lock<mutex> lock2(mGlobalMutex);
        unique_lock<mutex> lock(mMutexPos);
        mWorldPos.col(0) = struct_s;
        mWorldPos.col(1) = struct_e;
        mWorldPos.col(2) = crosspoint;
        Pos.copyTo(plane_pos);
    }

    void InsectLine::SetWorldPos(const Eigen::Vector3d &Pos_sP1,const Eigen::Vector3d &Pos_eP1, const Eigen::Vector3d insec,const Eigen::Vector3d &Pos_sP2,const Eigen::Vector3d &Pos_eP2)
    {
        unique_lock<mutex> lock2(mGlobalMutex);
        unique_lock<mutex> lock(mMutexPos);
        line1.head(3) = Pos_sP1;
        line1.tail(3) = Pos_eP1;
        crosspoint = insec;
        line2.head(3) = Pos_sP2;
        line2.tail(3) = Pos_eP2;
        // Pos.copyTo(plane_pos);
    }

    void InsectLine::SetWorldPos(const Eigen::Matrix<double,15,1> &LIL) {
        unique_lock<mutex> lock2(mGlobalMutex);
        unique_lock<mutex> lock(mMutexPos);
        line1 = LIL.segment<6>(0);
        line2 = LIL.segment<6>(6);
        crosspoint = LIL.segment<3>(12);
    }


    Matrix3d InsectLine::GetWorldPos()
    {
        unique_lock<mutex> lock(mMutexPos);
        return mWorldPos;
    }

    cv::Mat InsectLine::GetWorldPos_plane(){
        unique_lock<mutex> lock(mMutexPos);
        return plane_pos.clone();
    }

    void InsectLine::UpdateBoundary(const ORB_SLAM2::Frame &pF, int id)
    {
        // Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat(pF.mTcw);
        // pF.mvPlanes[id] = T.inverse().matrix()*pF.mvPlanes[id];

    }

    bool InsectLine::IsInKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return (mObservations.count(pKF));       
    }

    bool InsectLine::isBad()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        return mbBad;        
    }

    void InsectLine::Replace(InsectLine *pMP)
    {
        if(pMP->mnId==this->mnId)
            return;

        int nvisible, nfound;
        map<KeyFrame*,size_t> obs, verObs, parObs;
        {
            unique_lock<mutex> lock1(mMutexFeatures);
            unique_lock<mutex> lock2(mMutexPos);
            mbBad=true;
            obs = mObservations;
            mObservations.clear();
            nvisible = mnVisible;
            nfound = mnFound;
            mpReplaced = pMP;
        }

        for(auto & ob : obs)
        {
            // Replace measurement in keyframe
            KeyFrame* pKF = ob.first;

            *pMP->mvPlanes += pKF->mvPlanes[ob.second];
            // pMP->crosspoint = pKF->CrossPoint_3D[ob.second];
            // pMP->line1.head(3) = pKF->mvLines3D[pKF->mvPlaneLineNo[ob.second].first].first;
            // pMP->line1.tail(3) = pKF->mvLines3D[pKF->mvPlaneLineNo[ob.second].first].second;
            // pMP->line2.head(3) = pKF->mvLines3D[pKF->mvPlaneLineNo[ob.second].second].first;
            // pMP->line2.tail(3) = pKF->mvLines3D[pKF->mvPlaneLineNo[ob.second].second].second;

            if(!pMP->IsInKeyFrame(pKF))
            {
                pKF->ReplaceMapinsecMatch(ob.second, pMP);
                pMP->AddObservation(pKF,ob.second);
            }
            else
            {
                pKF->EraseMapinsecMatch(ob.second);
            }
        }

        pMP->IncreaseFound(nfound);
        pMP->IncreaseVisible(nvisible);
        pMP->UpdatePlaneNormal();

        mpMap->EraseMapInsec(this);
    }

    InsectLine* InsectLine::GetReplaced()
    {
        unique_lock<mutex> lock1(mMutexFeatures);
        unique_lock<mutex> lock2(mMutexPos);
        return mpReplaced;
    }

    void InsectLine::IncreaseVisible(int n)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        mnVisible+=n;
    }

    void InsectLine::IncreaseFound(int n)
    {
        unique_lock<mutex> lock(mMutexFeatures);
        mnFound+=n;
    }

    float InsectLine::GetFoundRatio()
    {
        unique_lock<mutex> lock(mMutexFeatures);
        return static_cast<float>(mnFound)/mnVisible;
    }

    cv::Mat InsectLine::GetPoseInFrame(ORB_SLAM2::Frame &pF) {
        unique_lock<mutex> lock(mMutexPos);
        cv::Mat temp;
        cv::transpose(pF.mTwc, temp);
        return temp*plane_pos;
    }    

    void InsectLine::UpdatePlaneNormal() {
        map<KeyFrame*, size_t> observations = GetObservations();
        for(auto & observation : observations){
            KeyFrame* frame = observation.first;
            int id = observation.second;
            cv::Vec4f pl = frame->mvPlanes[id];
            cv::Mat x3Dc = (cv::Mat_<float>(4, 1) << pl[0], pl[1], pl[2], pl[3]);
            cv::Mat mTwc = frame->GetPoseInverse();
            // cv::Mat temp;
            // cv::transpose(mTwc,temp);
            cv::Mat plane= mTwc*x3Dc;
            frame->mvPlanes[id] = cv::Vec4f(plane.at<float>(0),plane.at<float>(1),plane.at<float>(2),plane.at<float>(3));
        }
    }

    void InsectLine::UpdatePlaneNormal(ORB_SLAM2::Frame &pF, int id) {
        cv::Vec4f pl = pF.mvPlanes[id];
        cv::Mat x3Dc = (cv::Mat_<float>(4, 1) << pl[0], pl[1], pl[2], pl[3]);
        cv::Mat mTwc = pF.mTwc;
        // cv::Mat temp;
        // cv::transpose(mTwc,temp);
        cv::Mat plane= mTwc*x3Dc;
        pF.mvPlanes[id] = cv::Vec4f(plane.at<float>(0),plane.at<float>(1),plane.at<float>(2),plane.at<float>(3));
    }

}
