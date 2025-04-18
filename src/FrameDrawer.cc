/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<mutex>

namespace ORB_SLAM2
{

FrameDrawer::FrameDrawer(Map* pMap):mpMap(pMap)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));
}

cv::Mat FrameDrawer::DrawFrame()
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state

    vector<KeyLine> vCurrentKeyLines;
    vector<KeyLine> vIniKeyLines;
    vector<bool> vbLineVO, vbLineMap;

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;

            vCurrentKeyLines = mvCurrentKeyLines;
            vIniKeyLines = mvIniKeyLines;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;

            vCurrentKeyLines = mvCurrentKeyLines;
            vbLineVO = mvbLineVO;
            vbLineMap = mvbLineMap;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
            vCurrentKeyLines = mvCurrentKeyLines;
        }
    } // destroy scoped mutex -> release mutex

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,CV_GRAY2BGR);

    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }        
    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        mnTracked_l = 0;
        mnTrackedVO_l = 0;
        const float r = 5;
        const int n = vCurrentKeys.size();
        const int nl = vCurrentKeyLines.size();
        for(int i=0;i<n;i++)
        {
            if(vbVO[i] || vbMap[i])
            {
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;

                // This is a match to a MapPoint in the map
                if(vbMap[i])
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }
        }
        for(int i=0; i< nl; ++i)
        {
            if(vbLineVO[i] || vbLineMap[i])
                // if(true)
            {
                cv::Point2f sp, ep;
                sp.x = int(vCurrentKeyLines[i].startPointX);
                sp.y = int(vCurrentKeyLines[i].startPointY);
                ep.x = int(vCurrentKeyLines[i].endPointX);
                ep.y = int(vCurrentKeyLines[i].endPointY);
                if(vbLineMap[i]) {
                    cv::line(im, sp, ep, cv::Scalar(0,0,255), 1.5);                     // Red
                    ++mnTracked_l;
                }
                else {
                    cv::line(im, sp, ep, cv::Scalar(255,0,255), 1.5);                   // Magenta
                    ++mnTrackedVO_l;
                }
            }
        }

        //add
      float step = 0.01;
        for(int i = 0; i < mvPlanes.size(); i++){
            Vector3d planeNormal_temp = mvPlaneNormal[i];
            cv::Point3d planeNormal(planeNormal_temp.x(),planeNormal_temp.y(),planeNormal_temp.z());
            int li = mvPlaneLineNo[i].first;
            int lj = mvPlaneLineNo[i].second;

            cv::Vec3f temp1 = mvLineEq[li];
            cv::Vec3f temp2 = mvLineEq[lj];
            Vector3d p3Dis = mvLines3D[li].first;
            Vector3d p3Die = mvLines3D[li].second;
            Vector3d p3Djs = mvLines3D[lj].first;
            Vector3d p3Dje = mvLines3D[lj].second;

            cv::Point3d directioni(mvLineEq[li][0],mvLineEq[li][1],mvLineEq[li][2]);
            cv::Point3d directionj (mvLineEq[lj][0],mvLineEq[lj][1],mvLineEq[lj][2]);
            cv::Point3d endpointi(p3Dis.x(),p3Dis.y(),p3Dis.z());
            cv::Point3d endpointi2(p3Die.x(),p3Die.y(),p3Die.z());
            cv::Point3d endpointj(p3Djs.x(),p3Djs.y(),p3Djs.z());
            cv::Point3d endpointj2(p3Dje.x(),p3Dje.y(),p3Dje.z());

            cv::Point3d tempVect = planeNormal.cross(directioni);

            float piEnd = (endpointi2.x - endpointi.x)/directioni.x;
            for (float pi = 0; pi < piEnd;) {
                for (float pj = -0.2; pj < 0.2;) {
                    float px = endpointi.x + pi * directioni.x + pj * tempVect.x;
                    float py = endpointi.y + pi * directioni.y + pj * tempVect.y;
                    float pz = endpointi.z + pi * directioni.z + pj * tempVect.z;
                    pj = pj + step;
                    {
                        float pu = fx * px / pz + cx;
                        float pv = fy * py / pz + cy;
                        cv::circle(im, cv::Point(pu, pv), 1, cv::Scalar(255, 144, 30), -1);
                    }
                }
                pi = pi + step;
            }

            tempVect = planeNormal.cross(directionj);
            piEnd = (endpointj2.x - endpointj.x)/directionj.x;
            for (float pi = 0; pi < piEnd;) {
                for (float pj = -0.2; pj < 0.2;) {
                    float px = endpointi.x + pi * directioni.x + pj * tempVect.x;
                    float py = endpointi.y + pi * directioni.y + pj * tempVect.y;
                    float pz = endpointi.z + pi * directioni.z + pj * tempVect.z;
                    pj = pj + step;
                    {
                        float pu = fx * px / pz + cx;
                        float pv = fy * py / pz + cy;
                        cv::circle(im, cv::Point(pu, pv), 1, cv::Scalar(255, 144, 30), -1);
                    }
                }
                pi = pi + step;
            }
        }
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);

    return imWithInfo;
}


void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        int nMLs = mpMap->MapLinesInMap();
        int nInsecs = mpMap->MapInsecsInMap();
        // s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked<< ", MLs: " << nMLs << ", LMatches: " << mnTracked_l;
        s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked<< ", nInsecs: " << nInsecs ;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
        if(mnTrackedVO_l>0)
            s << ", + VO matches: " << mnTrackedVO_l;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    pTracker->mImGray.copyTo(mIm);
    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    N = mvCurrentKeys.size();
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    mbOnlyTracking = pTracker->mbOnlyTracking;

    mvCurrentKeyLines = pTracker->mCurrentFrame.mvKeylinesUn;
    NL = mvCurrentKeyLines.size();
    mvbLineVO = vector<bool>(NL, false);
    mvbLineMap = vector<bool>(NL, false);

    //add
    fx = pTracker->mCurrentFrame.fx;
    fy = pTracker->mCurrentFrame.fy;
    cx = pTracker->mCurrentFrame.cx;
    cy = pTracker->mCurrentFrame.cy;
    mvPlanes = pTracker->mCurrentFrame.mvPlanes;
    mvPlaneNormal = pTracker->mCurrentFrame.mvPlaneNormal;
    mvPlaneLineNo = pTracker->mCurrentFrame.mvPlaneLineNo;
    mvLines3D = pTracker->mCurrentFrame.mvLines3D;
    mvLineEq = pTracker->mCurrentFrame.mvLineEq;

    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
        mvIniKeyLines = pTracker->mInitialFrame.mvKeylinesUn;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;
                }
            }
        }

        for(int i=0; i<NL; i++)
        {
            MapLine* pML = pTracker->mCurrentFrame.mvpMapLines[i];
            if(pML)
            {
                if(!pTracker->mCurrentFrame.mvbLineOutlier[i])
                {
                    if(pML->Observations()>0)
                        mvbLineMap[i] = true;
                    else
                        mvbLineVO[i] = false;
                }
            }
        }
    }
    mState=static_cast<int>(pTracker->mLastProcessedState);
}

} //namespace ORB_SLAM
