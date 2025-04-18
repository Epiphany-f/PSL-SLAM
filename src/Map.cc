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

#include "Map.h"

#include<mutex>

namespace ORB_SLAM2 {
    Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
    {
    }

    void Map::AddKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspKeyFrames.insert(pKF);
        if(pKF->mnId>mnMaxKFid)
            mnMaxKFid=pKF->mnId;
    }

    void Map::AddMapPoint(MapPoint *pMP)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspMapPoints.insert(pMP);
    }

    void Map::EraseMapPoint(MapPoint *pMP)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspMapPoints.erase(pMP);

        // TODO: This only erase the pointer.
        // Delete the MapPoint
    }

    void Map::EraseKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspKeyFrames.erase(pKF);

        // TODO: This only erase the pointer.
        // Delete the MapPoint
    }

    void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
    {
        unique_lock<mutex> lock(mMutexMap);
        mvpReferenceMapPoints = vpMPs;
    }

    void Map::InformNewBigChange()
    {
        unique_lock<mutex> lock(mMutexMap);
        mnBigChangeIdx++;
    }

    int Map::GetLastBigChangeIdx()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mnBigChangeIdx;
    }

    vector<KeyFrame*> Map::GetAllKeyFrames()
    {
        unique_lock<mutex> lock(mMutexMap);
        return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
    }

    vector<MapPoint*> Map::GetAllMapPoints()
    {
        unique_lock<mutex> lock(mMutexMap);
        return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
    }

    long unsigned int Map::MapPointsInMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mspMapPoints.size();
    }

    long unsigned int Map::KeyFramesInMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mspKeyFrames.size();
    }

    vector<MapPoint*> Map::GetReferenceMapPoints()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mvpReferenceMapPoints;
    }

    long unsigned int Map::GetMaxKFid()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mnMaxKFid;
    }

    void Map::clear()
    {
        for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
            delete *sit;

        for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
            delete *sit;

        mspMapPoints.clear();
        mspKeyFrames.clear();
        mnMaxKFid = 0;
        mvpReferenceMapPoints.clear();
        mvpKeyFrameOrigins.clear();
    }

    void Map::AddMapLine(MapLine *pML)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspMapLines.insert(pML);
    }

    void Map::EraseMapLine(MapLine *pML)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspMapLines.erase(pML);
    }


    void Map::SetReferenceMapLines(const std::vector<MapLine *> &vpMLs)
    {
        unique_lock<mutex> lock(mMutexMap);
        mvpReferenceMapLines = vpMLs;
    }

    vector<MapLine*> Map::GetAllMapLines()
    {
        unique_lock<mutex> lock(mMutexMap);
        return vector<MapLine*>(mspMapLines.begin(), mspMapLines.end());
    }

    vector<MapLine*> Map::GetReferenceMapLines()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mvpReferenceMapLines;
    }

    long unsigned int Map::MapLinesInMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mspMapLines.size();
    }

    //add
    void Map::AddMapInsecs(InsectLine* pInsec)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspInsecs.insert(pInsec);
    }

    //add
    long unsigned int Map::MapInsecsInMap()
    {
        unique_lock<mutex> lock(mMutexMap);
        return mspInsecs.size();
    }

    //add
    void Map::SetReferenceMapInsecs(const vector<InsectLine *> &vpInsecs) {
        unique_lock<mutex> lock(mMutexMap);
        mvpReferenceMapInsecs = vpInsecs;
    }

    //add
    vector<InsectLine*> Map::GetAllMapInsecs()
    {
        unique_lock<mutex> lock(mMutexMap);
        return vector<InsectLine*>(mspInsecs.begin(),mspInsecs.end());
    }



    //add
    void Map::EraseMapInsec(InsectLine *pML)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspInsecs.erase(pML);
    }

    int Map::AssociatePlanesByBoundary(ORB_SLAM2::Frame &pF,float dTh, float aTh) {

        unique_lock<mutex> lock(mMutexMap);
        pF.mbNewPlane = false;
        // pF.mbKeyFramePlane = false;


        int nmatches = 0;

        for(int i = 0; i < pF.N_LJL; ++i) {
            Mat pM = pF.ComputeWorldPlane(i);

            int nlinei = pF.mvPlaneLineNo[i].first;
            int nlinej = pF.mvPlaneLineNo[i].second;
            // pF.mvpMapInsecs[i] = static_cast<InsectLine*>(nullptr);
            Vector3d p3Dis = pF.mvLines3D[nlinei].first;
            Vector3d p3Die = pF.mvLines3D[nlinei].second;
            Vector3d p3Djs = pF.mvLines3D[nlinej].first;
            Vector3d p3Dje = pF.mvLines3D[nlinej].second;
            Vector3d intersection_point = pF.CrossPoint_3D[i];
            float ldTh = dTh;
            float laTh = aTh;

            bool found = false;
            for(set<InsectLine*>::iterator sit=mspInsecs.begin(), send=mspInsecs.end(); sit!=send; sit++) {

                Mat pW = (*sit)->GetWorldPos_plane();
                if(pW.at<float>(3, 0) < 0){
                    pW = -pW;
                }
                float angle = pM.at<float>(0, 0) * pW.at<float>(0, 0) +
                              pM.at<float>(1, 0) * pW.at<float>(1, 0) +
                              pM.at<float>(2, 0) * pW.at<float>(2, 0);
                cout  << ":  angle : " << angle << endl;
                // associate plane
                if ((angle > aTh || angle < -aTh))
                {
                    float dis = pM.at<float>(3, 0) - pW.at<float>(3, 0);
                    cout  << ":  dis原 : " << dis << endl;
                    float disis = (pW.at<float>(0, 0) * p3Dis.x() + pW.at<float>(1, 0) * p3Dis.y() + pW.at<float>(2, 0) * p3Dis.z() + pW.at<float>(3, 0));
                    float disie = (pW.at<float>(0, 0) * p3Die.x() + pW.at<float>(1, 0) * p3Die.y() + pW.at<float>(2, 0) * p3Die.z() + pW.at<float>(3, 0));
                    float disjs = (pW.at<float>(0, 0) * p3Djs.x() + pW.at<float>(1, 0) * p3Djs.y() + pW.at<float>(2, 0) * p3Djs.z() + pW.at<float>(3, 0));
                    float disje = (pW.at<float>(0, 0) * p3Dje.x() + pW.at<float>(1, 0) * p3Dje.y() + pW.at<float>(2, 0) * p3Dje.z() + pW.at<float>(3, 0));
                    float disins = (pW.at<float>(0, 0) * intersection_point.x() + pW.at<float>(1, 0) * intersection_point.y() + pW.at<float>(2, 0) * intersection_point.z() + pW.at<float>(3, 0));
                    dis = (disis + disie + disjs + disje + disins)/5;
                    cout  << ":  dis : " << dis << endl;
                    if(std::abs(dis) < dTh) {
                        dTh = dis;
                        cout  << ":  dTh : " << dTh << endl;
                        pF.mvpMapInsecs[i] = static_cast<InsectLine*>(nullptr);
                        pF.mvpMapInsecs[i] = (*sit);
                        found = true;
                        nmatches++;
                        continue;
                    }
                }
            }
        }


        for(auto p : pF.mvpMapInsecs){
            if(p== nullptr)
                pF.mbNewPlane = true;
            if(p)
                p->AddFrameObservation(pF);
        }

        return nmatches;
    }

}

