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

#include "Frame.h"
#include "Converter.h"
#include "ORBmatcher.h"
#include <thread>

namespace ORB_SLAM2
{

long unsigned int Frame::nNextId=0;
bool Frame::mbInitialComputations=true;
float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;

Frame::Frame()
{}

//Copy Constructor
Frame::Frame(const Frame &frame)
    :mpORBvocabulary(frame.mpORBvocabulary), mpORBextractorLeft(frame.mpORBextractorLeft), mpORBextractorRight(frame.mpORBextractorRight),
     mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()),
     mbf(frame.mbf), mb(frame.mb), mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
     mvKeysRight(frame.mvKeysRight), mvKeysUn(frame.mvKeysUn),  mvuRight(frame.mvuRight),
     mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
     mDescriptors(frame.mDescriptors.clone()), mDescriptorsRight(frame.mDescriptorsRight.clone()),
     mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
     mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
     mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
     mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
     mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2),
     mnScaleLevelsLine(frame.mnScaleLevelsLine),
     mfScaleFactorLine(frame.mfScaleFactorLine), mfLogScaleFactorLine(frame.mfLogScaleFactorLine),
     mvScaleFactorsLine(frame.mvScaleFactorsLine), mvInvScaleFactorsLine(frame.mvInvScaleFactorsLine),
     mvLevelSigma2Line(frame.mvLevelSigma2Line), mvInvLevelSigma2Line(frame.mvInvLevelSigma2Line),
     mLdesc(frame.mLdesc), NL(frame.NL), mvKeylinesUn(frame.mvKeylinesUn),mvSupposedVectors(frame.mvSupposedVectors), vManhAxisIdx(frame.vManhAxisIdx), mvPerpLines(frame.mvPerpLines), mvParallelLines(frame.mvParallelLines), mvpMapLines(frame.mvpMapLines), mvLines3D(frame.mvLines3D),
     mvLineEq(frame.mvLineEq),
     mvbLineOutlier(frame.mvbLineOutlier), mvKeyLineFunctions(frame.mvKeyLineFunctions),
    N_LJL(frame.N_LJL),mvpMapInsecs(frame.mvpMapInsecs),mvbOutlier_Insec(frame.mvbOutlier_Insec),mbNewPlane(frame.mbNewPlane),mbKeyFramePlane(frame.mbKeyFramePlane),
    mvle_l(frame.mvle_l),mvPlanes(frame.mvPlanes),mvPlaneNormal(frame.mvPlaneNormal),mvPlaneLineNo(frame.mvPlaneLineNo),CrossPoint_3D(frame.CrossPoint_3D),CrossPoint_2D(frame.CrossPoint_2D)
{
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGrid[i][j]=frame.mGrid[i][j];

    // Lines
    for(int i=0;i<FRAME_GRID_COLS;i++)
        for(int j=0; j<FRAME_GRID_ROWS; j++)
            mGridForLine[i][j]=frame.mGridForLine[i][j];

    if(!frame.mTcw.empty())
        SetPose(frame.mTcw);
}


Frame::Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractorLeft),mpORBextractorRight(extractorRight), mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth),
     mpReferenceKF(static_cast<KeyFrame*>(NULL))
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    thread threadLeft(&Frame::ExtractORB,this,0,imLeft);
    thread threadRight(&Frame::ExtractORB,this,1,imRight);
    threadLeft.join();
    threadRight.join();

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    ComputeStereoMatches();

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));    
    mvbOutlier = vector<bool>(N,false);


    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imLeft);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}

Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,LINEextractor* lsdextractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),mpLSDextractorLeft(lsdextractor),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();    
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // Scale Level Info for line
    mnScaleLevelsLine = mpLSDextractorLeft->GetLevels();
    mfScaleFactorLine = mpLSDextractorLeft->GetScaleFactor();
    mfLogScaleFactorLine = log(mfScaleFactor);
    mvScaleFactorsLine = mpLSDextractorLeft->GetScaleFactors();
    mvInvScaleFactorsLine = mpLSDextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2Line = mpLSDextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2Line = mpLSDextractorLeft->GetInverseScaleSigmaSquares();

    // This is done only for the first Frame (or after a change in the calibration)
    if (mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv = static_cast<float>(FRAME_GRID_COLS) / static_cast<float>(mnMaxX - mnMinX);
        mfGridElementHeightInv = static_cast<float>(FRAME_GRID_ROWS) / static_cast<float>(mnMaxY - mnMinY);

        fx = K.at<float>(0, 0);
        fy = K.at<float>(1, 1);
        cx = K.at<float>(0, 2);
        cy = K.at<float>(1, 2);
        invfx = 1.0f / fx;
        invfy = 1.0f / fy;

        mbInitialComputations = false;
    }

    mb = mbf / fx;

    // ORB extraction
    ExtractORB(0,imGray);
    ExtractLSD(imGray, imDepth);
    // thread threadPoint(&Frame::ExtractORBNDepth, this, imGray, imGray);
    // thread threadLine(&Frame::ExtractLSD, this, imGray, imDepth);
    // threadPoint.join();
    // threadLine.join();

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();
    ComputeStereoFromRGBD(imDepth);
    NL = mvKeylinesUn.size();

    N_LJL = mvPlanes.size();

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);


    mvpMapLines = vector<MapLine *>(NL, static_cast<MapLine *>(NULL));
    mvbLineOutlier = vector<bool>(NL, false);

    //add
    mvpMapInsecs = vector<InsectLine *>(mvPlanes.size(), static_cast<InsectLine *>(NULL));
    mvbOutlier_Insec = vector<bool>(mvPlanes.size(),false);
    AssignFeaturesToGrid();
    AssignFeaturesToGridForLine();

}


Frame::Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
    :mpORBvocabulary(voc),mpORBextractorLeft(extractor),mpORBextractorRight(static_cast<ORBextractor*>(NULL)),
     mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
{
    // Frame ID
    mnId=nNextId++;

    // Scale Level Info
    mnScaleLevels = mpORBextractorLeft->GetLevels();
    mfScaleFactor = mpORBextractorLeft->GetScaleFactor();
    mfLogScaleFactor = log(mfScaleFactor);
    mvScaleFactors = mpORBextractorLeft->GetScaleFactors();
    mvInvScaleFactors = mpORBextractorLeft->GetInverseScaleFactors();
    mvLevelSigma2 = mpORBextractorLeft->GetScaleSigmaSquares();
    mvInvLevelSigma2 = mpORBextractorLeft->GetInverseScaleSigmaSquares();

    // ORB extraction
    ExtractORB(0,imGray);

    N = mvKeys.size();

    if(mvKeys.empty())
        return;

    UndistortKeyPoints();

    // Set no stereo information
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
    mvbOutlier = vector<bool>(N,false);

    // This is done only for the first Frame (or after a change in the calibration)
    if(mbInitialComputations)
    {
        ComputeImageBounds(imGray);

        mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
        mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

        fx = K.at<float>(0,0);
        fy = K.at<float>(1,1);
        cx = K.at<float>(0,2);
        cy = K.at<float>(1,2);
        invfx = 1.0f/fx;
        invfy = 1.0f/fy;

        mbInitialComputations=false;
    }

    mb = mbf/fx;

    AssignFeaturesToGrid();
}

void Frame::AssignFeaturesToGrid()
{
    int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGrid[i][j].reserve(nReserve);

    for(int i=0;i<N;i++)
    {
        const cv::KeyPoint &kp = mvKeysUn[i];

        int nGridPosX, nGridPosY;
        if(PosInGrid(kp,nGridPosX,nGridPosY))
            mGrid[nGridPosX][nGridPosY].push_back(i);
    }
}

void Frame::AssignFeaturesToGridForLine()
{
    int nReserve = 0.5f*NL/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
    for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
        for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
            mGridForLine[i][j].reserve(nReserve);

    //#pragma omp parallel for
    for(int i=0;i<NL;i++)
    {
        const KeyLine &kl = mvKeylinesUn[i];

        list<pair<int, int>> line_coords;

        LineIterator* it = new LineIterator(kl.startPointX * mfGridElementWidthInv, kl.startPointY * mfGridElementHeightInv, kl.endPointX * mfGridElementWidthInv, kl.endPointY * mfGridElementHeightInv);

        std::pair<int, int> p;
        while (it->getNext(p))
            if (p.first >= 0 && p.first < FRAME_GRID_COLS && p.second >= 0 && p.second < FRAME_GRID_ROWS)
                mGridForLine[p.first][p.second].push_back(i);

        delete [] it;
    }
}

void Frame::ExtractORB(int flag, const cv::Mat &im)
{
    if(flag==0)
        (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors);
    else
        (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);
}

void Frame::ExtractORBNDepth(const cv::Mat &im, const cv::Mat &im_depth)
{
    (*mpORBextractorLeft)(im, cv::Mat(), mvKeys, mDescriptors);

    N = mvKeys.size();
    if (!mvKeys.empty())
    {
        UndistortKeyPoints();
        ComputeStereoFromRGBD(im_depth);
    }
}

Vector3d Frame::Frame_get_point(const Vector6d &v1, const Vector3d &ins)
{
    int dist1;
    Vector3d line_3D;
    Vector3d line_aux1_SP = v1.head(3);
    Vector3d line_aux1_EP = v1.tail(3);
    Vector3d dx1 = line_aux1_SP - ins;
    Vector3d dx2 = line_aux1_EP - ins;
    if (dx1.norm() > dx2.norm())
    {
        dist1 = 1;
        // v1.tail(3) = ins;
        line_3D = line_aux1_SP;
    }
    else
    {
        dist1 = 0;
        //  v1.head(3) = ins;
        line_3D = line_aux1_EP;
    }
    return line_3D;
}


void Frame::keyLinesToMat(const std::vector<cv::line_descriptor::KeyLine> &keylines)
{
    // 将矩阵的行数设置为键线的数量，列数设置为4（每条线有两个点，每个点有两个坐标）
    mLines.release();
    fans.release();
    mLines.create(keylines.size(), 4, CV_32F);

    // 遍历键线向量，将每条线段的起点和终点坐标存入矩阵中的每一行
    for (size_t i = 0; i < keylines.size(); ++i)
    {
        const auto &line = keylines[i];
        // 起点坐标
        mLines.at<float>(i, 0) = line.startPointX;
        mLines.at<float>(i, 1) = line.startPointY;
        // 终点坐标
        mLines.at<float>(i, 2) = line.endPointX;
        mLines.at<float>(i, 3) = line.endPointY;
    }
}

    double Frame::Frame_dotProduct(const Vector3d &v1, const Vector3d &v2)
{
    return v1.dot(v2);
}

    std::pair<bool, Vector3d> Frame::Frame_shortestDistance(Vector6d line1, Vector6d line2, double threshold)
{
    Vector3d p1 = line1.head(3);
    Vector3d p2 = line2.head(3);
    Vector3d d1 = line1.tail(3) - line1.head(3);
    Vector3d d2 = line2.tail(3) - line2.head(3);
    Vector3d root1, root2;
    Vector3d p2p1 = p1 - p2;
    double angle;
    Vector3d crosspoint;
    double dot_d1_d1 = Frame_dotProduct(d1, d1);
    double dot_d1_d2 = Frame_dotProduct(d1, d2);
    double dot_d2_d2 = Frame_dotProduct(d2, d2);
    double dot_p2p1_d1 = Frame_dotProduct(p2p1, d1);
    double dot_p2p1_d2 = Frame_dotProduct(p2p1, d2);

    Matrix2d A;
    A << dot_d1_d1, -dot_d1_d2,
        dot_d1_d2, -dot_d2_d2;

    Vector2d b(-dot_p2p1_d1, -dot_p2p1_d2);

    if (A.determinant() == 0)
    {
        return std::make_pair(false, Vector3d::Zero());
    }
    Vector2d x = A.colPivHouseholderQr().solve(b);

    root1 = p1 + x[0] * d1;
    root2 = p2 + x[1] * d2;
    crosspoint = (root1 + root2) * 0.5;
    double distance = (root2 - root1).norm();
    Vector3d midpoint_x = (line1.head(3) + line2.head(3))*0.5;
    Vector3d midpoint_y = (line1.tail(3) + line2.tail(3))*0.5;

    double distmid = ((midpoint_x - midpoint_y).norm())*2;

    // // std::cout<<"distance:"<<distance<<std::endl;
    if (distmid < (line1.norm()+line2.norm()))
    {
        return std::make_pair(true, crosspoint);
    }

    // return std::make_pair(false, Vector3d::Zero());
}

        vLIL Frame::convertFansToKeyLines(const cv::Mat &fans, const std::vector<cv::line_descriptor::KeyLine> &mLines)
    {
        std::vector<cv::line_descriptor::KeyLine> keyLines;
        vLIL LILs;
        LIL temp;
        intersection_lines_plane.clear();
        // intersection_point.clear();
        for (int i = 0; i < fans.rows; ++i)
        {
            // Extract information from fans matrix
            float x = fans.at<float>(i, 0);
            float y = fans.at<float>(i, 1);
            temp.first = Point2f(x, y);
            int index1 = static_cast<int>(fans.at<float>(i, 2));
            int index2 = static_cast<int>(fans.at<float>(i, 3));

            // Get start and end points of the corresponding lines
            cv::line_descriptor::KeyLine line1_ = mLines[index1];
            cv::line_descriptor::KeyLine line2_ = mLines[index2];

            Vector3d sp3D1= mvLines3D[index1].first;
            Vector3d ep3D1= mvLines3D[index1].second;
            Vector3d sp3D2= mvLines3D[index2].first;
            Vector3d ep3D2= mvLines3D[index2].second;
            Vector6d line1, line2;
            line1.head(3) = sp3D1;
            line1.tail(3) = ep3D1;
            line2.head(3) = sp3D2;
            line2.tail(3) = ep3D2;
            // std::cout<<"line1:"<<line1.transpose()<<std::endl;
            // std::cout<<"line2:"<<line2.transpose()<<std::endl;
            std::pair<bool, Vector3d> results = Frame_shortestDistance(line1, line2, 8);
            if(results.first){

                    if(results.second.norm() > std::numeric_limits<double>::epsilon())
                    {
                        intersection_lines_plane.push_back(make_pair(make_pair(index1,index2),make_pair(Point2f(x, y) ,results.second)));
                    }

            }
            // mvle_l.push_back(make_pair(le_l1,le_l2));
            temp.second.first = line1_;
            temp.second.second = line2_;
            LILs.push_back(temp);
        }
        return LILs;
    }

bool Frame::OldPlane(cv::Vec4f &pl)
{
    for (auto pli : mvPlanes)
    {
        float d = pl[3] - pli[3];
        float angle = pl[0] * pli[0] + pl[1] * pli[1] + pl[2] * pli[2];
        if (d > 0.2 || d < -0.2)
            continue;
        if (angle < 0.9397 && angle > -0.9397) // 30 degrees
            continue;
        return true;
    }
    return false;
}

void Frame::ExtractLSD(const cv::Mat &im, const cv::Mat &im_depth)
{
     std::chrono::steady_clock::time_point t1_line_detect = std::chrono::steady_clock::now();

    cv::Mat mask;
    (*mpLSDextractorLeft)(im,mask,mvKeylinesUn, mLdesc, mvKeyLineFunctions);

     std::chrono::steady_clock::time_point t2_line_detect = std::chrono::steady_clock::now();
     chrono::duration<double> t_line_detect = chrono::duration_cast<chrono::duration<double>>(t2_line_detect - t1_line_detect);

     std::chrono::steady_clock::time_point t1_line_good = std::chrono::steady_clock::now();

    // Option 1: git yanyan-li/PlanarSLAM
     isLineGood(im, im_depth, mK);

    keyLinesToMat(mvKeylinesUn);
    CPartiallyRecoverConnectivity p1(mLines, expandWidth, fans, im,fanThr);
    //二维
    LIL_gather = convertFansToKeyLines(fans, mvKeylinesUn);
    // UndistortKeyPoints_LJL();
    // ComputeStereoFromRGBD_LJL(im_depth);

    N_crosslineWithPlane = intersection_lines_plane.size();
        for (int i = 0; i < N_crosslineWithPlane; i++)
        {
            int l1 = intersection_lines_plane[i].first.first;
            int l2 = intersection_lines_plane[i].first.second;
            cv::Point2f CrossPoint_2d= intersection_lines_plane[i].second.first;
            cv::line_descriptor::KeyLine line1 = mvKeylinesUn[l1];
            cv::line_descriptor::KeyLine line2 = mvKeylinesUn[l2];

            Vector3d sp_lun1; sp_lun1 << line1.startPointX, line1.startPointY, 1.0;
            Vector3d ep_lun1; ep_lun1 << line1.endPointX,   line1.endPointY,   1.0;
            Vector3d le_l1; le_l1 << sp_lun1.cross(ep_lun1); le_l1 = le_l1 / std::sqrt( le_l1(0)*le_l1(0) + le_l1(1)*le_l1(1) );

            Vector3d sp_lun2; sp_lun2 << line2.startPointX, line2.startPointY, 1.0;
            Vector3d ep_lun2; ep_lun2 << line2.endPointX,   line2.endPointY,   1.0;
            Vector3d le_l2; le_l2 << sp_lun2.cross(ep_lun2); le_l2 = le_l2 / std::sqrt( le_l2(0)*le_l2(0) + le_l2(1)*le_l2(1) );

            mvle_l.push_back(make_pair(le_l1,le_l2));
            if (mvLineEq[l1][0] == 0 && mvLineEq[l1][1] == 0 && mvLineEq[l1][2] == 0)
                continue;
            if (mvLineEq[l2][0] == 0 && mvLineEq[l2][1] == 0 && mvLineEq[l2][2] == 0)
                continue;
            // if(CrossPoint_3D_old[i].z() < 0)
            //     continue;
            if (mvLines3D[l1].first.isZero() && mvLines3D[l1].second.isZero())
                continue;
            if (mvLines3D[l2].first.isZero() && mvLines3D[l2].second.isZero())
                continue;
            // if(std::isnan(mvLine3DDirections[l1].x()) || std::isnan(mvLine3DDirections[l1].y()) || std::isnan(mvLine3DDirections[l1].z()))
            // {
            //     continue;
            // }

            // if(std::isnan(mvLine3DDirections[l2].x()) || std::isnan(mvLine3DDirections[l2].y()) || std::isnan(mvLine3DDirections[l2].z()))
            // {
            //     continue;
            // }
            // float dot = abs(mvLine3DDirections[l1].dot(mvLine3DDirections[l2]));
            // float mfParallelAngle = cos(15 / 180 * M_PI);
            // cout<<"mfParallelAngle:"<<mfParallelAngle<<std::endl;
            // if (dot > mfParallelAngle)
            //     continue;
            cv::Vec3f planeNormal = mvLineEq[l1].cross(mvLineEq[l2]);
            // std::cout<<"planeNormal:"<<planeNormal[0]<<","<<planeNormal[1]<<","<<planeNormal[2]<<std::endl;
            // std::cout<<"planeNormal[l2]"<< planeNormal.transpose()<<std::endl;
            // 共面平面
            //     if(planeNormal.isZero())
            //     {
            //         continue;
            //     }
            // if(std::isnan(planeNormal.x()) || std::isnan(planeNormal.y()) || std::isnan(planeNormal.z()))
            // {
            //     continue;
            // }
            // if(std::isnan(planeNormal.x()) && std::isnan(planeNormal.y()) && std::isnan(planeNormal.z()))
            // continue;

            float norm = sqrt(planeNormal[0] * planeNormal[0] + planeNormal[1] * planeNormal[1] +
                              planeNormal[2] * planeNormal[2]);
            // cout<<"norm: "<< norm <<endl;
            planeNormal[0] = planeNormal[0] / norm;
            planeNormal[1] = planeNormal[1] / norm;
            planeNormal[2] = planeNormal[2] / norm;
            // std::cout<<"planeNormal/norm:"<<planeNormal[0]<<","<<planeNormal[1]<<","<<planeNormal[2]<<std::endl;
            Vector3d p3Dis = mvLines3D[l1].first;
            Vector3d p3Die = mvLines3D[l1].second;
            Vector3d p3Djs = mvLines3D[l2].first;
            Vector3d p3Dje = mvLines3D[l2].second;
            Vector3d p3ins = intersection_lines_plane[i].second.second;
            // cv::Point3d p3Dins(p3ins.x(),p3ins.y(),p3ins.z());
            // float distance1 = sqrt(pow(p3ins.x-p3Dis.x,2) + pow(p3ins.y-p3Dis.y,2) + pow(p3ins.z-p3Dis.z,2));
            Vector3d planeNormal_(planeNormal[0] ,planeNormal[1],planeNormal[2]);

            // std::cout<<"p3Dis:"<<p3Dis.x()<<","<<p3Dis.y()<<","<<p3Dis.z()<<std::endl;
            // std::cout<<"p3Die:"<<p3Die.x()<<","<<p3Die.y()<<","<<p3Die.z()<<std::endl;
            // std::cout<<"p3Djs:"<<p3Djs.x()<<","<<p3Djs.y()<<","<<p3Djs.z()<<std::endl;
            // std::cout<<"p3Dje:"<<p3Dje.x()<<","<<p3Dje.y()<<","<<p3Dje.z()<<std::endl;
            // std::cout<<"p3Dins:"<<p3ins.x()<<","<<p3ins.y()<<","<<p3ins.z()<<std::endl;
            // std::cout << "planeNormal: " << planeNormal.x << "," << planeNormal.y << "," << planeNormal.z << std::endl;

            float d1 = planeNormal_.x() * p3Dis.x() + planeNormal_.y() * p3Dis.y() + planeNormal_.z() * p3Dis.z();
            float d2 = planeNormal_.x() * p3Die.x() + planeNormal_.y() * p3Die.y() + planeNormal_.z() * p3Die.z();
            float d3 = planeNormal_.x() * p3Djs.x() + planeNormal_.y() * p3Djs.y() + planeNormal_.z() * p3Djs.z();
            float d4 = planeNormal_.x() * p3Dje.x() + planeNormal_.y() * p3Dje.y() + planeNormal_.z() * p3Dje.z();

            float d5 = planeNormal_.x()*p3ins.x() + planeNormal_.y() * p3ins.y() + planeNormal_.z() * p3ins.z();

            // std::cout<<"d1: "<<d1<<std::endl;
            // std::cout<<"d2: "<<d2<<std::endl;
            // std::cout<<"d3: "<<d3<<std::endl;
            // std::cout<<"d4: "<<d4<<std::endl;
            // std::cout<<"d5: "<<d5<<std::endl;
            float dmin = 10000, dmax = -10000;
            dmin = dmin < d1 ? dmin : d1;
            dmin = dmin < d2 ? dmin : d2;
            dmin = dmin < d3 ? dmin : d3;
            dmin = dmin < d4 ? dmin : d4;
            dmax = dmax > d1 ? dmax : d1;
            dmax = dmax > d2 ? dmax : d2;
            dmax = dmax > d3 ? dmax : d3;
            dmax = dmax > d4 ? dmax : d4;
            //new
            dmin = dmin < d5 ? dmin : d5;
            dmax = dmax > d5 ? dmax : d5;
            // std::cout<<"dmax: "<<dmax<<std::endl;
            // std::cout<<"dmin: "<<dmin<<std::endl;
            // std::cout << "dmax - dmin: " << dmax - dmin << std::endl;
            // std::cout << "mfSamePlaneDis: " << mfSamePlaneDis << std::endl;
            if (dmax - dmin > 0.05)
                continue;

            // float planeDis = -(d1 + d2 + d3 + d4 ) / 4;
            float planeDis = -(d1 + d2 + d3 + d4 + d5) / 5;
            // std::cout << "d1: " << d1 << std::endl;
            // std::cout << "d2: " << d2 << std::endl;
            // std::cout << "d3: " << d3 << std::endl;
            // std::cout << "d4: " << d4 << std::endl;
            // std::cout << "planeDis: " << planeDis << std::endl;
            cv::Vec4f plane(planeNormal_.x(), planeNormal_.y(), planeNormal_.z(), planeDis);
            // std::cout << "plane: " << plane[0] << "," << plane[1] << "," << plane[2] << "," << plane[3] << std::endl;
            if (plane[3] < 0)
            {
                plane = -plane;
                planeNormal_ = -planeNormal_;
            }
            if (OldPlane(plane))
                continue;
            mvPlanes.push_back(plane);
            mvPlaneNormal.push_back(planeNormal_);
            mvPlaneLineNo.push_back(make_pair(l1, l2));
            CrossPoint_3D.push_back(p3ins);
            Vector2d temp(CrossPoint_2d.x,CrossPoint_2d.y);
            CrossPoint_2D.push_back(temp);
            // std::cout<<"N_crosslineWithPlane: "<<N_crosslineWithPlane<<std::endl;
        }
}

Vector6d Frame::obtain3DLine(const int &i) {
    Vector6d Lines3D;
    Lines3D.head(3) = mvLines3D[i].first;
    Lines3D.tail(3) = mvLines3D[i].second;
    cv::Mat Ac = (Mat_<float>(3, 1) << Lines3D(0), Lines3D(1), Lines3D(2));
    cv::Mat A = mRwc * Ac + mOw;
    cv::Mat Bc = (Mat_<float>(3, 1) << Lines3D(3), Lines3D(4), Lines3D(5));
    cv::Mat B = mRwc * Bc + mOw;
    Lines3D << A.at<float>(0, 0), A.at<float>(1, 0), A.at<float>(2, 0),
            B.at<float>(0, 0), B.at<float>(1,0), B.at<float>(2, 0);
    return Lines3D;
}

// Optimize Lines --> Small mods of the code from YanYan Li ICRA 2021
void Frame::isLineGood(const cv::Mat &imGray, const cv::Mat &imDepth, const cv::Mat &K)
{
    mvLineEq.clear();
    mvLineEq.resize(mvKeylinesUn.size(),Vec3f(-1.0, -1.0, -1.0));
    mvLines3D.resize(mvKeylinesUn.size(), std::make_pair(Vector3d(0.0, 0.0, 0.0), Vector3d(0.0, 0.0, 0.0)));

    for (int i = 0; i < mvKeylinesUn.size(); ++i)
    { // each line
        double len = cv::norm(mvKeylinesUn[i].getStartPoint() - mvKeylinesUn[i].getEndPoint());
        vector<cv::Point3d> pts3d;
        // iterate through a line
        double numSmp = (double)min((int)len, 20); //number of line points sampled

        pts3d.reserve(numSmp);
        for (int j = 0; j <= numSmp; ++j)
        {
            // use nearest neighbor to querry depth value
            // assuming position (0,0) is the top-left corner of image, then the
            // top-left pixel's center would be (0.5,0.5)
            cv::Point2d pt = mvKeylinesUn[i].getStartPoint() * (1 - j / numSmp) +
                             mvKeylinesUn[i].getEndPoint() * (j / numSmp);

            if (pt.x < 0 || pt.y < 0 || pt.x >= imDepth.cols || pt.y >= imDepth.rows)
            {
                continue;
            }
            int row, col; // nearest pixel for pt
            if ((floor(pt.x) == pt.x) && (floor(pt.y) == pt.y))
            { // boundary issue
                col = max(int(pt.x - 1), 0);
                row = max(int(pt.y - 1), 0);
            }
            else
            {
                col = int(pt.x);
                row = int(pt.y);
            }

            float d = -1;
            if (imDepth.at<float>(row, col) <= 0.01)
            {
                continue;
            }
            else
            {
                d = imDepth.at<float>(row, col);
            }
            cv::Point3d p;

            p.z = d;
            p.x = (col - cx) * p.z * invfx;
            p.y = (row - cy) * p.z * invfy;

            pts3d.push_back(p);
        }

        if (pts3d.size() < 5){
            continue;
        }

        RandomLine3d tmpLine;
        vector<RandomPoint3d> rndpts3d;
        rndpts3d.reserve(pts3d.size());

        // compute uncertainty of 3d points
        for (int j = 0; j < pts3d.size(); ++j)
        {
            rndpts3d.push_back(mpLSDextractorLeft->compPt3dCov(pts3d[j], K, 1));
        }
        // using ransac to extract a 3d line from 3d pts
        tmpLine = mpLSDextractorLeft->extract3dline_mahdist(rndpts3d);

        if (
        cv::norm(tmpLine.A - tmpLine.B) > 0.02)
        {

            Eigen::Vector3d st_pt3D(tmpLine.A.x, tmpLine.A.y, tmpLine.A.z);
            Eigen::Vector3d e_pt3D(tmpLine.B.x, tmpLine.B.y, tmpLine.B.z);

            cv::Vec3f line_eq(tmpLine.B.x - tmpLine.A.x, tmpLine.B.y- tmpLine.A.y, tmpLine.B.z - tmpLine.A.z);

            float magn  = sqrt(line_eq[0] * line_eq[0] + line_eq[1] * line_eq[1]+ line_eq[2] * line_eq[2]);
            std::pair<Eigen::Vector3d, Eigen::Vector3d> line_ep_3D(st_pt3D, e_pt3D);
            mvLines3D[i] = line_ep_3D;

            mvLineEq[i] = line_eq/magn;
        }
    }
}

vector<size_t> Frame::GetFeaturesInAreaForLine(const float &x1, const float &y1, const float &x2, const float &y2, const float  &r, const int minLevel, const int maxLevel,const float TH) const
{
    vector<size_t> vIndices;
    vIndices.reserve(NL);
    unordered_set<size_t> vIndices_set;

    float x[3] = {x1, (x1+x2)/2.0, x2};
    float y[3] = {y1, (y1+y2)/2.0, y2};

    float delta1x = x1-x2;
    float delta1y = y1-y2;
    float norm_delta1 = sqrt(delta1x*delta1x + delta1y*delta1y);
    delta1x /= norm_delta1;
    delta1y /= norm_delta1;

    for(int i = 0; i<3;i++){
        const int nMinCellX = max(0,(int)floor((x[i]-mnMinX-r)*mfGridElementWidthInv));
        if(nMinCellX>=FRAME_GRID_COLS)
            continue;

        const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x[i]-mnMinX+r)*mfGridElementWidthInv));
        if(nMaxCellX<0)
            continue;

        const int nMinCellY = max(0,(int)floor((y[i]-mnMinY-r)*mfGridElementHeightInv));
        if(nMinCellY>=FRAME_GRID_ROWS)
            continue;

        const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y[i]-mnMinY+r)*mfGridElementHeightInv));
        if(nMaxCellY<0)
            continue;

        for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
        {
            for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
            {
                const vector<size_t> vCell = mGridForLine[ix][iy];
                if(vCell.empty())
                    continue;

                for(size_t j=0, jend=vCell.size(); j<jend; j++)
                {
                    if(vIndices_set.find(vCell[j]) != vIndices_set.end())
                        continue;

                    const KeyLine &klUn = mvKeylinesUn[vCell[j]];

                    float delta2x = klUn.startPointX - klUn.endPointX;
                    float delta2y = klUn.startPointY - klUn.endPointY;
                    float norm_delta2 = sqrt(delta2x*delta2x + delta2y*delta2y);
                    delta2x /= norm_delta2;
                    delta2y /= norm_delta2;
                    float CosSita = abs(delta1x * delta2x + delta1y * delta2y);

                    if(CosSita < TH)
                        continue;

                    Eigen::Vector3d Lfunc = mvKeyLineFunctions[vCell[j]];
                    const float dist = Lfunc(0)*x[i] + Lfunc(1)*y[i] + Lfunc(2);

                    if(fabs(dist)<r)
                    {
                        if(vIndices_set.find(vCell[j]) == vIndices_set.end())
                        {
                            vIndices.push_back(vCell[j]);
                            vIndices_set.insert(vCell[j]);
                        }
                    }
                }
            }
        }
    }

    return vIndices;
}

bool Frame::isInFrustum(MapLine *pML, float viewingCosLimit)
{
    pML->mbTrackInView = false;

    Vector6d P = pML->GetWorldPos();

    cv::Mat SP = (Mat_<float>(3,1) << P(0), P(1), P(2));
    cv::Mat EP = (Mat_<float>(3,1) << P(3), P(4), P(5));

    const cv::Mat SPc = mRcw*SP + mtcw;
    const float &SPcX = SPc.at<float>(0);
    const float &SPcY = SPc.at<float>(1);
    const float &SPcZ = SPc.at<float>(2);

    const cv::Mat EPc = mRcw*EP + mtcw;
    const float &EPcX = EPc.at<float>(0);
    const float &EPcY = EPc.at<float>(1);
    const float &EPcZ = EPc.at<float>(2);

    if(SPcZ<0.0f || EPcZ<0.0f)
        return false;

    const float invz1 = 1.0f/SPcZ;
    const float u1 = fx * SPcX * invz1 + cx;
    const float v1 = fy * SPcY * invz1 + cy;

    if(u1<mnMinX || u1>mnMaxX)
        return false;
    if(v1<mnMinY || v1>mnMaxY)
        return false;

    const float invz2 = 1.0f/EPcZ;
    const float u2 = fx*EPcX*invz2 + cx;
    const float v2 = fy*EPcY*invz2 + cy;

    if(u2<mnMinX || u2>mnMaxX)
        return false;
    if(v2<mnMinY || v2>mnMaxY)
        return false;

    const float maxDistance = pML->GetMaxDistanceInvariance();
    const float minDistance = pML->GetMinDistanceInvariance();

    const cv::Mat OM = 0.5*(SP+EP) - mOw;
    const float dist = cv::norm(OM);

    if(dist<minDistance || dist>maxDistance)
        return false;

    // Check viewing angle
    Vector3d Pn = pML->GetNormal();
    cv::Mat pn = (Mat_<float>(3,1) << Pn(0), Pn(1), Pn(2));
    const float viewCos = OM.dot(pn)/dist;

    if(viewCos<viewingCosLimit)
        return false;

    // Predict scale in the image
    const int nPredictedLevel = pML->PredictScale(dist, mfLogScaleFactor);

    // Data used by the tracking
    pML->mbTrackInView = true;
    pML->mTrackProjX1 = u1;
    pML->mTrackProjY1 = v1;
    pML->mTrackProjX2 = u2;
    pML->mTrackProjY2 = v2;
    pML->mnTrackScaleLevel = nPredictedLevel;
    pML->mTrackViewCos = viewCos;

    return true;
}

void Frame::SetPose(cv::Mat Tcw)
{
    mTcw = Tcw.clone();
    UpdatePoseMatrices();
}

void Frame::UpdatePoseMatrices()
{ 
    mRcw = mTcw.rowRange(0,3).colRange(0,3);
    mRwc = mRcw.t();
    mtcw = mTcw.rowRange(0,3).col(3);
    mOw = -mRcw.t()*mtcw;

    mTwc = cv::Mat::eye(4, 4, mTcw.type());
    mRwc.copyTo(mTwc.rowRange(0, 3).colRange(0, 3));
    mOw.copyTo(mTwc.rowRange(0, 3).col(3));
}

cv::Mat Frame::ComputeWorldPlane(int i)
{
    cv::Vec4f pl = mvPlanes[i];
    cv::Mat x3Dc = (cv::Mat_<float>(4, 1) << pl[0], pl[1], pl[2], pl[3]);
    cv::Mat temp;
    cv::transpose(mTcw, temp);
    return temp * x3Dc;
}

bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)
{
    pMP->mbTrackInView = false;

    // 3D in absolute coordinates
    cv::Mat P = pMP->GetWorldPos(); 

    // 3D in camera coordinates
    const cv::Mat Pc = mRcw*P+mtcw;
    const float &PcX = Pc.at<float>(0);
    const float &PcY= Pc.at<float>(1);
    const float &PcZ = Pc.at<float>(2);

    // Check positive depth
    if(PcZ<0.0f)
        return false;

    // Project in image and check it is not outside
    const float invz = 1.0f/PcZ;
    const float u=fx*PcX*invz+cx;
    const float v=fy*PcY*invz+cy;

    if(u<mnMinX || u>mnMaxX)
        return false;
    if(v<mnMinY || v>mnMaxY)
        return false;

    // Check distance is in the scale invariance region of the MapPoint
    const float maxDistance = pMP->GetMaxDistanceInvariance();
    const float minDistance = pMP->GetMinDistanceInvariance();
    const cv::Mat PO = P-mOw;
    const float dist = cv::norm(PO);

    if(dist<minDistance || dist>maxDistance)
        return false;

   // Check viewing angle
    cv::Mat Pn = pMP->GetNormal();

    const float viewCos = PO.dot(Pn)/dist;

    if(viewCos<viewingCosLimit)
        return false;

    // Predict scale in the image
    const int nPredictedLevel = pMP->PredictScale(dist,this);

    // Data used by the tracking
    pMP->mbTrackInView = true;
    pMP->mTrackProjX = u;
    pMP->mTrackProjXR = u - mbf*invz;
    pMP->mTrackProjY = v;
    pMP->mnTrackScaleLevel= nPredictedLevel;
    pMP->mTrackViewCos = viewCos;

    return true;
}

vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const
{
    vector<size_t> vIndices;
    vIndices.reserve(N);

    const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
    if(nMinCellX>=FRAME_GRID_COLS)
        return vIndices;

    const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
    if(nMaxCellX<0)
        return vIndices;

    const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
    if(nMinCellY>=FRAME_GRID_ROWS)
        return vIndices;

    const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
    if(nMaxCellY<0)
        return vIndices;

    const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

    for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
    {
        for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
        {
            const vector<size_t> vCell = mGrid[ix][iy];
            if(vCell.empty())
                continue;

            for(size_t j=0, jend=vCell.size(); j<jend; j++)
            {
                const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                if(bCheckLevels)
                {
                    if(kpUn.octave<minLevel)
                        continue;
                    if(maxLevel>=0)
                        if(kpUn.octave>maxLevel)
                            continue;
                }

                const float distx = kpUn.pt.x-x;
                const float disty = kpUn.pt.y-y;

                if(fabs(distx)<r && fabs(disty)<r)
                    vIndices.push_back(vCell[j]);
            }
        }
    }

    return vIndices;
}

bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)
{
    posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
    posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);

    //Keypoint's coordinates are undistorted, which could cause to go out of the image
    if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
        return false;

    return true;
}


void Frame::ComputeBoW()
{
    if(mBowVec.empty())
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
        mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
    }
}

void Frame::UndistortKeyPoints()
{
    if(mDistCoef.at<float>(0)==0.0)
    {
        mvKeysUn=mvKeys;
        return;
    }

    // Fill matrix with points
    cv::Mat mat(N,2,CV_32F);
    for(int i=0; i<N; i++)
    {
        mat.at<float>(i,0)=mvKeys[i].pt.x;
        mat.at<float>(i,1)=mvKeys[i].pt.y;
    }

    // Undistort points
    mat=mat.reshape(2);
    cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
    mat=mat.reshape(1);

    // Fill undistorted keypoint vector
    mvKeysUn.resize(N);
    for(int i=0; i<N; i++)
    {
        cv::KeyPoint kp = mvKeys[i];
        kp.pt.x=mat.at<float>(i,0);
        kp.pt.y=mat.at<float>(i,1);
        mvKeysUn[i]=kp;
    }
}

// void Frame::UndistortKeyLines()
// {
//     // 如果没有图像是矫正过的，没有失真
//     if(mDistCoef.at<float>(0)==0.0)
//     {
//         mvKeylinesUn=mvKeylines;
//         return;
//     }
//
//     // NL为提取的特征线数量，起点和终点分别保存在NL*2的mat中
//     cv::Mat matS(NL,2,CV_32F);
//     cv::Mat matE(NL,2,CV_32F);
//     for(int i=0; i<NL; i++)
//     {
//         matS.at<float>(i,0)=mvKeylines[i].startPointX;
//         matS.at<float>(i,1)=mvKeylines[i].startPointY;
//         matE.at<float>(i,0)=mvKeylines[i].endPointX;
//         matE.at<float>(i,1)=mvKeylines[i].endPointY;
//     }
//
//     matS = matS.reshape(2);
//     cv::undistortPoints(matS,matS,mK,mDistCoef,cv::Mat(),mK);
//     matS = matS.reshape(1);
//
//     matE = matE.reshape(2);
//     cv::undistortPoints(matE,matE,mK,mDistCoef,cv::Mat(),mK);
//     matE = matE.reshape(1);
//
//     mvKeylinesUn.resize(NL);
//     for(int i=0; i<NL; i++)
//     {
//         KeyLine kl = mvKeylines[i];
//         kl.startPointX = matS.at<float>(i,0);
//         kl.startPointY = matS.at<float>(i,1);
//         kl.endPointX = matE.at<float>(i,0);
//         kl.endPointY = matE.at<float>(i,1);
//         mvKeylinesUn[i] = kl;
//     }
//
// }

void Frame::ComputeImageBounds(const cv::Mat &imLeft)
{
    if(mDistCoef.at<float>(0)!=0.0)
    {
        cv::Mat mat(4,2,CV_32F);
        mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
        mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;
        mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;
        mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;

        // Undistort corners
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
        mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
        mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
        mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));

    }
    else
    {
        mnMinX = 0.0f;
        mnMaxX = imLeft.cols;
        mnMinY = 0.0f;
        mnMaxY = imLeft.rows;
    }
}

void Frame::ComputeStereoMatches()
{
    mvuRight = vector<float>(N,-1.0f);
    mvDepth = vector<float>(N,-1.0f);

    const int thOrbDist = (ORBmatcher::TH_HIGH+ORBmatcher::TH_LOW)/2;

    const int nRows = mpORBextractorLeft->mvImagePyramid[0].rows;

    //Assign keypoints to row table
    vector<vector<size_t> > vRowIndices(nRows,vector<size_t>());

    for(int i=0; i<nRows; i++)
        vRowIndices[i].reserve(200);

    const int Nr = mvKeysRight.size();

    for(int iR=0; iR<Nr; iR++)
    {
        const cv::KeyPoint &kp = mvKeysRight[iR];
        const float &kpY = kp.pt.y;
        const float r = 2.0f*mvScaleFactors[mvKeysRight[iR].octave];
        const int maxr = ceil(kpY+r);
        const int minr = floor(kpY-r);

        for(int yi=minr;yi<=maxr;yi++)
            vRowIndices[yi].push_back(iR);
    }

    // Set limits for search
    const float minZ = mb;
    const float minD = 0;
    const float maxD = mbf/minZ;

    // For each left keypoint search a match in the right image
    vector<pair<int, int> > vDistIdx;
    vDistIdx.reserve(N);

    for(int iL=0; iL<N; iL++)
    {
        const cv::KeyPoint &kpL = mvKeys[iL];
        const int &levelL = kpL.octave;
        const float &vL = kpL.pt.y;
        const float &uL = kpL.pt.x;

        const vector<size_t> &vCandidates = vRowIndices[vL];

        if(vCandidates.empty())
            continue;

        const float minU = uL-maxD;
        const float maxU = uL-minD;

        if(maxU<0)
            continue;

        int bestDist = ORBmatcher::TH_HIGH;
        size_t bestIdxR = 0;

        const cv::Mat &dL = mDescriptors.row(iL);

        // Compare descriptor to right keypoints
        for(size_t iC=0; iC<vCandidates.size(); iC++)
        {
            const size_t iR = vCandidates[iC];
            const cv::KeyPoint &kpR = mvKeysRight[iR];

            if(kpR.octave<levelL-1 || kpR.octave>levelL+1)
                continue;

            const float &uR = kpR.pt.x;

            if(uR>=minU && uR<=maxU)
            {
                const cv::Mat &dR = mDescriptorsRight.row(iR);
                const int dist = ORBmatcher::DescriptorDistance(dL,dR);

                if(dist<bestDist)
                {
                    bestDist = dist;
                    bestIdxR = iR;
                }
            }
        }

        // Subpixel match by correlation
        if(bestDist<thOrbDist)
        {
            // coordinates in image pyramid at keypoint scale
            const float uR0 = mvKeysRight[bestIdxR].pt.x;
            const float scaleFactor = mvInvScaleFactors[kpL.octave];
            const float scaleduL = round(kpL.pt.x*scaleFactor);
            const float scaledvL = round(kpL.pt.y*scaleFactor);
            const float scaleduR0 = round(uR0*scaleFactor);

            // sliding window search
            const int w = 5;
            cv::Mat IL = mpORBextractorLeft->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduL-w,scaleduL+w+1);
            IL.convertTo(IL,CV_32F);
            IL = IL - IL.at<float>(w,w) *cv::Mat::ones(IL.rows,IL.cols,CV_32F);

            int bestDist = INT_MAX;
            int bestincR = 0;
            const int L = 5;
            vector<float> vDists;
            vDists.resize(2*L+1);

            const float iniu = scaleduR0+L-w;
            const float endu = scaleduR0+L+w+1;
            if(iniu<0 || endu >= mpORBextractorRight->mvImagePyramid[kpL.octave].cols)
                continue;

            for(int incR=-L; incR<=+L; incR++)
            {
                cv::Mat IR = mpORBextractorRight->mvImagePyramid[kpL.octave].rowRange(scaledvL-w,scaledvL+w+1).colRange(scaleduR0+incR-w,scaleduR0+incR+w+1);
                IR.convertTo(IR,CV_32F);
                IR = IR - IR.at<float>(w,w) *cv::Mat::ones(IR.rows,IR.cols,CV_32F);

                float dist = cv::norm(IL,IR,cv::NORM_L1);
                if(dist<bestDist)
                {
                    bestDist =  dist;
                    bestincR = incR;
                }

                vDists[L+incR] = dist;
            }

            if(bestincR==-L || bestincR==L)
                continue;

            // Sub-pixel match (Parabola fitting)
            const float dist1 = vDists[L+bestincR-1];
            const float dist2 = vDists[L+bestincR];
            const float dist3 = vDists[L+bestincR+1];

            const float deltaR = (dist1-dist3)/(2.0f*(dist1+dist3-2.0f*dist2));

            if(deltaR<-1 || deltaR>1)
                continue;

            // Re-scaled coordinate
            float bestuR = mvScaleFactors[kpL.octave]*((float)scaleduR0+(float)bestincR+deltaR);

            float disparity = (uL-bestuR);

            if(disparity>=minD && disparity<maxD)
            {
                if(disparity<=0)
                {
                    disparity=0.01;
                    bestuR = uL-0.01;
                }
                mvDepth[iL]=mbf/disparity;
                mvuRight[iL] = bestuR;
                vDistIdx.push_back(pair<int,int>(bestDist,iL));
            }
        }
    }

    sort(vDistIdx.begin(),vDistIdx.end());
    const float median = vDistIdx[vDistIdx.size()/2].first;
    const float thDist = 1.5f*1.4f*median;

    for(int i=vDistIdx.size()-1;i>=0;i--)
    {
        if(vDistIdx[i].first<thDist)
            break;
        else
        {
            mvuRight[vDistIdx[i].second]=-1;
            mvDepth[vDistIdx[i].second]=-1;
        }
    }
}


void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
{
    mvuRight = vector<float>(N,-1);
    mvDepth = vector<float>(N,-1);

    for(int i=0; i<N; i++)
    {
        const cv::KeyPoint &kp = mvKeys[i];
        const cv::KeyPoint &kpU = mvKeysUn[i];

        const float &v = kp.pt.y;
        const float &u = kp.pt.x;

        const float d = imDepth.at<float>(v,u);

        if(d>0)
        {
            mvDepth[i] = d;
            mvuRight[i] = kpU.pt.x-mbf/d;
        }
    }
}

cv::Mat Frame::UnprojectStereo(const int &i)
{
    const float z = mvDepth[i];
    if(z>0)
    {
        const float u = mvKeysUn[i].pt.x;
        const float v = mvKeysUn[i].pt.y;
        const float x = (u-cx)*z*invfx;
        const float y = (v-cy)*z*invfy;
        cv::Mat x3Dc = (cv::Mat_<float>(3,1) << x, y, z);
        return mRwc*x3Dc+mOw;
    }
    else
        return cv::Mat();
}

} //namespace ORB_SLAM
