#ifndef LJL_H
#define LJL_H


#include <iostream>
#include <string>
#include <math.h>
#include <time.h>
#include <fstream>
#include "opencv2/imgproc.hpp"
#include <opencv/cv.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/line_descriptor.hpp>
#include "uselongline.h"

#include <cv.h>
#include <highgui.h>
#include "IO.h"
#include "PartiallyRecoverConnectivity.h"
#include "LineMatching.h"
#include <chrono>
#include "MatOperation.h"
#include <map>
#include <algorithm>
// #include "precomp.hpp"
// #include "opencl_kernels_features2d.hpp"
// #include <iterator>


using namespace std;
using namespace cv;
using namespace cv::line_descriptor;

namespace ORB_SLAM2 {
    class ORB_Impl_LJL
{
public:
    float fanThr = 1.0/3 * CV_PI;
    float expandWidth = 12.0f;
    const float HARRIS_K = 0.04f;
    Mat img;
    // cv::Mat mLines1,mLines2;
    cv::Mat fans1,fans2;
    struct LIL {
        Point2f Point;
        cv::line_descriptor::KeyLine line1;
        cv::line_descriptor::KeyLine line2;
        int idx1;
        int idx2;
        double angle;
    };
    typedef vector<LIL> vLIL;
    struct pairline {
        cv::line_descriptor::KeyLine KeyLine1;
        cv::line_descriptor::KeyLine KeyLine2;
        Mat Desc;
        float angle;
        Point2f intersection;
        int idx1;
        int idx2;
        int idx_point;
    };
    typedef vector<pairline> vPL;
    vPL LIL_gathers;
    vector<pair<int,pairline>> vpairline_level;
    // vector<pair<int,pairline>> vpairline_all;
    vector<vector<pair<int,pairline>>> vpairline_image_pyramid;
    // map<int,pairline> map_pairline_level;
    // map<int,pairline> map_pairline;
    vLIL LIL_gather1; //储存左图交点
    vLIL LIL_gather2; //储存右图交点
    vPL temp;
    double calculateAngle(const cv::line_descriptor::KeyLine& line1, const cv::line_descriptor::KeyLine& line2);
    void drawPartiallyConnectedLine(Mat Img, string imgname, vLIL fans);
    void keyLinesToMat(const std::vector<cv::line_descriptor::KeyLine>& keylines, cv::Mat& mLines);
    vLIL convertFansToKeyLines(const cv::Mat& fans, const std::vector<cv::line_descriptor::KeyLine>& mLines,int level) ;
    void drawLILMatches(Mat& img1, Mat& img2, vector<strFanMatch>& vstrFanMatch, vector<strPointMatch>& vstrPointMatch, Mat& outImg);
    float computeAngle(const KeyLine& kl1, const KeyLine& kl2);
    void my_getPointsonPolarline(vector<Point2f> &PointSet1,vector<Point2f> &PointSet2, Mat_<double> F, double T, bool *pbIsKept);
    void my_findRobustFundamentalMat(vector<Point2f> &PointSet1,vector<Point2f> &PointSet2, Mat &FMat, bool *pbIsKept);
    void saveKeypoints(const vector<KeyPoint>& keypoints, const string& filename);
    void loadKeypoints(const string& filename, vector<KeyPoint>& keypoints_3);
    void saveDescriptors(const Mat& descriptors, const string& filename);
    void loadDescriptors(const string& filename, Mat& descriptors);
    void saveLILGathersAllToFile(const std::vector<vPL>& LIL_gathers_all, const std::string& filename);
    void saveVpairlineLevelToFile(const vector<pair<int,pairline>>& vpairline_level, const std::string& filename);
    void saveVpairlineImagePyramidToFile(const vector<vector<pair<int,pairline>>>& vpairline_image_pyramid, const std::string& filename);
    void saveVpairlineAllToFile(const vector<pair<int,pairline>>& vpairline_all, const std::string& filename);
    vLIL ExactJunctionFromImage(Mat img,int level);
    vPL ExactPairlinesFromJuction(Mat Img,int level);
    vector<KeyPoint> LILtoVectorKeypoint(vLIL a);
    vector<KeyPoint> PairlinetoVectorKeypoint(const vPL& a, int& max_class_id);
    vector<pair<int,pairline>> GetPairline(const vPL& a, const vector<KeyPoint>& b);
    vector<pair<int,pairline>> PairlineFilter(vector<pair<int,pairline>>& temp,vector<KeyPoint> key_points);
    void checkLineExtremes( cv::Vec4f& extremes, cv::Size imageSize );
    ORB_Impl_LJL::vPL foroctave(vPL& temp,int level, const vector<float> & layer, int& max_id,const Mat& imagePyramid,const std::vector<Rect>& layerInfo);
    void DrawJunction(Mat img);
    ORB_Impl_LJL(int _nlevels);
    void computeOrientation(const Mat& image, vector<KeyPoint>& keypoints, const vector<int>& umax, ORB_Impl_LJL::vPL temp);
    void detectfromimages(InputArray image,
                        std::vector<KeyPoint>& keypoints,vector<pair<int,pairline>>& vPL_all1,
                        OutputArray descriptors);
    void computeKeyPoints(const Mat& imagePyramid,
                             const Mat& maskPyramid,
                             const std::vector<Rect>& layerInfo,
                             const UMat& ulayerInfo,
                             const std::vector<float>& layerScale,
                             std::vector<KeyPoint>& allKeypoints,
                             vector<pair<int,pairline>>& vpairline_all,
                             int nfeatures, double scaleFactor,
                             int edgeThreshold, int patchSize, int scoreType,
                             int fastThreshold  );
    void setMaxFeatures(int maxFeatures) { nfeatures = maxFeatures; }
    int getMaxFeatures() const { return nfeatures; }

    void setScaleFactor(double scaleFactor_) { scaleFactor = scaleFactor_; }
    double getScaleFactor() const { return scaleFactor; }

    void setNLevels(int nlevels_) { nlevels = nlevels_; }
    int getNLevels() const { return nlevels; }

    void setEdgeThreshold(int edgeThreshold_) { edgeThreshold = edgeThreshold_; }
    int getEdgeThreshold() const { return edgeThreshold; }

    void setFirstLevel(int firstLevel_) { firstLevel = firstLevel_; }
    int getFirstLevel() const { return firstLevel; }

    void setWTA_K(int wta_k_) { wta_k = wta_k_; }
    int getWTA_K() const { return wta_k; }

    void setScoreType(int scoreType_) { scoreType = scoreType_; }
    int getScoreType() const { return scoreType; }

    void setPatchSize(int patchSize_) { patchSize = patchSize_; }
    int getPatchSize() const { return patchSize; }

    void setFastThreshold(int fastThreshold_) { fastThreshold = fastThreshold_; }
    int getFastThreshold() const { return fastThreshold; }

    // returns the descriptor size in bytes
    int descriptorSize() const;
    // returns the descriptor type
    int descriptorType() const;
    // returns the default norm type
    int defaultNorm() const;

    // Compute the ORB_Impl features and descriptors on an image
    void detectAndCompute( InputArray _image, InputArray _mask,
                                 std::vector<KeyPoint>& keypoints,
                                 vector<pair<int,pairline>>& vpairlines_all,
                                 OutputArray _descriptors, bool useProvidedKeypoints );

protected:

    int nfeatures;
    double scaleFactor;
    int nlevels;
    int edgeThreshold;
    int firstLevel;
    int wta_k;
    int scoreType;
    int patchSize;
    int fastThreshold;
    // int nfeatures=500;
    // float scaleFactor=1.2f;
    // int nlevels=8;
    // int edgeThreshold=31;
    // int firstLevel=0;
    // int wta_k=2;
    // int scoreType=ORB::HARRIS_SCORE;
    // int patchSize=31;
    // int fastThreshold=20;
    vector<float> expandWidth_level;
};

}

#endif