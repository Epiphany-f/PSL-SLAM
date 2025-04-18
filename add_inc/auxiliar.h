//
// Created by lan on 17-12-18.
//

#pragma once

#include <iostream>

#include <cv.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/line_descriptor/descriptor.hpp>
using namespace cv;
using namespace cv::line_descriptor;

#include <vector>
using namespace std;

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Eigenvalues>
using namespace Eigen;

typedef Matrix<double,6,1> Vector6d;
typedef Matrix<double,6,6> Matrix6d;

struct compare_by_maxDepth  //这里写成bool compare_by_maxDepth()函数不行吗 测试 TODO
{
    inline bool operator() (const pair<pair<float,float>, int>& a, const pair<pair<float,float>, int>& b)
    {
        float aDepthMax = max(a.first.first, a.first.second);
        float bDepthMax = max(b.first.first, b.first.second);

        return aDepthMax < bDepthMax;   //按照升序排列

    }
};

struct compare_descriptor_by_NN_dist
{
    inline bool operator()(const vector<DMatch>& a, const vector<DMatch>& b){
        return ( a[0].distance < b[0].distance);
    }
};

struct conpare_descriptor_by_NN12_dist
{
    inline bool operator()(const vector<DMatch>& a, const vector<DMatch>& b){
        return ((a[1].distance - a[0].distance) > (b[1].distance - b[0].distance));
    }
};

struct sort_descriptor_by_queryIdx
{
    inline bool operator()(const vector<DMatch>& a, const vector<DMatch>& b){
        return ( a[0].queryIdx < b[0].queryIdx );
    }
};

struct sort_lines_by_response
{
    inline bool operator()(const KeyLine& a, const KeyLine& b){
        return ( a.response > b.response );
    }
};

inline Mat SkewSymmetricMatrix(const cv::Mat &v)
{
    return (cv::Mat_<float>(3,3) <<  0, -v.at<float>(2), v.at<float>(1),
                        v.at<float>(2),               0,-v.at<float>(0),
                       -v.at<float>(1),  v.at<float>(0),             0);
}

inline double vector_mad(vector<double> residues)
{
    if(residues.size()!=0)
    {
        // Return the standard deviation of vector with MAD estimation
        int n_samples = residues.size();
        sort(residues.begin(), residues.end());
        double median = residues[n_samples/2];
        for(int i=0; i<n_samples; i++)
            residues[i] = fabs(residues[i]-median);
        sort(residues.begin(), residues.end());
        double MAD = residues[n_samples/2];
        return 1.4826*MAD;
    } else
        return 0.0;
}
