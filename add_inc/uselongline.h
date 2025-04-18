#pragma once

#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/line_descriptor.hpp>
#include <set>
#include <unordered_map>
#include <unordered_set>

void FilterShortLines(std::vector<Eigen::Vector4f>& lines, float length_thr);
std::vector<Eigen::Vector4f> convertVec4dToVec4f(const std::vector<Eigen::Vector4d>& vec_d) ;
std::vector<Eigen::Vector4d> convertVec4fToVec4d(const std::vector<Eigen::Vector4f>& vec_f) ;
// void LineExtractor(cv::Mat& image, std::vector<Eigen::Vector4d>& lines);
std::vector<Eigen::Vector4f> cvVecToEigenVec(const std::vector<cv::Vec<float, 4>>& cvVec);
std::vector<cv::Vec4f> convertEigenToCV(std::vector<Eigen::Vector4f>& eigenVec);
float PointLineDistance(Eigen::Vector4f line, Eigen::Vector2f point);
float AngleDiff(float &angle1, float &angle2);
Eigen::Vector4f MergeTwoLines(const Eigen::Vector4f &line1, const Eigen::Vector4f &line2);
void MergeLines(std::vector<cv::Vec4f> &source_lines, std::vector<cv::Vec4f> &dst_lines,
                float angle_threshold, float distance_threshold, float endpoint_threshold);
std::vector<cv::Vec4f> convertKeyLinesToVec4f(const std::vector<cv::line_descriptor::KeyLine>& keylines);
std::vector<cv::line_descriptor::KeyLine> convertVec4fToKeyLine(const std::vector<cv::Vec4f>& lines , cv::Mat img);
void optimizeAndMergeLines_lsd(std::vector<cv::line_descriptor::KeyLine>& lines, const cv::Mat& img);
void optimizeAndMergeLines_fld(std::vector<cv::Vec4f>& fld_lines);
