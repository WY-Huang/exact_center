#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;
using namespace cv;


class EdgeDetection
{
    cv::Mat m_img;
    cv::Mat bgr_img;
    cv::Mat m_canny;
    // cv::Mat thresholdSegImg;
    vector<Point2f> candidatePairPoint;
    vector<Point2f> targetCenter;

public:
    EdgeDetection(cv::Mat image);
    bool cannyProcess(unsigned int downThreshold, unsigned int upThreshold);
    bool thresholdSeg(unsigned int downThreshold, unsigned int upThreshold);
    bool getContours();
    bool findTargetPairPoint();

    ~EdgeDetection();
};