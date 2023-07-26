#include "edgeDetection.h"

EdgeDetection::EdgeDetection(cv::Mat image)
{
    m_img = image;
    cv::cvtColor(image, bgr_img, cv::COLOR_GRAY2BGR);
}

bool EdgeDetection::cannyProcess(unsigned int downThreshold, unsigned int upThreshold)
{
    bool ret = true;

    if (m_img.empty())
    {
        ret = false;
    }

    cv::Canny(m_img, m_canny, downThreshold, upThreshold);
    cv::imshow("Canny", m_canny);

    return ret;
}

bool EdgeDetection::thresholdSeg(unsigned int downThreshold, unsigned int upThreshold)
{
    bool ret = true;

    if (m_img.empty())
    {
        ret = false;
    }
    cv::threshold(m_img, m_canny, downThreshold, upThreshold, cv::THRESH_TOZERO);

    return ret;
}

bool EdgeDetection::getContours()
{
    bool ret = true;
    if (m_canny.empty())
    {
        ret = false;
    }

    cv::Mat k = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(-1, -1));
    cv::dilate(m_canny, m_canny, k);
    imshow("dilate", m_canny);

    // cv::erode(m_canny, m_canny, k);
    // imshow("erode", m_canny);

    // 轮廓发现与绘制
    vector<vector<cv::Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(m_canny, contours, cv::RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());

    for (size_t i = 0; i < contours.size(); ++i)
    {
        // 最大外接轮廓
        // cv::Rect rect = cv::boundingRect(contours[i]);
        // cv::rectangle(m_img, rect, cv::Scalar(0,255,0), 2, LINE_8);

        // 最小外接轮廓
        RotatedRect rrt = minAreaRect(contours[i]);
        Point2f pts[4];
        rrt.points(pts);    // 矩形四个点的位置，左下，左上，右上，右下
        // 绘制旋转矩形与中心位置
        for (int i = 0; i < 4; i++) 
        {
            line(bgr_img, pts[i % 4], pts[(i + 1) % 4], Scalar(0, 0, 255), 1, 8, 0);
        }
        Point2f cpt = rrt.center;
        circle(bgr_img, cpt, 2, Scalar(255, 0, 0), 1, 8, 0);

        float rectWidth = pts[2].x - pts[1].x;
        float rectHigth = pts[1].y - pts[0].y;
        // if (rectWidth)
    }

    imshow("contours", bgr_img);
    waitKey(0);
    return ret;
}

EdgeDetection::~EdgeDetection()
{
}