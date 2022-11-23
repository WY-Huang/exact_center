#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <time.h>
#include <string>

using std::placeholders::_1;
using namespace cv;

class LaserCenter : public rclcpp::Node
{
public:
    LaserCenter();


private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    Mat cvImage;

    void topic_callback(const sensor_msgs::msg::Image msg);

    // 高斯滤波+大津阈值+灰度质心法
    void gray_centroid(Mat img);
    int getOTSUthread(const Mat& src);// 大津阈值法2
    int GetMatOTSU(const Mat& img);// 大津阈值法1
    int thd_otsu;
    int GetLineOTSU(const Mat& img, int index);// 单行大津阈值法

    // 高斯滤波+zhang细化+灰度质心法
    void thining_gray_centroid(Mat img);
    void zhang(Mat& input, Mat& output);
    double ijpixel(double& x, double& y, Mat& m);
    void CalcNormVec(Point2d&& ptA, Point2d&& ptB, Point2d&& ptC, double& pfCosSita, double& pfSinSita);
    void thining_point(Mat& inputimg, std::vector<Point2d>& pt);

    // 大津阈值法3
    void getHistogram(Mat &src, int *dst);
    int myOtsu(Mat & src);
    
    time_t begin, end, gauss_time, otsu_time;
    double gauss_cost, otsu_cost, gray_cost, total_cost;
    std::string fps;

};