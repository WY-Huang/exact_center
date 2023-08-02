#include <opencv2/opencv.hpp>
#include <opencv2/plot.hpp>


float erodeDilate(cv::Mat image, uchar* row_ptr);                                       // 开运算/闭运算提取特征峰
void plotGrayCurve(cv::Mat img);                                                        // 绘制图像一行的灰度值分布
void grayTransform(const cv::Mat &imgIn, cv::Mat &imgOut, int transformMode);           // 灰度变换
void sobelEdge(cv::Mat grayimg);                                                        // sobel算子
void imgRoi(const cv::Mat imgIn, cv::Mat &imgOut, int x, int y, int width, int height); // 图像ROI
void samallAreaRemove(const cv::Mat imgIn, cv::Mat &imgOut, int areaSize);              // 小面积轮廓去除
void houghCircles(const cv::Mat &grayImg);                                              // 霍夫圆检测
void onMouse(int event, int x, int y, int flags, void* param);                          // 鼠标事件的回调函数签名
void imgGrayVisualize(const cv::Mat grayImg);                                           // 点击显示像素点的灰度值