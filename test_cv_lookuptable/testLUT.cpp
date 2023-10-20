/*
测试图片逐像素转换运算时，for循环和LUT的耗时对比。20231017
结论：LUT比ForLoop快一个数量级
*/


#include <opencv2/opencv.hpp>


int main()
{
    cv::Mat img = cv::imread("/home/wanyel/vs_code/exact_center/allData/srcImg/src1.bmp", cv::IMREAD_COLOR);

    cv::Mat new_image = cv::Mat::zeros(img.size(), img.type());

    // for loop time cost
    double t1 = (double)cv::getTickCount();

    for( int y = 0; y < img.rows; y++ ) 
    {
        for( int x = 0; x < img.cols; x++ ) 
        {
            for( int c = 0; c < img.channels(); c++ ) 
            {
                new_image.at<cv::Vec3b>(y,x)[c] =
                cv::saturate_cast<uchar>(0.8 * img.at<cv::Vec3b>(y,x)[c] + 50 );
            }
        }
    }

    double t2 = ((double)cv::getTickCount() - t1) / cv::getTickFrequency();
    std::cout << "For loop Times passed in seconds t2: " << t2 << std::endl;


    cv::Mat res = img.clone();

    // LUT time cost
    double t3 = (double)cv::getTickCount();

    cv::Mat lookUpTable(1, 256, CV_8U);
    uchar* p = lookUpTable.ptr();
    for( int i = 0; i < 256; ++i)
    p[i] = cv::saturate_cast<uchar>(0.8 * i + 50);

    cv::LUT(img, lookUpTable, res);

    double t4 = ((double)cv::getTickCount() - t3) / cv::getTickFrequency();
    std::cout << "LUT Times passed in seconds t4: " << t4 << std::endl;


    cv::imshow("Original Image", img);
    cv::imshow("New Image", new_image);
    cv::imshow("New res Image", res);
    cv::waitKey(0);
}