#include <opencv2/opencv.hpp>


int main()
{
    cv::Mat img = cv::imread("/home/wanyel/vs_code/exact_center/allData/srcImg/src1.bmp", cv::IMREAD_COLOR);

    cv::Mat new_image = cv::Mat::zeros(img.size(), img.type());

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

    cv::Mat lookUpTable(1, 256, CV_8U);
    uchar* p = lookUpTable.ptr();
    for( int i = 0; i < 256; ++i)
    p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, 0.67) * 255.0);

    cv::Mat res = img.clone();
    cv::LUT(img, lookUpTable, res);

    cv::imshow("Original Image", img);
    cv::imshow("New Image", new_image);
    cv::imshow("New  res Image", res);
    cv::waitKey();
}