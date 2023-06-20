/*
透明反射物体中心线提取（20230620）
*/

#include <opencv2/opencv.hpp>
#include <opencv2/plot.hpp>

// 绘制图像一行的灰度值分布
void plotGrayCurve(cv::Mat img)
{
    int rows = img.rows;
    int cols = img.cols;

    cv::Mat data_x(1, cols, CV_64F);
    cv::Mat data_y(1, cols, CV_64F);

    // 按行遍历图像
    for (int i = 0; i < 1; i++)
    {
        uchar* rowPtr = img.ptr<uchar>(i);  // 获取当前行的指针

        for (int j = 0; j < cols; j++)
        {
            uchar pixelValue = rowPtr[j];   // 获取当前像素的灰度值

            data_x.at<double>(0, j) = j;
            data_y.at<double>(0, j) = pixelValue;
        }
    }

    cv::Mat plot_result;

    cv::Ptr<cv::plot::Plot2d> plot = cv::plot::Plot2d::create(data_x,data_y);
    //自定义参数
    plot->setShowText(false);
    // plot->setShowGrid(true);
    // plot->setPlotBackgroundColor(cv::Scalar(255, 255, 255));
    // plot->setPlotLineColor(cv::Scalar(255, 0, 0));
    // plot->setPlotLineWidth(1);
    // plot->setInvertOrientation(true);//左右颠倒绘制

    plot->render(plot_result);//根据参数进行渲染
    cv::imshow("plot 2d data", plot_result);
    cv::waitKey(0);
}




int main()
{
    cv::Mat srcimg = cv::imread("/home/wanyel/vs_code/exact_center/allData/srcImg/bmp/test2r.jpg");
    cv::Mat grayimg;
    cv::cvtColor(srcimg, grayimg, cv::COLOR_BGR2GRAY);

    // 20-30ms
    clock_t begin, end;
    begin = clock();

    // algorithm test
    plotGrayCurve(grayimg);
    
    end = clock();
    std::cout << "algorithm costs:" << double(end - begin) / 1000 << "ms" << std::endl;

    // cv::Mat rotatedImage;
    // cv::rotate(srcimg, rotatedImage, cv::ROTATE_90_COUNTERCLOCKWISE);
    // cv::imshow("centerline", rotatedImage);
    // // cv::imwrite("alg103_test2r_round.jpg", srcimg0);
    // cv::waitKey(0);
    return 0;
}