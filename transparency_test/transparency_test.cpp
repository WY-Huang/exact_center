/*
透明反射物体中心线提取（20230620）
*/

#include <opencv2/opencv.hpp>
#include <opencv2/plot.hpp>

float erodeDilate(cv::Mat image, uchar* row_ptr);

float erodeDilate(cv::Mat image, uchar* row_ptr)
{
    int rows = image.rows;
    int cols = image.cols;
    int laserWidth = 15;

    // cv::Mat element = cv::Mat::zeros(1, 1000, CV_8U);     // 定义零振幅结构元素
    // cv::Mat structuringElement(1, 11, CV_8UC1, cv::Scalar(0));
    cv::Mat structuringElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(laserWidth, 1));
    cv::Mat data_x(1, cols, CV_64F);
    cv::Mat data_y(1, cols, CV_64F);

    // 将一维数据转换为 cv::Mat 对象
    cv::Mat row_data(1, cols, CV_8UC1, row_ptr);
    // std::cout << "row_data: " << row_data << std::endl;
    
    cv::Mat row_close;        // 闭运算
    cv::morphologyEx(row_data, row_close, cv::MORPH_CLOSE, structuringElement);

    cv::Mat row_open;         // 开运算
    cv::morphologyEx(row_close, row_open, cv::MORPH_OPEN, structuringElement);

    /*// ============================================================
    cv::Mat dilated_row_close;        // 进行膨胀操作(闭运算)
    cv::dilate(row_data, dilated_row_close, structuringElement);
    // std::cout << "dilated_row: " << dilated_row << std::endl;
    
    cv::Mat eroded_row_close;         // 进行腐蚀操作
    cv::erode(dilated_row_close, eroded_row_close, structuringElement);

    cv::Mat eroded_row_open;         // 进行腐蚀操作（开运算）
    cv::erode(eroded_row_close, eroded_row_open, structuringElement);

    cv::Mat dilated_row_open;        // 进行膨胀操作
    cv::dilate(eroded_row_open, dilated_row_open, structuringElement);
    // =============================================================*/

    cv::Mat diff_row = row_close - row_open;

    cv::Mat sortedMat;
    cv::sortIdx(diff_row, sortedMat, cv::SORT_DESCENDING);  // 对矩阵进行降序排序

    int maxValueIndex1 = sortedMat.at<int>(0);  // 获取排序后的第一个最大值的索引
    int maxValueIndex2 = sortedMat.at<int>(1);  // 获取排序后的第二个最大值的索引
    // std::cout << "Max value 1 index: " << maxValueIndex1 << std::endl;
    // std::cout << "Max value 2 index: " << maxValueIndex2 << std::endl;

    int linePos = (maxValueIndex1 < maxValueIndex2) ? maxValueIndex1 : maxValueIndex2;
    // std::cout << "Min linePos: " << linePos << std::endl;

    //计算灰度重心
    float sum_valuecoor = 0;
    float sum_value = 0;
    for (int k = (-laserWidth/2);k < laserWidth/2;k++) 
    {
        int indexPos = linePos + k;
        sum_valuecoor += row_ptr[indexPos] * indexPos;
        sum_value += row_ptr[indexPos];
    }
    float x_centroid = sum_valuecoor / sum_value;
    // std::cout << "x_centroid: " << x_centroid << std::endl;

    int show = 1;
    if (show)
    {
        for (int j = 0; j < cols; j++)
            {
                uchar pixelValue = diff_row.at<uchar>(0, j);   // 获取当前像素的灰度值

                data_x.at<double>(0, j) = j;
                data_y.at<double>(0, j) = pixelValue;

                // std::cout << std::to_string(pixelValue) << std::endl;
            }

            cv::Mat plot_result;
            cv::Ptr<cv::plot::Plot2d> plot = cv::plot::Plot2d::create(data_x, data_y);
            //自定义参数
            plot->setPlotSize(1024, 300);
            plot->setShowText(false);

            plot->render(plot_result);//根据参数进行渲染
            cv::imshow("Plot Rows DilateErode", plot_result);
            cv::waitKey(100);
    }
    

    return x_centroid;

}

// 绘制图像一行的灰度值分布
void plotGrayCurve(cv::Mat img)
{
    int rows = img.rows;
    int cols = img.cols;

    cv::Mat data_x(1, cols, CV_64F);
    cv::Mat data_y(1, cols, CV_64F);

    cv::Point2f centerPoint;
    std::vector<cv::Point2f> centerLine;
    // 按行遍历图像
    for (int i = 0; i < rows; i++)
    {
        uchar* rowPtr = img.ptr<uchar>(i);  // 获取当前行的指针

        // test erode and dilate
        float centerPos = erodeDilate(img, rowPtr);
        centerPoint.x = i;
        centerPoint.y = centerPos;
        // std::cout << "centerPoint:" << centerPoint << std::endl;

        centerLine.push_back(centerPoint);

        int showPlot = 0;
        if (showPlot)
        {
            for (int j = 0; j < cols; j++)
            {
                uchar pixelValue = rowPtr[j];   // 获取当前像素的灰度值

                data_x.at<double>(0, j) = j;
                data_y.at<double>(0, j) = pixelValue;

                // std::cout << std::to_string(pixelValue) << std::endl;
            }
            // std::cout << "Row:" << i << std::endl;

            cv::Mat plot_result;
            cv::Ptr<cv::plot::Plot2d> plot = cv::plot::Plot2d::create(data_x, data_y);
            //自定义参数
            plot->setPlotSize(1024, 300);
            plot->setShowText(false);
            // plot->setShowGrid(true);
            // plot->setPlotBackgroundColor(cv::Scalar(255, 255, 255));
            // plot->setPlotLineColor(cv::Scalar(255, 0, 0));
            // plot->setPlotLineWidth(1);
            // plot->setInvertOrientation(true);//左右颠倒绘制

            plot->render(plot_result);//根据参数进行渲染
            cv::imshow("Plot Rows GrayValue", plot_result);
            cv::waitKey(0);
                    
        }
    }
    // cv::waitKey(0);

    // std::cout << "centerPoint:" << centerLine << std::endl;

    cv::Mat centerLineImage;
    cv::cvtColor(img, centerLineImage, cv::COLOR_GRAY2BGR);
    for (int i=0; i<centerLine.size(); i++)
    {
        cv::circle(centerLineImage, cv::Point(round(centerLine[i].y), centerLine[i].x), 0, cv::Scalar(0, 0, 255), -1, 8);
        // std::cout << pointcloud0[i].x << "\t" << round(pointcloud0[i].x) << std::endl;
    }

    cv::Mat rotatedImage;
    cv::rotate(centerLineImage, rotatedImage, cv::ROTATE_90_COUNTERCLOCKWISE);
    cv::imshow("centerLineImage", rotatedImage);
    // cv::imwrite("alg103_test2r_round.jpg", srcimg0);
    cv::waitKey(0);
}


int main()
{
    cv::Mat srcimg = cv::imread("/home/wanyel/vs_code/exact_center/transparency_test/test_img/2023_06_21_15_22_57_545.bmp");
    cv::Mat grayimg;
    cv::cvtColor(srcimg, grayimg, cv::COLOR_BGR2GRAY);

    cv::Mat rotatedImage;
    cv::rotate(grayimg, rotatedImage, cv::ROTATE_90_COUNTERCLOCKWISE);

    // 20-30ms
    clock_t begin, end;
    begin = clock();

    // algorithm test
    plotGrayCurve(rotatedImage);
    
    end = clock();
    std::cout << "algorithm costs:" << double(end - begin) / 1000 << "ms" << std::endl;

    // cv::Mat rotatedImage;
    // cv::rotate(srcimg, rotatedImage, cv::ROTATE_90_COUNTERCLOCKWISE);
    // cv::imshow("centerline", rotatedImage);
    // // cv::imwrite("alg103_test2r_round.jpg", srcimg0);
    // cv::waitKey(0);
    return 0;
}

