/*
透明反射物体中心线提取（20230620）
*/

#include <opencv2/opencv.hpp>
#include <opencv2/plot.hpp>

void erodeDilate(cv::Mat image, uchar* row_ptr);


// 绘制图像一行的灰度值分布
void plotGrayCurve(cv::Mat img)
{
    int rows = img.rows;
    int cols = img.cols;

    cv::Mat data_x(1, cols, CV_64F);
    cv::Mat data_y(1, cols, CV_64F);

    // 按行遍历图像
    for (int i = 0; i < rows; i++)
    {
        uchar* rowPtr = img.ptr<uchar>(i);  // 获取当前行的指针

        // test erode and dilate
        erodeDilate(img, rowPtr);

        for (int j = 0; j < cols; j++)
        {
            uchar pixelValue = rowPtr[j];   // 获取当前像素的灰度值

            data_x.at<double>(0, j) = j;
            data_y.at<double>(0, j) = pixelValue;

            // std::cout << std::to_string(pixelValue) << std::endl;
        }

        cv::Mat plot_result;
        cv::Ptr<cv::plot::Plot2d> plot = cv::plot::Plot2d::create(data_x, data_y);
        //自定义参数
        plot->setPlotSize(1024, 256);
        plot->setShowText(false);
        // plot->setShowGrid(true);
        // plot->setPlotBackgroundColor(cv::Scalar(255, 255, 255));
        // plot->setPlotLineColor(cv::Scalar(255, 0, 0));
        // plot->setPlotLineWidth(1);
        // plot->setInvertOrientation(true);//左右颠倒绘制

        plot->render(plot_result);//根据参数进行渲染
        cv::imshow("Plot Rows GrayValue", plot_result);
        cv::waitKey(100);
    }
    cv::waitKey(0);
}


void erodeDilate(cv::Mat image, uchar* row_ptr)
{
    int rows = image.rows;
    int cols = image.cols;

    // cv::Mat element = cv::Mat::zeros(1, 1000, CV_8U);     // 定义零振幅结构元素
    // cv::Mat structuringElement(1, 11, CV_8UC1, cv::Scalar(0));
    cv::Mat structuringElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(11, 1));
    cv::Mat data_x(1, cols, CV_64F);
    cv::Mat data_y(1, cols, CV_64F);

    // 将一维数据转换为 cv::Mat 对象
    cv::Mat row_data(1, cols, CV_8UC1, row_ptr);
    // std::cout << "row_data: " << row_data << std::endl;
    
    cv::Mat dilated_row;        // 进行膨胀操作
    cv::dilate(row_data, dilated_row, structuringElement);
    // std::cout << "dilated_row: " << dilated_row << std::endl;
    
    cv::Mat eroded_row;         // 进行腐蚀操作
    cv::erode(dilated_row, eroded_row, structuringElement);

    for (int j = 0; j < cols; j++)
    {
        uchar pixelValue = dilated_row.at<uchar>(0, j);   // 获取当前像素的灰度值

        data_x.at<double>(0, j) = j;
        data_y.at<double>(0, j) = pixelValue;

        // std::cout << std::to_string(pixelValue) << std::endl;
    }

    cv::Mat plot_result;
    cv::Ptr<cv::plot::Plot2d> plot = cv::plot::Plot2d::create(data_x, data_y);
    //自定义参数
    plot->setPlotSize(1024, 256);
    plot->setShowText(false);

    plot->render(plot_result);//根据参数进行渲染
    cv::imshow("Plot Rows DilateErode", plot_result);
    // cv::waitKey(0);

}



int main()
{
    cv::Mat srcimg = cv::imread("/home/wanyel/vs_code/exact_center/transparency_test/test_img/2023_06_26_14_34_55_598.bmp");
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

#include <opencv2/opencv.hpp>

int mainaa()
{
    // 创建待处理的一维行数据
    cv::Mat row_data = (cv::Mat_<uchar>(1, 20) << 1,   1,   1,   1,   1,   1,   0,   1,   1,   1,   1,   1,   0,   1,   1,   1,   1,   1,   0,   1,   1,   1,   0,   1,   0,   0,   0,   1,   1,   1,   0,   1,   1,   1,   1,   1,   1,   1,   0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   0,   1,   1,   1,   1,   1,   1,   1,   1,   0,   1,   1,   1,   0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   0,   1,   1,   1,   1,   1,   1,   1,   0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2,   2,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   1,   1,   1,   2,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   1,   1,   1,   1,   1,   1,   2,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   1,   1,   1,   1,   2,   2,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   1,   1,   2,   1,   1,   2,   1,   2,   1,   2,   2,   2,   2,   3,   4,   4,   4,   4,   3,   4,   4,   4,   5,   5,   5,   4,   4,   5,   4,   3,   4,   3,   2,   3,   4,   4,   3,   3,   3,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   1,   2,   2,   1,   2,   2,   2,   2,   2,   1,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   1,   2,   2,   2,   2,   2,   3,   2,   2,   1,   2,   2,   2,   2,   2,   2,   2,   2,   3,   2,   2,   2,   3,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   1,   2,   2,   2,   2,   2,   2,   2,   3,   3,   3,   3,   2,   2,   3,   2,   3,   2,   2,   2,   2,   2,   3,   3,   3,   3,   2,   3,   2,   2,   2,   2,   2,   3,   3,   3,   3,   2,   2,   3,   3,   2,   3,   2,   3,   3,   3,   3,   3,   3,   2,   3,   2,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   4,   3,   3,   2,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   4,   4,   4,   4,   4,   3,   4,   3,   4,   4,   3,   4,   4,   4,   4,   4,   4,   4,   5,   3,   4,   4,   4,   5,   4,   3,   4,   4,   4,   5,   4,   4,   3,   4,   4,   3,   5,   4,   5,   5,   5,   5,   4,   6,   5,   5,   5,   4,   4,   5,   5,   4,   5,   6,   5,   5,   6,   6,   5,   5,   5,   6,   6,   5,   6,   7,   7,   6,   5,   5,   6,   6,   7,   6,   6,   6,   7,   6,   7,   6,   7,   7,   6,   8,   8,   7,   6,   7,   7,   7,   7,   8,   6,   7,   7,   7,   7,   8,   9,   8,   7,   9,   8,   9,   9,   9,   8,  10,   7,  10,  10,  10,   9,   7,   9,  10,   9,  11,  13,  12,  12,  10,  15,  15,  14,  13,  13,  13,  15,  14,  15,  12,  14,  13,  17,  15,  16,  16,  15,  18,  18,  17,  19,  16,  22,  15,  16,  22,  20,  18,  21,  16,  18,  20,  20,  19,  23,  23,  16,  18,  18,  19,  19,  17,  17,  20,  20,  25,  20,  18,  20,  19,  14,  17,  19,  16,  17,  22,  17,  17,  19,  23,  18,  18,  22,  20,  21,  21,  22,  20,  20,  20,  18,  21,  21,  22,  18,  19,  21,  17,  18,  26,  24,  22,  20,  20,  18,  29,  20,  23,  30,  25,  33,  38,  39,  38,  32,  42,  49,  62,  76, 118, 122, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 215, 166, 230, 176, 192, 144, 213, 177, 148, 157, 118,  66,  73,  83, 133, 112,  77,  61,  47,  37,  36,  38,  29,  25,  25,  23,  22,  25,  28,  21,  20,  24,  20,  22,  25,  26,  20,  20,  21,  21,  22,  24,  23,  23,  21,  22,  21,  21,  25,  26,  27,  23,  21,  23,  23,  27,  20,  28,  31,  25,  25,  22,  21,  31,  25,  25,  31,  31,  21,  25,  28,  25,  17,  23,  17,  22,  22,  23,  23,  21,  21,  20,  19,  20,  19,  18,  19,  18,  17,  16,  16,  15,  15,  16,  16,  13,  15,  13,  13,  12,  11,  13,  12,  12,  12,  14,  13,  12,  10,  13,  11,  11,  11,  10,  11,  10,  10,  10,  11,  10,   9,   8,   8,   8,   8,   8,  10,   8,  10,   9,  10,  10,   9,   9,   8,   9,   9,   7,   9,   7,   8,   8,   7,   8,   7,   8,   8,   7,   7,   6,   6,   7,   6,   8,   7,   6,   6,   6,   6,   7,   5,   6,   5,   6,   7,   6,   5,   6,   6,   6,   6,   6,   4,   5,   5,   5,   5,   5,   6,   6,   6,   5,   5,   5,   5,   6,   5,   5,   5,   4,   5,   4,   4,   4,   4,   5,   4,   5,   5,   5,   5,   4,   5,   4,   5,   5,   4,   4,   4,   3,   4,   4,   5,   4,   4,   4,   4,   4,   5,   4,   4,   5,   4,   5,   3);

    // 创建结构元素
    cv::Mat structuringElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 1));
    // cv::Mat structuringElement = cv::Mat::zeros(1, 5, CV_8U);

    // 创建存储结果的变量
    cv::Mat dilated_row;

    // 执行膨胀操作
    cv::dilate(row_data, dilated_row, structuringElement);

    // 输出结果
    std::cout << "Dilated Row: " << dilated_row << std::endl;

    return 0;
}
