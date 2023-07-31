/*
透明反射物体中心线提取（start:20230620; last:20230724）
*/

#include <opencv2/opencv.hpp>
#include <opencv2/plot.hpp>
#include <boost/filesystem.hpp>

#include "edgeDetection.h"

namespace fs = boost::filesystem;

float erodeDilate(cv::Mat image, uchar* row_ptr);                                       // 开运算/闭运算提取特征峰
void plotGrayCurve(cv::Mat img);                                                        // 绘制图像一行的灰度值分布
void grayTransform(const cv::Mat &imgIn, cv::Mat &imgOut, int transformMode);           // 灰度变换
void sobelEdge(cv::Mat grayimg);                                                        // sobel算子
void imgRoi(const cv::Mat imgIn, cv::Mat &imgOut, int x, int y, int width, int height); // 图像ROI
void samallAreaRemove(const cv::Mat imgIn, cv::Mat &imgOut, int areaSize);              // 小面积轮廓去除
void houghCircles(const cv::Mat &grayImg);                                              // 霍夫圆检测
void onMouse(int event, int x, int y, int flags, void* param);                          // 鼠标事件的回调函数签名
void imgGrayVisualize(const cv::Mat grayImg);                                           // 点击显示像素点的灰度值


// 开运算/闭运算提取特征峰
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

    cv::Mat filterMat;
    int contourPosFinal = 10000;
    for (int n=0; n<20; n++)
    {
        int contourPos = sortedMat.at<int>(n);
        int grayValue = row_ptr[contourPos];
        // std::cout << grayValue << " ";
        if (grayValue >75 && grayValue < 125 && contourPos < contourPosFinal)
        {
            contourPosFinal = contourPos;
            
        }
        
    }
    // std::cout << contourPosFinal << " ";
    // std::cout << row_ptr[contourPosFinal] << std::endl;
    
    int maxValueIndex1 = sortedMat.at<int>(0);  // 获取排序后的第一个最大值的索引
    int maxValueIndex2 = sortedMat.at<int>(1);  // 获取排序后的第二个最大值的索引
    // std::cout << "Max value 1 index: " << maxValueIndex1 << std::endl;
    // std::cout << "Max value 2 index: " << maxValueIndex2 << std::endl;

    int linePos = (maxValueIndex1 < maxValueIndex2) ? maxValueIndex1 : maxValueIndex2;
    // std::cout << "Min linePos: " << linePos << std::endl;
    linePos = contourPosFinal;

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

    int show = 0;
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

            cv::putText(plot_result, std::to_string(i),cv::Point(0, 100), 3, 2, cv::Scalar(0, 255, 255));
            cv::imshow("Plot Rows GrayValue", plot_result);
            cv::waitKey(100);
                    
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
    cv::rotate(centerLineImage, rotatedImage, cv::ROTATE_90_CLOCKWISE);
    cv::imshow("centerLineImage", rotatedImage);
    // cv::imwrite("alg103_test2r_round.jpg", srcimg0);
    cv::waitKey(0);
}

// 灰度变换
void grayTransform(const cv::Mat &imgIn, cv::Mat &imgOut, int transformMode)
{
    switch (transformMode)
    {
        // 灰度反转 1
        case 1:
            imgOut = imgIn.clone();
            for (int i = 0; i < imgIn.rows; i++)
            {
                for (int j = 0; j < imgIn.cols; j++)
                {
                    imgOut.at<uchar>(i, j) = 255 - imgIn.at<uchar>(i, j);  //灰度反转
                }
            }
            break;
        // 灰度对数变换 2
        case 2:
            imgOut = imgIn.clone();
            for (int i = 0; i < imgIn.rows; i++)
            {
                for (int j = 0; j < imgIn.cols; j++)
                {
                    imgOut.at<uchar>(i, j) = 2 * log((double)(imgIn.at<uchar>(i, j)) + 1) / log(100);  //对数变换 s=6*log(r+1)
                }
            }
            cv::normalize(imgOut, imgOut, 0, 255, cv::NORM_MINMAX);  //图像归一化，转到0~255范围内
            cv::convertScaleAbs(imgOut, imgOut);  //数据类型转换到CV_8U
            break;

        // 灰度幂律变换 3
        case 3:
            imgOut = imgIn.clone();
            for (int i = 0; i < imgIn.rows; i++)
            {
                for (int j = 0; j < imgIn.cols; j++)
                {
                    imgOut.at<uchar>(i, j) = 6 * pow((double)imgIn.at<uchar>(i, j), 0.5);  //幂律变换 s=6*r^0.5
                }
            }
            cv::normalize(imgOut, imgOut, 0, 255, cv::NORM_MINMAX);  //图像归一化，转到0~255范围内
            cv::convertScaleAbs(imgOut, imgOut);  //数据类型转换到CV_8U
            break;
        
        // 直方图均衡化 4
        case 4:
            cv::equalizeHist(imgIn, imgOut);
            break;

        default:
            break;
    }

	cv::imshow("grayTransform", imgOut);  //显示图像
    // cv::waitKey(0);
}

// sobel算子
void sobelEdge(cv::Mat grayimg)
{
    cv::Mat grad_x, grad_y;
    cv::Mat abs_grad_x, abs_grad_y, dst;

    //求x方向梯度
    cv::Sobel(grayimg, grad_x, CV_16S, 1, 0, 3, 1, 1, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_x, abs_grad_x);
    cv::imshow("x方向soble", abs_grad_x);

    //求y方向梯度
    cv::Sobel(grayimg, grad_y, CV_16S, 0, 1, 3, 1, 1, cv::BORDER_DEFAULT);
    cv::convertScaleAbs(grad_y, abs_grad_y);
    cv::imshow("y向soble", abs_grad_y);

    //合并梯度
    cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, dst);
    cv::imshow("整体方向soble", dst);
}

// ROI, 定义ROI区域的位置和大小
void imgRoi(const cv::Mat imgIn, cv::Mat &imgOut, int x, int y, int width, int height, string savePath="")
{
    // 创建ROI区域的矩形
    cv::Rect roiRect(x, y, width, height);

    // 提取ROI区域
    cv::Mat roi = imgIn(roiRect);

    // 显示原始图像和ROI区域
    cv::imshow("Original Image", imgIn);
    cv::imshow("ROI", roi);
    if (savePath.empty() == false)
    {
        cv::imwrite(savePath, roi);
    }
    
    cv::waitKey(0);
}

/** @brief 小面积区域移除
@param imgIn 输入图像
@param imgOut 输出图像
@param areaSize 面积阈值
*/
void samallAreaRemove(const cv::Mat imgIn, cv::Mat &imgOut, int areaSize)
{
    cv::threshold(imgIn, imgOut, 5, 255, cv::THRESH_BINARY);
	// cv::imshow("binary", imgOut);   

	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));   //针对高亮部分腐蚀
	cv::erode(imgOut, imgOut, element);
	// cv::imshow("erode", imgOut);
 
	// 提取连通区域，并剔除小面积联通区域
	std::vector<std::vector<cv::Point>> contours;           //二值图像轮廓的容器
	std::vector<cv::Vec4i> hierarchy;                  //4个int向量，分别表示后、前、父、子的索引编号
	cv::findContours(imgOut, contours, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);             //检测所有轮廓
	//contours.erase(remove_if(contours.begin(), contours.end(),[](const vector<Point>& c) {return contourArea(c) < 800; }), contours.end());  //vector.erase 删除元素
	// 显示图像并保存
	/*imgHSVMask.setTo(0);
	drawContours(imgHSVMask, contours, -1, Scalar(255), FILLED);
	imshow("处理图", imgHSVMask); */
 
	cv::Mat ImageContours = cv::Mat::zeros(imgOut.size(), CV_8UC1);  //绘制
	cv::Mat ImgContours= cv::Mat::zeros(imgOut.size(), CV_8UC1);
 
	std::vector<std::vector<cv::Point>>::iterator k;                    //迭代器，访问容器数据
 
	for (k = contours.begin(); k != contours.end();)      //遍历容器,设置面积因子
	{
		if (cv::contourArea(*k, false) < areaSize)
		{
            //删除指定元素，返回指向删除元素下一个元素位置的迭代器，删除小面积轮廓
			k = contours.erase(k);
		}
		else
			++k;
	}

    //contours[i]代表第i个轮廓，contours[i].size()代表第i个轮廓上所有的像素点
	for (int i = 0; i < contours.size(); i++)
	{
		for (int j = 0; j < contours[i].size(); j++)
		{
			//获取轮廓上点的坐标
			cv::Point contourPoint = cv::Point(contours[i][j].x, contours[i][j].y);
			ImgContours.at<uchar>(contourPoint) = 255;
		}
		cv::drawContours(ImageContours, contours, i, cv::Scalar(255), -1, 8);   // 填充轮廓内部
	}

    // 提取目标轮廓线
    vector<Point2f> contourIndex;
    cv::Mat plotImg;
    cv::cvtColor(imgIn, plotImg, cv::COLOR_GRAY2BGR);
    for (int i = 0; i < ImgContours.cols; i++)
    {
        for (int j = 0; j < ImgContours.rows; j++)
        {
            if(ImgContours.at<uchar>(j, i) == 255)
            {
                contourIndex.push_back(cv::Point2f(i, j));
                circle(plotImg, cv::Point2f(i, j), 1, Scalar(0, 0, 255), 1, 8, 0);
                break;
            }
        }
    }

    // 计算灰度重心
    int laserWidth = 15;
    for (int pointX = 0; pointX < contourIndex.size(); pointX++)
    {
        float sum_valuecoor = 0;
        float sum_value = 0;
        for (int k = 0; k < laserWidth; k++) 
        {
            int indexPos = contourIndex[pointX].y + k;
            sum_valuecoor += imgIn.at<uchar>(indexPos, contourIndex[pointX].x) * indexPos;
            sum_value += imgIn.at<uchar>(indexPos, contourIndex[pointX].x);
        }
        float y_centroid = sum_valuecoor / sum_value;

        circle(plotImg, cv::Point2f(contourIndex[pointX].x, y_centroid), 1, Scalar(0, 255, 0), 1, 8, 0);
    }


	// cv::imshow("ImageContoursFilled", ImageContours);
	cv::imshow("Contours", ImgContours);
    cv::imshow("plotImg", plotImg);
	cv::waitKey(0);
}

/**
 @brief 霍夫圆形检测
*/
void houghCircles(const cv::Mat &grayImg) 
{
	std::vector<cv::Vec3f> circles;    //minDist 和 param2 数值的设定是关键
	cv::HoughCircles(grayImg, circles, cv::HOUGH_GRADIENT, 1, 200, 100, 30, 500, 0);
	cv::Mat srcImgTri;
    cv::cvtColor(grayImg, srcImgTri, cv::COLOR_GRAY2BGR);

	for (int i = 0; i < circles.size(); i++) 
    {
		cv::circle(srcImgTri, cv::Point(circles[i][0], circles[i][1]), circles[i][2], cv::Scalar(0, 0, 255), 2);
	}
	cv::imshow("houghCircles", srcImgTri);
}

 
// 定义鼠标点击的回调函数
void onMouse(int event, int x, int y, int flags, void* param)
{
	cv::Mat *im = reinterpret_cast<cv::Mat*>(param);
	switch (event)
	{
		case cv::EVENT_LBUTTONDOWN:
			std::cout << "Img at (" << x << "," << y << ") value is: " 
				      << static_cast<int>(im->at<uchar>(cv::Point(x, y))) << std::endl;

            // 图上显示灰度值
            cv::circle(*im, cv::Point(x, y), 2, cv::Scalar(0, 0, 255));
            cv::putText(*im, std::to_string(im->at<uchar>(cv::Point(x, y))), cv::Point(x, y), 
                        cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0));
			break;

		case cv::EVENT_RBUTTONDOWN:
			std::cout << "input(x,y)" << std::endl;
			std::cout << "x = " << std::endl;
			std::cin >> x;
			std::cout << "y = " << std::endl;
			std::cin >> y;
			std::cout << "Img at(" << x << "," << y << ") value is: "
				      << static_cast<int>(im->at<uchar>(cv::Point(x,y))) << std::endl;
			break;

        default:
            break;			
	}
}

// 点击显示像素点的灰度值
void imgGrayVisualize(const cv::Mat grayImg)
{
    cv::Mat bgrImgShow;
    cv::cvtColor(grayImg, bgrImgShow, cv::COLOR_GRAY2BGR);  // gray to BGR
    cv::namedWindow("grayimg");        
    cv::setMouseCallback("grayimg", onMouse, reinterpret_cast<void*>(&bgrImgShow));   // 注册回调函数 
    while (true)
    {
        cv::imshow("grayimg", bgrImgShow);
        int key = cv::waitKey(30);
        if (key == 'q')
        {
            break;
        }
    }
}


int main()
{
    // 批量读取文件
    std::string folderPath = "/home/wanyel/vs_code/exact_center/transparency_test/test_img/NBU_20230720_location/ROI";

    fs::path directory(folderPath);

    // 检查文件夹是否存在
    if (!fs::exists(directory) || !fs::is_directory(directory)) 
    {
        std::cout << "Folder does not exist: " << folderPath << std::endl;
        return 1;
    }

    // 遍历文件夹内的所有文件和子目录
    for (const auto& entry : fs::directory_iterator(directory)) 
    {
        std::string filePath = entry.path().string();

        // 检查是否为普通文件
        if (fs::is_regular_file(entry)) 
        {
            // 在这里进行文件的读取和处理
            std::cout << "Reading file: " << filePath << std::endl;            
            std::string imgPath = filePath;

            cv::Mat srcimg = cv::imread(imgPath);
            cv::Mat grayimg;
            cv::cvtColor(srcimg, grayimg, cv::COLOR_BGR2GRAY);
            cv::imshow("grayimg", grayimg);

            // cv::GaussianBlur(grayimg, grayimg, cv::Size(3, 3), 0);          // 高斯模糊处理

            /* ==================== 匹配杯壁上的定位点对 ======================== */
            // EdgeDetection ed(grayimg);                                       // 边缘检测，最小外接矩形
            // // ed.cannyProcess(240, 255);
            // ed.thresholdSeg(250, 255);
            // ed.getContours();
            // ed.findTargetPairPoint();
            // std::vector<cv::Point2f> targetPoint;                            // 寻找目标点对
            // targetPoint = ed.getTagetPoints();


            /* ==================== 根据定位点对进行ROI处理 ===================== */
            // cv::Mat grayRoi;                                             // ROI处理
            // string savePath = imgPath.substr(0, imgPath.rfind(".")) + "_ROI_.bmp";
            // int roiX = targetPoint[0].x + 50;
            // int roiY = targetPoint[0].y;
            // int roiWidth = targetPoint[1].x - targetPoint[0].x - 100;
            // int roiHeight = 200;
            // imgRoi(grayimg, grayRoi, roiX, roiY, roiWidth, roiHeight, savePath);


            // cv::Mat thresholdSegImg;                                        // 普通阈值分割
            // cv::threshold(grayimg, thresholdSegImg, 250, 255, cv::THRESH_TOZERO);
            // cv::imshow("thresholdSegImg", thresholdSegImg);
            // cv::waitKey(0);

            // imgGrayVisualize(grayimg);                                   // 点击显示像素点灰度值           
            
            // houghCircles(grayimg);                                       // 圆形检测

            // sobelEdge(grayimg);                                          // sobel边缘检测

            /* ============= 灰度变换->小轮廓移除->灰度质心法提取轮廓中心线 ============ */
            cv::Mat counterGrayImg;
            grayTransform(grayimg, counterGrayImg, 2);                   // 灰度变换

            // cv::Mat cannyImg;
            // cv::Canny(counterGrayImg, cannyImg, 5, 100);                 // canny边缘检测
            // cv::imshow("cannyImg", cannyImg);

            cv::Mat filterImg;
            samallAreaRemove(grayimg, filterImg, 5000);                  // 移除小面积斑点，提取轮廓线


            // cv::Mat rotatedImage;                                        // 旋转图像
            // cv::rotate(counterGrayImg, rotatedImage, cv::ROTATE_90_COUNTERCLOCKWISE);

            /* ============== 算法耗时分析 ========================== */
            // 20-30ms
            clock_t begin, end;
            begin = clock();

            // algorithm test
            // plotGrayCurve(rotatedImage);
            
            end = clock();
            std::cout << "algorithm costs:" << double(end - begin) / 1000 << "ms" << std::endl;

            // cv::Mat rotatedImage;
            // cv::rotate(srcimg, rotatedImage, cv::ROTATE_90_COUNTERCLOCKWISE);
            // cv::imshow("centerline", rotatedImage);
            // cv::imwrite("alg103_test2r_round.jpg", srcimg0);
            // cv::waitKey(0);

        }
    }

    return 0;
}

