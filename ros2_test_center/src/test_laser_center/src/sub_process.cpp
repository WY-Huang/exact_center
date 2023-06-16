/*
ros2 param set /gpio_raspberry_node laser true
ros2 param set /camera_tis_node power true

colcon build
source install/setup.bash
ros2 run test_laser_center sub_process
*/

#include "test_laser_center/sub_process.hpp"

#define QUICK_TRANSMIT    //快速传输
// video/compressed   camera_tis_node/image    rotate_image_node/image_rotated


LaserCenter::LaserCenter() : Node("sub_images")
{
	subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera_tis_node/image", rclcpp::SensorDataQoS(), std::bind(&LaserCenter::topic_callback, this, _1)
    );

	save = 1;
}

void LaserCenter::topic_callback(const sensor_msgs::msg::Image msg)
{
    
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, msg.encoding);

    cvImage = cv_ptr->image.clone();
    // RCLCPP_INFO(this->get_logger(), "receive images...");

    cv::Mat rotatedImage;
    cv::rotate(cvImage, rotatedImage, cv::ROTATE_90_COUNTERCLOCKWISE);
	// imshow("srcimg", rotatedImage);
    // waitKey(5);
	// if (num_img < 300)
	// 	grap_images(cvImage);
    
    // gray_centroid(cvImage);

    // thining_gray_centroid(cvImage);

	std::vector <cv::Point2f> pointcloud0;
    std::vector <Targetpoint> namepoint0;
    bool solderjoints0;

    // 20-30ms
    clock_t begin, end;
    begin = clock();

    alg103_runimage(rotatedImage, pointcloud0, namepoint0, solderjoints0, 1);

    end = clock();
    std::cout << "alg103runimage costs:" << double(end - begin) / 1000 << "ms" << std::endl;

    // for (int i=0; i<pointcloud0.size(); i++)
    // {
    //     cv::circle(rotatedImage, cv::Point(round(pointcloud0[i].x), pointcloud0[i].y), 0, cv::Scalar(0, 0, 255), -1, 8);
    //     // std::cout << pointcloud0[i].x << "\t" << round(pointcloud0[i].x) << std::endl;
    // }
	// cv::Mat rotatedImage2;
    // cv::rotate(rotatedImage, rotatedImage2, cv::ROTATE_90_COUNTERCLOCKWISE);
	// std::cout << rotatedImage2.size() << std::endl;
	// cv::Mat rotatedImages = rotatedImage.clone();
	if (save < 10)
	{
		cv::imwrite(std::to_string(save) + "alg103_test.jpg", rotatedImage);
		save++;
	}

	// cv::namedWindow("centerline", cv::WINDOW_NORMAL);  // 创建新的图像窗口
	// cv::imshow("centerline", rotatedImage);
    // cv::waitKey(5);
	// cv::destroyWindow("centerline");  // 销毁之前的窗口
	
}

void LaserCenter::grap_images(Mat & img)
{
	if (start != "1")
	{
		std::cout << "输入1开始:" << std::endl;
		std::cin >> start;
	}	
	num_img++;
	std::cout << "开始采集：" << num_img << std::endl;

	std::string filename = "srcImgs/" + std::to_string(num_img) + ".png";
	imwrite(filename, img);
}

void LaserCenter::gray_centroid(Mat srcimg)
{ 
    Mat pyrimg;
    Mat gaussimg;
	Mat binaryimg;
    // std::cout<< srcimg.channels() <<std::endl;
    // cvtColor(srcimg, grayimg, COLOR_BGR2GRAY);
    // std::cout<<"3"<<std::endl;
    // imshow("srcImg", srcimg);

    //计时开始 
    begin = clock();

    pyrDown(srcimg, pyrimg);
    // std::cout<< pyrimg.size() <<std::endl;
    imshow("pyrimg", pyrimg);

    // GaussianBlur(pyrimg, gaussimg, Size(3, 3), 0);
	gaussimg = pyrimg;
    gauss_time = clock();

    // thd_otsu = GetMatOTSU(grayimg);
    thd_otsu = myOtsu(gaussimg);
    otsu_time = clock();

	threshold(gaussimg, binaryimg, thd_otsu, 255, 0);
    imshow("binaryimg", binaryimg);
    // waitKey(0);
    //遍历每一列
    // float x0 = 0;

    std::vector<float> pyr_coordinat;
    std::vector<float> src_coordinat;
    for (int i = 0;i < gaussimg.rows;i++) 
    {
        float sum_value = 0;
        float sum_valuecoor = 0;
        // std::vector<float> current_value;
        // std::vector<float> current_coordinat;
        

        // 计算单行otsu
        // thd_otsu = GetLineOTSU(grayimg, i);
		int current_j1 = 0;
		int count_j = 0;
        for (int j = 0;j < gaussimg.cols;j++) 
        {
            float current = gaussimg.at<uchar>(i, j);
            //将符合阈值的点灰度值和坐标存入数组
            if (current > thd_otsu) 
            {
                // current_value.push_back(current);
                // current_coordinat.push_back(j);
				count_j++;
				if (count_j<2)
				{
					sum_valuecoor += current * j;
                	sum_value += current;
					current_j1 = j;
				}
				else
				{
					// 筛选距离小于10*2个像素的点
					if ((float)(j-10) < current_j1)
					{
						sum_valuecoor += current * j;
                		sum_value += current;
					}
				}
                
            }
        }

        // //计算灰度重心
        // for (int k = 0;k < current_value.size();k++) 
        // {
        //     sum_valuecoor += current_value[k]*current_coordinat[k];
        //     sum_value += current_value[k];
        // }
        float x = sum_valuecoor / sum_value;
        pyr_coordinat.push_back(x);

        // x0 = x;
        // circle(pyrimg, Point(i, x), 0, Scalar(0, 0, 255), -1, 8);
        // current_value.clear();
        // current_coordinat.clear();
    }
    // 扩充到原始大小
    for (int k = 0; k < (pyr_coordinat.size() - 1); k++)
    {
        if (abs(pyr_coordinat[k+1]-pyr_coordinat[k]) < 8)
		{
			src_coordinat.push_back(pyr_coordinat[k] * 2);
        	src_coordinat.push_back(pyr_coordinat[k] + pyr_coordinat[k+1]);
		}
		else
		{
			src_coordinat.push_back(pyr_coordinat[k] * 2);
        	src_coordinat.push_back(pyr_coordinat[k+1] *2);
		}
		 
        if ((k + 2) == pyr_coordinat.size())
        {
            src_coordinat.push_back(pyr_coordinat[k+1] * 2);
            src_coordinat.push_back(pyr_coordinat[k+1] * 2);
        }
    }

    // std::cout << pyr_coordinat.size() << "\t" << src_coordinat.size() << std::endl;
    // 绘制中心线
	cvtColor(srcimg, srcimg,cv::COLOR_GRAY2BGR);
    for (int n = 0; n < src_coordinat.size(); n++)
    {
        
		circle(srcimg, Point(src_coordinat[n], n), 0, Scalar(0, 0, 255), -1, 8);
        // std::cout <<  src_coordinat[n] << std::endl;
    }

    //计时结束
    end = clock();
    gauss_cost = double(gauss_time - begin) /1000;
    otsu_cost = double(otsu_time - gauss_time) /1000;
    gray_cost = double(end - otsu_time) /1000;
    total_cost = double(end - begin) /1000;
    std::cout << "gauss_cost: " << gauss_cost << "ms\t" << 
    "otsu costs: " <<  otsu_cost << "ms\t" <<
    "gray_centroid costs: " <<  gray_cost << "ms\t" <<
    "total costs: " <<  total_cost << "ms" << std::endl;

    std::cout << "thd: " << thd_otsu << std::endl;

    //绘制帧率
    fps = std::to_string(round(1000 / total_cost));
    std::cout << "fps: " << fps << std::endl;
    putText(srcimg, fps, cv::Point(20, 20), cv::FONT_ITALIC, 0.8, cv::Scalar(255, 255, 255));
	resize(srcimg, srcimg, Size(512, 768));
    imshow("pyrimg_line", srcimg);
    waitKey(5);

}

#if 0
// 2 大津法获取自适应阈值
int LaserCenter::getOTSUthread(const Mat& src)
{
	// int size = 256;

	int *NoArry = new int[256];// 直方图矩阵

	for (int i = 0; i < 256; ++i)// 初始化
	    NoArry[i] = 0;
    // NoArry[256] = {0};

	int r = src.rows;
	int c = src.cols;
	int sum = r*c;

//  建立直方图矩阵
	for (int i = 0; i < r; ++i)
	{
		for (int j = 0; j < c; ++j)
		{	
			uchar pix = src.at<uchar>(i, j);
			NoArry[pix]++;
		}
	}
	
	int thd = 0; // 阈值
	float w1 = 0, w2 = 0, u1 = 0, u2 = 0, u = 0, thg = 0, MaxTh = 0;
	int cnt1 = 0, cnt2 = 0;
	for (int i = 1; i <= 255; ++i)
	{
		u1 = u2 = w1 = w2 = 0; // 均值，比例初始化
		cnt1 = 0;cnt2 = 0;
		int wt1 = 0, wt2 = 0;// weight初始化

		for (int j = 0; j <i; ++j)
		{
			cnt1 += NoArry[j];
			wt1 += j*NoArry[j];
		}
		u1 = wt1 / cnt1;
		w1 = cnt1*1.0 / sum;

		for (int j = i; j < 256; ++j)
		{
			cnt2 += NoArry[j];
			wt2 += j*NoArry[j];
		}
		u2 = wt2 / cnt2;
		w2 = cnt2*1.0 / sum;

		thg = w1*w2*(u1 - u2)*(u1 - u2);
		if (MaxTh < thg)// 找最大类间方差阈值
		{
			MaxTh = thg;
			thd = i;
		}
	}
    delete[] NoArry;
	return thd;
}
#endif
// 1 使用大津法Mat的阈值
int LaserCenter::GetMatOTSU(const Mat& img)
{
  //判断如果不是单通道直接返回128
    // if (img.channels() > 1) return 128;
    int rows = img.rows;
    int cols = img.cols;
    //定义数组
    float mathists[256] = { 0 };
    //遍历计算灰度值为0-255的个数，索引为灰度值，对应的值为当前灰度值的个数
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            int val = img.at<uchar>(row, col);
            mathists[val]++;
        }
    }
 
    //定义灰度级像素在整个图像中的比例
    float grayPro[256] = { 0 };
    int matSize = rows * cols;
    for (int i = 0; i < 256; ++i) {
        grayPro[i] = (float)mathists[i] / (float)matSize;
    }
 
    //大津法OTSU，前景与背景分割，计算出方差最大的灰度值
    int calcval;
    int calcMax = 0;
    for (int i = 0; i < 256; ++i) {
        float w0 = 0, w1 = 0, u0tmp = 0, u1tmp = 0, u0 = 0, u1 = 0, u = 0, calctmp = 0;
      
        for (int k = 0; k < 256; ++k) {
            float curGray = grayPro[k];
            //计算背景部分
            if (k <= i) {
                //以i为阈值分类，第一类总的概率
                w0 += curGray;
                u0tmp += curGray * k;
            }
            //计算前景部分
            else {
                //以i为阈值分类，第一类总的概率
                w1 += curGray;
                u1tmp += curGray * k;
            }
        }
 
        //求出第一类和第二类的平均灰度
        u0 = u0tmp / w0;
        u1 = u1tmp / w1;
        //求出整幅图像的平均灰度
        u = u0tmp + u1tmp;
    
        //计算类间方差
        calctmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
    
        //更新最大类间方差，并设置阈值
        if (calctmp > calcMax) {
        calcMax = calctmp;
        calcval = i;
        }
    }
 
    return calcval;
}

// 1-1 单行otsu计算
int LaserCenter::GetLineOTSU(const Mat& img, int index)
{
  //判断如果不是单通道直接返回128
    // if (img.channels() > 1) return 128;
    int rows = img.rows;
    int cols = img.cols;
    //定义数组
    float mathists[256] = { 0 };
    //遍历计算灰度值为0-255的个数，索引为灰度值，对应的值为当前灰度值的个数
    for (int row = 0; row < rows; ++row) {
        int val = img.at<uchar>(row, index);
        mathists[val]++;
    }
 
    //定义灰度级像素在整个图像中的比例
    float grayPro[256] = { 0 };
    for (int i = 0; i < 256; ++i) {
        grayPro[i] = (float)mathists[i] / (float)rows;
    }
 
    //大津法OTSU，前景与背景分割，计算出方差最大的灰度值
    int calcval;
    int calcMax = 0;
    for (int i = 0; i < 256; ++i) {
        float w0 = 0, w1 = 0, u0tmp = 0, u1tmp = 0, u0 = 0, u1 = 0, u = 0, calctmp = 0;
      
        for (int k = 0; k < 256; ++k) {
            float curGray = grayPro[k];
            //计算背景部分
            if (k <= i) {
                //以i为阈值分类，第一类总的概率
                w0 += curGray;
                u0tmp += curGray * k;
            }
            //计算前景部分
            else {
                //以i为阈值分类，第一类总的概率
                w1 += curGray;
                u1tmp += curGray * k;
            }
        }
 
        //求出第一类和第二类的平均灰度
        u0 = u0tmp / w0;
        u1 = u1tmp / w1;
        //求出整幅图像的平均灰度
        u = u0tmp + u1tmp;
    
        //计算类间方差
        calctmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
    
        //更新最大类间方差，并设置阈值
        if (calctmp > calcMax) {
        calcMax = calctmp;
        calcval = i;
        }
    }
 
    return calcval;
}

void LaserCenter::getHistogram(Mat &src, int *dst)
{
	Mat hist;
	int channels[1] = { 0 };
	int histSize[1] = { 256 };
	float hranges[2] = { 0, 256.0 };
	const float *ranges[1] = { hranges };
	calcHist(&src, 1, channels, Mat(), hist, 1, histSize, ranges);
	//cout << hist ;
	for (int i = 0; i < 256; i++)
	{ 
		float binVal = hist.at<float>(i);
		dst[i] = int(binVal);
	}
}
 
 // 大津阈值法3
// int LaserCenter::myOtsu(Mat & src, Mat &dst)
int LaserCenter::myOtsu(Mat & src)
{
	
	int th = 0;
	const int GrayScale = 256;	//单通道图像总灰度256级
	int pixCount[GrayScale] = { 0 };//每个灰度值所占像素个数
	int pixSum = src.cols * src.rows;//图像总像素点
	float pixPro[GrayScale] = { 0 };//每个灰度值所占总像素比例
	float SumpixPro[GrayScale] = { 0 }; // 比例的和
	float WpixPro[GrayScale] = { 0 }; //比例 * 权重
	float SumWpixPro[GrayScale] = { 0 };//比例 * 权重 的 和
	float w0, w1, u0tmp, u1tmp, u0, u1, deltaTmp, deltaMax = 0;
	double start = getTickCount(); //开始时间
	
	getHistogram(src, pixCount);
 
	double c1 = getTickCount(); 
	//cout << "c1 >> " << (c1 - start) / getTickFrequency() << endl;//输出时间
	for (int i = 0; i < GrayScale; i++)
	{
		pixPro[i] = pixCount[i] * 1.0 / pixSum;//计算每个灰度级的像素数目占整幅图像的比例  
		WpixPro[i] = i * pixPro[i];
		if (i == 0)
		{
			SumWpixPro[i] += WpixPro[i];
			SumpixPro[i] += pixPro[i];
		}
		else
		{
			SumWpixPro[i] = WpixPro[i] + SumWpixPro[i - 1];
			SumpixPro[i] = pixPro[i] + SumpixPro[i - 1];
		}
	}
 
	for (int i = 0; i < GrayScale; i++)//遍历所有从0到255灰度级的阈值分割条件，测试哪一个的类间方差最大
	{
		w0 = w1 = u0tmp = u1tmp = u0 = u1 = deltaTmp = 0;

		w0 = SumpixPro[i];
		w1 = 1 - w0;
		
		if (w0 == 0 || w1 == 0)
			continue;
		u0tmp = SumWpixPro[i];
		u1tmp = SumWpixPro[255] - SumWpixPro[i];
		
		u0 = u0tmp / w0;
		u1 = u1tmp / w1;
		deltaTmp = (float)(w0 *w1* pow((u0 - u1), 2)); //类间方差公式 g = w1 * w2 * (u1 - u2) ^ 2
		if (deltaTmp > deltaMax)
		{
			deltaMax = deltaTmp;
			th = i;
		}
	}
	// double c2 = getTickCount();
	// //cout << "c2 >> " << (c2 - c1) / getTickFrequency() << endl;//输出时间
	// uchar lutData[256];
	// for (int i = 0; i < 256; i++)
	// { 
	// 	if (i > th)
	// 		lutData[i] = 255;
	// 	else
	// 		lutData[i] = 0;
	// }
	// Mat lut(1, 256, CV_8UC1, lutData);
	// LUT(src, lut, dst);

	// double c3 = getTickCount();
	//cout << "c3 >> " << (c3 - c2) / getTickFrequency() << endl;//输出时间
	return th;
}


void LaserCenter::thining_gray_centroid(Mat img)
{
    //计时开始 
    begin = clock();
    
    Mat img1, img2;

	GaussianBlur(img, img, Size(3, 3), 0);
	img.copyTo(img2);
	threshold(img, img, 95, 255, 3);// 可改
	zhang(img, img1);

	std::vector<Point2d> points;
	thining_point(img1, points);
	std::vector<double> kcal;

	// Point2d & ptA = ;
	for (int i = 1; i < points.size() - 1; i++)
	{
		//normal
		double pfCosSita=0, pfSinSita=0;
		CalcNormVec(Point2d(points[i - 1].x, points[i - 1].y), Point2d(points[i].x, points[i].y), 
					Point2d(points[i + 1].x, points[i + 1].y), pfCosSita, pfSinSita);
		//gdd
		double sum=0, sum_sumx=0, sum_sumy=0;
		for (int j = 0; j < 2; j++)
		{
			if (j == 0)
			{
				double cj = points[i].x;
				double ci = points[i].y;
				sum = ijpixel(cj, ci, img2);
				sum_sumx = ijpixel(cj, ci, img2)*cj;
				sum_sumy = ijpixel(cj, ci, img2)*ci;
			}
			else
			{
				double x_cor = points[i].x + j*pfCosSita;
				double y_cor = points[i].y + j*pfSinSita;
				double x_cor1 = points[i].x - j*pfCosSita;
				double y_cor1 = points[i].y - j*pfSinSita;
				sum = sum + ijpixel(x_cor, y_cor, img2) + ijpixel(x_cor1, y_cor1, img2);
				sum_sumx = sum_sumx + ijpixel(x_cor, y_cor, img2)*x_cor + ijpixel(x_cor1, y_cor1, img2)*x_cor1;
				sum_sumy = sum_sumy + ijpixel(x_cor, y_cor, img2)*y_cor + ijpixel(x_cor1, y_cor1, img2)*y_cor1;
			}
		}
		
		circle(img2, Point(points[i].x, points[i].y), 0, Scalar(0, 0, 255), -1, 8);

	}
    //计时结束
    end = clock();
    total_cost = double(end - begin) / 1000;
    std::cout << "thin_gray cost: " << total_cost << "ms" << std::endl;

	// imshow("h", img);
	imshow("w", img2);
	waitKey(1);

}

//zhang细化算法
void LaserCenter::zhang(Mat& input, Mat& output)
{
	Mat copymat;
	input.copyTo(copymat);
	int k = 0;
	//防止溢出
	while (1)
	{
		k++;
		bool stop = false;
		//step1
		for (int i = 1; i < input.cols - 1; i++)
			for (int j = 1; j < input.rows - 1; j++)
			{
				if (input.at<uchar>(j, i)>0)
				{
					int p1 = int(input.at<uchar>(j, i))>0 ? 1 : 0;
					int p2 = int(input.at<uchar>(j - 1, i))>0 ? 1 : 0;
					int p3 = int(input.at<uchar>(j - 1, i + 1))>0 ? 1 : 0;
					int p4 = int(input.at<uchar>(j, i + 1))>0 ? 1 : 0;
					int p5 = int(input.at<uchar>(j + 1, i + 1))>0 ? 1 : 0;
					int p6 = int(input.at<uchar>(j + 1, i))>0 ? 1 : 0;
					int p7 = int(input.at<uchar>(j + 1, i - 1))>0 ? 1 : 0;
					int p8 = int(input.at<uchar>(j, i - 1))>0 ? 1 : 0;
					int p9 = int(input.at<uchar>(j - 1, i - 1))>0 ? 1 : 0;
					int np1 = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
					int sp2 = (p2 == 0 && p3 == 1) ? 1 : 0;
					int sp3 = (p3 == 0 && p4 == 1) ? 1 : 0;
					int sp4 = (p4 == 0 && p5 == 1) ? 1 : 0;
					int sp5 = (p5 == 0 && p6 == 1) ? 1 : 0;
					int sp6 = (p6 == 0 && p7 == 1) ? 1 : 0;
					int sp7 = (p7 == 0 && p8 == 1) ? 1 : 0;
					int sp8 = (p8 == 0 && p9 == 1) ? 1 : 0;
					int sp9 = (p9 == 0 && p2 == 1) ? 1 : 0;
					int sp1 = sp2 + sp3 + sp4 + sp5 + sp6 + sp7 + sp8 + sp9;
	
					if (np1 >= 2 && np1 <= 6 && sp1 == 1 && ((p2*p4*p6) == 0) && ((p4*p6*p8) == 0))
					{
						stop = true;
						copymat.at<uchar>(j, i) = 0;
					}
				}
			}
		//step2
		for (int i = 1; i < input.cols - 1; i++)
		{
			for (int j = 1; j < input.rows - 1; j++)
			{
				if (input.at<uchar>(j, i)>0)
				{
					int p2 = int(input.at<uchar>(j - 1, i))>0 ? 1 : 0;
					int p3 = int(input.at<uchar>(j - 1, i + 1)) > 0 ? 1 : 0;
					int p4 = int(input.at<uchar>(j, i + 1)) > 0 ? 1 : 0;
					int p5 = int(input.at<uchar>(j + 1, i + 1)) > 0 ? 1 : 0;
					int p6 = int(input.at<uchar>(j + 1, i)) > 0 ? 1 : 0;
					int p7 = int(input.at<uchar>(j + 1, i - 1)) > 0 ? 1 : 0;
					int p8 = int(input.at<uchar>(j, i - 1)) > 0 ? 1 : 0;
					int p9 = int(input.at<uchar>(j - 1, i - 1)) > 0 ? 1 : 0;
					int np1 = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
					int sp2 = (p2 == 0 && p3 == 1) ? 1 : 0;
					int sp3 = (p3 == 0 && p4 == 1) ? 1 : 0;
					int sp4 = (p4 == 0 && p5 == 1) ? 1 : 0;
					int sp5 = (p5 == 0 && p6 == 1) ? 1 : 0;
					int sp6 = (p6 == 0 && p7 == 1) ? 1 : 0;
					int sp7 = (p7 == 0 && p8 == 1) ? 1 : 0;
					int sp8 = (p8 == 0 && p9 == 1) ? 1 : 0;
					int sp9 = (p9 == 0 && p2 == 1) ? 1 : 0;
					int sp1 = sp2 + sp3 + sp4 + sp5 + sp6 + sp7 + sp8 + sp9;
					if (np1 >= 2 && np1 <= 6 && sp1 == 1 && (p2*p4*p8) == 0 && (p2*p6*p8) == 0)
					{
						stop = true;
						copymat.at<uchar>(j, i) = 0;
					}
				}
			}
		}
		//将新得到的图片赋给新的图片
		copymat.copyTo(input);
		if (!stop)
		{
			break;
		}
	}
	copymat.copyTo(output);
}

//第i，j个点像素值
double LaserCenter::ijpixel(double& x, double& y, Mat& m)
{
	int x_0 = int(x);
	int x_1 = int(x + 1);
	int y_0 = int(y);
	int y_1 = int(y + 1);
	int px_0y_0 = int(m.at<uchar>(y_0, x_0));
	int px_0y_1 = int(m.at<uchar>(y_1, x_0));
	int px_1y_0 = int(m.at<uchar>(y_0, x_1));
	int px_1y_1 = int(m.at<uchar>(y_1, x_1));
	double x_y0 = px_0y_0 + (x - double(x_0))*(px_1y_0 - px_0y_0);
	double x_y1 = px_0y_1 + (x - double(x_0))*(px_1y_1 - px_0y_1);
	double x_y = x_y0 + (y - double(y_0))*(x_y1 - x_y0);
	return x_y;
}

//normal vector
void LaserCenter::CalcNormVec(Point2d&& ptA, Point2d&& ptB, Point2d&& ptC, double& pfCosSita, double& pfSinSita)
{
	double fVec1_x, fVec1_y, fVec2_x, fVec2_y;
	if (ptA.x == 0 && ptA.y == 0)
	{
		ptA.x = ptC.x;
		ptA.y = ptC.y;
		//先用B点坐标减A点坐标
		fVec1_x = -(ptB.x - ptA.x);
		fVec1_y = -(ptB.y - ptA.y);
	}
	else
	{
		//先用B点坐标减A点坐标
		fVec1_x = ptB.x - ptA.x;
		fVec1_y = ptB.y - ptA.y;
	}

	if (ptC.x == 0 && ptC.y == 0)
	{
		ptC.x = ptA.x;
		ptC.y = ptA.y;
		//再用C点坐标减B点坐标
		fVec2_x = (ptB.x - ptC.x);
		fVec2_y = (ptB.y - ptC.y);
	}
	else
	{
		//再用C点坐标减B点坐标
		fVec2_x = ptC.x - ptB.x;
		fVec2_y = ptC.y - ptB.y;
	}

	//单位化
	double fMod = sqrt(fVec1_x * fVec1_x + fVec1_y * fVec1_y);
	fVec1_x /= fMod;
	fVec1_y /= fMod;
	//计算垂线
	double fPerpendicularVec1_x = -fVec1_y;
	double fPerpendicularVec1_y = fVec1_x;

	//单位化
	fMod = sqrt(fVec2_x * fVec2_x + fVec2_y * fVec2_y);
	fVec2_x /= fMod;
	fVec2_y /= fMod;
	//计算垂线
	double fPerpendicularVec2_x = -fVec2_y;
	double fPerpendicularVec2_y = fVec2_x;
	//求和
	double fSumX = fPerpendicularVec1_x + fPerpendicularVec2_x;
	double fSumY = fPerpendicularVec1_y + fPerpendicularVec2_y;
	//单位化
	fMod = sqrt(fSumX * fSumX + fSumY * fSumY);
	double fCosSita = fSumX / fMod;
	double fSinSita = fSumY / fMod;
	pfCosSita = fCosSita;
	pfSinSita = fSinSita;
}

void LaserCenter::thining_point(Mat& inputimg, std::vector<Point2d>& pt)
{
	pt.push_back(Point2d(0, 0));
	for (int i = 0; i < inputimg.cols; i++)
		for (int j = 0; j < inputimg.rows; j++)
		{
			if (inputimg.at<uchar>(j, i) >= 95)
			{
				Point2d curr = Point2d(i, j);
				pt.push_back(curr);
			}
		}
	pt.push_back(Point2d(0, 0));
}

// 103算法测试

int LaserCenter::alg103_runimage(cv::Mat &cvimgIn,
                    std::vector <cv::Point2f> &pointcloud,
                    std::vector <Targetpoint> &namepoint,
                    bool &solderjoints,
                    int step)    //输出结果点信息
{
    Uint8 bryvalue;
    Int32 i32_bryvalue;
    Int32 i,j,n,t;
    Myhalcv2::Mat imageIn;
    Myhalcv2::Mat imageGasu;
    Myhalcv2::Mat imageBry;
    Myhalcv2::Mat m16_filterIma;
    Myhalcv2::Mat m_brygujia;
    Myhalcv2::Mat m_filter;
    Myhalcv2::Mat imageGasupain;
    Myhalcv2::Mat m_tempmatIn;
    Myhalcv2::MyConect ImageConect,ImageConectlong;

    Int32 nWidth=cvimgIn.cols;	//输入图像宽
    Int32 nHeight=cvimgIn.rows;	//输入图像高
    Uint8 filterdata[25]={0,0,0,0,0,
                          0,0,0,0,0,
                          1,1,1,1,1,
                          0,0,0,0,0,
                          0,0,0,0,0};

    Myhalcv2::L_Point32 f_center={-1,-1};
    Int32 X_Linestarty=0;
    Int32 X_Lineendy=0;
    cv::Point2f cv_point;
    Int32 jiguangTop,jiguangDeep,jiguangLeft,jiguangRight;
    Int32 nstarti,nendi,nstartj,nendj;
    Targetpoint targetpoint;

    // add start
    char *cv8uc1_Imagebuff_image;
    char *cv8uc1_Imagebuff1;
    char *cv8uc1_Imagebuff2;
    char *cv8uc1_Imagebuff3;
    char *cv8uc1_Imagebuff4;
    char *cv8uc1_Imagebuff5;
    char *cv8uc1_Imagebuff6;
    char *cv8uc1_Imagebuff7;
    char *cv8uc1_Imagebuff8;
    char *cv8uc1_Imagebuff9;

    Int32 *X_line;
    float *f_line;
    Uint8 *X_lineMark;
    Int32 *X_linedif32,*niheX,*niheY;
    Myhalcv2::MyConect ImageConectlongPX,Imageheadline;

    Int32 firstsearch;
    Int32 firstsearch_stx,firstsearch_sty,firstsearch_edx,firstsearch_edy;
    Int32 jishuST_x,jishuST_y,jishuED_x,jishuED_y,jishuNum;
    Int32 firstdimian;
    Int32 fuzhuxielv,b_fuzhuxielv,jishuxielv;

    Myhalcv2::L_Point32 fuzhufindST,fuzhufindED;//结果线2拟合区域,(上方)

//
    static int oldwidth=0, oldHeight=0;

  // 申请内存空间
  if(oldwidth!=cvimgIn.cols||oldHeight!=cvimgIn.rows)
  {
    // 释放内存
    if(oldwidth!=0||oldHeight!=0)
    {
      Myhalcv2::MyhalcvMemFree();
      delete [] cv8uc1_Imagebuff_image;
      delete [] cv8uc1_Imagebuff1;
      delete [] cv8uc1_Imagebuff2;
      delete [] cv8uc1_Imagebuff3;
      delete [] cv8uc1_Imagebuff4;
      delete [] cv8uc1_Imagebuff5;
      delete [] cv8uc1_Imagebuff6;
      delete [] cv8uc1_Imagebuff7;
      delete [] cv8uc1_Imagebuff8;
      delete [] X_line;
      delete [] X_lineMark;
      delete [] X_linedif32;
      delete [] niheX;
      delete [] niheY;
      delete [] f_line;
    }
    oldwidth=cvimgIn.cols;// 1536
    oldHeight=cvimgIn.rows;// 1024
    Myhalcv2::MyhalcvMemInit(oldHeight,oldwidth); // 内存初始化

    cv8uc1_Imagebuff_image=new char [oldwidth*oldHeight*4];
    cv8uc1_Imagebuff1=new char [oldwidth*oldHeight];
    cv8uc1_Imagebuff2=new char [Myhalcv2::getHoughsize()];
    cv8uc1_Imagebuff3=new char [Myhalcv2::getConectsize()*oldwidth*oldHeight];
    cv8uc1_Imagebuff4=new char [oldwidth*oldHeight];
    cv8uc1_Imagebuff5=new char [oldwidth*oldHeight];
    cv8uc1_Imagebuff6=new char [oldwidth*oldHeight*2];
    cv8uc1_Imagebuff7=new char [oldwidth*oldHeight];
    cv8uc1_Imagebuff8=new char [oldwidth*oldHeight];
    cv8uc1_Imagebuff9=new char [oldwidth*oldHeight];

    Int32 bigsize;//1536
    bigsize=oldwidth>oldHeight?oldwidth:oldHeight;
    X_line=new Int32 [bigsize];
    f_line=new float [bigsize];
    X_lineMark=new Uint8 [bigsize*4];
    X_linedif32=new Int32 [bigsize];
    niheX=new Int32 [bigsize];
    niheY=new Int32 [bigsize];
  }
  // add end


/*********************/
    //算法参数
    Int32 pingjun=15;//15;
    Int32 gujiaerzhi=160;//160;
    Int32 widthliantongdis=5;//5;
    Int32 highliantongdis=5;//5;
    Int32 jiguanglong=5;//5;//激光长度
    Int32 jiguangkuandu=10;//10;//激光宽度
    Int32 jiguangduibidu=5;//5;
    Int32 lvbomod=1;        // 高斯滤波模式3*3， 5*5， 7*7

    if(step==2)
    {
      return 0;
    }
    imageIn=Myhalcv2::MatCreat(nWidth,nHeight,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff_image);
    Myhalcv2::CvMatToMat(cvimgIn,&imageIn,cv8uc1_Imagebuff_image);
    imageGasu=Myhalcv2::MatCreat(nWidth,nHeight,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff5);
    imageBry=Myhalcv2::MatCreat(nWidth,nHeight,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff4);
    m_brygujia=Myhalcv2::MatCreat(nWidth,nHeight,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff7);
    m16_filterIma=Myhalcv2::MatCreat(nWidth,nHeight,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff6);
    
    Myhalcv2::Mygausspyramid_2levl(imageIn,&imageGasu);

    if(step==3)
    {
        Myhalcv2::MatToCvMat(imageGasu,&cvimgIn);
        return 0;
    }

    if(step!=0)
    {
        imageGasupain=Myhalcv2::MatCreatClone(imageGasu,cv8uc1_Imagebuff8);
    }

    Myhalcv2::Mybinaryval(imageGasu,&bryvalue,Myhalcv2::MHC_BARINYVAL_MEAN);

    i32_bryvalue=(Int32)bryvalue+pingjun;//求平均值二值化阈值
    Myhalcv2::Mybinary(imageGasu,&imageBry,Myhalcv2::MHC_BARINY_VALUE_IMG,255,i32_bryvalue,0);
    if(step==4)
    {
        Myhalcv2::MatToCvMat(imageBry,&cvimgIn);
        return 0;
    }
    m_brygujia=Myhalcv2::MatCreatzero(nHeight/4,nWidth/4,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff7);
    Myhalcv2::Mynormalize_lineXY(imageGasu,&m_brygujia,jiguangduibidu);

    if(step==5)
    {
        Myhalcv2::MatToCvMat(m_brygujia,&cvimgIn);
        return 0;
    }

    i32_bryvalue=gujiaerzhi;
    Myhalcv2::Mybinary(m_brygujia,&m_brygujia,Myhalcv2::MHC_BARINY_VALUE_IMG,255,i32_bryvalue,0);

    if(step==6)
    {
        Myhalcv2::Mymat_to_binself(&m_brygujia,255);
        Myhalcv2::MatToCvMat(m_brygujia,&cvimgIn);
        return 0;
    }

    Myhalcv2::Myconnection2(m_brygujia,&ImageConect,jiguanglong,widthliantongdis,highliantongdis,Myhalcv2::MHC_MORPH_RECT,Myhalcv2::MHC_8LT,cv8uc1_Imagebuff3);//创建8联通区域ImageConect,最小面积120,两区域距离小于2认为同一区域
    Myhalcv2::Myselect_shape(&ImageConect,&ImageConectlong,Myhalcv2::MHC_CONNECT_WIDTH_HEIGHT,jiguanglong,MAX(ImageConect.nWidth,ImageConect.nHeight));//在ImageConect中筛选出高度大于50的联通域
    if(ImageConectlong.AllMarkPointCount==0)
    {
    #ifdef QUICK_TRANSMIT
        Myhalcv2::MatToCvMat(imageGasu,&cvimgIn);
    #endif
        return 1;
    }
    Myhalcv2::MyGetthinNoHough(&ImageConectlong,Myhalcv2::THIN_X,jiguangkuandu,&m_brygujia);
    if(step==7)
    {
        Myhalcv2::MatToCvMat(m_brygujia,&cvimgIn);
        return 0;
    }
    Myhalcv2::Mysmallest_rectangle(&ImageConectlong,&jiguangLeft,&jiguangRight,&jiguangTop,&jiguangDeep);

    Myhalcv2::Mydilation_circle2(m_brygujia,&imageBry,2,0,Myhalcv2::MHC_MORPH_RECT);

    nstartj=MAX(jiguangTop*4,0);
    nendj=MIN(jiguangDeep*4,nHeight-1);
    nstarti=MAX(jiguangLeft*4-30,0);
    nendi=MIN(jiguangRight*4+30,nWidth-1);

    Myhalcv2::MyCutRoi(imageIn,&m_tempmatIn,Myhalcv2::MHC_CUT_NOTCOPY,nstarti,nstartj,nendi-nstarti+1,nendj-nstartj+1);

    if(lvbomod==0)
    {

    }
    else if(lvbomod==1)
    {
        Myhalcv2::Mygaussia(m_tempmatIn,&m_brygujia,Myhalcv2::GAUSS_WIN_3x3);
        m_tempmatIn=m_brygujia;
    }
    else if(lvbomod==2)
    {
        Myhalcv2::Mygaussia(m_tempmatIn,&m_brygujia,Myhalcv2::GAUSS_WIN_5x5);
        m_tempmatIn=m_brygujia;
    }
    else if(lvbomod==3)
    {
        Myhalcv2::Mygaussia(m_tempmatIn,&m_brygujia,Myhalcv2::GAUSS_WIN_7x7);
        m_tempmatIn=m_brygujia;
    }

    for(j=m_tempmatIn.starty;j<m_tempmatIn.starty+m_tempmatIn.height;j++)
    {
        Int32 sum_valuecoor=0;
        Int32 sum_value=0;

        for(i=m_tempmatIn.startx;i<m_tempmatIn.startx+m_tempmatIn.width;i++)
        {
            Int32 di=i>>2;
            Int32 dj=j>>2;
            if(imageBry.data[dj*imageBry.nWidth+di]!=0)
            {
                sum_valuecoor=sum_valuecoor+(Int32)m_tempmatIn.data[j*m_tempmatIn.nWidth+i]*i;
                sum_value=sum_value+m_tempmatIn.data[j*m_tempmatIn.nWidth+i];
            }
        }
        if(sum_value!=0)
        {
            f_line[j]=(float)sum_valuecoor/sum_value;
            if(X_Linestarty==0)
            {
                X_Linestarty=j;//骨架起点
            }
            X_Lineendy=j;//骨架终点
            X_lineMark[j]=1;
        }
        if(step==8)
        {
            if(sum_value!=0)
            {
                imageGasupain.data[j*imageGasupain.nWidth+X_line[j]]=128;
            }
        }
    }
    if(step==8)
    {
        Myhalcv2::MatToCvMat(imageGasupain,&cvimgIn);
        return 0;
    }
    /***********************/

    /*
    //以下的图像几乎都是完美图像,需要检测出结果
    //以下对高斯图做卷积
    m16_filterIma=Myhalcv2::MatCreatzero(nHeight/4,nWidth/4,Myhalcv2::CCV_16UC1,cv8uc1_Imagebuff6);
    m_filter=Myhalcv2::MatCreat(5,5,Myhalcv2::CCV_8UC1,filterdata);
    Myhalcv2::Myfilter(imageGasu,m_filter,&m16_filterIma,Myhalcv2::CCV_16UC1,0,f_center);
    memset(X_line,0,sizeof(Int32)*nHeight/4);
    memset(X_lineMark,0,nHeight/4);
    X_Linestarty=0;
    X_Lineendy=0;
    //以下取出二值图结果中每行卷积最大值
    m_brygujia=Myhalcv2::MatCreatzero(nHeight/4,nWidth/4,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff7);
    for(j=m16_filterIma.starty;j<m16_filterIma.starty+m16_filterIma.height;j++)
    {
        Uint16 max=0;
        Uint16 maxX=m16_filterIma.startx;
        Uint16 maxXn=0;
        for(i=m16_filterIma.startx;i<m16_filterIma.startx+m16_filterIma.width;i++)
        {
            if(imageBry.ptr_uchar[j*imageBry.nWidth+i]!=0)
            {
                if(max<m16_filterIma.ptr_ushort[j*m16_filterIma.nWidth+i])
                {
                    max=m16_filterIma.ptr_ushort[j*m16_filterIma.nWidth+i];
                    maxXn=1;
                    maxX=i;
                }
                else if(max==m16_filterIma.ptr_ushort[j*m16_filterIma.nWidth+i])
                {
                    maxXn++;
                    maxX=i+maxX;
                }
            }
        }
        if(maxXn!=0)
        {
            X_line[j]=(maxX<<1)/maxXn;
            if(X_Linestarty==0)
            {
                X_Linestarty=j;//骨架起点
            }
            X_Lineendy=j;//骨架终点
            m_brygujia.data[j*imageGasu.nWidth+(X_line[j]>>1)]=255;
        }
        if(step==8)
        {
            if(X_line[j]!=0&&maxX!=imageBry.startx)
            {
                imageGasupain.data[j*imageGasu.nWidth+(X_line[j]>>1)]=0;
            }
        }
    }
    if(step==8)
    {
        Myhalcv2::MatToCvMat(imageGasupain,&cvimgIn);
        return 0;
    }
    if(X_Lineendy==0)//没找到骨架
    {
    #ifdef QUICK_TRANSMIT
        Myhalcv2::MatToCvMat(imageGasu,&cvimgIn);
    #endif
        return 1;
    }
    Myhalcv2::Myconnection(m_brygujia,&ImageConect,jiguanglong,1,Myhalcv2::MHC_8LT,cv8uc1_Imagebuff3);//先去掉离散点
    if(ImageConect.AllMarkPointCount==0)
    {
    #ifdef QUICK_TRANSMIT
        Myhalcv2::MatToCvMat(imageGasu,&cvimgIn);
    #endif
        return 1;
    }
    for(j=0;j<ImageConect.AllMarkPointCount;j++)
    {
        for(i=0;i<ImageConect.AllMarkPoint[j].PointArea;i++)
        {
            Int32 y=ImageConect.AllMarkPoint[j].point[i].y;
            X_lineMark[y]=1;
        }
    }
    Myhalcv2::Myfixdata(X_line,X_lineMark,nHeight/4);//修复空的线
    m_brygujia=Myhalcv2::MatCreatzero(nHeight/4,nWidth/4,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff7);
    for(j=X_Linestarty;j<=X_Lineendy;j++)
    {
        m_brygujia.data[j*m_brygujia.nWidth+(X_line[j]>>1)]=255;
    }
    if(step==9)
    {
        Myhalcv2::MatToCvMat(m_brygujia,&cvimgIn);
        return 0;
    }
    */
/***************************************/
//  Myhalcv2::Myresizefix2bitdata_4fSize(X_line,X_lineMark,f_line,nHeight/4);
    for(i=0;i<nHeight;i++)
    {
        Int32 y=(Int32)(((float)i/4)+0.5);
        Int32 x=(Int32)(f_line[i]/4+0.5);
        if(x>=0&&x<imageBry.nWidth&&y>=0&&y<imageBry.nHeight)
        {
            if(imageBry.data[y*imageBry.nWidth+x]==0)
            {
                f_line[i]=-1;
            }
        }
        cv::Point2f point(f_line[i],i);
        pointcloud.push_back(point);
    }
    if(step==1)
    {
        Myhalcv2::MatToCvMat(imageGasu,&cvimgIn);
        if(cvimgIn.type()==CV_8UC1)
            cv::cvtColor(cvimgIn,cvimgIn,cv::COLOR_GRAY2BGR);
        for(j=0;j<nHeight;j++)
        {
            if(f_line[j]>=0)
            {
                Int32 di=(f_line[j]/4+0.5);
                Int32 dj=j/4;
                if(di>=0&&di<imageGasu.nWidth)
                {
                    cvimgIn.data[dj*imageGasu.nWidth*3+di*3]=255;
                    cvimgIn.data[dj*imageGasu.nWidth*3+di*3+1]=0;
                    cvimgIn.data[dj*imageGasu.nWidth*3+di*3+2]=0;
                }
            }
        }
    }
    solderjoints=false;
    cv_point.x=pointcloud[0].x;
    cv_point.y=pointcloud[0].y;
    targetpoint.pointf=cv_point;
    targetpoint.name="point_0";
    namepoint.push_back(targetpoint);  
    cv_point.x=0;
    cv_point.y=0;
    targetpoint.pointf=cv_point;
    targetpoint.name="normal";
    namepoint.push_back(targetpoint);  
    cv_point.x=pointcloud[pointcloud.size()-1].x;
    cv_point.y=pointcloud[pointcloud.size()-1].y;
    targetpoint.pointf=cv_point;
    targetpoint.name="point_1";
    namepoint.push_back(targetpoint);  

    return 0;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserCenter>());
  rclcpp::shutdown();
  return 0;
}