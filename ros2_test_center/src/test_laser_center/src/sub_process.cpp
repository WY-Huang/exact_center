#include "test_laser_center/sub_process.hpp"
// video/compressed   camera_tis_node/image    rotate_image_node/image_rotated


LaserCenter::LaserCenter() : Node("sub_images")
{
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera_tis_node/image", rclcpp::SensorDataQoS(), std::bind(&LaserCenter::topic_callback, this, _1)
    );
}

void LaserCenter::topic_callback(const sensor_msgs::msg::Image msg)
{
    
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, msg.encoding);

    cvImage = cv_ptr->image.clone();
    // RCLCPP_INFO(this->get_logger(), "receive images...");

    // imshow("srcimg", cvImage);
    // waitKey(5);
    
    gray_centroid(cvImage);

    // thining_gray_centroid(cvImage);
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
    for (int i = 0;i < gaussimg.cols;i++) 
    {
        float sum_value = 0;
        float sum_valuecoor = 0;
        // std::vector<float> current_value;
        // std::vector<float> current_coordinat;
        

        // 计算单行otsu
        // thd_otsu = GetLineOTSU(grayimg, i);
        float mean_value;
		int current_j = 0;
		int count_j = 0;
        for (int j = 0;j < gaussimg.rows;j++) 
        {
            float current = gaussimg.at<uchar>(j, i);
            //将符合阈值的点灰度值和坐标存入数组
            if (current > thd_otsu) 
            {
                // current_value.push_back(current);
                // current_coordinat.push_back(j);
				count_j++;
				if (count_j<4)
				{
					sum_valuecoor += current * j;
                	sum_value += current;
					current_j += j;
					mean_value = current_j / count_j;
				}
				else
				{
					// 
					if ((float)(j-15) < mean_value)
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
        if (abs(pyr_coordinat[k+1]-pyr_coordinat[k]) < 10)
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
        
		circle(srcimg, Point(n, src_coordinat[n]), 0, Scalar(0, 0, 255), -1, 8);
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


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LaserCenter>());
  rclcpp::shutdown();
  return 0;
}