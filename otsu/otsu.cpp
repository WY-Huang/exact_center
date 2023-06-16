/*
大津阈值分割算法
*/

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <time.h>

using namespace cv;
using namespace std;

////////////////////////////////////////////////////////////////
int getOTSUthread(Mat& src)
{
	int size = 256;

	int *NoArry = new int[size];// 直方图矩阵

	for (int i = 0; i < size; ++i)// 初始化
		NoArry[i] = 0;

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
	//delete[] NoArry;

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

	return thd;
}

///////////////////////////////////////////////////////////////
//使用大津法Mat的阈值
int GetMatOTSU(const Mat& img)
{
  //判断如果不是单通道直接返回128
  if (img.channels() > 1) return 128;
  int rows = img.rows;
  int cols = img.cols;
  //定义数组
  float mathists[256] = { 0 };
  //遍历计算0-255的个数
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
/////////////////////////////////////////////////////////////////////////////
void getHistogram(Mat &src, int *dst)
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
int myOtsu(Mat & src)
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

int main()
{  
  cv::Mat Src= cv::imread("/home/wanyel/vs_code/exact_center/srcImg/bmp/test4.bmp",1);
  if(!Src.data){
        printf("fail to open the image!\n");
        return -1;
  }
  // 80-90ms
  time_t begin, otsu1, otsu2, otsu3;
    
  int thd_otsu1, thd_otsu2, thd_otsu3;
	begin = clock();
  // thd_otsu1 = getOTSUthread(Src);
  for (int i=0; i<1000; i++)
  {
    thd_otsu1 = GetMatOTSU(Src);
  }

  otsu1 = clock();

	// thd_otsu2 = GetMatOTSU(Src);
  for (int i=0; i<1000; i++)
  {
    thd_otsu2 = getOTSUthread(Src);
  }
  otsu2 = clock();

  for (int i=0; i<1000; i++)
  {
    thd_otsu3 = myOtsu(Src);
  }
	otsu3 = clock();

  std::cout << "thd_otsu1: " << thd_otsu1 << "\tthd_otsu2: " << thd_otsu2 << "\tthd_otsu3: " << thd_otsu3 << std::endl;
  std::cout << "GetMatOTSU costs:" << double(otsu1 - begin) / 1000000 << "ms" <<
  "\tgetOTSUthread costs:" << double(otsu2 - otsu1) / 1000000 << "ms" << 
  "\tmyOtsu costs:" << double(otsu3 - otsu2) / 1000000 << "ms" << std::endl;

  // lines.imshow();
  return 0;

}