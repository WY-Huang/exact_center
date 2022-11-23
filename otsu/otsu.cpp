#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <time.h>

using namespace cv;
using namespace std;


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


int main()
{  
    cv::Mat Src= cv::imread("/home/wanyel/vs_code/exact_center/srcImg/bmp/test2.bmp",1);
    if(!Src.data){
        printf("fail to open the image!\n");
        return -1;
    }
    // 80-90ms
    time_t begin, mid, end;
    
    int thd_otsu1, thd_otsu2;
	begin = clock();
    // thd_otsu1 = getOTSUthread(Src);
	thd_otsu2 = GetMatOTSU(Src);

    mid = clock();
	// thd_otsu2 = GetMatOTSU(Src);
	thd_otsu1 = getOTSUthread(Src);
	end = clock();

    std::cout << "thd_otsu1: " << thd_otsu1 << "\tthd_otsu2: " << thd_otsu2 << std::endl;
    std::cout << "getOTSUthread1 costs:" << double(mid - begin) / 1000 << "ms" <<
	 "\tgetOTSUthread2 costs:" << double(end - mid) / 1000 << "ms" <<std::endl;

    // lines.imshow();
    return 0;

}