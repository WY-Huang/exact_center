/*
zhang氏细化算法+灰度质心法
*/

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <stdlib.h>


using namespace std;
using namespace cv;


//zhang细化算法
void zhang(Mat& input, Mat& output)
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

double ijpixel(double& x, double& y, Mat& m)
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
void CalcNormVec(Point2d&& ptA, Point2d&& ptB, Point2d&& ptC, double& pfCosSita, double& pfSinSita)
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

void point(Mat& inputimg, vector<Point2d>& pt)
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


int main()
{
	// ofstream of1, of2;
	// of1.open("x.txt", ios::trunc);
	// of2.open("y.txt", ios::trunc);
	Mat img, img1, img2;
	img = imread("/home/wanyel/vs_code/exact_center/srcImg/bmp/test0.bmp");
	cvtColor(img, img, COLOR_RGB2GRAY);

	// 75ms
    clock_t begin, end;
    begin = clock();

	GaussianBlur(img, img, Size(3, 3), 0);
	img.copyTo(img2);
	threshold(img, img, 95, 255, 3);
	zhang(img, img1);

	imshow("w", img1);
	waitKey(0);

	vector<Point2d> points;
	point(img1, points);
	vector<double> kcal;

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
		// of1 << points[i].x << ",";
		// of2 << points[i].y << ",";
		circle(img2, Point(points[i].x, points[i].y), 0, Scalar(0, 0, 255), -1, 8);

	}

	end = clock();
    std::cout << "extractLine_steger costs:" << double(end - begin) / 1000 << "ms" << std::endl;

	imshow("h", img);
	imshow("w", img2);
	// imwrite("20221114094923_test.jpg", img2);
	waitKey(0);
	return 0;
}