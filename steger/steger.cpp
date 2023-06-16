/*
中心线提取的steger算法
*/

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <ctime>


using namespace cv;
using namespace std;

void StegerCV()
{
    // 读取图像并转为灰度图 
    Mat srcImg = imread("srcImg/bmp/test4.bmp", 1);
    // Size dsize = Size(1536*0.5, 1024*0.5);
    resize(srcImg, srcImg, Size(1536, 1024));
    Mat grayImg;
    cvtColor(srcImg, grayImg, COLOR_BGR2GRAY);
    // cout << "srcImag size: " << srcImg.channels() << "\ngrayImg size:" << grayImg.channels() << endl;

    // imshow("srcImg", srcImg);
    // imshow("grayImg", grayImg);
    // waitKey(0);

    // 高斯滤波,32位浮点型单通道
    // grayImg.convertTo(grayImg, CV_32FC1);
    // imshow("grayImg", grayImg);
    // waitKey(0);
    GaussianBlur(grayImg, grayImg, Size(0, 0), 6, 6);
    

    imshow("Gaussian_grayImg", grayImg);
    waitKey(0);

    // 一阶偏导数
    Mat m1, m2;
    m1 = (Mat_<float>(1, 2) << 1, -1);  // x偏导,定义一行两列的矩阵，[1, -1]
    m2 = (Mat_<float>(2, 1) << 1, -1);  // y偏导, 定义两行一列的矩阵，[[1], [-1]]

    Mat dx, dy;
    filter2D(grayImg, dx, CV_32FC1, m1);
    filter2D(grayImg, dy, CV_32FC1, m2);

    // imshow("dx", dx);
    // imshow("dy", dy);
    // waitKey(0);

    //二阶偏导数
    Mat m3, m4, m5;
    m3 = (Mat_<float>(1, 3) << 1, -2, 1);        // 二阶x偏导[1, -2, 1]
    m4 = (Mat_<float>(3, 1) << 1, -2, 1);        // 二阶y偏导[[1], [-2], [1]]
    m5 = (Mat_<float>(2, 2) << 1, -1, -1, 1);    // 二阶xy偏导[[1, -1], [-1, 1]]

    Mat dxx, dyy, dxy;
    filter2D(grayImg, dxx, CV_32FC1, m3);
    filter2D(grayImg, dyy, CV_32FC1, m4);
    filter2D(grayImg, dxy, CV_32FC1, m5);

    // imshow("dxx", dxx);
    // imshow("dyy", dyy);
    // imshow("dxy", dxy);
    // waitKey(0);

    // hessian矩阵
    double maxD = -1;
    int imgcol = grayImg.cols;// 768
    int imgrow = grayImg.rows;// 512
    // cout << grayImg.cols << '\t'<< grayImg.rows <<endl;
    vector<double> Pt;
    for (int i=0;i<grayImg.cols;i++)
    {
        for (int j=0;j<imgrow;j++)
        {
            if (srcImg.at<uchar>(j,i)>200)
            {
                Mat hessian(2, 2, CV_32FC1);// 海森矩阵赋值
                hessian.at<float>(0, 0) = dxx.at<float>(j, i);
                hessian.at<float>(0, 1) = dxy.at<float>(j, i);
                hessian.at<float>(1, 0) = dxy.at<float>(j, i);
                hessian.at<float>(1, 1) = dyy.at<float>(j, i);

                Mat eValue;
                Mat eVectors;
                eigen(hessian, eValue, eVectors);// 计算海森矩阵的特征值和特征向量

                double nx, ny;
                double fmaxD = 0;
                if (fabs(eValue.at<float>(0,0))>= fabs(eValue.at<float>(1,0)))  //求特征值最大时对应的特征向量
                {
                    nx = eVectors.at<float>(0, 0);
                    ny = eVectors.at<float>(0, 1);
                    fmaxD = eValue.at<float>(0, 0);
                }
                else
                {
                    nx = eVectors.at<float>(1, 0);
                    ny = eVectors.at<float>(1, 1);
                    fmaxD = eValue.at<float>(1, 0);
                }

                double t = -(nx*dx.at<float>(j, i) + ny*dy.at<float>(j, i)) / (nx*nx*dxx.at<float>(j,i)+2*nx*ny*dxy.at<float>(j,i)+ny*ny*dyy.at<float>(j,i));

                if (fabs(t*nx) <= 0.5 && fabs(t*ny) <= 0.5)
                {
                    double x_sub = i + t*nx;
                    double y_sub = j + t*ny;
                    // cout << x_sub << "\t"<< y_sub << endl;
                    Pt.push_back(x_sub);
                    Pt.push_back(y_sub);
                }
            }
        }
    }

    // cout << "Pt.size: " << Pt.size()/2 << endl;
    static int point_x = -1;
    for (int k = 0;k<Pt.size()/2;k++)
    {
        Point rpt;
        rpt.x = Pt[2 * k + 0];
        rpt.y = Pt[2 * k + 1];
        // cout << "x,y = " << rpt << endl;
        
        if (point_x < rpt.x ) 
        {
            circle(srcImg, rpt, 1, Scalar(0, 0, 255));
            point_x = rpt.x;

            // cout << "x,y = " << rpt << endl;
        }
        
    }

    imshow("result", srcImg);
    waitKey(0);
}


int main()
{
    clock_t begin, end;
    begin = clock();

    StegerCV();

    end = clock();
    cout << "steger costs:" << double(end - begin) / 1000 << "ms" << endl;

    return 0;
    
}