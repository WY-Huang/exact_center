#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>
#include <time.h>

using namespace cv;
using namespace std;

void gray_centroid()
{
    Mat srcimg = imread("/home/wanyel/vs_code/exact_center/gray_centroid_base/test.png");
    rotate(srcimg, srcimg, 0);
    // Size newsize = Size(srcimg.cols*0.5, srcimg.rows*0.5);
    // resize(srcimg, srcimg, newsize);
    Mat grayimg;
    cvtColor(srcimg, grayimg, COLOR_BGR2GRAY);
    threshold(grayimg, grayimg, 128, 255, 0);
    imshow("grayimg", grayimg);
    waitKey(0);

    //计时开始 45ms
    time_t begin, end;
    // clock_t begin, end;
    begin = clock();

    GaussianBlur(grayimg, grayimg, Size(0, 0), 6, 6);
    // imshow("grayImg", grayimg);
    // waitKey(0);
    //遍历每一列
    for (int i = 0;i < grayimg.rows;i++) 
    {
        float sum_value = 0;
        float sum_valuecoor = 0;
        vector<float>current_value;
        vector<float>current_coordinat;
        for (int j = 0;j < grayimg.cols;j++) 
        {
            float current = grayimg.at<uchar>(i, j);
            //将符合阈值的点灰度值和坐标存入数组
            if (current > 30) 
            {
                current_value.push_back(current);
                current_coordinat.push_back(j);
            }
        }
        //计算灰度重心
        for (int k = 0;k < current_value.size();k++) 
        {
            sum_valuecoor += current_value[k]*current_coordinat[k];
            sum_value += current_value[k];
        }
        float x = sum_valuecoor / sum_value;
        cv::Point2f point(x, i);

        circle(srcimg, point, 0, Scalar(0, 0, 255), -1, 8);
        current_value.clear();
        current_coordinat.clear();
    }
    //计时结束
    end = clock();
    cout << "gray_centroid costs:" << double(end - begin) / 1000 << "ms" << endl;

    imshow("srcimg", srcimg);
    // imwrite("20221114100129_center.bmp", srcimg);
    waitKey(0);

}

int main()
{
    // 60-70ms
    // time_t begin, end;
    // clock_t begin, end;
    // begin = clock();

    gray_centroid();

    // end = clock();
    // cout << "gray_centroid costs:" << double(end - begin) / 1000 << "ms" << endl;
    
    return 0;
}