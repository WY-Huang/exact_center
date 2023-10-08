// 倾斜镜头的标定测试 20231007-


#include <opencv2/opencv.hpp>
#include "find_ellipses.hpp"

using namespace cv;
using namespace std;


int main()
{
    // 1.准备标定棋盘图像
    int boardWidth = 7;  // 棋盘格横向内角点数量
    int boardHeight = 7; // 棋盘格纵向内角点数量
    // float squareSize = 1.f; // 棋盘格格子的大小，单位为米, 随便设置，不影响相机内参计算
    float squareSize = 0.0025; // 半圆心距的大小，单位为米
    Size boardSize(boardWidth, boardHeight);

    vector<vector<Point3f>> objectPoints;
    vector<vector<Point2f>> imagePoints;
    vector<Point2f> corners;

    // 2.拍摄棋盘图像
    Mat image, gray;
    // namedWindow("calibration image", WINDOW_NORMAL);
    vector<String> fileNames;
    glob("images/testImg/*.jpg", fileNames);

    for (size_t i = 0; i < fileNames.size(); i++)
    {
        image = imread(fileNames[i], IMREAD_COLOR);
        cvtColor(image, gray, COLOR_BGR2GRAY);

        /*
        // 3.读入图像数据，并提取棋盘格标定点角点
        bool found = findChessboardCorners(image, boardSize, corners, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
        if (found)
        {
            cornerSubPix(gray, corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));
            drawChessboardCorners(image, boardSize, corners, found);
            imshow("image", image);
            waitKey();

            vector<Point3f> objectCorners;
            for (int j = 0; j < boardHeight; j++)
            {
                for (int k = 0; k < boardWidth; k++)
                {
                    objectCorners.push_back(Point3f(k * squareSize, j * squareSize, 0));
                }
            }
            objectPoints.push_back(objectCorners);
            imagePoints.push_back(corners);
        }
        */


        /* 提取圆形标定板圆心点 */
        vector<Vec6f> result;
        cv::ximgproc::findEllipses(gray, result, 0.5, 0.6, 0.05);

        vector<Point2f> center_r;
        for (int i=0; i<result.size(); i++)
        {
            Point2f center_p = Point2f(result[i][0], result[i][1]);
            // std::cout << center_p << std::endl;
            cv::circle(image, center_p, 2, cv::Scalar(0, 0, 255), 2);
            cv::ellipse(image, center_p, Size(result[i][2], result[i][3]), 0, 0, 360, cv::Scalar(0, 255, 0), 2);

            center_r.push_back(center_p);
        }

    
        cv::Mat showImg;
        cv::resize(image, showImg, cv::Size(1536, 1024));
        imshow("draw image", showImg);
        waitKey(0);
        continue;

        drawChessboardCorners(image, boardSize, center_r, 1);
        cv::resize(image, image, cv::Size(1536, 1024));
        imshow("draw image", image);
        waitKey();

        if (false == findCirclesGrid(gray, boardSize, corners, CALIB_CB_SYMMETRIC_GRID + CALIB_CB_CLUSTERING))
        {
            cout << fileNames[i] << " failed" << endl;  // 找不到角点

            // cv::resize(gray, gray, cv::Size(1536, 1024));
            // imshow("gray image", gray);
            // waitKey();
            continue;
        }
        else
        {
            drawChessboardCorners(image, boardSize, corners, 1);
            cv::resize(image, image, cv::Size(1536, 1024));
            imshow("draw image", image);
            waitKey();

            vector<Point3f> objectCorners;
            for (int j = 0; j < boardHeight; j++)
            {
                for (int k = 0; k < boardWidth; k++)
                {
                    objectCorners.push_back(Point3f(k * squareSize, j * squareSize, 0));
                }
            }
            objectPoints.push_back(objectCorners);
            imagePoints.push_back(corners);
        }
    }

    // 4.标定相机
    Mat cameraMatrix, distCoeffs;
    vector<Mat> rvecs, tvecs;
    calibrateCamera(objectPoints, imagePoints, image.size(), cameraMatrix, distCoeffs, rvecs, tvecs);

    cout << "Camera matrix:" << endl << cameraMatrix << endl;
    cout << "Distortion coefficients:" << endl << distCoeffs << endl;

    // 5.畸变校正
    for (int i = 0; i < fileNames.size(); i++)
    {
        Mat dst;
        Mat image = imread(fileNames[i]);
        undistort(image, dst, cameraMatrix, distCoeffs);
        imshow("image", image);
        imshow("undistort image", dst);
        waitKey(0);
    }

    return 0;
}