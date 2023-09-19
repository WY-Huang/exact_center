#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

int main()
{
    // 批量读取文件
    std::string folderPath = "/home/wanyel/contours/halcon_calibration/20230916-17-45-cab/rawImgs";

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

            cv::Mat srcimg = cv::imread(imgPath, 0);

            // 灰度变换
            cv::Mat grayTransImg = srcimg.clone();
            for (int i = 0; i < srcimg.rows; i++)
            {
                for (int j = 0; j < srcimg.cols; j++)
                {
                    grayTransImg.at<uchar>(i, j) = 2 * log((double)(grayTransImg.at<uchar>(i, j)) + 1) / log(100);  //对数变换 s=6*log(r+1)
                }
            }
            cv::normalize(grayTransImg, grayTransImg, 0, 255, cv::NORM_MINMAX);  //图像归一化，转到0~255范围内
            cv::convertScaleAbs(grayTransImg, grayTransImg);  //数据类型转换到CV_8U

            // 高斯滤波
            // cv::Mat gaussImg;
            // cv::GaussianBlur(srcimg, gaussImg, cv::Size(0, 0), 3, 3);

            // 二值化
            // cv::Mat binaryImg;
            // cv::threshold(srcimg, binaryImg, 80, 255, cv::THRESH_BINARY);

            // 重设置图像大小
            cv::resize(grayTransImg, grayTransImg, cv::Size(1536, 1024));
            // cv::imshow("grayTransImg", grayTransImg);
            // cv::waitKey(0);

            // 重命名图像路径
            fs::path path_temp(filePath);
            std::string stem = "/home/wanyel/contours/halcon_calibration/20230916-17-45-cab/grayTransImgs_1536/" + path_temp.stem().string() + "_1536.jpg";
            std::cout << stem << std::endl;
            cv::imwrite(stem, grayTransImg);

        }
    }

    return 0;
}