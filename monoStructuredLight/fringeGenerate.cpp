/*
相移条纹生成（20230801）
*/

#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>


void makePhaseShiftPatterns(double A, double B, int T, int N, int W, int H, 
                            std::vector<cv::Mat>& Is, std::vector<cv::Mat>& Is_img) 
{
    Is.resize(N);
    Is_img.resize(N);

    cv::Mat xs(1, W, CV_64F);

    double f_2pi = 1.0 / static_cast<double>(T) * 2.0 * M_PI;

    for (int k = 0; k < N; k++) 
    {
        double phaseShiftK = 2 * k / static_cast<double>(N) * M_PI;
        for (int wi = 0; wi < W; wi++)
        {
            xs.at<double>(0, wi) = A + B * std::cos(f_2pi * (wi + 1) + phaseShiftK);
            double temp = A + B * std::cos(f_2pi * (wi + 1) + phaseShiftK);
        }

        Is[k] = xs;
        Is_img[k] = cv::repeat(xs, H, 1);
    }
}


int main() 
{
    int W = 2184; // T1, T2, T3
    int H = 720;
    int N = 12;
    double A = 130;
    double B = 90;

    int T1 = 28;
    int T2 = 26;
    int T3 = 24;

    bool show = true;
    std::string save_folder = "data/ideaPhase"; 
    // cv::utils::fs::createDirectories(save_folder);

    std::vector<cv::Mat> Is_T1;
    std::vector<cv::Mat> Is_T1_img;
    makePhaseShiftPatterns(A, B, T1, N, W, H, Is_T1, Is_T1_img);

    std::vector<cv::Mat> Is_T2;
    std::vector<cv::Mat> Is_T2_img;
    makePhaseShiftPatterns(A, B, T2, N, W, H, Is_T2, Is_T2_img);

    std::vector<cv::Mat> Is_T3;
    std::vector<cv::Mat> Is_T3_img;
    makePhaseShiftPatterns(A, B, T3, N, W, H, Is_T3, Is_T3_img);

    int idxN = 0;
    for (const auto& imgs : {Is_T1_img, Is_T2_img, Is_T3_img}) 
    {
        idxN++;
        for (int i = 0; i < N; i++) 
        {
            const auto& I_img = imgs[i];
            if (show) 
            { 
                std::string save_file = save_folder + "/" + std::to_string(idxN) + "_"+ std::to_string(i + 1) + ".bmp";
                cv::imwrite(save_file, I_img);
                // cv::imshow("Image", I_img);
                // cv::waitKey(0);
            }
        }
    }

    return 0;
}
