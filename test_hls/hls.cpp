#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

struct m_cam
{
    /* data */
    int m_saveheight=400;
    int m_savewidth=1000;
} m_cam ;

int GetBuliaoColor(uchar* pImage);

int main()
{
	Mat src;
    src = imread("/home/wanyel/vs_code/exact_center/histogram/data/img/4.bmp");
    uchar* src_p = (uchar*)src.data;
    GetBuliaoColor(src_p);

}

int GetBuliaoColor(uchar* pImage)
{
	/******在这里以下添加布料分析函数*******/
	Mat MatIn(m_cam.m_saveheight, m_cam.m_savewidth, CV_8UC3, pImage);
	Rect m_select = Rect(m_cam.m_savewidth/12*5, m_cam.m_saveheight/12*5, m_cam.m_savewidth/12*2, m_cam.m_saveheight/12*2);
	Mat ROI = MatIn(m_select);
    // Mat ROI = pImage;

	Mat hls;
	cvtColor(ROI, hls, COLOR_BGR2HLS);
	vector<Mat> hlsPlane;
	split(hls, hlsPlane);
	//显示各个通道
	Mat hlsPlane2(m_cam.m_saveheight/6,m_cam.m_savewidth/6,CV_8UC1,Scalar(127));
	Scalar S;
	S=sum(hlsPlane[1]);//明度平均值
	int mingdupinjun=S[0]/(m_cam.m_saveheight/6*m_cam.m_savewidth/6);
	vector<Mat> mbgr(3); 
    mbgr[0] = hlsPlane[0];
    mbgr[1] = hlsPlane2;
    mbgr[2] = hlsPlane[2];
	Mat imageB,image;
	merge(mbgr, imageB);
	cvtColor(imageB, image, COLOR_HLS2BGR);
	int bins=256;
	int hist_size[]={bins};
	float range[]={0,256};
	const float *ranges[]={range};
	MatND redHist,grayHist,blueHist;
	int channels_r[]={2};
	int channels_g[]={1};
	int channels_b[]={0};
	calcHist(&image,1,channels_r,Mat(),redHist,1,hist_size,ranges,true,false);
	calcHist(&image,1,channels_g,Mat(),grayHist,1,hist_size,ranges,true,false);
	calcHist(&image,1,channels_b,Mat(),blueHist,1,hist_size,ranges,true,false);
	double maxValue_red,maxValue_green,maxValue_blue;
	Point maxLoc_red,maxLoc_green,maxLoc_blue;
	minMaxLoc(redHist, 0, &maxValue_red, 0, &maxLoc_red);
	minMaxLoc(grayHist, 0, &maxValue_green, 0, &maxLoc_green);
	minMaxLoc(blueHist, 0, &maxValue_blue, 0, &maxLoc_blue);
	Mat MatRGB(m_cam.m_saveheight/6,m_cam.m_savewidth/6,CV_8UC3,Scalar(maxLoc_blue.y,maxLoc_green.y,maxLoc_red.y));
// #ifdef savebmptest
	// imwrite("/home/wanyel/vs_code/exact_center/histogram/data/img/Cutin.bmp",MatIn);
	imwrite("/home/wanyel/vs_code/exact_center/histogram/data/img/Cutroi.bmp",ROI);
	imwrite("/home/wanyel/vs_code/exact_center/histogram/data/img/CutHSL_50.bmp",image); 
	imwrite("/home/wanyel/vs_code/exact_center/histogram/data/img/Cutrgb.bmp",MatRGB); 
/*#endif
	u8_lightresultR=255-maxLoc_red.y;
	u8_lightresultG=255-maxLoc_green.y;
	u8_lightresultB=255-maxLoc_blue.y;
	Mat RGBlight(1,1,CV_8UC3,Scalar(u8_lightresultB,u8_lightresultG,u8_lightresultR));
	Mat RGBlighthls;
	cvtColor(RGBlight, RGBlighthls, COLOR_BGR2HLS);
	vector<Mat> lightmbgr; 
	split(RGBlighthls, lightmbgr);
	Mat lightmbgr2;
	lightmbgr2=Mat(1,1,CV_8UC1,Scalar(GetLightMingdu(mingdupinjun)));//明度
	vector<Mat> lightmbgr3(3); 
	lightmbgr3[0] = lightmbgr[0];
    lightmbgr3[1] = lightmbgr2;
    lightmbgr3[2] = lightmbgr[2];
	Mat imageC,imageD;
	merge(lightmbgr3, imageC);
	cvtColor(imageC, imageD, COLOR_HLS2BGR);
	u8_lightresultR=imageD.data[2];
	u8_lightresultG=imageD.data[1];
	u8_lightresultB=imageD.data[0];
	return 0;
    */
	/*******************************/
    
}