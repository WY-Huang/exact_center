#ifndef MYHALCV2_H
#define MYHALCV2_H


#define USE_XIMGPROC    0

#include "opencv2/opencv.hpp"
#if USE_XIMGPROC == 1
#include "opencv2/ximgproc.hpp"
#endif
#include "tistdtypes.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdint.h>

#define restrict
#define DDR2HEAP		  0
/*******************************************/
//此处是库函数一些内存上限数据
#define MHC_MARK_POINTNUM		254 			//联通域点个数最多支持存255个
#define MHC_TETARANGE			407				//霍夫变换西塔范围
#define MHC_ROLENGTH			401				//霍夫变换肉范围
#define MHC_TETARANGE_MID		3				//霍夫变换西塔上下限
#define MHC_SEARCH_SIZE			18				//霍夫变换完成后的微调搜索范围rho
#define MHC_SEARCH_LINENUM		20				//霍夫变换最多找的线条数
#define MHC_PEAK_NUM			2000			//波形图最多支持的点个数
#define MHC_VER_NUM				2000			//VER缓存最多支持的缓冲元素个数
#define MHC_ROATE				601				//旋转细分度
#define MHC_MIX_NUM_VAR_SQARE	50				//最小二乘法小点集斜率拟合的个数
/********************************************/

#define SQRTof2_Q8 		  362
#define RANDOM_MAX		  0x7FFFFFFF
#define RANDOM_MIN		  -2147483648

#define MIN(a,b)  ((a) > (b) ? (b) : (a))
#define MAX(a,b)  ((a) < (b) ? (b) : (a))

#define XY_ONE 	 	65536		//位运算标记
#define XY_SHIFT 	16			//位运算标记

#ifndef UINT_BIT64
#define UINT_BIT64
typedef uint64_t  Uint64;
typedef int64_t   Int64;
#endif

namespace Myhalcv2
{
    typedef struct {
        int area;
        int xsum;
        int ysum;

        int xmin;
        int ymin;
        int xmax;
        int ymax;

    } VLIB_CC;

    typedef enum TYPE
    {
        CCV_8UC1,	//8位单通道无符号型
        CCV_8UC3,	//8位三通道无符号型
        CCV_8SC1,	//8位单通道有符号型
        CCV_8SC3,	//8位三通道有符号型
        CCV_16UC1,	//16位单通道无符号型
        CCV_16UC3,	//16位三通道无符号型
        CCV_16SC1,	//16位单通道有符号型
        CCV_16SC3,	//16位三通道有符号型
        CCV_32SC1,	//32位单通道有符号型
        CCV_32SC3,	//32位单通道有符号型
        CCV_32FC1,	//单精度浮点数单通道型
        CCV_32FC3,	//单精度浮点数三通道型
        CCV_64FC1,	//双精度浮点数单通道型
        CCV_64FC3	//双精度浮点数三通道型
    }type;

    //BMP格式位图文件头
    typedef struct MHC_tagBITMAPFILEHEADER
    {
        Uint16 bfType;
        Uint32 bfSize;
        Uint16 bfReserved1;
        Uint16 bfReserved2;
        Uint32 bfOffbits;
    }MHC_BITMAPFILEHEADER;

    //BMP格式位图文件头
    typedef struct MHC_tagBITMAPFILEHEADERZOOM
    {
        Uint32 bfType;
        Uint32 bfSize;
        Uint16 bfReserved1;
        Uint16 bfReserved2;
        Uint32 bfOffbits;
    }MHC_BITMAPFILEHEADERZOOM;

    //BMP格式位图信息头
    typedef struct MHC_tagBITMAPINFOHEADER
    {
      Uint32  biSize;
      Uint32  biWidth;
      Uint32  biHeight;
      Uint16  biPlanes;
      Uint16  biBitCount;
      Uint32  biCompression;
      Uint32  biSizeImage;
      Uint32  biXPelsPerMeter;
      Uint32  biYPelsPerMeter;
      Uint32  biClrUsed;
      Uint32  biClrImportant;
    }MHC_BITMAPINFOHEADER;


//切割图像模式
    typedef enum CUTROI
    {
        MHC_CUT_NOTCOPY,	//截取矩阵时不拷贝内存（图像数据存储位置在原矩阵上）
        MHC_CUT_COPY		//截取矩阵时拷贝内存（图像数据新的存储位置上,需要给输出矩阵申请内存）
    }cutroi;

//切割出小块图像模式
    typedef enum CUTMINROI
    {
        MHC_CUTMIN_1,	//小图宽度等于截图宽度,
        MHC_CUTMIN_4,	//小图宽度等于截图宽度按4的倍数补零,
        MHC_CUTMIN_8,	//小图宽度等于截图宽度按8的倍数补零,
        MHC_CUTMIN_32,	//小图宽度等于截图宽度按32的倍数补零,
        MHC_CUTMIN_64,	//小图宽度等于截图宽度按64的倍数补零
        MHC_CUTMIN_4x4	//小图宽度和高度等于截图宽度和高度按4的倍数补零
    }cutminroi;

//逐行逐列均衡化
    typedef enum HISTOLINE
    {
        MHC_HISTO_X,		//逐行均衡化
        MHC_HISTO_Y			//逐列均衡化
    }histoline;

    //图像翻转模式
    typedef enum FLIPTYPE
    {
        MHC_FLIP_X,				//沿X轴翻转
        MHC_FLIP_Y				//沿Y轴翻转
    }fliptype;

    //线模式
    typedef enum LINETYPE
    {
        CV_LINE_4LT=4,	//4连通直线
        CV_LINE_8LT=8	//8连通直线
    }linetype;

//圆模式
    typedef enum CIRCLETYPE
    {
        CV_CLRCLE_FILL=-1,	//实心圆
        CV_CLRCLE_UNFILL=1	//空心圆
    }circletype;

    //虚线模式
    typedef enum DOTTEDTYPE
    {
        CV_DOTTEDLINE_LINE,     //直线虚线
        CV_DOTTEDLINE_POINT 	//点虚线
    }dottedtype;

    //高斯算子窗口大小
    typedef enum GAUSS_WIN
    {
        GAUSS_WIN_3x3,	//3x3窗口
        GAUSS_WIN_5x5,	//5x5窗口
        GAUSS_WIN_7x7	//7x7窗口
    }gauss_win;

    //盖博窗口大小
    typedef enum GABOR_WIN
    {
        GABOR_WIN_3x3,	//3x3窗口
        GABOR_WIN_5x5,	//5x5窗口
        GABOR_WIN_11x11	//11x11窗口
    }gabor_win;

    //索贝儿算子窗口大小
    typedef enum SOBEL_WIN
    {
        SOB_WIN_3x3,	//3x3窗口
        SOB_WIN_5x5,	//5x5窗口
        SOB_WIN_7x7		//7x7窗口
    }sobel_win;

    //细化算法类型
    typedef enum SKELETON_CN
    {
        SKELETON_CN_ROSENFELD,		//Rosenfeld类型细化
        SKELETON_CN_HILDITCH_1,		//hilditch类型方法1
        SKELETON_CN_HILDITCH_2,		//hilditch类型方法2
        SKELETON_CN_ZHANGSUNEN,     //opencv的zhangsuen类型方法
        SKELETON_CN_GUOHALL         //opencv的guohall类型方法
    }skeleton_cn;

    typedef enum BINARYMOD
    {
        MHC_BARINY_MEAN,				//平均值法求二值图,目标为color,背景为0,大于等于mean+value1为目标,小于mean+value1为背景
        MHC_BARINY_MEAN_INV,			//平均值法求二值图,目标为0,背景为color,大于等于mean+value1为目标,小于mean+value1为背景
        MHC_BARINY_MEAN_IMG,			//平均值法求二值图,目标为原图,背景为0,大于等于mean+value1为目标,小于mean+value1为背景
        MHC_BARINY_VALUE_INSIZE,		//双阈值法求区间内二值图,目标为color,背景为0,大于等于value1且小于value2为目标,其他为背景
        MHC_BARINY_VALUE_INSIZE_INV,	//双阈值法求区间外二值图,目标为color,背景为0,大于等于value1且小于value2为背景,其他为目标
    //	MHC_BARINY_GAUSSIAN_C,			//区域高斯加权求二值
    //	MHC_BARINY_GAUSSIAN_C_INV,		//区域高斯加权求二值
        MHC_BARINY_MEAN_C,				//区域平均求二值,目标为color,背景为0,大于等于mean+value1为目标,小于mean+value1为背景,value2为区域窗口尺寸
        MHC_BARINY_MEAN_C_INV,			//区域平均求二值,目标为0,背景为color,大于等于mean+value1为目标,小于mean+value1为背景,value2为区域窗口尺寸
        MHC_BARINY_MEAN_C_IMG,			//区域平均求二值,目标为原图,背景为0,大于等于mean+value1为目标,小于mean+value1为背景,value2为区域窗口尺寸
        MHC_BARINY_OTSU,				//大律法求二值,目标为color,背景为0,大于等于mean+value1为目标,小于mean+value1为背景
        MHC_BARINY_OTSU_INV,			//大律法求二值,目标为0,背景为color,大于等于mean+value1为目标,小于mean+value1为背景
        MHC_BARINY_OTSU_IMG,			//大律法求二值,目标为原图,背景为0,大于等于mean+value1为目标,小于mean+value1为背景
        MHC_BARINY_VALUE,				//单阈值法求二值图,目标为color,背景为0,大于等于value1为目标,小于value1为背景
        MHC_BARINY_VALUE_INV,			//单阈值法求二值图,目标为0,背景为color,大于等于value1为目标,小于value1为背景
        MHC_BARINY_VALUE_IMG			//单阈值法求二值图,目标为原图,背景为0,大于等于value1为目标,小于value1为背景
    }binarymod;

    //求二值阈值模式
    typedef enum BINARYVALMOD
    {
        MHC_BARINYVAL_MEAN,				//求图像平均值
        MHC_BARINYVAL_OTSU				//求图像大律法阈值
    }binaryvalmod;

    //梯度金字塔模式
    typedef enum GRADPYRAMIDTYPE
    {
        MHC_GRADPYRAMID_X,		//水平梯度
        MHC_GRADPYRAMID_Y,		//垂直梯度
        MHC_GRADPYRAMID_X_Y,	//水平垂直梯度后取平均
        MHC_GRADPYRAMID_XoY		//水平垂直梯度后取大的值
    }gradpyramidtype;

    //腐蚀膨胀模式
    typedef enum DILAERO
    {
        MHC_MORPH_ELLIPSE,	//椭圆膨胀
        MHC_MORPH_RECT		//矩形膨胀
    }dilaero;

    //背景和目标颜色
    typedef enum BACKGROUND
    {
        MHC_BACK,		//背景为0,目标为Color
        MHC_BACK_INV,	//背景为Color,目标为0
        MHC_BACK_WNV	//背景为255,目标为Color
    }background;

    //非极大值抑制模式
    typedef enum NON_MAX
    {
        NON_MAX_3x3,	//3x3搜索
        NON_MAX_5x5,	//5x5搜索
        NON_MAX_7x7		//7x7搜索
    }non_max;

    //排序直线模式
    typedef enum SORTLINE
    {
        MHC_LONG_LINEPAIXU,		//长度从小到大排序
        MHC_UP_LINEPAIXU,		//按最高端点从上到下排序
        MHC_DOWN_LINEPAIXU,		//按最低端点从下到上排序
        MHC_LEFT_LINEPAIXU,		//按最左端点从左到右排序
        MHC_RIGHT_LINEPAIXU,	//按最右端点从右到左排序
        MHC_CENTER_H_LINEPAIXU,	//按中点从上到下排序
        MHC_CENTER_W_LINEPAIXU,	//按中点从左到右排序
        MHC_K_LINEPAIXU			//按斜率从小到大排序
    }sortline;

    //图像旋转模式
    typedef enum TRANSTYPE
    {
        MHC_TRANS_INIMAGE,		//旋转或平移后，区域内存数据只保留图像内的点
        MHC_TRANS_OUTIMAGE		//旋转或平移后，区域内存数据保留包括图像外的全部点
    }transtype;

//获取狭窄部位模式
    typedef enum GETTHIN
    {
        THIN_X,		//获取X轴方向狭窄部位
        THIN_Y,		//获取Y轴方向狭窄部位
        THIN_45,	//获取45度方向狭窄部位
        THIN_135	//获取135度方向狭窄部位
    }getthin;

    //搜索模式
    typedef enum GETEVER
    {
        MHC_R_TO_L,		//从右往左搜寻
        MHC_L_TO_R,		//从左往右搜寻
        MHC_T_TO_B,		//从上往下搜寻
        MHC_B_TO_T,		//从下往上搜寻
        MHC_T_TO_B_45,	//从右上往左下搜索
        MHC_B_TO_T_45,	//从左下往右上搜索
        MHC_T_TO_B_135,	//从左上往右下搜索
        MHC_B_TO_T_135	//从右下往左上搜索
    }getever;

    //直线拟合模式
    typedef enum LINEFITTYPE
    {
        MHC_LINER_FITTING,			//头尾线性拟合
        MHC_LEAST_SQUARE_FITTING	//最小二乘法拟合
    }linefittype;

    //流水循环模式
    typedef enum CIRCULATYPE
    {
        MHC_CIRCULA_DOWN_TO_TOP,		//下往上流
        MHC_CIRCULA_TOP_TO_DOWN			//上往下流
    }circulatype;

    //二值化方向
    typedef enum BINARYSIDEMOD
    {
        MHC_BARINYSIDE_VALUE,			//大于等于bry_value为目标
        MHC_BARINYSIDE_VALUE_INV		//小于等于bry_value为目标
    }binarysidemod;


    //求连通模式
    typedef enum CONECTLT
    {
        MHC_4LT=0,		//求4连通
        MHC_8LT=1		//求8连通
    }conectlt;

//Uint16坐标型数据
    typedef struct L_POINT
    {
        Uint16 x;			//x坐标
        Uint16 y;			//y坐标
    }L_Point;

    //Int32坐标型数据
    typedef struct L_POINT32
    {
        Int32 x;			//x坐标
        Int32 y;			//y坐标
    }L_Point32;

    //float坐标型数据
    typedef struct L_POINT32F
    {
        float x;			//x坐标
        float y;			//y坐标
    }L_Point32F;

    //线的起点终点
    typedef struct L_LINE
    {
        L_Point st;			//起点
        L_Point ed;			//终点
    }L_line;

    //线的起点终点
    typedef struct L_LINE32
    {
        L_Point32 st;		//起点
        L_Point32 ed;		//终点
    }L_line32;

    //Int32尺寸坐标型数据
    typedef struct L_SIZE32
    {
        Int32 width;		//宽度
        Int32 height;		//高度
    }L_Size32;

    //float尺寸坐标型数据
    typedef struct L_SIZE32F
    {
        float width;		//宽度
        float height;		//高度
    }L_Size32F;

//CV_8UC3型数据
    typedef struct VEC3B
    {
        Uint8 data1;		//通道1数据
        Uint8 data2;		//通道2数据
        Uint8 data3;		//通道3数据
    }Vec3b;

    //CV_8SC3型数据
    typedef struct VEC3IB
    {
        Int8 data1;			//通道1数据
        Int8 data2;			//通道2数据
        Int8 data3;			//通道3数据
    }Vec3ib;

    //CV_16UC3型数据
    typedef struct VEC3S
    {
        Uint16 data1;		//通道1数据
        Uint16 data2;		//通道2数据
        Uint16 data3;		//通道3数据
    }Vec3s;

    //2维坐标型数据
    typedef struct VEC2W
    {
        Int16 data1;		//通道1数据
        Int16 data2;		//通道2数据
    }Vec2w;

    //2维坐标型数据
    typedef struct VEC2I
    {
        Int32 data1;		//通道1数据
        Int32 data2;		//通道2数据
    }Vec2i;

    //CV_16SC3型数据
    typedef struct VEC3W
    {
        Int16 data1;		//通道1数据
        Int16 data2;		//通道2数据
        Int16 data3;		//通道3数据
    }Vec3w;

    //4维坐标型数据
    typedef struct VEC4W
    {
        Int16 data1;		//通道1数据
        Int16 data2;		//通道2数据
        Int16 data3;		//通道3数据
        Int16 data4;		//通道4数据
    }Vec4w;

    //CV_32SC3型数据
    typedef struct VEC3I
    {
        Int32 data1;		//通道1数据
        Int32 data2;		//通道2数据
        Int32 data3;		//通道3数据
    }Vec3i;

    //CV_32FC3型数据
    typedef struct VEC3F
    {
        float data1;		//通道1数据
        float data2;		//通道2数据
        float data3;		//通道3数据
    }Vec3f;

    //CV_64FC3型数据
    typedef struct VEC3D
    {
        double data1;		//通道1数据
        double data2;		//通道2数据
        double data3;		//通道3数据
    }Vec3d;

    //二维维向量结构体
    typedef struct VEC2V
    {
        L_Point32 vec1;		//向量1
        L_Point32 vec2;		//向量2
    }Vec2v;

    //三维向量结构体
    typedef struct VEC3V
    {
        L_Point32 vec1;		//向量1
        L_Point32 vec2;		//向量2
        L_Point32 vec3;		//向量3
    }Vec3v;


    //椭圆结构体
    typedef struct ROTATEDRECT
    {
        float angle;		//偏移角(正反向是逆时针转,角度制)
        L_Point32F center;	//椭圆圆心
        L_Size32F size;		//椭圆长短轴半径尺寸
    }RotatedRect;

    //Verb缓存结构体(缓存0和非0),结构体内部数据不推荐从外部直接访问或更改,最好调用接口函数获得数据
    typedef struct VERB
    {
        Int32 buf32_num;		//32位化后的缓存总个数(不推荐从外部访问或更改)
        Uint32 *buf32_data;		//32位化后的缓存的数据首地址(不推荐从外部访问或更改)
        Uint32 highst_data;		//最高位与上的数字(不推荐从外部访问或更改)
        Int32 num;				//当前缓存内的元素个数
    }Verb;

    //Ver1i缓存结构体
    typedef struct VER1I
    {
        Int32 buf32_num;		//缓存总个数
        Int32 *point;			//缓存数据首地址
        Int32 num;				//当前缓存内的元素个数
    }Ver1i;

    //Ver2i缓存结构体
    typedef struct VER2I
    {
        Int32 buf32_num;		//缓存总个数
        L_Point32 *point;		//缓存数据首地址
        Int32 num;				//当前缓存内的元素个数
    }Ver2i;

    //Ver3i缓存结构体
    typedef struct VER3I
    {
        Int32 buf32_num;		//缓存总个数
        Vec3i *point;			//缓存数据首地址
        Int32 num;				//当前缓存内的元素个数
    }Ver3i;

    //Ver2v缓存结构体
    typedef struct VER2V
    {
        Int32 buf32_num;		//缓存总个数
        Vec2v *point;			//缓存数据首地址
        Int32 num;				//当前缓存内的元素个数
    }Ver2v;

    //Ver3v缓存结构体
    typedef struct VER3V
    {
        Int32 buf32_num;		//缓存总个数
        Vec3v *point;			//缓存数据首地址
        Int32 num;				//当前缓存内的元素个数
    }Ver3v;

    //波形信息
    typedef struct PEAK_TROUGH
    {
        Int32 *value;				//全部数据信息
        Int8 *valpeaktrough;		//峰谷信息1是峰,-1是谷,-2是下降沿,2是上升沿
        Int32 valuenum;				//全部数据个数
    }Peak_Trough;

    //波形信息
    typedef struct PEAK_TROUGH32FC
    {
        float *value;				//全部数据信息
        Int8 *valpeaktrough;		//峰谷信息1是峰,-1是谷,-2是下降沿,2是上升沿
        Int32 valuenum;				//全部数据个数
    }Peak_Trough32FC;

    //漫水信息
    typedef struct WATERSHED
    {
        Int32 volume;				//水池体积
        Int16 deep;					//水池深度
        Int16 width;				//水面宽度
    }watershed;

    //漫水信息
    typedef struct WATERSHED32FC
    {
        float volume;				//水池体积
        float deep;					//水池深度
        Int16 width;				//水面宽度
    }watershed32FC;

    //波谷波峰膨胀模式
    typedef enum PEAKDILATIONTYPE
    {
        MHC_PEAKTROUGH_BOGU,	//膨胀波谷
        MHC_PEAKTROUGH_BOFENG	//膨胀波峰
    }peakdilationtype;

    //筛选峰峰值模式1
    typedef enum PEAKSELECTTYPE_1
    {
        MHC_PEAKTROUGH_BOGU_NEAR_PRESENT_ABS,				//峰谷值判定，峰谷值阈值采用范围内最大值比波谷大于最大值的百分比阈值,选出波谷
        MHC_PEAKTROUGH_BOGU_NEAR_ABS,						//峰谷值判定，峰谷值阈值采用范围内最大值比波谷大于最大值的阈值，选出波谷
        MHC_PEAKTROUGH_BOFENG_NEAR_PRESENT_ABS,				//峰谷值判定，峰谷值阈值采用范围内最小值比波峰大于波峰值的百分比阈值,选出波峰
        MHC_PEAKTROUGH_BOFENG_NEAR_ABS,						//峰谷值判定，峰谷值阈值采用范围内最小值比波峰大于波峰值的阈值，选出波峰
        MHC_PEAKTROUGH_BOGU_NEAR2_PRESENT_ABS,				//峰谷值判定，峰谷值阈值采用除谷值部分外,范围内最大值比波谷大于最大值的百分比阈值,选出波谷
        MHC_PEAKTROUGH_BOGU_NEAR2_ABS,						//峰谷值判定，峰谷值阈值采用除谷值部分外,范围内最大值比波谷大于最大值的阈值，选出波谷
        MHC_PEAKTROUGH_BOFENG_NEAR2_PRESENT_ABS,			//峰谷值判定，峰谷值阈值采用除峰值部分外,范围内最小值比波峰大于波峰值的百分比阈值,选出波峰
        MHC_PEAKTROUGH_BOFENG_NEAR2_ABS,					//峰谷值判定，峰谷值阈值采用除峰值部分外,范围内最小值比波峰大于波峰值的阈值，选出波峰
        MHC_PEAKTROUGH_BOGU_TOWSIZENEAR_PRESENT_ABS,		//峰谷值判定，峰谷值阈值采用范围内左右两侧最大值都比波谷大于最大值的百分比阈值,选出波谷
        MHC_PEAKTROUGH_BOGU_TOWSIZENEAR_ABS,				//峰谷值判定，峰谷值阈值采用范围内左右两侧最大值都比波谷大于最大值的阈值，选出波谷
        MHC_PEAKTROUGH_BOFENG_TOWSIZENEAR_PRESENT_ABS,		//峰谷值判定，峰谷值阈值采用范围内左右两侧最小值都比波峰大于波峰值的百分比阈值,选出波峰
        MHC_PEAKTROUGH_BOFENG_TOWSIZENEAR_ABS,				//峰谷值判定，峰谷值阈值采用范围内左右两侧最小值都比波峰大于波峰值的阈值，选出波峰
        MHC_PEAKTROUGH_BOGU_TOWSIZENEAR2_PRESENT_ABS,		//峰谷值判定，峰谷值阈值采玫部分外,范围内左右两侧最大值都比波谷大于最大值的百分比阈值,选出波谷
        MHC_PEAKTROUGH_BOGU_TOWSIZENEAR2_ABS,				//峰谷值判定，峰谷值阈值采用除谷值部分外,范围内左右两侧最大值都比波谷大于最大值的阈值，选出波谷
        MHC_PEAKTROUGH_BOFENG_TOWSIZENEAR2_PRESENT_ABS,		//峰谷值判定，峰谷值阈值采用除峰值部分外,范围内左右两侧最小值都比波峰大于波峰值的百分比阈值,选出波峰
        MHC_PEAKTROUGH_BOFENG_TOWSIZENEAR2_ABS				//峰谷值判定，峰谷值阈值采用除峰值部分外,范围内左右两侧最小值都比波峰大于波峰值的阈值，选出波峰
    }peakselecttype_1;

    //筛选峰峰值模式2
    typedef enum PEAKSELECTTYPE_2
    {
        MHC_PEAKTROUGH_BOGU_ABS,							//峰谷值判定，峰谷值阈值采用两侧波峰绝对值数值大于谷值模式,选出波谷
        MHC_PEAKTROUGH_BOGU_ONESIDE,						//峰谷值判定，峰谷值阈值采用单侧波峰绝对值数值大于谷值模式,选出波谷
        MHC_PEAKTROUGH_BOFENG_ABS,							//峰谷值判定，峰谷值阈值采用两侧波谷绝对值数值小于峰值模式,选出波峰
        MHC_PEAKTROUGH_BOFENG_ONESIDE						//峰谷值判定，峰谷值阈值采用单侧波谷绝对值数值小于峰值模式,选出波峰
    }peakselecttype_2;

    //峰峰值滤波模式
    typedef enum PEAKLVBOTYPE
    {
        MHC_PEAKTROUGH_BOGU_LVBO,			//波谷滤波（去掉孤立谷）
        MHC_PEAKTROUGH_BOFENG_LVBO			//波峰滤波（去掉孤立的波峰）
    }peaklvbotype;

    //漫水填充波形模式
    typedef enum PEAKWATERTYPE
    {
        MHC_PEAKTROUGH_BOGU_WATER,			//波谷填充
        MHC_PEAKTROUGH_BOFENG_WATER			//波峰填充
    }peakwatertype;

    //筛选连通模式
    typedef enum MYSELECT
    {
        MHC_CONNECT_WIDTH,			//筛选出长度大于阈值value1,小于等于阈值value2的区域
        MHC_CONNECT_HEIGHT,			//筛选出高度大于阈值value1,小于等于阈值value2的区域
        MHC_CONNECT_WIDTH_HEIGHT,	//筛选出长度或高度大于阈值value1,小于等于阈值value2的区域
        MHC_CONNECT_DIAGONAL,		//筛选出对角线大于阈值value1,小于等于阈值value2的区域
        MHC_CONNECT_MIANJI,			//筛选出面积大于阈值value1,小于等于阈值value2的区域
        MHC_WEIGHT_X_ROOM,			//筛选出重心X轴坐标大于阈值value1,小于等于阈值value2的区域
        MHC_WEIGHT_Y_ROOM			//筛选出重心Y轴坐标大于阈值value1,小于等于阈值value2的区域
    }myselect;

    //查询信息
    typedef enum REGION_INFO
    {
        MHC_HEIGHT_TOTAL,			//获取每个区域的高度的和
        MHC_WIDTH_TOTAL				//获取每个区域的宽度的和
    }region_info;

    //拟合模式
    typedef enum SQARETYPE
    {
        MHC_LINE_SQARE,			//线性拟合
        MHC_VAR_SQARE,			//二乘拟合
        MHC_HEADTAIL_SQARE,		//首尾拟合
        MHC_HOUGH_SQARE,		//霍夫拟合
        MHC_MIXDIS_SQARE        //最小距离拟合
    }sqaretype;

    //排序连通模式
    typedef enum SORTREGION
    {
        MHC_MIANJI_PAIXU,				//按照面积从小到大排序
        MHC_LEFTTORIGHT_PAIXU,			//按照连通点中心从左往右排序
        MHC_RIGHTTOLEFT_PAIXU,			//按照连通点中心从右往左排序
        MHC_TOPTOBOTTOM_PAIXU,			//按照连通点中心从上往下排序
        MHC_BOTTOMTOTOP_PAIXU,			//按照连通点中心从下往上排序
        MHC_LEFT_LEFTTORIGHT_PAIXU,		//按照连通点左边点从左往右排序
        MHC_RIGHT_LEFTTORIGHT_PAIXU,	//按照连通点右边点从左往右排序
        MHC_LEFT_RIGHTTOLEFT_PAIXU,		//按照连通点左边点从右往左排序
        MHC_RIGHT_RIGHTTOLEFT_PAIXU,	//按照连通点右边点从右往左排序
        MHC_TOP_TOPTOBOTTOM_PAIXU,		//按照连通点上边点从上往下排序
        MHC_BOTTOM_TOPTOBOTTOM_PAIXU,	//按照连通点下边点从上往下排序
        MHC_TOP_BOTTOMTOTOP_PAIXU,		//按照连通点上边点从下往上排序
        MHC_BOTTOM_BOTTOMTOTOP_PAIXU,	//按照连通点下边点从下往上排序
        MHC_WIDTH_PAIXU,				//按照宽度从小到大排序
        MHC_HEIGHT_PAIXU,				//按照高度从小到大排序
        MHC_COLOR_PAIXU					//按照颜色从小到大排序
    }sortregion;

    //链表
    typedef struct POLYEDGE
    {
        Int32 y0, y1;
        Int32 x, dx;
        struct POLYEDGE *next;
    }PolyEdge;

    //霍夫图平面提取信息
    typedef struct	MHC_HOU_LINEINFO
    {
        Int32 theta;			//横坐标
        Int32 rho;				//纵坐标
        Int32 maxlongsize;		//线长度
        Int32 drho;				//微调
    }mhc_hou_lineinfo;

    //排序信息
    typedef struct MYPAIXU
    {
        float value;	//排序值
        Int32 i;		//排序下标
    }Mypaixu;

//Mat矩阵结构体
    typedef struct MAT{
        Uint32 nWidth;			//全矩阵长
        Uint32 nHeight;			//全矩阵宽
        Uint32 startx;			//有效矩阵起点位置X
        Uint32 starty;			//有效矩阵起点位置Y
        Uint32 width;			//有效矩阵长
        Uint32 height;			//有效矩阵宽
        Uint8 *data;			//矩阵数据首地址
        Uint8 *ptr_uchar;		//矩阵数据首地址(Uint8)寻址	用于CV_8UC1型矩阵
        Int8 *ptr_char;			//矩阵数据首地址(Int8)寻址	用于CV_8SC1型矩阵
        Uint16 *ptr_ushort;		//矩阵数据首地址(Uint16)寻址 用于CV_16UC1型矩阵
        Int16 *ptr_short;		//矩阵数据首地址(Int16)寻址	用于CV_16SC1型矩阵
        Int32 *ptr_int;			//矩阵数据首地址(Int32)寻址	用于CV_32SC1型矩阵
        float *ptr_float;		//矩阵数据首地址(float)寻址	用于CV_32FC1型矩阵
        double *ptr_double;		//矩阵数据首地址(double)寻址用于CV_64FC1型矩阵
        Vec3b *ptr_Vec3b;		//矩阵数据首地址(Vec3b)寻址 用于CV_8UC3型矩阵
        Vec3ib *ptr_Vec3ib;		//矩阵数据首地址(Vec3ib)寻址 用于CV_8SC3型矩阵
        Vec3s *ptr_Vec3s;		//矩阵数据首地址(Vec3s)寻址 用于CV_16UC3型矩阵
        Vec3w *ptr_Vec3w;		//矩阵数据首地址(Vec3w)寻址 用于CV_16SC3型矩阵
        Vec3i *ptr_Vec3i;		//矩阵数据首地址(Vec3i)寻址 用于CV_32SC3型矩阵
        Vec3f *ptr_Vec3f;		//矩阵数据首地址(Vec3f)寻址 用于CV_32FC3型矩阵
        Vec3d *ptr_Vec3d;		//矩阵数据首地址(Vec3d)寻址 用于CV_64FC3型矩阵
        type _type;				//矩阵类型
    }Mat;

    //单元联通信息
    typedef struct MARKPOINT
    {
        Uint32 left;	 //单元点左侧
        Uint32 right;	 //单元点右侧
        Uint32 top;		 //单元点上侧
        Uint32 bottom;	 //单元点下侧
        Uint32 xTotal;	 //单元点X轴总数
        Uint32 yTotal;	 //单元点Y轴总数
        Uint32 PointArea;//单元点面积
        Uint8 color;	 //color标记,单元点映射标记,一般用于旋转\映射等函数变化,可对原联通区寻下标
        L_Point *point;	 //单元点的坐标
    }MarkPoint;

    //联通信息
    typedef struct MYCONECT
    {
        L_Point *memPoint;							//单元点的内存存放的真实位置,最好不要从外部访问,除非你很有信心
        Uint32 AllMarkPointCount;					//全部连通区域的个数
        Uint32 mianji;								//全部连通区域的总面积
        Uint32 nWidth;								//矩阵数据长度
        Uint32 nHeight;								//矩阵数据宽度
        Uint32 startx;								//有效矩阵起点位置X
        Uint32 starty;								//有效矩阵起点位置Y
        Uint32 width;								//有效矩阵长
        Uint32 height;								//有效矩阵宽
        MarkPoint AllMarkPoint[MHC_MARK_POINTNUM];	//每一个联通单元信息,从大到小最多存放255个
    }MyConect;

    //线的霍夫面坐标信息
    typedef struct HOUGHLINEINFO
    {
        Uint16 theta;								//霍夫平面上的纵坐标(平面原点在霍夫图象中心)
        Uint16 rho;									//霍夫平面上的横坐标(平面原点在霍夫图象中心)
    }houghlineinfo;

    //线堆信息
    typedef struct MYHOUGHLINE
    {
        L_line line[MHC_SEARCH_LINENUM];				//全部线的起终点的内存存放的位置
        houghlineinfo houghinfo[MHC_SEARCH_LINENUM];	//线的霍夫面坐标信息
        Uint8 linenum;									//全部线条数
        Uint32 nWidth;									//矩阵数据长度
        Uint32 nHeight;									//矩阵数据宽度
        Uint32 startx;									//有效矩阵起点位置X
        Uint32 starty;									//有效矩阵起点位置Y
        Uint32 width;									//有效矩阵长
        Uint32 height;									//有效矩阵宽
    }MyCountLine;

    //波形图导数信息
    typedef struct PEAKTROUGH_INFO
    {
        Int8 Left_diff;			//左测导数（1上升,-1下降）
        Int8 Right_diff;		//右侧导数（1上升,-1下降）
        Int32 Left_xiabiao;		//左测数据下标
        Int32 Right_xiabiao;	//右侧数据下标
        Int32 value;			//本身数值
    }peaktrough_info;

    //波形图堆栈信息
    typedef struct PEAKTROUGH_QUE
    {
        Int32 value;	//数值
        Int32 n;		//原数组的下标
    }peaktrough_Que;

    //波形图堆栈信息
    typedef struct PEAKTROUGH_QUE32FC
    {
        float value;	//数值
        Int32 n;		//原数组的下标
    }peaktrough_Que32FC;


    void *MEM_alloc(Int32 add ,Int32 size,Int32 heep);

    void MatToCvMat(Mat matIn,cv::Mat *matOut);     //Mat与opencv的Mat转换

    void CvMatToMat(cv::Mat matIn,Mat *matOut,void *buffer);    //opencv的Mat与Mat转换


    /*******************以下函数为Myhalcv库初始化类*************************/

    //内存初始化,想使用这个库前必须先调用它一次申请内存,nWidth和nHeight是算法处理过程中最大图的宽和高,且必须为64的整数倍
    void MyhalcvMemInit(Uint32 nHeight,Uint32 nWidth);

    //内存初始化释放,不再使用这个库前必须先调用它一次释放内存
    void MyhalcvMemFree();

    /*******************以下函数为创建Mat类*************************/
    Mat MatCreat(Uint32 nHeight,	//要创建的Mat数据高度
                 Uint32 nWidth,		//要创建的Mat数据宽度
                 type Mod,			//要创建的Mat类型,详见type定义
                 void *bufferIn);	//输入要创建的Mat数据内存存放空间,bufferIn大小至少为nHeight*nWidth*getsizeof(Mat._type)

    //创建一个Mat矩阵,并赋上初值color
    Mat MatCreat1col(Uint32 nHeight,	//要创建的Mat数据高度
                     Uint32 nWidth,		//要创建的Mat数据宽度
                     type Mod,			//要创建的Mat类型,详见type定义
                     void *bufferIn,	//输入要创的Mat数据内存存放空间,bufferIn大小至少为nHeight*nWidth*getsizeof(Mat._type)
                     double color);		//要创建的Mat数据内存中的初值

    //创建一个Mat矩阵,并按通道分别赋上初值color1,color2,color3,一般用于3通道型Mat,如果用于单通型Mat,则初值为三者的平均数
    Mat MatCreat3col(Uint32 nHeight,	//要创建的Mat数据高度
                     Uint32 nWidth,		//要创建的Mat数据宽度
                     type Mod,			//要创建的Mat类型,详见type定义
                     void *bufferIn,	//输入要创建的Mat数据内存存放空间,bufferIn大小至少为nHeight*nWidth*getsizeof(Mat._type)
                     double color1,		//要创建的Mat数据内存中通道1的初值
                     double color2,		//要创建的Mat数据内存中通道2的初值
                     double color3);	//要创建的Mat数据内存中通道3的初值

    //创建一Mat矩阵,并赋上初值0
    Mat MatCreatzero(Uint32 nHeight,	//要创建的Mat数据高度
                     Uint32 nWidth,		//要创建的Mat数据宽度
                     type Mod,			//要创建的Mat类型,详见type定义
                     void *bufferIn);	//输入要创建的Mat数据内存存放空间,bufferIn大小至少为nHeight*nWidth*getsizeof(Mat._type)

    //创建一个从Mat矩阵,内容在matIn上复制
    Mat MatCreatClone(Mat matIn,			//要复制的Mat矩阵对象
                      void *bufferIn);		//输入要创建的Mat数据内存存放空间,bufferIn大小至少为nHeight*nWidth*getsizeof(Mat._type);

    //创建一个Mat，地址指向matIn，一般用于给matIn再添加一个名称
    Mat MatPointto(Mat *matIn);				//输入要指向的Mat矩阵对象

    //创建一个Hough型Mat
    Mat HoughCreat(void *bufferIn);	//输入要创建的Mat数据内存存放空间,bufferIn大小至少为getHoughsize();


    /*********************以下函数如果有输出*Mat部分，则需要保证输出*Mat图像内存有足够的空间，否则将越界,函数运行成功返回0，否则返回非0************************/
    //把matIn_Out数据类型转换,(需要保证转换之后的类型有足够的内存，所以一般建议从大类型转换到小类型.如果转换后与转换前的类型不同，则转换后出来的图像内存信息数据需要重新赋值)
    Int8 MyconvertTo(Mat *matIn_Out,type Mod);

    //把matIn连同内存数据一同转换到matOut,多通道转单通道会取三通道的平均数,单通道转多通道会用单通道的值,
    Int8 MyconvertDataTo(Mat matIn,	  //输入想要转换的Mat型矩阵
                         Mat *matOut, //输出想要转换的Mat型矩阵
                         type Mod);   //想要转换过去的类型,需要注意转换后的Mat类型是否有足够空间

    //把Mat灰度图转换为彩色RGB图(只作用于有效区域)
    Int8 MyBRYtoRGB(Mat matIn,		//输入灰度图,仅支持CV_8UC1类型
                    Mat *matOut);	//输出彩色图,类型为CV_8UC3,需要注意转换后的Mat类型是否有足够空间


    //把彩色RGB图转换为Mat灰度图图(只作用于有效区域)
    Int8 MyRGBtoBRY(Mat matIn,		//输入彩色图,仅支持CV_8UC3类型
                    Mat *matOut);	//输出灰度图,类型为CV_8UC1,需要注意转换后的Mat类型是否有足够空间


    //给Mat矩阵全员赋值color
    Int8 MatSet1col(Mat *matIn_Out,		//输入/输出想要赋值的Mat型矩阵，
                    double color);		//想要赋的值

    //给Mat矩阵全员按通道分别赋值color1,color2,color3,一般用于3通道型Mat,如果用于单通道型Mat,则初值为三者的平均数
    Int8 MatSet3col(Mat *matIn_Out,		//输入/输出想要赋值的Mat型矩阵
                    double color1,		//想要给通道1赋的值
                    double color2,		//想要给通道2赋的值
                    double color3);		//想要给通道3赋的值

    //把matIn矩阵拷贝到matOut矩阵
    Int8 MatClone(Mat matIn,			//输入想要拷贝的Mat矩阵
                  Mat *matOut);			//输出想要拷贝过去的Mat矩阵,需要注意是否有足够空间

    //把matIn矩阵部分拷贝添加到matOut矩阵
    Int8 MatClonesizeadd(Mat matIn,				//输入想要拷贝的Mat矩阵
                         Mat *matIn_Out,		//输入/输出想要拷贝过去的Mat矩阵,与matIn同类型同尺寸,需要注意是否有足够空间
                         Uint32 nStartX,		//拷贝区域X起点
                         Uint32 nStartY,		//拷贝区域Y起点
                         Uint32 CopyWidth,		//拷贝区域的宽度
                         Uint32 CopyHeight);	//拷贝区域的高度

    //更新Mat矩阵有效区域
    Int8 MyCutRoi(Mat matIn,		//输入待更新的Mat矩阵
                  Mat *matOut,		//输出更新完毕的Mat矩阵
                  cutroi Mod,		//Mod=MHC_CUT_NOTCOPY为不拷贝内存模式,此时matOut可不用内存,数据指向原图,Mod=MHC_CUT_COPY为拷贝更新,此时matOut需要内存,数据指向新内存
                  Uint32 nStartX,	//更新有效区域X起点
                  Uint32 nStartY,	//更新有效区域Y起点
                  Uint32 CutWidth,	//更新有效区域的宽度
                  Uint32 CutHeight);//更新有效区域的高度

    //更新Mat矩阵有效区域
    Int8 MyCutselfRoi(Mat *matIn_Out,	//输入/输出待更新的Mat矩阵
                      Uint32 nStartX,	//更新有效区域X起点
                      Uint32 nStartY,	//更新有效区域Y起点
                      Uint32 CutWidth,	//更新有效区域的宽度
                      Uint32 CutHeight);//更新有效区域的高度

    //切割下小块图片
    Int8 MyCutminRoi(Mat matIn,			//输入待更新的Mat矩阵,仅支持CV_8UC1,CV_8SC1,CV_8UC3,CV_16UC1,CV_16SC1,CV_32SC1,CV_32FC1类型
                     Mat *matOut,		//输出更新完毕的Mat矩阵,类型与matIn相同,注意需要有足够内存
                     cutminroi Mod,		//Mod=MHC_CUTMIN_1时matOut宽度等于CutWidth,Mod=MHC_CUTMIN_4时matOut宽度按CutWidth的4的倍数补零,Mod=MHC_CUTMIN_8时matOut宽度按CutWidth的8的倍数补零,Mod=MHC_CUTMIN_32时matOut宽度按CutWidth的32的倍数补零,Mod=MHC_CUTMIN_64时matOut宽度按CutWidth的64的倍数补零
                     Uint32 nStartX,	//切割小块X起点
                     Uint32 nStartY,	//切割小块Y起点
                     Uint32 CutWidth,	//切割小块的宽度
                     Uint32 CutHeight);	//切割小块的高度

    //把小块图片添加到大图上
    Int8 MyAddminRoi(Mat matIn,			//输入待更新的Mat矩阵,仅支持CV_8UC1,CV_8SC1,CV_8UC3,CV_16UC1,CV_16SC1,CV_32SC1,CV_32FC1类型(只刷新其有效部分)
                     Mat matRoiIn,		//输入待添加的小Mat矩阵,类型与matIn相同(只添加其有效区域部分)
                     Mat *matOut,		//输出更新完毕的Mat矩阵,类型与matIn相同,注意需要有足够内存
                     Uint32 addStartX,	//添加位置X起点
                     Uint32 addStartY);	//添加位置Y起点

    //把Mat有效区域外赋值为0
    Int8 MyCutRoiSetZero(Mat *matIn_Out);	//输入/输出待更新的Mat矩阵

    //均衡化(只处理Mat矩阵的有效区域)
    Int8 Myequ_histo_image(Mat matIn,		//输入待处理矩阵,仅支持输入CV_8UC1型
                           Mat *matOut);	//输出处理结果,输出CV_8UC1型,注意需要有足够内存

    //均衡化(只处理Mat矩阵的有效区域)
    Int8 Myequ_histo_image_mask(Mat matIn,		//输入待处理矩阵,仅支持输入CV_8UC1型
                                Mat *matOut,	//输出处理结果,输出CV_8UC1型,注意需要有足够内存
                                Mat mask);		//只对mask中不为0的区域做均衡化,mask需为CV_8UV1型,且与matIn等尺寸

    //逐行或者逐列为单位均衡化(只处理Mat矩阵的有效区域)
    Int8 Myequ_histo_image_lineXY(Mat matIn,		//输入待处理矩阵,仅支持输入CV_8UC1型
                                  Mat *matOut,		//输出处理结果,输出CV_8UC1型,注意需要有足够内存
                                  histoline Mod);	//Mod=MHC_HISTO_X为逐行Mod=MHC_HISTO_Y为逐列

    //归一化(只处理Mat矩阵的有效区域)
    Int8 Mynormalize(Mat matIn,		//输入待处理矩阵,仅支持输入CV_8UC1,CV_8SC1,CV_16UC1,CV_16SC1,CV_32SC1,CV_32FC1,CV_64FC1型
                     Mat *matOut);	//输出处理结果,输出CV_8UC1型,注意需要有足够内存

    //归一化(只处理Mat矩阵的有效区域)
    Int8 Mynormalize_mask(Mat matIn,	//输入待处理矩阵,仅支持输入CV_8UC1,CV_8SC1,CV_16UC1,CV_16SC1,CV_32SC1,CV_32FC1,CV_64FC1型
                          Mat *matOut,	//输出处理结果,输出CV_8UC1型,注意需要有足够内存
                          Mat mask);	//只对mask中不为0的区域归一化,mask需为CV_8UV1型,且与matIn等尺寸

    //逐行归一化(只处理Mat矩阵的有效区域)
    Int8 Mynormalize_lineXY(Mat matIn,		//输入待处理矩阵,仅支持输入CV_8UC1,CV_8SC1,CV_16UC1,CV_16SC1,CV_32SC1,CV_32FC1,CV_64FC1型
                            Mat *matOut,	//输出处理结果,输出CV_8UC1型,注意需要有足够内存
                            double value);	//value表示每行数据最大与最小值的差距在value值以上的,这行做归一化,否则这行数据为0,value最小值为1

    //逐行归一化(只处理Mat矩阵的有效区域)
    Int8 Mynormalize_lineXY_mask(Mat matIn,			//输入待处理矩阵,仅支持输入CV_8UC1,CV_8SC1,CV_16UC1,CV_16SC1,CV_32SC1,CV_32FC1,CV_64FC1型
                                 Mat *matOut,		//输出处理结果,输出CV_8UC1型,注意需要有足够内存
                                 Mat mask,			//只对mask中不为0的区域归一化,mask需为CV_8UV1型,且与matIn等尺寸
                                 double value);		//value表示每列数据最大与最小值的差距在value值以上的,这行做归一化,否则这行数据为0,value最小值为1

    //逐行归一化(只处理Mat矩阵的有效区域)
    Int8 Mynormalize_lineXY_line(Mat matIn,			//输入待处理矩阵,仅支持输入CV_8UC1,CV_8SC1,CV_16UC1,CV_16SC1,CV_32SC1,CV_32FC1,CV_64FC1型
                                 Mat *matOut,		//输出处理结果,输出CV_8UC1型,注意需要有足够内存
                                 L_line lineIn,		//只对lineIn附近做区域归一化
                                 Int32 lineInsize,	//做归一化的lineIn直线的宽度
                                 double value);		//value表示每列数据最大与最小值的差距在value值以上的,这行做归一化,否则这行数据为0,value最小值为1

    //逐行归一化(只处理Mat矩阵的有效区域)
    Int8 Mynormalize_lineXY_line32(Mat matIn,			//输入待处理矩阵,仅支持输入CV_8UC1,CV_8SC1,CV_16UC1,CV_16SC1,CV_32SC1,CV_32FC1,CV_64FC1型
                                   Mat *matOut,			//输出处理结果,输出CV_8UC1型,注意需要有足够内存
                                   L_line32 l32_line,	//只对l32_line附近做区域归一化
                                   Int32 lineInsize,	//做归一化的lineIn直线的宽度
                                   double value);		//value表示每列数据最大与最小值的差距在value值以上的,这行做归一化,否则这行数据为0,value最小值为1

    //逐行归一化(只处理Mat矩阵的有效区域)
    Int8 Mynormalize_lineXY_downvalue(Mat matIn,			//输入待处理矩阵,仅支持输入CV_8UC1,CV_8SC1,CV_16UC1,CV_16SC1,CV_32SC1,CV_32FC1,CV_64FC1型
                                      Mat *matOut,			//输出处理结果,输出CV_8UC1型,注意需要有足够内存
                                      double value,			//value表示每列数据最大与最小值的差距在value值以上的,这行做归一化,否则这行数据为0,value最小值为1
                                      double downvalue);	//大于downvalue像素为过曝点,不纳入统计

    //逐行归一化(只处理Mat矩阵的有效区域)
    Int8 Mynormalize_lineXY_downvalue_mask(Mat matIn,			//输入待处理矩阵,仅支持输入CV_8UC1,CV_8SC1,CV_16UC1,CV_16SC1,CV_32SC1,CV_32FC1,CV_64FC1型
                                           Mat *matOut,			//输出处理结果,输出CV_8UC1型,注意需要有足够内存
                                           Mat mask,			//只对mask中不为0的区域归一化,mask需为CV_8UV1型,且与matIn等尺寸
                                           double value,		//value表示每列数据最大与最小值的差距在value值以上的,这行做归一化,否则这行数据为0,value最小值为1
                                           double downvalue);	//大于downvalue像素为过曝点,不纳入统计

    //逐列归一化(只处理Mat矩阵的有效区域)
    Int8 Mynormalize_rowXY(Mat matIn,		//输入待处理矩阵,仅支持输入CV_8UC1,CV_8SC1,CV_16UC1,CV_16SC1,CV_32SC1,CV_32FC1,CV_64FC1型
                           Mat *matOut,		//输出处理结果,输出CV_8UC1型,注意需要有足够内存
                           double value);	//value表示每列数据最大与最小值的差距在value值以上的,这行做归一化,否则这行数据为0,value最小值为1

    //逐列归一化(只处理Mat矩阵的有效区域)
    Int8 Mynormalize_rowXY_mask(Mat matIn,			//输入待处理矩阵,仅支持输入CV_8UC1,CV_8SC1,CV_16UC1,CV_16SC1,CV_32SC1,CV_32FC1,CV_64FC1型
                                Mat *matOut,		//输出处理结果,输出CV_8UC1型,注意需要有足够内存
                                Mat mask,			//只对mask中不为0的区域归一化,mask需为CV_8UV1型,且与matIn等尺寸
                                double value);		//value表示每列数据最大与最小值的差距在value值以上的,这行做归一化,否则这行数据为0,value最小值为1

    //逐列归一化(只处理Mat矩阵的有效区域)
    Int8 Mynormalize_rowXY_line(Mat matIn,			//输入待处理矩阵,仅支持输入CV_8UC1,CV_8SC1,CV_16UC1,CV_16SC1,CV_32SC1,CV_32FC1,CV_64FC1型
                                Mat *matOut,		//输出处理结果,输出CV_8UC1型,注意需要有足够内存
                                L_line lineIn,		//只对lineIn附近做区域归一化
                                Int32 lineInsize,	//做归一化的lineIn直线的宽度
                                double value);		//value表示每列数据最大与最小值的差距在value值以上的,这行做归一化,否则这行数据为0,value最小值为1

    //逐列归一化(只处理Mat矩阵的有效区域)
    Int8 Mynormalize_rowXY_line32(Mat matIn,			//输入待处理矩阵,仅支持输入CV_8UC1,CV_8SC1,CV_16UC1,CV_16SC1,CV_32SC1,CV_32FC1,CV_64FC1型
                                  Mat *matOut,			//输出处理结果,输出CV_8UC1型,注意需要有足够内存
                                  L_line32 l32_line,	//只对l32_line附近做区域归一化
                                  Int32 lineInsize,		//做归一化的lineIn直线的宽度
                                  double value);		//value表示每列数据最大与最小值的差距在value值以上的,这行做归一化,否则这行数据为0,value最小值为1

    //逐列归一化(只处理Mat矩阵的有效区域)
    Int8 Mynormalize_rowXY_downvalue(Mat matIn,			//输入待处理矩阵,仅支持输入CV_8UC1,CV_8SC1,CV_16UC1,CV_16SC1,CV_32SC1,CV_32FC1,CV_64FC1型
                                     Mat *matOut,		//输出处理结果,输出CV_8UC1型,注意需要有足够内存
                                     double value,		//value表示每列数据最大与最小值的差距在value值以上的,这行做归一化,否则这行数据为0,value最小值为1
                                     double downvalue);	//大于downvalue像素为过曝点,不纳入统计

    //逐列归一化(只处理Mat矩阵的有效区域)
    Int8 Mynormalize_rowXY_downvalue_mask(Mat matIn,			//输入待处理矩阵,仅支持输入CV_8UC1,CV_8SC1,CV_16UC1,CV_16SC1,CV_32SC1,CV_32FC1,CV_64FC1型
                                          Mat *matOut,			//输出处理结果,输出CV_8UC1型,注意需要有足够内存
                                          Mat mask,				//只对mask中不为0的区域归一化,mask需为CV_8UV1型,且与matIn等尺寸
                                          double value,			//value表示每列数据最大与最小值的差距在value值以上的,这行做归一化,否则这行数据为0,value最小值为1
                                          double downvalue);	//大于downvalue像素为过曝点,不纳入统计

    //平均值滤波(只处理Mat矩阵的有效区域)
    Int8 Mymean(Mat matIn,		//输入待处理矩阵,仅支持输入CV_8UC1
                Mat *matOut,	//输出处理结果,输出CV_8UC1型,注意需要有足够内存
                Uint8 sizew,	//输入宽方向上的滤波尺寸,可以为偶数,最小为1
                Uint8 sizeh);	//输入高方向上的滤波尺寸,可以为偶数,最小为1

    //平均值滤波(只处理Mat矩阵的有效区域)
    Int8 Mymean_mask(Mat matIn,		//输入待处理矩阵,仅支持输入CV_8UC1
                     Mat *matOut,	//输出处理结果,输出CV_8UC1型,注意需要有足够内存
                     Mat mask,		//只对mask中不为0的区域做均值,mask需为CV_8UV1型,且与matIn等尺寸
                     Uint8 sizew,	//输入宽方向上的滤波尺寸,可以为偶数,最小为1
                     Uint8 sizeh);	//输入高方向上的滤波尺寸,可以为偶数,最小为1

    //3x3窗口中值滤波(只处理Mat矩阵的有效区域)
    Int8 Mymedian(Mat matIn,		//输入待处理矩阵,仅支持输入CV_8UC1
                  Mat *matOut);		//输出处理结果,输出CV_8UC1型,注意需要有足够内存

    //中值滤波(只处理Mat矩阵的有效区域)
    Int8 Mymedian2(Mat matIn,		//输入待处理矩阵,仅支持输入CV_8UC1
                   Mat *matOut,		//输出处理结果,输出CV_8UC1型,注意需要有足够内存
                   Uint32 sizew,	//输入宽方向上的滤波尺寸,可以为偶数,最小为1
                   Uint32 sizeh);	//输入高方向上的滤波尺寸,可以为偶数,最小为1
                                    //当sizew*sizeh为偶数时,中值为sizew*sizeh/2后截尾,需要注意,在排序中值时,相同灰度值的像素点不能多于65535个

    //高斯滤波(只处理Mat矩阵的有效区域)
    Int8 Mygaussia(Mat matIn,		//输入待处理矩阵,仅支持输入CV_8UC1
                   Mat *matOut,		//输出处理结果,输出CV_8UC1型,注意需要有足够内存
                   gauss_win Mod);	//高斯窗口大小

    //Gabor实部滤波(只处理Mat矩阵的有效区域)(一般盖博实部滤波用来平滑)
    Int8 MyGabor(Mat matIn,			//输入待处理矩阵,仅支持输入CV_8UC1
                 Mat *matOut,		//输出卷积结果,输出CV_8UC1型,注意需要有足够内存
                 gabor_win size,	//生成盖博窗口大小
                 double theta,		//滤波方向弧度值,theta=0.0时,滤波器为竖状纹路,滤竖直条纹,theta=CV_PI/2时,滤波器为横状纹路,滤水平条纹
                 double sigma,		//标准差,盖博峰峰值,推荐1.0
                 double lambd,		//周期弧度值
                 double gamma,		//椭圆率纵横比,gamma=1.0时为标准圆
                 double psi);		//相位弧度值,psi=0时,高峰值为于正中央

    //Gabor虚部边缘检测(只处理Mat矩阵的有效区域)(一般先用盖博实部滤波平滑后,再用虚部滤波来边缘检测)
    Int8 MyImGabor(Mat matIn,			//输入待处理矩阵,仅支持输入CV_8UC1
                   Mat *matOut,			//输出卷积结果,输出CV_8UC1型,注意需要有足够内存
                   gabor_win size,		//生成盖博窗口大小
                   double theta,		//滤波方向弧度值,theta=0.0时,滤波器为竖状纹路,滤竖直条纹,theta=CV_PI/2时,滤波器为横状纹路,滤水平条纹
                   double sigma,		//标准差,盖博峰峰值,推荐1.0
                   double lambd,		//周期弧度值
                   double gamma,		//椭圆率纵横比,gamma=1.0时为标准圆
                   double psi);			//相位弧度值,psi=0时,高峰值为于正中央

    //索贝儿算子边缘检测(只处理Mat矩阵的有效区域)
    Int8 Mysobel(Mat matIn,			//输入待处理矩阵,仅支持输入CV_8UC1
                 Mat *matOut,		//输出处理结果,输出CV_8UC1型,注意需要有足够内存
                 sobel_win Mod,		//索贝儿窗口大小
                 Uint16 value);		//二值化域值

    //Canny算子边缘检测(只处理Mat矩阵的有效区域)
    Int8 Mycanny(Mat matIn,			//输入待处理矩阵,仅支持输入CV_8UC1
                 Mat *matOut,		//出处理结果,输出CV_8UC1型,注意需要有足够内存
                 gauss_win Mod,		//高斯窗口大小
                 Uint8 valueLow,	//低二值阈值,推荐如2
                 Uint8 valueHight); //高二值阈值,推荐如20

    //哈里斯角点检测(只处理Mat矩阵的有效区域)
    Int8 Myharris(Mat matIn,		//输入待处理矩阵,仅支持输入CV_8UC1
                  Mat *matOut,		//输出处理结果,输出CV_8UC1型,注意需要有足够内存
                  float paraMeter,	//敏感参数,推荐值如0.04
                  Uint16 value);	//二值化域值,推荐值如26

    //二值图细化(C语言版)(只处理Mat矩阵的有效区域)
    Int8 Myskeleton_cn(Mat matIn,			//输入待处理矩阵,仅支持输入CV_8UC1二值图
                       Mat *matOut,			//输出处理结果,输出CV_8UC1型,注意需要有足够内存
                       skeleton_cn Mod);	//细化算法类型

    //图像缩放(会作用于全部区域,需要小心使用)
    Int8 Myrescale(Mat matIn,			//输入待处理矩阵,仅支持输入CV_8UC1
                   Mat *matOut,			//输出处理结果,输出CV_8UC1型,注意需要有足够内存
                   float paraMeter);	//缩放倍数,大于1时为放大,小于1时为缩小,放大时有效区域也会一同放大

    //二值图边缘提取(只处理Mat矩阵的有效区域)
    Int8 MyMat2Contours(Mat matIn,		//输入待处理矩阵,仅支持输入CV_8UC1二值图
                        Mat *matOut);	//输出处理结果,输出CV_8UC1型,注意需要有足够内存

    //求交集(会作用于全部区域,需要小心使用)
    Int8 Myintersection(Mat matIn1,		//输入待处理矩阵,仅支持输入CV_8UC1,CV_16UC1二值图
                        Mat matIn2,		//输入待处理矩阵,与matIn1同类型同尺寸
                        Mat *matOut);	//输出处理结果,输出与matIn1同类型同尺寸,注意需要有足够内存

    //求并集(会作用于全部区域,需要小心使用)
    Int8 Myunion2(Mat matIn1,	//输入待处理矩阵,仅支持输入CV_8UC1,CV_16UC1二值图
                  Mat matIn2,	//输入待处理矩阵,与matIn1同类型同尺寸
                  Mat *matOut);	//输出处理结果,输出与matIn1同类型同尺寸,注意需要有足够内存

    //图象取反(只处理Mat矩阵的有效区域)
    Int8 Myinvert_image(Mat matIn,	//输入待处理矩阵,仅支持输入CV_8UC1,CV_16UC1二值图
                        Mat *matOut);//输出处理结果,输出与matIn1同类型同尺寸,注意需要有足够内存

    //求两图和(会作用于全部区域,需要小心使用)
    Int8 Myadd_image(Mat matIn1,	//输入待处理矩阵,仅支持输入CV_8UC1,CV_16UC1,CV_8SC1,CV_16SC1,
                     Mat matIn2,	//输入待处理矩阵,与matIn1同类型同尺寸
                     Mat *matOut);	//输出处理结果,输出与matIn1同类型同尺寸,注意需要有足够内存

    //求两图差(会作用于全部区域,需要小心使用)
    Int8 Mysub_image(Mat matIn1,	//输入待处理矩阵,仅支持输入CV_8UC1,CV_16UC1,CV_8SC1,CV_16SC1,
                     Mat matIn2,	//输入待处理矩阵,与matIn1同类型同尺寸
                     Mat *matOut);	//输出处理结果,输出与matIn1同类型同尺寸,注意需要有足够内存

    //求两图积(会作用于全部区域,需要小心使用)
    Int8 Mymul_image(Mat matIn1,	//输入待处理矩阵,仅支持输入CV_8UC1,CV_16UC1,CV_8SC1,CV_16SC1,
                     Mat matIn2,	//输入待处理矩阵,与matIn1同类型同尺寸
                     Mat *matOut);	//输出处理结果,输出CV_16SC1型和CV_32SC1型.与matIn1同尺寸,注意需要有足够内存

    //求反色(会作用于全部区域,需要小心使用)
    Int8 Myinvert_value(Mat matIn,		//输入待处理矩阵,仅支持输入CV_8UC1,CV_16UC1,CV_8SC1,CV_16SC1,
                        Mat *matOut);	//输出处理结果,与matIn同类型同尺寸,注意需要有足够内存

    //求每个数组元素加value(会作用于全部区域,需要小心使用)
    Int8 Myadd_value(Mat matIn,		//输入待处理矩阵,仅支持输入CV_8UC1,CV_16UC1,CV_8SC1,CV_16SC1,
                     Mat *matOut,	//输出处理结果,与matIn同类型同尺寸,注意需要有足够内存
                     Int32 value);	//输入想要加上的阈值

    //求每个数组元素减value(会作用于全部区域,需要小心使用)
    Int8 Mysub_value(Mat matIn,		//输入待处理矩阵,仅支持输入CV_8UC1,CV_16UC1,CV_8SC1,CV_16SC1,
                     Mat *matOut,	//输出处理结果,与matIn同类型同尺寸,注意需要有足够内存
                     Int32 value);	//输入想要减去的阈值

    //求每个数组元素乘value(会作用于全部区域,需要小心使用)
    Int8 Mymul_value(Mat matIn,		//输入待处理矩阵,仅支持输入CV_8UC1,CV_16UC1,CV_8SC1,CV_16SC1,
                     Mat *matOut,	//输出处理结果,输出CV_16SC1型和CV_32SC1型.与matIn1同尺寸,注意需要有足够内存
                     Int32 value);	//输入想要乘以的阈值

    //求绝对值
    Int8 Myabs_value(Mat matIn,		//输入待处理矩阵,仅支持输入CV_8UC1,CV_16UC1,CV_8SC1,CV_16SC1,
                     Mat *matOut);	//输出处理结果,与matIn同类型同尺寸,注意需要有足够内存

    //求卷积(只处理Mat矩阵的有效区域)
    Int8 Myfilter(Mat matIn,			//输入待卷积的矩阵支持CV_8UC1,CV_8SC1,CV_16UC1,CV_16SC1,CV_32SC1,CV_32FC1
                  Mat filterIn,			//输入卷积窗
                  Mat *matOut,			//输出卷积结果,类型由Mod决定,注意需要有足够内存
                  type Mod,				//想输出的结果类型,支持类型如下,返回值非零表示不支持
                  Int16 shift,			//想输出的结果做右移运算的位数,防止饱和,为0时结果保持原值,超出类型最大值时取最大值
                  L_Point32 center);	//卷积对齐中心,如果是(-1,-1),则自动为卷积窗中心
                                        //matIn为CV_8UC1,CV_8SC1时，输出可以为CV_8UC1,CV_8SC1,CV_16UC1,CV_16SC1,CV_32SC1,CV_32FC1
                                        //matIn为CV_16UC1,CV_16SC1时，输出可以为CV_16UC1,CV_16SC1,CV_32SC1,CV_32FC1
                                        //matIn为CV_32SC1,CV_32FC1时,输出可以为CV_32SC1,CV_32FC1

    //求图象金字塔,用2x2采样,求4点平均值(截尾)法,(会作用于全部区域,需要小心使用)
    Int8 Mypyramid(Mat matIn,		//输入待处理矩阵
                   Mat *matOut1,	//输出第一层图象,注意内存大小,至少为getsizeof(matIn._type)*(matIn.nHeight/2*matIn.nWidth/2)
                   Mat *matOut2,	//输出第二层图象,注意内存大小,至少为getsizeof(matIn._type)*(matIn.nHeight/4*matIn.nWidth/4)
                   Mat *matOut3);	//输出第三层图象,注意内存大小,至少为getsizeof(matIn._type)*(matIn.nHeight/8*matIn.nWidth/8)

    //求下一层的高斯金字塔,2x2采样,用5x5高斯窗口,(会作用于全部区域,需要小心使用)
    Int8 Mygausspyramid(Mat matIn,		//输入待处理矩阵
                        Mat *matOut);	//输出下一层高斯金字塔图象,注意内存大小,至少为getsizeof(matIn._type)*(matIn.nHeight/2*matIn.nWidth/2)

    //求下二层的高斯金字塔,2x2采样,用5x5高斯窗口,(会作用于全部区域,需要小心使用)
    Int8 Mygausspyramid_2levl(Mat matIn,        //输入待处理矩阵
                              Mat *matOut);     //输出下二层高斯金字塔图象,注意内存大小,至少为getsizeof(matIn._type)*(matIn.nHeight/4*matIn.nWidth/4)

    //求上一层高斯金字塔,(会作用于全部区域,需要小心使用)(如果再加上上一层的拉普拉斯塔(需要注意无符号类型的拉普拉斯塔数据的正负值可能是居中显示)就会变为上一层原图)
    Int8 MygausspyramidUp(Mat matIn,		//输入待处理矩阵
                          Mat *matOut);		//输出上一层高斯金字塔图象,注意内存大小,至少为getsizeof(matIn._type)*(matIn.nHeight*2*matIn.nWidth*2)

    //求当前层的拉普拉斯塔,2x2采样,用5x5高斯窗口,(会作用于全部区域,需要小心使用)
    Int8 Mylapalcepyramid(Mat matIn,				//输入当前层图象,仅支持输入CV_8UC1,CV_8SC1,CV_16UC1,CV_16SC1型
                          Mat gausspyramidIn,		//输入matIn对应的下一层的高斯金字塔图象
                          Mat *matOut);				//输出当前一层的拉普拉斯图象,注意内存大小,至少为getsizeof(matIn._type)*(matIn.nHeight*matIn.nWidth),为了处理正负数灰度值居中显示

    //求梯度金字塔,2x2采样,用5x5高斯梯度窗口,(会作用于全部区域,需要小心使用)
    Int8 Mygradpyramid(Mat matIn,				//输入待处理矩阵,仅支持输入CV_8UC1,CV_8SC1
                       gradpyramidtype Mod,		//输入水平/垂直梯度
                       Mat *matOut);			//输出下一层图象,注意内存大小,至少为getsizeof(matIn._type)*(matIn.nHeight/2*matIn.nWidth/2),为了处理正负数灰度值居中显示

    //统计matIn在mask非零部位的灰度和
    Int8 Myaddimagebry(Mat matIn,		//输入待处理矩阵,仅支持输入CV_8UC1,CV_8SC1,CV_16UC1,CV_16SC1,CV_32SC1型
                       Mat mask,		//输入掩模,仅支持输入CV_8UC1型二值图
                       Int32 *valout);	//输出灰度和

    //二值化(只处理Mat矩阵的有效区域)
    Int8 Mybinary(Mat matIn,		//输入待处理矩阵,仅支持输入CV_8UC1型
                  Mat *matOut,		//输出处理结果,输出CV_8UC1型,注意需要有足够内存
                  binarymod Mod,	//二值化模式,详见binarymod
                  double color,		//二值图结果中的目标显示的颜色(一般为255)
                  double value1,	//二值化阈值1,根据不同Mod有不同作用,详见binarymod
                  double value2);	//二值化阈值2,根据不同Mod有不同作用,详见binarymod


    //二值化(只处理Mat矩阵的有效区域)
    Int8 Mybinaryself(Mat *matIn_Out,	//输入/输出待处理矩阵,仅支持输入CV_8UC1型
                      binarymod Mod,	//二值化模式,详见binarymod
                      double color,		//二值图结果中的目标显示的颜色(一般为255)
                      double value1,	//二值化阈值1,根据不同Mod有不同作用,详见binarymod
                      double value2);	//二值化阈值2,根据不同Mod有不同作用,详见binarymod


    //根据全图有效区域进行统计,返回二值阈值
    Int8 Mybinaryval(Mat matIn,			//输入待处理矩阵,仅支持输入CV_8UC1型
                     Uint8 *valueOut,	//输出处理结果阈值
                     binaryvalmod Mod);	//二值化模式,详见binaryvalmod


    //根据掩膜有效区域进行统计,返回二值阈值(只处理Mat矩阵的有效区域)
    Int8 Mybinaryval_mask(Mat matIn,			//输入待处理矩阵,仅支持输入CV_8UC1型
                          Mat mask,				//只对mask中不为0的区域统计,mask需为CV_8UV1型,且与matIn等尺寸
                          Uint8 *valueOut,		//输出处理结果阈值
                          binaryvalmod Mod);	//二值化模式,详见binaryvalmod

    //找到矩阵中的最大最小元素，以及其坐标(只处理Mat矩阵的有效区域)
    Int8 Mymin_max_gray(Mat matIn,			//输入待处理矩阵,仅支持输入CV_8UC1,CV_8SC1,CV_16UC1,CV_16SC1,CV_32SC1,CV_32FC1,CV_64FC1
                        Int32 *min,			//找到的矩阵数据最小值
                        Int32 *max,			//找到的矩阵数据最大值
                        L_Point *minLoc,	//找到的矩阵数据最小值坐标
                        L_Point *maxLoc);	//找到数据最大值坐标


    //找到矩阵中的最大最小元素，以及其坐标(只处理Mat矩阵的有效区域)
    Int8 Mymin_max_dgray(Mat matIn,			//输入待处理矩阵,仅支持输入CV_8UC1,CV_8SC1,CV_16UC1,CV_16SC1,CV_32SC1,CV_32FC1,CV_64FC1型
                         double *min,		//找到的矩阵数据最小值
                         double *max,		//找到的矩阵数据最大值
                         L_Point *minLoc,	//找到的矩阵数据最小值坐标
                         L_Point *maxLoc);	//找到的矩阵数据最大值坐标


    //找到矩阵中在掩模mask区域内的最大最小元素，以及其坐标(只处理Mat矩阵的有效区域)
    Int8 Mymin_max_gray_mask(Mat matIn,			//输入待处理矩阵,仅支持输入CV_8UC1,CV_8SC1,CV_16UC1,CV_16SC1,CV_32SC1,CV_32FC1,CV_64FC1型
                             Int32 *min,		//找到的矩阵数据最小值
                             Int32 *max,		//找到的矩阵数据最大值
                             L_Point *minLoc,	//找到的矩阵数据最小值坐标
                             L_Point *maxLoc,	//找到的矩阵数据最大值坐标
                             Mat mask);			//只对mask中不为0的区域寻找,mask需为CV_8UV1型,且与matIn等尺寸


    //找到矩阵中的最大最小元素，以及其坐标(只处理Mat矩阵的有效区域)
    Int8 Mymin_max_dgray_mask(Mat matIn,		//输入待处理矩阵,仅支持输入CV_8UC1,CV_8SC1,CV_16UC1,CV_16SC1,CV_32SC1,CV_32FC1,CV_64FC1型
                              double *min,		//找到的矩阵数据最小值
                              double *max,		//找到的矩阵数据最大值
                              L_Point *minLoc,	//找到的矩阵数据最小值坐标
                              L_Point *maxLoc,	//找到的矩阵数据最大值坐标
                              Mat mask);		//只对mask中不为0的区域寻找,mask需为CV_8UV1型,且与matIn等尺寸

    //找到矩阵中灰度值分布最大的一行(只处理Mat矩阵的有效区域)
    Int8 MygetMaxLine(Mat matIn,		//输入待处理矩阵,仅支持输入CV_8UC1
                      Int32 size,		//输入上下行的统计尺寸
                      Int32 *valout);	//输出最大行的Y坐标

    //找到矩阵中灰度值分布最大的一列(只处理Mat矩阵的有效区域)
    Int8 MygetMaxRow(Mat matIn,			//输入待处理矩阵,仅支持输入CV_8UC1
                     Int32 size,		//输入左右列的统计尺寸
                     Int32 *valout);	//输出最大列的X坐标

    //保留矩阵中非0值个数大于num的一行(只处理Mat矩阵的有效区域)
    Int8 MygetbryLine(Mat matIn,		//输入待处理矩阵,仅支持输入CV_8UC1
                      Mat *matOut,		//输出处理结果
                      Int32 num,		//矩阵中非0值个数大于num则保留
                      Int32 size);		//输入上下行的统计尺寸

    //保留矩阵中非0值个数大于num的一列(只处理Mat矩阵的有效区域)
    Int8 MygetbryRow(Mat matIn,			//输入待处理矩阵,仅支持输入CV_8UC1
                     Mat *matOut,		//输出处理结果
                     Int32 num,			//矩阵中非0值个数大于num则保留
                     Int32 size);		//输入左右列的统计尺寸

    //把矩阵在mask等于0的部分赋值为0,其余部分保留原数据(只处理Mat矩阵的有效区域)
    Int8 MyMask_image(Mat *matIn_Out,	//输入/输出待处理矩阵
                      Mat mask);		//只对mask中不为0的区域寻找,其余赋值为0,mask需为CV_8UV1型,且与matIn等尺寸

    //统计阵非零区域面积(只处理Mat矩阵的有效区域)
    Int8 Mymianji_obj(Mat matIn,		//输入待处理矩阵
                      Int32 *mianji);	//得到的非零区域像素面积

    //统计阵非零区域的行数(只处理Mat矩阵的有效区域)
    Int8 MyLinenum_obj(Mat matIn,		//输入待处理矩阵
                       Int32 *line);	//得到的非零区域像素行数

    //统计阵非零区域的列数(只处理Mat矩阵的有效区域)
    Int8 MyRownum_obj(Mat matIn,		//输入待处理矩阵
                      Int32 *row);		//得到的非零区域像素列数

    //膨胀函数(只处理Mat矩阵的有效区域)
    Int8 Mydilation_circle(Mat matIn,		//输入待处理矩阵仅支持输入CV_8UC1二值图
                           Mat *matOut,		//输出处理结果,输出CV_8UC1型,背景为0,目标值为1,需要注意是否有足够空间
                           Uint32 size,		//输入膨胀的尺寸,size=0时输出为不膨胀的图
                           dilaero Mod);	//膨胀模式Mod=MHC_MORPH_ELLIPSE为椭圆Mod=MHC_MORPH_RECT为矩形

    //膨胀函数(只处理Mat矩阵的有效区域)
    Int8 Mydilation_circle2(Mat matIn,		//输入待处理矩阵仅支持输入CV_8UC1二值图
                            Mat *matOut,	//输出处理结果,输出CV_8UC1型,背景为0,目标值为1,需要注意是否有足够空间
                            Uint32 sizew,	//输入宽度方向膨胀的尺寸,sizew=0时宽度不膨胀
                            Uint32 sizeh,	//输入高度方向膨胀的尺寸,sizeh=0时高度不膨胀
                            dilaero Mod);	//膨胀模組Mod=MHC_MORPH_ELLIPSE为椭圆Mod=MHC_MORPH_RECT为矩形

    //腐蚀函数(只处理Mat矩阵的有效区域)
    Int8 Myerosion_circle(Mat matIn,		//输入处理矩阵仅支持输入CV_8UC1二值图
                          Mat *matOut,		//输出处理结果,输出CV_8UC1型,背景为0,目标值为1,需要注意是否有足够空间
                          Uint32 size,		//输入腐蚀的尺寸,size=0时输出为不腐蚀的图
                          dilaero Mod);		//腐蚀模式Mod=MHC_MORPH_ELLIPSE为椭圆Mod=MHC_MORPH_RECT为矩形

    //腐蚀函数(只处理Mat矩阵的有效区域)
    Int8 Myerosion_circle2(Mat matIn,		//输入待处理矩阵支持输入CV_8UC1二值图
                           Mat *matOut,		//输出处理结果,输出CV_8UC1型,背景为0,目标值为1,需要注意是否有足够空间
                           Uint32 sizew,	//输入宽度方向蚀的尺寸,sizew=0时宽度不腐蚀
                           Uint32 sizeh,	//输入高度方向腐蚀的尺寸,sizeh=0时高度不腐蚀
                           dilaero Mod);	//膨胀模式Mod=MHC_MORPH_ELLIPSE为椭圆Mod=MHC_MORPH_RECT为矩形

    //求连通(只处理Mat矩阵的有效区域)
    Int8 Myconnection(Mat matIn,					//输入待处理矩阵仅支持输入CV_8UC1二值图
                      MyConect *ImageConectOut,		//输出处理结果
                      Int32 value,					//value为联通目标面积,大于等于这个值为目标,最小为1,
                      Int32 distance,				//两块区域相距小于等于distance距离内被判为同块联通域,最小为0,
                      conectlt Mod,					//Mod=MHC_4LT为4连通,Mod=MHC_8LT为8连通
                      void *bufferIn);				//bufferIn为联通图内存地址,内存大小至少为Height*nWidth*getConectsize();

    //求连通2(只处理Mat矩阵的有效区域)
    Int8 Myconnection2(Mat matIn,					//输入待处理矩阵仅支持输入CV_8UC1二值图
                       MyConect *ImageConectOut,	//输出处理结果
                       Int32 value,					//value为联通目标面积,大于等于这个值为目标,最小为1,
                       Int32 distancew,				//两块区域横向相距小于等于distancew距离内被判为同块联通域,最小为0,
                       Int32 distanceh,				//两块区域纵向相距小于等于distanceh距离内被判为同块联通域,最小为0,
                       dilaero distancemod,			//distancemod=MHC_MORPH_ELLIPSE为椭圆距离distancemod=MHC_MORPH_RECT为矩形距离,当distancew和distanceh都为0时,distancemod可随意取其中之一
                       conectlt Mod,				//Mod=MHC_4LT为4连通,Mod=MHC_8LT为8连通
                       void *bufferIn);				//bufferIn为联通图内存地址,内存大小至少为Height*nWidth*getConectsize();

    //找到最大连通图,赋值为255,其余赋值为0(会作用于全部区域,需要小心使用)
    Int8 MyfindBigstconnection(Mat matIn,	 		//输入待处理矩阵仅支持输入CV_8UC1二值图
                               Mat *matOut,			//输出处理结果
                               Int32 value,			//value为联通目标面积,大于等于这个值为目标,最小为1,
                               Int32 distance,	 	//两块区域相距小于等于distance距离内被判为同块联通域,最小为0,
                               conectlt Mod,	 	//Mod=MHC_4LT为4连通,Mod=MHC_8LT为8连通
                               background Cod,		//Cod=MHC_BACK,背景为0,目标为color,Cod=MHC_BACK_INV背景为color,目标为0,Cod=MHC_BACK_WNV,背景为255,目标为color
                               Int32 color,			//输出目标颜色
                               VLIB_CC *PointInfo);	//输出最大连通的信息

    //找到最大连通图2,赋值为255,其余赋值为0(会作用于全部区域,需要小心使用),图像的宽需为32的倍数
    Int8 MyfindBigstconnection2(Mat matIn,	 			//输入待处理矩阵仅支持输入CV_8UC1二值图
                                Mat *matOut,			//输出处理结果
                                Int32 value,			//value为联通目标面积,大于等于这个值为目标,最小为1,
                                Int32 distancew,		//两块区域横向相距小于等于distancew距离内被判为同块联通域,最小为0,
                                Int32 distanceh,		//两块区域纵向相距小于等于distanceh距离内被判为同块联通域,最小为0,
                                dilaero distancemod,	//distancemod=MHC_MORPH_ELLIPSE为椭圆距离distancemod=MHC_MORPH_RECT为矩形距离,当distancew和distanceh都为0时,distancemod可随意取其中之一
                                conectlt Mod,	 		//Mod=MHC_4LT为4连通,Mod=MHC_8LT为8连通
                                background Cod,			//Cod=MHC_BACK,背景为0,目标为color,Cod=MHC_BACK_INV背景为color,目标为0,Cod=MHC_BACK_WNV,背景为255,目标为color
                                Int32 color,			//输出目标颜色
                                VLIB_CC *PointInfo);	//输出最大连通的信息

    //二值图中删去连通面积小于value的图(会作用于全部区域,需要小心使用)
    Int8 Mydeleteconnection(Mat matIn,			//输入待处理矩阵仅支持输入CV_8UC1二值图
                            Mat *matOut,		//输出处理结果
                            Int32 value,		//value为联通目标面积,小于这个值的被删去,最小为1,
                            Int32 distance,	 	//两块区域相距小于等于distance距离内被判为同块联通域,最小为0,
                            conectlt Mod);	 	//Mod=MHC_4LT为4连通,Mod=MHC_8LT为8连通

    //二值图中删去连通面积小于value的图2(会作用于全部区域,需要小心使用)
    Int8 Mydeleteconnection2(Mat matIn,				//输入待处理矩阵仅支持输入CV_8UC1二值图
                             Mat *matOut,			//输出处理结果
                             Int32 value,			//value为联通目标面积,小于这个值的被删去,最小为1,
                             Int32 distancew,		//两块区域横向相距小于等于distancew距离内被判为同块联通域,最小为0,
                             Int32 distanceh,		//两块区域纵向相距小于等于distanceh距离内被判为同块联通域,最小为0,
                             dilaero distancemod,	//distancemod=MHC_MORPH_ELLIPSE为椭圆距离distancemod=MHC_MORPH_RECT为矩形距离,当distancew和distanceh都为0时,distancemod可随意取其中之一
                             conectlt Mod);	 		//Mod=MHC_4LT为4连通,Mod=MHC_8LT为8连通

    //统计灰度直方图(会作用于全部区域,需要小心使用)
    Int8 MyhistoALLgram(Mat matIn,			//输入待处理图象仅支持输入CV_8UC1
                        Uint32 *hisOut); 	//输出处理结果,内存必须大于等于sizeof(Uint32)*256

    //统计灰度直方图(只处理Mat矩阵的有效区域)
    Int8 Myhistogram(Mat matIn,			//输入待处理图象仅支持输入CV_8UC1
                     Uint32 *hisOut); 	//输出处理结果,内存必须大于等于sizeof(Uint32)*256

    //统计灰度直方图(只统计mask不等于0部分)
    Int8 MyhistoMaskgram(Mat matIn,			//输入待处理图象仅支持输入CV_8UC1
                         Mat mask,			//只对mask中不为0的区域做统计,mask需为CV_8UV1型,且与matIn等尺寸
                         Uint32 *hisOut); 	//输出处理结果,内存必须大于等于sizeof(Uint32)*256

    //直方图匹配(只处理Mat矩阵的有效区域)
    Int8 MyhistMatch(Mat matIn,			//输入待处理矩阵,仅支持输入CV_8UC1型
                     Mat *matOut,		//输出处理结果,类型与matIn相同,注意需要有足够内存
                     Mat histmatIn);	//输入要匹配的模板图,仅支持输入CV_8UC1型

    //积分图像(只处理Mat矩阵的有效区域)
    Int8 Myintegral(Mat matIn,		//输入待处理矩阵,仅支持输入CV_8UC1,CV_8SC1,CV_16UC1,CV_16SC1型
                    Mat *matOut);	//输出处理结果,输出CV_32SC1型,注意需要有足够内存

    //计算数组积分8位
    Int8 Myintegral_8(Uint8 *pIn,			//输入数组
                      Uint16 width,			//数组宽度(需4的倍数)
                      Uint16 height,		//数组高度
                      Uint32 *pOut);		//输出数组

    //计算数组积分16位
    Int8 Myintegral_16(Uint16 *pIn,			//输入数组
                       Uint16 width,		//数组宽度(需4的倍数)
                       Uint16 height,		//数组高度
                       Uint32 *pOut);		//输出数组

    //float型转化为SQ0.15型
    Int8 My_float_to_q15(float *pIn,		//输入数组
                         Int16 num,			//输入数组个数,必须为偶数
                         Int16 *pOut);		//输出数组

    //SQ0.15型转化为float型
    Int8 My_q15_to_float(Int16 *pIn,		//输入数组
                         Int16 num,			//输入数组个数,必须为偶数
                         float *pOut);		//输出数组

    //把连通区域里的color标记按连通区域顺序重新更新
    Int8 Myregion_coloruse_label(MyConect *ImageConectIn_Out);	//输入/输出连通区域


    //连通区域转换成Mat(只处理Mat矩阵的有效区域)
    Int8 Myregion_to_bin(MyConect *ImageConectIn,	//输入连通区域
                         Mat *matOut,				//输出处理结果,输出CV_8UC1型,需要注意是否有足够空间
                         Int32 value);				//连通区域以value值在处理结果中显示(一般设255),背景为0，


    //把连通区域添加到Mat图像上(只处理Mat矩阵的有效区域)
    Int8 Myregion_to_add(MyConect *ImageConectIn,	//输入连通区域
                         Mat *matIn_Out,			//输入/输出要添加的Mat图,目前支持CV_8UC1,CV_8UC3,且要与ImageConectIn具有相同尺寸
                         Int32 color);				//color为添加的点要设置的颜色,如果是3通道,则3通道都设为color值


    //把连通区域添加到Mat图像上(只处理Mat矩阵的有效区域)
    Int8 Myregion_to_add3col(MyConect *ImageConectIn,	//输入连通区域
                             Mat *matIn_Out,			//输入/输出要添加的Mat图,目前支持CV_8UC1,CV_8UC3,且要与ImageConectIn具有相同尺寸
                             Int32 color1,				//color1为添加的点要设置的1通道的值,如果是单通道,则取3个color的平均值
                             Int32 color2,				//color2为添加的点要设置的2通道的值
                             Int32 color3);				//color3为添加的点要设置的3通道的值


    //连通区域按序转换成Label图(只处理Mat矩阵的有效区域) ImageConectOutLab=0为背景,ImageConectOutLab=1-255对应为0-254号连通区域
    Int8 Myregion_to_label(MyConect *ImageConectIn,	 //输入连通区域
                           Mat *ImageConectOutLab);	 //输出处理结果,输出CV_8UC1型,需要注意是否有足够空间

    //线集添加到Mat图像中显示
    Int8 Mylinegion_to_add(MyCountLine *LineIn,		//输入线集
                           Mat *matIn_Out,			//输入/输出要添加的Mat图,目前支持CV_8UC1,CV_8UC3,且要与LineIn具有相同尺寸
                           Int32 color,				//color为添加的线要设置的颜色,如果是3通道,则3通道都设为color值
                           linetype connectivity,	//直线连通模式
                           Int32 thickness);		//直线粗细

    //线集添加到Mat图像中显示
    Int8 Mylinegion_to_add3col(MyCountLine *LineIn,		//输入线集
                               Mat *matIn_Out,			//输入/输出要添加的Mat图,目前支持CV_8UC1,CV_8UC3,且要与LineIn具有相同尺寸
                               Int32 color1,			//color1为添加的线要设置的1通道的值,如果是单通道,则取3个color的平均值
                               Int32 color2,			//color2为添加的线要设置的2通道的值
                               Int32 color3,			//color3为添加的线要设置的3通道的值
                               linetype connectivity,	//直线连通模式
                               Int32 thickness);		//直线粗细

    //斜率截距式转MyCountLine类
    Int8 Mylinekb_to_gion(double *k_In,				//输入直线斜率数组
                          double *b_In,				//输入直线截距数组
                          Int32 num,				//输入数组个数,num需小于MHC_SEARCH_LINENUM
                          Uint32 nHeight,			//输入直线的矩阵高度
                          Uint32 nWidth,			//输入直线的矩阵宽度
                          MyCountLine *LineOut);	//输出直线

    //把Mat矩阵里所有非零区域设为value值,一般用于单通道图像(只处理Mat的有效区域)
    Int8 Mymat_to_binself(Mat *matIn_Out,			//输入/输出Mat矩阵
                          double value);			//value为把Mat矩阵中的所有非零值设为value,如果是3通道图像,则3通道都为value值

    //把Mat矩阵里所有非零区域设为value值,一般用于单通道图像(只处理Mat的有效区域)
    Int8 Mymat_to_bin(Mat matIn,		//输入Mat矩阵
                      Mat *matOut,		//输出Mat矩阵
                      double value);	//value为把Mat矩阵中的所有非零值设为value,如果是3通道图像,则3通道都为value值

    //把Mat矩阵里所有非零区域按各个通道分别设value1,value2,value3值,一般用于三通道图像,如果是用于单通道图,则为3个value值取平均
    Int8 Mymat_to_bin3colself(Mat *matIn_Out,			//输入/输出Mat矩阵
                              double value1,			//value1为把Mat矩阵中的所有非零值的通道1设为value1值
                              double value2,			//value2为把Mat矩阵中的所有非零值的通道1设为value1值
                              double value3);			//value3为把Mat矩阵中的所有非零值的通道1设为value1值

    //把Mat矩阵里所有非零区域按各个通道分别设value1,value2,value3值,一般用于三通道图像,如果是用于单通道图,则为3个value值取平均
    Int8 Mymat_to_bin3col(Mat matIn,			//输入Mat矩阵
                          Mat *matOut,			//输出Mat矩阵,类型与matIn相同,需要注意是否有足够空间
                          double value1,		//value1为把Mat矩阵中的所有非零值的通道1设为value1值
                          double value2,		//value2为把Mat矩阵中的所有非零值的通道1设为value1值
                          double value3);		//value3为把Mat矩阵中的所有非零值耐道1设为value1值

    //复制区域(复制信息,复制实际内存)
    Int8 MyCopyObj(MyConect *ImageConectIn,		//输入想要复制的区域
                   MyConect *ImageConectOut,	//输出结果
                   void *bufferIn);				//bufferIn为联通图内存地址,内存大小至少为Height*nWidth*getConectsize();

    //快速复制区域(只复制信息而不复制实际内存,实际内存还是指向原地址)
    Int8 MyCopyObjQuick(MyConect *ImageConectIn,	//输入想要复制的区域
                        MyConect *ImageConectOut);	//输出结果

    //筛选区域
    Int8 Myselect_shape(MyConect *ImageConectIn,	//输入想要筛选的连通区域
                        MyConect *ImageConectOut,	//输出筛选结果
                        myselect Mod,				//筛选模式,详见myselect
                        double value1,				//筛选阈值1,不同Mod情况下有不同的作用,详见myselect
                        double value2);				//筛选阈值2,不同Mod情况下有不同的作用,详见myselect

    //排序区域
    Int8 Mysort_region(MyConect *ImageConectIn,		//输入想要排序的区域
                       MyConect *ImageConectOut,	//输出排序结果
                       sortregion Mod);				//排序模式,详见sortregion

    //根据原图的灰度平均数排序区域
    Int8 Mysort_region_matIn(MyConect *ImageConectIn,	//输入想要排序的区域
                             Mat matIn,					//输入原图,类型为CV_8UC1,且与ImageConectIn尺寸相同
                             MyConect *ImageConectOut);	//输出排序结果

    //选择第Index号区域
    Int8 Myselect_obj(MyConect *ImageConectIn, 		//输入想要筛选的区域
                      MyConect *ImageConectOut, 	//输出筛选的结果
                      Int32 Index);					//输入想筛选出的区域序号

    //获取全区域的最小外接矩形
    Int8 Mysmallest_rectangle(MyConect *ImageConectIn, 		//输入想要求最小外接矩形的区域
                              Int32 *LeftOut,				//输出矩形左边
                              Int32 *RightOut,				//输出矩形右边
                              Int32 *TopOut,				//输出矩形上边
                              Int32 *BottomOut);			//输出矩形下边

    //获取全区域的最小外接矩形(只处理Mat矩阵的有效区域)
    Int8 Mymatsmallest_rectangle(Mat matIn, 				//输入想要求最小外接矩形的Mat矩阵,需为CV_8UC1型二值图
                                 Int32 *LeftOut,			//输出矩形左边
                                 Int32 *RightOut,			//输出矩形右边
                                 Int32 *TopOut,				//输出矩形上边
                                 Int32 *BottomOut);			//输出矩形下边

    //获取每个区域的高度或者宽度的和
    Int8 MyGet_region_info(MyConect *ImageConectIn, 		//输入想要查询信息的区域
                           region_info Mod,					//输入想要获取的类型信息
                           Int32 *dataOut);					//输出该信息的查询结果数据

    //最小二乘法拟合直线(只处理Mat的有效区域)
    Int8 MyMat_sqare_line(Mat matIn,					//输入想要拟合的Mat矩阵,需为CV_8UC1型二值图
                          sqaretype Mod,				//输入拟合模式
                          L_line *lineOut,				//输出拟合的直线端点结果,lineOut->st为直线在图像的上部点,lineOut->ed为直线在图像的下部点
                          houghlineinfo *houghinfoOut); //输出拟合的直线哈夫面信息结果

    //最小二乘法拟合直线(只处理Mat的有效区域的mask非零部分)
    Int8 MyMat_masksqare_line(Mat matIn,					//输入想要拟合的Mat矩阵,需为CV_8UC1型二值图
                              Mat mask,						//输入掩膜,支持CV_8UC1型二值图,且与matIn等尺寸
                              sqaretype Mod,				//输入拟合模式
                              L_line *lineOut,				//输出拟合的直线端点结果,lineOut->st为直线在图像的上部点,lineOut->ed为直线在图像的下部点
                              houghlineinfo *houghinfoOut); //输出拟合的直线哈夫面信息结果

    //最小二乘法拟合直线
    Int8 MyConect_sqare_line(MyConect *ImageConectIn,		//输入想要拟合的区域
                             sqaretype Mod,					//输入拟合模式
                             L_line *lineOut,				//输出拟合的直线端点结果,lineOut->st为直线在图像的上部点,lineOut->ed为直线在图像的下部点
                             houghlineinfo *houghinfoOut);	//输出拟合的直线哈夫面信息结果

    //区域的点平移后再做最小二乘法拟合直线
    Int8 MyConectmove_sqare_line(MyConect *ImageConectIn,		//输入想要拟合的区域
                                 sqaretype Mod,					//输入拟合模式
                                 Int32 movex,					//区域点平移x坐标
                                 Int32 movey,					//区域点平移y坐标
                                 L_line *lineOut,				//输出拟合的直线端点结果,lineOut->st为直线在图像的上部点,lineOut->ed为直线在图像的下部点
                                 houghlineinfo *houghinfoOut);	//输出拟合的直线哈夫面信息结果

    //最小二乘法拟合直线
    Int8 MyData_sqare_line(Int32 *dataXIn,				//输入想要拟合的X数组
                           Int32 *dataYIn,				//输入想要拟合的Y数组
                           Int32 num,					//输入想要拟合的数组个数
                           Int32 nWidth,				//输入想要直线图像的宽度
                           Int32 nHeight,				//输入想要直线图像的高度
                           sqaretype Mod,				//输入拟合模式
                           L_line *lineOut,				//输出拟合的直线端点结果,lineOut->st为直线在图像的上部点,lineOut->ed为直线在图像的下部点
                           houghlineinfo *houghinfoOut);//输出拟合的直线哈夫面信息结果

    //最小二乘法拟合圆
    Int8 MyMat_sqare_circle(Mat matIn,					//输入想要拟合的Mat矩阵,需为CV_8UC1型二值图
                            sqaretype Mod,				//输入拟合模式,目前只支持MHC_VAR_SQARE
                            double *centerXOut,			//输出圆心x坐标
                            double *centerYOut,			//输出圆心y坐标
                            double *radiusOut);			//输出圆心半径

    //最小二乘法拟合圆(只处理Mat的有效区域的mask非零部分)
    Int8 MyMat_masksqare_circle(Mat matIn,					//输入想要拟合的Mat矩阵,需为CV_8UC1型二值图
                                Mat mask,					//输入掩膜,支持CV_8UC1型二值图,且与matIn等尺寸
                                sqaretype Mod,				//输入拟合模式,目前只支持MHC_VAR_SQARE
                                double *centerXOut,			//输出圆心x坐标
                                double *centerYOut,			//输出圆心y坐标
                                double *radiusOut);			//输出圆心半径

    //最小二乘法拟合圆
    Int8 MyConect_sqare_circle(MyConect *ImageConectIn,		//输入想要拟合的区域
                               sqaretype Mod,				//输入拟合模式,目前只支持MHC_VAR_SQARE
                               double *centerXOut,			//输出圆心x坐标
                               double *centerYOut,			//输出圆心y坐标
                               double *radiusOut);			//输出圆心半径

    //最小二乘法拟合圆
    Int8 MyData_sqare_circle(Int32 *dataXIn,			//输入想要拟合的X数组
                             Int32 *dataYIn,			//输入想要拟合的Y数组
                             Int32 num,					//输入想要拟合的数组个数
                             sqaretype Mod,				//输入拟合模式,目前只支持MHC_VAR_SQARE
                             double *centerXOut,		//输出圆心x坐标
                             double *centerYOut,		//输出圆心y坐标
                             double *radiusOut);		//输出圆心半径

    //Mat到Hough变化
    Int8 MyMat_to_Hough(Mat matIn,					//输入想要变化的Mat矩阵,需为CV_8UC1型二值图
                        Mat *HoughOut);				//输出Hough变化结果,Mat类型为CV_16UC1,内存大小至少为getHoughsize()

    //连通域到Hough变化
    Int8 Myregion_to_Hough(MyConect *ImageConectIn,	//输入想要变化的连通区域
                           Mat *HoughOut);			//输出Hough变化结果,Mat类型为CV_16UC1,内存大小至少为getHoughsize()

    //点集到Hough变化
    Int8 MyPoint_to_Hough(Uint16 *pEdgeMapList,		//输入非零点集按x=,y=顺序输入
                          Uint16 nWidth,			//点集的宽度
                          Uint16 nHeight,			//点集的高度
                          Uint16 pEdgeMapListNum,	//点集的个数
                          Mat *HoughOut);			//输出Hough变换结果,Mat类型为CV_16UC1,内存大小至少为getHoughsize()

    //传入Hough图和二值图,结果输出直线(注意HoughIn应该从与MatIn_Out具有相同尺寸的矩阵变化而来,对于找斜率接近0的直线,需要改动MHC_TETARANGE_MID的值)
    Int8 MyHough_to_line(Mat HoughIn,				//输入Hough图,Mat类型为CV_16UC1
                         Mat matIn,					//输入二值图,Mat类型为CV_8UC1,它必须是Hough变化的原图
                         MyCountLine *LineOut,		//输出直线数据
                         Int32 lineTotal,			//想找到的线总条数,不大于MHC_SEARCH_LINENUM条
                         Int32 lineLongmin,			//想找到的线的最小长度
                         Int32 lineGapmax,			//允许一条线断开的最大距离
                         Int32 neighbordictance,	//两条直线之间的起终点都小于这一距离被认为同一条直线
                         Int32 value,				//HoughIn极值最小限制阈值,一般为400以上
                         non_max Mod);				//HoughIn极值筛选模式

    //传入Hough图和二值图,结果输出一条斜率为正数的最长直线,一条斜率为负数的最长直线
    Int8 MyHough_to_2sideline(Mat HoughIn,				//输入Hough图,Mat类型为CV_16UC1
                              Mat matIn,				//输入二值图,Mat类型为CV_8UC1,它必须是Hough变化的原图
                              MyCountLine *LineOut,		//输出直线数据
                              Int32 lineLongmin,		//想找到的线的最小长度
                              Int32 lineGapmax,			//允许一条线断开的最大距离
                              Int32 abssidevalue);		//斜率最小绝对值限制,最小值为0

    //传入Hough图和二值图,结果输出一条斜率为正数的最长直线区域,一条斜率为负数的最长直线区域
    Int8 MyHough_to_2sideline_area(Mat HoughIn,				//输入Hough图,Mat类型为CV_16UC1
                                   Mat matIn,				//输入二值图,Mat类型为CV_8UC1,它必须是Hough变化的原图
                                   MyCountLine *LineOut,	//输出直线数据
                                   Int32 abssidevalue);		//斜率最小绝对值限制,最小值为0

    //传入Hough图和二值图,在极大值抑制下结果输出一条斜率为正数的最长直线,一条斜率为负数的最长直线
    Int8 MyHough_to_2sidenonmaxline(Mat HoughIn,			//输入Hough图,Mat类型为CV_16UC1
                                    Mat matIn,				//输入二值图,Mat类型为CV_8UC1,它必须是Hough变化的原图
                                    MyCountLine *LineOut,	//输出直线数据
                                    Int32 lineLongmin,		//想找到的线的最小长度
                                    Int32 lineGapmax,		//允许一条线断开的最大距离
                                    Int32 value,			//HoughIn极值最小限制阈值,一般为400以上
                                    non_max Mod,			//HoughIn极值筛选模式
                                    Int32 abssidevalue);	//斜率最小绝对值限制,最小值为0

    //传入Hough图,找到Hough面最大值的直线
    Int8 MyHough_to_maxline(Mat HoughIn,				//输入Hough图,Mat类型为CV_16UC1
                            Uint16 nWidth,				//原图图像的宽度
                            Uint16 nHeight,				//原图图像的高度
                            MyCountLine *LineOut);		//输出直线数据

    //传入斜率(霍夫面上的y轴theta),在这个斜率附近搜索一条最长的直线
    Int8 MyHough_to_lineNeartheta(Mat HoughIn,				//输入Hough图,Mat类型为CV_16UC1
                                  Mat matIn,				//输入二值图,Mat类型为CV_8UC1,它必须是Hough变化的原图
                                  MyCountLine *LineOut,		//输出直线数据
                                  Int32 thetaIn,			//输入要搜索的霍夫斜率
                                  Int32 thetaNearby,		//输入要搜索的霍夫斜率范围
                                  Int32 lineLongmin,		//想找到的线的最小长度
                                  Int32 lineGapmax);		//允许一条线断开的最大距离

    //最小二乘法更新霍夫直线
    Int8 MyGetCountLineSqare(MyCountLine *LineIn,		//输入直线数据
                             Mat matIn,					//输入二值图,Mat类型为CV_8UC1,它必须是Hough变化的原图
                             MyCountLine *LineOut,		//输出新直线数据
                             sqaretype Mod,				//输入拟合模式
                             Int32 rhoIn,				//输入扩展搜索范围
                             Int32 lineGapmax,			//允许一条线断开的最大距离
                             Bool *ResultMark);			//输出标记位,下标对应相应的线,TRUE表示该下标位置的线坐标更新成功,FALSE表示该下标位置的线坐标更新失败,内存大小至少为MHC_SEARCH_LINENUM*sizeof(Bool)

    //扩展搜索霍夫直线首尾端点
    Int8 MyGetCountLineBreakpoint(MyCountLine *LineIn,		//输入直线数据
                                  Mat matIn,				//输入二值图,Mat类型为CV_8UC1,它必须是Hough变化的原图
                                  MyCountLine *LineOut,		//输出新直线数据
                                  Int32 rhoIn,				//输入扩展搜索范围
                                  Int32 lineGapmax,			//端点附近的线的允许断开的最大长度
                                  Int32 lineminlong,		//端点附近的线的最小长度
                                  Bool *ResultMark);		//输出标记位,下标对应相应的线,TRUE表示该下标位置的线坐标更新成功,FALSE表示该下标位置的线坐标更新失败,内存大小至少为MHC_SEARCH_LINENUM*sizeof(Bool)

    //排序直线
    Int8 Mysort_line(MyCountLine *LineIn,	//输入想要排序的直线
                     MyCountLine *LineOut,	//输出排序结果
                     sortline Mod);			//排序模式,详见sortline

    //选择第Index号直线
    Int8 Myselect_line(MyCountLine *LineIn, 	//输入想要筛选的直线
                       MyCountLine *LineOut, 	//输出筛选的结果
                       Int32 Index);			//输入想筛选出的直线序号

    //图象X轴与Y轴对调(如同顺时针旋转90度),会作用于全矩阵区域,图像的宽和高需为4的倍数
    Int8 MyX_to_Y_trans(Mat matIn,		//输入待处理矩阵
                        Mat *matOut);	//输出处理结果,类型与MatIn相同

    //图像翻转(会作用于全部区域,需要小心使用)
    Int8 Myflip(Mat matIn,		//输入待处理矩阵仅支持输入CV_8UC1,CV_8UC3
                Mat *matOut,	//输出处理结果,类型与MatIn相同
                fliptype Mod);  //翻转模式

    //区域旋转(支持-90度到90度旋转)(会作用于全部区域,需要小心使用)
    Int8 Myaffine_trans_region_sincos(MyConect *ImageConectIn,	//输入旋转区域
                                      MyConect *ImageConectOut, //输出旋转结果
                                      Int32 Range,				//旋转细分度从0到MHC_ROATE,在Range=MHC_ROATE/2时为旋转0度,Range=0时为逆时针旋转90度,Range=MHC_ROATE时为顺时针旋转90度
                                      Int32 Ox,					//旋转中心X坐标
                                      Int32 Oy,					//旋转中心Y坐标
                                      transtype Mod);			//Mod=MHC_TRANS_INIMAGE时,内存数据只保留图像内的点,否则保留包括图像外的全部点

    //图像平移(会作用于全部区域,需要小心使用)
    Int8 Myhom_mat2d_translate(Mat matIn,				//输入Mat矩阵,仅支持CV_8UC1,CV_8UC3
                               Mat *matOut,				//输出Mat矩阵,类型与matIn相同,需要注意是否有足够空间
                               Int32 movex,				//数据平移x位移(右移为正数)
                               Int32 movey);			//数据平移y位移(下移为正数)

    //区域平移(会作用于全部区域,需要小心使用)
    Int8 Myhom_conect2d_translate(MyConect *ImageConectIn,		//输入平移区域
                                  MyConect *ImageConectOut,		//输出平移结果
                                  Int32 movex,					//区域平移x位移
                                  Int32 movey,					//区域平移y位移
                                  transtype Mod);				//Mod=MHC_TRANS_INIMAGE时,内存数据只保留图像内的点,否则保留包括图像外的全部点

    //非极大值抑制(会作用于全部区域,需要小心使用)
    Int8 MyNon_Maximal(Mat matIn,		//输入Mat矩阵仅支持输入CV_8UC1,CV_16UC1型
                       Mat *matOut,		//输出极大值,类型为CV_8UC1,极大值为255,非极大值为0
                       Int16 value,		//极大值要大于阈值value
                       non_max Mod);	//极大值周围收索范围

    //保存Mat矩阵到PC机,目前只支持BMP格式
    Int8 Myimrite(const char *str,			//保存文件路径和文件名
                  Mat matIn);				//输入想要保存的Mat矩阵,仅支持CV_8UC1,CV_8UC3

    //从PC机读取Mat矩阵,目前只支持BMP格式
    Int8 Myimread(const char *str,			//读取文件路径和文件名,目前只支持BMP格式
                  Mat *matIn_Out);			//输入/输出读取到的Mat矩阵,注意内存大小,大小至少为nHeight*nWidth*getsizeof(Mat._type)

    //保存Mat矩阵数据到txt,按matIn的成员宽度换行
    Int8 MyMatSaveTxt(const char *str,		//保存文件名
                      Mat matIn);			//要保存的数据


    //读取txt矩阵数据到matOut,按matIn的成员宽度换行
    Int8 MyMatReadTxt(const char *str,		//读取的文件名
                      Mat *matIn_Out);		//输入/输出读取到的Mat矩阵,注意内存大小,类型由txt矩阵信息决定,需要注意是否有足够空间

    //L_Point类型转换为L_Point32类型
    Int8 MyPoint16to32(L_Point pointIn,		//输入L_Point类型的点
                       L_Point32 *pointOut);//输出L_Point32类型的点


    //L_Point32类型转换为L_Point类型
    Int8 MyPoint32to16(L_Point32 pointIn,	//输入L_Point32类型的点
                       L_Point *pointOut);	//输出L_Point类型的点


    //L_line类型转换为L_line32类型
    Int8 Myline16to32(L_line lineIn,		//输入L_Point类型的点
                      L_line32 *lineOut);	//输出L_Point32类型的点


    //L_line32类型转换为L_line类型
    Int8 Myline32to16(L_line32 lineIn,		//输入L_Point32类型的点
                      L_line *lineOut);		//输出L_Point类型的点

    //两点法画直线
    Int8 MyLine(Mat *matIn_Out,				//输入/输出矩阵,支持CV_8UC1,CV_8UC3类型
                L_Point32 st,				//直线起点
                L_Point32 ed,				//直线终点
                Int32 color,				//直线颜色,CV_8UC3类型时3通道值均为color
                linetype connectivity,		//直线连通模式
                Int32 thickness);			//直线粗细

    //两点法画直线
    Int8 MyLine3col(Mat *matIn_Out,				//输入/输出矩阵,支持CV_8UC1,CV_8UC3类型
                    L_Point32 st,				//直线起点
                    L_Point32 ed,				//直线终点
                    Int32 color1,				//通道1直线颜色,CV_8UC1类型时为color1,color2,color3色取平均
                    Int32 color2,				//通道2直线颜色
                    Int32 color3,				//通道3直线颜色
                    linetype connectivity,		//直线连通模式
                    Int32 thickness);			//直线粗细

    //两点法画虚线
    Int8 MyDottedLine(Mat *matIn_Out,           //输入/输出矩阵,支持CV_8UC1,CV_8UC3类型
                      L_Point32 st,				//虚线起点
                      L_Point32 ed,				//虚线终点
                      Int32 color,				//虚线颜色,CV_8UC3类型时3通道值均为color
                      Int32 dis,                //虚间隔距离
                      dottedtype Mod,           //虚线模式
                      linetype connectivity,    //虚线连通模式
                      Int32 thickness);			//虚线粗细

    //两点法画虚线
    Int8 MyDottedLine3col(Mat *matIn_Out,           //输入/输出矩阵,支持CV_8UC1,CV_8UC3类型
                          L_Point32 st,				//虚线起点
                          L_Point32 ed,				//虚线终点
                          Int32 color1,				//通道1虚线颜色,CV_8UC1类型时为color1,color2,color3色取平均
                          Int32 color2,				//通道2虚线颜色
                          Int32 color3,				//通道3虚线颜色
                          Int32 dis,                //虚间隔距离
                          dottedtype Mod,           //虚线模式
                          linetype connectivity,    //虚线连通模式
                          Int32 thickness);			//虚线粗细


    //圆心半径法画圆(如果要画粗线圆,请使用椭圆函数)
    Int8 MyCircle(Mat *matIn_Out,			//输入/输出矩阵,支持CV_8UC1,CV_8UC3类型
                  L_Point32 center,			//圆心
                  Int32 radius,				//半径
                  Int32 color,				//圆颜色,CV_8UC3类型时3通道值均为color
                  circletype connectivity);	//圆模式

    //圆心半径法画圆(如果要画粗线圆,请使用椭圆函数)
    Int8 MyCircle3col(Mat *matIn_Out,			//输入/输出矩阵,支持CV_8UC1,CV_8UC3类型
                      L_Point32 center,			//圆心
                      Int32 radius,				//半径
                      Int32 color1,				//通道1圆颜色,CV_8UC1类型时为color1,color2,color3色取平均
                      Int32 color2,				//通道2圆颜色,CV_8UC1类型时为color1,color2,color3色取平均
                      Int32 color3,				//通道3圆颜色,CV_8UC1类型时为color1,color2,color3色取平均
                      circletype connectivity);	//圆模式

    //圆心长短轴旋转角画法画椭圆
    Int8 MyEllipse(Mat *matIn_Out,				//输入/输出矩阵,支持CV_8UC1,CV_8UC3类型
                   RotatedRect box,				//输入椭圆信息
                   Int32 color,					//椭圆颜色,CV_8UC3类型时3通道值均为color
                   linetype connectivity,		//椭圆连通模式
                   Int32 thickness);			//椭圆粗细


    //圆心长短轴旋转角法画椭圆
    Int8 MyEllipse3col(Mat *matIn_Out,				//输入/输出矩阵,支持CV_8UC1,CV_8UC3类型
                       RotatedRect box,				//输入椭圆信息
                       Int32 color1,				//通道1圆颜色,CV_8UC1类型时为color1,color2,color3色取平均
                       Int32 color2,				//通道2圆颜色,CV_8UC1类型时为color1,color2,color3色取平均
                       Int32 color3,				//通道3圆颜色,CV_8UC1类型时为color1,color2,color3色取平均
                       linetype connectivity,		//椭圆连通模式
                       Int32 thickness);			//椭圆粗细

    //四点法画矩形
    Int8 Myrectangle(Mat *matIn_Out,				//输入/输出矩阵,支持CV_8UC1,CV_8UC3类型
                     Int32 left,					//矩形左边
                     Int32 right,					//矩形右边
                     Int32 top,						//矩形上边
                     Int32 deep,					//矩形下边
                     Int32 color,					//矩形颜色,CV_8UC3类型时3通道值均为color
                     Int32 thickness);				//矩形边长直线粗细,thickness为负数时为实心矩形

    //四点法画矩形
    Int8 Myrectangle3col(Mat *matIn_Out,				//输入/输出矩阵,支持CV_8UC1,CV_8UC3类型
                         Int32 left,					//矩形左边
                         Int32 right,					//矩形右边
                         Int32 top,						//矩形上边
                         Int32 deep,					//矩形下边
                         Int32 color1,					//通道1矩形颜色,CV_8UC1类型时为color1,color2,color3色取平均
                         Int32 color2,					//通道2矩形颜色,CV_8UC1类型时为color1,color2,color3色取平均
                         Int32 color3,					//通道3矩形颜色,CV_8UC1类型时为color1,color2,color3色取平均
                         Int32 thickness);				//矩形边长直线粗细,thickness为负数时为实心矩形

    //两点法画直线,生成直线点集区域,当thickness=1时,点序会从st点往ed点依次延伸
    Int8 MyLinetoPoint(Uint32 nHeight,			//输入直线的矩阵高度
                       Uint32 nWidth,			//输入直线的矩阵宽度
                       L_Point32 st,			//输入直线起点
                       L_Point32 ed,			//输入直线终点
                       linetype connectivity,	//输入直线联通类型
                       Int32 thickness,			//输入直线粗细
                       MyConect *ImageConectOut,//生成直线连通域
                       void *bufferIn);			//bufferIn为ImageConectOut内存地址,内存大小至少为nHeight*nWidth*getConectsize()


    //圆心半径法画圆,生成圆点集区域
    Int8 MyCircletoPoint(Uint32 nHeight,			//输入圆的矩阵高度
                         Uint32 nWidth,				//输入圆的矩阵宽度
                         L_Point32 center,			//输入圆的圆心
                         Int32 radius,				//输入圆的半径
                         circletype connectivity,	//输入圆的模式
                         MyConect *ImageConectOut,	//生成圆的连通域
                         void *bufferIn);			//bufferIn为ImageConectOut内存地址,内存大小至少为nHeight*nWidth*getConectsize()

    //圆心长短轴旋转角法画椭圆,生成椭圆点集区域
    Int8 MyEllipsetoPoint(Uint32 nHeight,			//输入椭圆的矩阵高度
                          Uint32 nWidth,			//输入椭圆的矩阵宽度
                          RotatedRect box,			//输入椭圆信息
                          linetype connectivity,	//输入椭圆连通模式
                          Int32 thickness,			//输入椭圆粗细
                          MyConect *ImageConectOut,	//生成椭圆的连通域
                          void *bufferIn);			//bufferIn为ImageConectOut内存地址,内存大小至少为nHeight*nWidth*getConectsize()

    //四点法画矩形,生成矩形点集区域
    Int8 MyrectangletoPoint(Uint32 nHeight,				//输入矩形的矩阵高度
                            Uint32 nWidth,				//输入矩形的矩阵宽度
                            Int32 left,					//矩形左边
                            Int32 right,				//矩形右边
                            Int32 top,					//矩形上边
                            Int32 deep,					//矩形下边
                            Int32 thickness,			//矩形边长直线粗细,thickness为负数时为实心矩形
                            MyConect *ImageConectOut,	//生成矩形的连通域
                            void *bufferIn);			//bufferIn为ImageConectOut内存地址,内存大小至少为nHeight*nWidth*getConectsize()

    //求两直线交点
    Int8 MyGetLinefocal(L_line lineIn1,			//输入直线1
                        L_line lineIn2,			//输入直线2
                        L_Point32 *focal);		//求得两直线交点

    //求两直线交点
    Int8 MyGetLinefocal32(L_line32 lineIn1,			//输入直线1
                          L_line32 lineIn2,			//输入直线2
                          L_Point32 *focal);		//求得两直线交点

    //求直线与圆交点
    Int8 MyGetLineCirclefocal(L_line lineIn,		//输入直线
                              double centerXIn,		//输入圆心X
                              double centerYIn,		//输入圆心Y
                              double radiusIn,		//输入圆心半径
                              L_Point32 *focal1,	//求得圆与直线交点1,位于上方或同行靠左的点
                              L_Point32 *focal2);	//求得圆与直线交点2,位于下方或同行靠右的点

    //求圆与圆交点
    Int8 MyGetCirclefocal(double centerXIn1,		//输入圆心1X
                          double centerYIn1,		//输入圆心1Y
                          double radiusIn1,			//输入圆心1半径
                          double centerXIn2,		//输入圆心2X
                          double centerYIn2,		//输入圆心2Y
                          double radiusIn2,			//输入圆心2半径
                          L_Point32 *focal1,		//求得两圆交点1,位于左侧或同列靠上的点
                          L_Point32 *focal2);		//求得两圆交点2,位于右侧或同列靠下的点

    //求直线上Y轴所对应的X轴坐标,如果斜率为0时,X轴返回0,且函数值返回非0
    Int8 MyGetLineXpos(L_line lineIn,			//输入直线
                       Int32 YIn,				//输入Y轴
                       Int32 *XOut);			//输出X轴坐标


    //求直线上X轴所对应的Y轴坐标,如果斜率为不存在时,Y轴返回0,且函数值返回非0
    Int8 MyGetLineYpos(L_line lineIn,			//输入直线
                       Int32 XIn,				//输入X轴
                       Int32 *YOut);			//输出Y轴坐标


    //求直线上Y轴所对应的X轴坐标,如果斜率为0时,X轴返回0,且函数值返回非0
    Int8 MyGetLine32Xpos(L_line32 lineIn,		//输入直线
                         Int32 YIn,				//输入Y轴
                         Int32 *XOut);			//输出X轴坐标


    //求直线上X轴所对应的Y轴坐标,如果斜率为不存在时,Y轴返回0,且函数值返回非0
    Int8 MyGetLine32Ypos(L_line32 lineIn,		//输入直线
                         Int32 XIn,				//输入X轴
                         Int32 *YOut);			//输出Y轴坐标

    //求直线上Y轴所对应的X轴坐标,如果斜率为0时,X轴返回0,且函数值返回非0
    Int8 MyGetLine32XD64pos(L_line32 lineIn,		//输入直线
                            Int32 YIn,				//输入Y轴
                            double *XOut);			//输出X轴坐标


    //求直线上X轴所对应的Y轴坐标,如果斜率为不存在时,Y轴返回0,且函数值返回非0
    Int8 MyGetLine32YD64pos(L_line32 lineIn,		//输入直线
                            Int32 XIn,				//输入X轴
                            double *YOut);			//输出Y轴坐标

    //在mask为0的部分把数组做线性修复
    Int8 Myfixdata(Int32 *dataIn_Out,		//输入_输出数组
                   Uint8 *mask,				//输入掩摸,修复mask为0的部分
                   Int32 num);				//输入数组长度

    //在mask为0的部分把数组做线性修复
    Int8 Myfixfdata(float *dataIn_Out,		//输入_输出数组
                    Uint8 *mask,			//输入掩摸,修复mask为0的部分
                    Int32 num);				//输入数组长度

    //在mask为0的部分把数组做线性修复并放大4倍
    Int8 Myresizefixdata_4fSize(Int32 *data,        //输入整形数组
                                Uint8 *mask,		//输入掩摸,修复mask为0的部分
                                float *dataOut,     //输出浮点型数组,注意内存大小,至少为sizeof(float)*num*4
                                Int num);           //输入数组长度

    //在mask为0的部分把右移两位数组做线性修复并放大4倍
    Int8 Myresizefix2bitdata_4fSize(Int32 *data2bit_In,  //输入整形数组(已经做右移动2位计算)
                                    Uint8 *mask,		//输入掩摸,修复mask为0的部分
                                    float *dataOut,     //输出浮点型数组,注意内存大小,至少为sizeof(float)*num*4
                                    Int num);           //输入数组长度

    //把整型的数组插值为浮点型数组
    Int8 MychangeI32toF32data(Int32 *dataIn,		//输入整形数组
                              float *dataOut,		//输出浮点型数组,注意内存大小,至少为sizeof(float)*num
                              Int32 num);			//输入数组长度

    /*******************以下函数为创建缓存Ver类,返回值为Ver*************************/
    //创建Verb缓存,用来存储0和非0数据,默认初始值为非0
    Verb VerbCreat(Int32 num,			//输入Verb缓存数据个数
                   void *bufferIn);		//bufferIn为Verb缓存地址,内存大小至少为getVerbsizeof(num)


    //创建Ver1i缓存,用来存储一维数据
    Ver1i Ver1iCreat(Int32 num,			//输入Ver1i缓存数据个数
                     void *bufferIn);	//bufferIn为Ver1i缓存地址,内存大小至少为getVer1isizeof(num)


    //创建Ver2i缓存,用来存储二维数据
    Ver2i Ver2iCreat(Int32 num,			//输入Ver2i缓存数据个数
                     void *bufferIn);	//bufferIn为Ver2i缓存地址,内存大小至少为getVer2isizeof(num)


    //创建Ver3i缓存,用来存储三维数据
    Ver3i Ver3iCreat(Int32 num,			//输入Ver3i缓存数据个数
                     void *bufferIn);	//bufferIn为Ver3i缓存地址,内存大小至少为getVer3isizeof(num)


    //创建Ver2v捍,用来存储二维向量
    Ver2v Ver2vCreat(Int32 num,			//输入Ver2v缓存数据个数
                     void *bufferIn);	//bufferIn为Ver2v缓存地址,内存大小至少为getVer2vsizeof(num)


    //创建Ver3v缓存,用来存储三维向量
    Ver3v Ver3vCreat(Int32 num,			//输入Ver3v缓存数据个数
                     void *bufferIn);	//bufferIn为Ver3v缓存地址,内存大小至少为getVer3vsizeof(num)


    /*******************以下函数为调度缓存Ver类,成功返回0，否则返回非0*************************/
    //往Verb缓存中推入最新的值,缓存满后挤掉最早的值
    Int8 VerbPush(Verb *verIn_Out,	//输入/输出的Verb缓存
                  Bool value);		//要推进Verb缓存去的最新值,非0为value=TRUE,0为value=FALSE


    //输出当前Verb缓存中为非0的值的总数
    Int8 VerbGetTRUEnum(Verb verIn,		//输入Verb缓存
                        Int32 *valout);	//输出非0的值的总数


    //输出当前Verb缓存中为0的值的总数
    Int8 VerbGetFALSEnum(Verb verIn,	//输入Verb缓存
                        Int32 *valout);	//输出非0的值的总数


    //清除Verb缓存中的数据
    Int8 VerbClear(Verb *verIn_Out);	//输入/输出Verb缓存


    //往Ver1i缓存中推入最新的值,缓存满后挤掉最早的值
    Int8 Ver1iPush(Ver1i *verIn_Out,	//输入/输出的Ver1i缓存
                   Int32 point);		//要推进Ver1i缓存去的最新值


    //清除Ver1i缓存中的数据
    Int8 Ver1iClear(Ver1i *verIn_Out);	//输入/输出Ver1i缓存


    //往Ver2i缓存中推入最新的值,缓存满后挤掉最早的值
    Int8 Ver2iPush(Ver2i *verIn_Out,	//输入/输出的Ver2i缓存
                   L_Point32 point);	//要推进Ver2i缓存去的最新值


    //清除Ver2i缓存中的数据
    Int8 Ver2iClear(Ver2i *verIn_Out);	//输入/输出Ver2i缓存


    //往Ver3i缓存中推入最新的值,缓存满后挤掉最早的值
    Int8 Ver3iPush(Ver3i *verIn_Out,	//输入/输出的Ver3i缓存
                   Vec3i point);		//要推进Ver3i缓存去的最新值


    //清除Ver3i缓存中的数据
    Int8 Ver3iClear(Ver3i *verIn_Out);	//输入/输出Ver3i缓存


    //往Ver2v缓存中推入最新的值,缓存满后挤掉最早的值
    Int8 Ver2vPush(Ver2v *verIn_Out,	//输入/输出的Ver2v缓存
                   Vec2v point);		//要推进Ver2v缓存去的最新值


    //清除Ver2v缓存中的数据
    Int8 Ver2vClear(Ver2v *verIn_Out);	//输入/输出Ver2v缓存


    //往Ver3v缓存中推入最新的值,缓存满后挤掉最早的值
    Int8 Ver3vPush(Ver3v *verIn_Out,	//输入/输出的Ver3v缓存
                   Vec3v point);		//要推进Ver3v缓存去的最新值


    //清除Ver3v缓存中的数据
    Int8 Ver3vClear(Ver3v *verIn_Out);	//输入/输出Ver3v缓存

    /***************以下函数为创建波形Peak_Trough类,返回值为Peak_Trough****************/

    //创建Peak_Trough空的波形结构体
    Peak_Trough MyCreat_peaktrough(Int32 num,			//波形的数据个数,且num<MHC_PEAK_NUM;
                                   void *bufferIn1,		//波形的数据缓存1,大小至少为sizeof(Int32)*num
                                   void *bufferIn2);	//波形的数据缓存2,大小至少为sizeof(Int8)*num


    //创建Peak_Trough空的波形结构体,并初始化信息为0
    Peak_Trough MyCreat_zeropeaktrough(Int32 num,			//波形的数据个数,且num<MHC_PEAK_NUM;
                                       void *bufferIn1,		//波形的数据缓存1,大小至少为sizeof(Int32)*num
                                       void *bufferIn2);	//波形的数据缓存2,大小至少为sizeof(Int8)*num


    //创建Peak_Trough波形结构体,从数组value中产生
    Peak_Trough Find_Peak_Trough(Int32 *value,		//输入要搜索的波形数组
                                 Int32 valuenum,	//输入数组个数,且valuenum<MHC_PEAK_NUM;
                                 Bool circleMod,	//circleMod=TRUE表示为头尾相接的数组(value[0]与value[valuenum-1]是相邻的),circleMod=FALSE,表示不相邻
                                 void *bufferIn1,	//波形的数据缓存1,大小至少为sizeof(Int32)*valuenum
                                 void *bufferIn2);	//波形的数据缓存2,大小至少为sizeof(Int8)*valuenum


    //创建Peak_Trough32FC空的波形结构体
    Peak_Trough32FC MyCreat_peaktrough32FC(Int32 num,			//波形的数据个数,且num<MHC_PEAK_NUM;
                                           void *bufferIn1,		//波形的数据缓存1,大小至少为sizeof(float)*num
                                           void *bufferIn2);	//波形的数据缓存2,大小至少为sizeof(Int8)*num


    //创建Peak_Trough32FC空的波形结构体,并初始化信息为0
    Peak_Trough32FC MyCreat_zeropeaktrough32FC(Int32 num,			//波形的数据个数,且num<MHC_PEAK_NUM;
                                               void *bufferIn1,		//波形的数据缓存1,大小至少为sizeof(float)*num
                                               void *bufferIn2);	//波形的数据缓存2,大小至少为sizeof(Int8)*num


    //创建Peak_Trough32FC波形结构体,从数组value中产生
    Peak_Trough32FC Find_Peak_Trough32FC(float *value,		//输入要搜索的波形数组
                                         Int32 valuenum,	//输入数组个数,且valuenum<MHC_PEAK_NUM;
                                         Bool circleMod,	//circleMod=TRUE表示为头尾相接的数组(value[0]与value[valuenum-1]是相邻的),circleMod=FALSE,表示不相邻
                                         void *bufferIn1,	//波形的数据缓存1,大小至少为sizeof(float)*valuenum
                                         void *bufferIn2);	//波形的数据缓存2,大小至少为sizeof(Int8)*valuenum


    /***************以下函数为操作波形Peak_Trough类,成功返回0，否则返回非0****************/
    //拷贝一个波峰波谷
    Int8 MyCopy_peaktrough(Peak_Trough *peaktroughIn,	//输入要拷贝的波形数组
                           Peak_Trough *peaktroughOut);	//输出结果,注意需要有足够内存


    //拷贝一个波峰波谷
    Int8 MyCopy_peaktrough32FC(Peak_Trough32FC *peaktroughIn,	//输入要拷贝的波形数组
                               Peak_Trough32FC *peaktroughOut);	//输出结果,注意需要有足够内存


    //求上升沿与下降沿交集
    Int8 Myintersection_peaktroughside(Peak_Trough *peaktroughIn1,	//输入波形1
                                       Peak_Trough *peaktroughIn2,	//输入波形2
                                       Peak_Trough *peaktroughOut);	//输出结果,注意需要有足够内存


    //求上升沿与下降沿交集
    Int8 Myintersection_peaktroughside32FC(Peak_Trough32FC *peaktroughIn1,	//输入波形1
                                           Peak_Trough32FC *peaktroughIn2,	//输入波形2
                                           Peak_Trough32FC *peaktroughOut);	//输出结果,注意需要有足够内存


    //求波谷波峰并集
    Int8 Myunion2_peaktrough(Peak_Trough *peaktroughIn1,	//输入波形1
                             Peak_Trough *peaktroughIn2,	//输入波形2
                             Peak_Trough *peaktroughOut);	//输出结果,注意需要有足够内存


    //求波谷波峰并集
    Int8 Myunion2_peaktrough32FC(Peak_Trough32FC *peaktroughIn1,	//输入波形1
                                 Peak_Trough32FC *peaktroughIn2,	//输入波形2
                                 Peak_Trough32FC *peaktroughOut);	//输出结果,注意需要有足够内存


    //求波谷波峰交集
    Int8 Myintersection_peaktrough(Peak_Trough *peaktroughIn1,	//输入波形1
                                   Peak_Trough *peaktroughIn2,	//输入波形2
                                   Peak_Trough *peaktroughOut);	//输出结果,注意需要有足够内存


    //求波谷波峰交集
    Int8 Myintersection_peaktrough32FC(Peak_Trough32FC *peaktroughIn1,	//输入波形1
                                       Peak_Trough32FC *peaktroughIn2,	//输入波形2
                                       Peak_Trough32FC *peaktroughOut);	//输出结果,注意需要有足够内存


    //求波谷波峰膨胀,Mod为膨胀波谷还是波峰,
    Int8 Mydilation_peaktrough(Peak_Trough *peaktroughIn,		//输入波形
                               Peak_Trough *peaktroughOut,		//输出结果,注意需要有足够内存
                               peakdilationtype Mod,			//输入膨胀模式,膨胀波谷还是波峰
                               Int32 peaktrough_val,			//输入左右膨胀值
                               Bool circleMod);					//circleMod=TRUE表示为头尾相接的数组(value[0]与value[valuenum-1]是相邻),circleMod=FALSE,表示不相邻


    //求波谷波峰膨胀,Mod为膨胀波谷还是波峰,
    Int8 Mydilation_peaktrough32FC(Peak_Trough32FC *peaktroughIn,	//输入波形
                                   Peak_Trough32FC *peaktroughOut,	//输出结果,注意需要有足够内存
                                   peakdilationtype Mod,			//输入膨胀模式,膨胀波谷还是波峰
                                   Int32 peaktrough_val,			//输入左右膨胀值
                                   Bool circleMod);					//circleMod=TRUE表示为头尾相接的数组(value[0]与value[valuenum-1]是相邻),circleMod=FALSE,表示不相邻


    //提取出单个波形,peaktroughnum为输入波形下标，
    Int8 Myselect_peaktrough_obj(Peak_Trough *peaktroughIn,		//输入波形
                                 Peak_Trough *peaktroughOut,	//输出结果,注意需要有足够内存
                                 Int32 peaktroughnum,			//输入筛选的波形下标,peaktroughIn.valpeaktrough[peaktroughnum]值必须为1或-1,
                                                                //peaktroughnum为1时选出一个peaktroughIn.value[peaktroughnum]为峰的波形,
                                                                //peaktroughnum为-1时选出一个peaktroughIn.value[peaktroughnum]为谷的波形
                                 Bool circleMod);				//circleMod=TRUE表示为头尾相接的数组(value[0]与value[valuenum-1]是相邻的),circleMod=FALSE,表示不相邻


    //提取出单个波形,peaktroughnum为输入波形下标，
    Int8 Myselect_peaktrough_obj32FC(Peak_Trough32FC *peaktroughIn,		//输入波形
                                     Peak_Trough32FC *peaktroughOut,	//输出结果,注意需要有足够内存
                                     Int32 peaktroughnum,				//输入筛选的波形下标,peaktroughIn.valpeaktrough[peaktroughnum]值必须为1或-1,
                                                                        //peaktroughnum为1时选出一个peaktroughIn.value[peaktroughnum]为峰的波形,
                                                                        //peaktroughnum为-1时选出一个peaktroughIn.value[peaktroughnum]为谷的波形
                                     Bool circleMod);					//circleMod=TRUE表示为头尾相接的数组(value[0]与value[valuenum-1]是相邻的),circleMod=FALSE,表示不相邻


    //筛选峰峰值1
    Int8 Myselect_peaktrough_1(Peak_Trough *peaktroughIn,		//输入波形
                               Peak_Trough *peaktroughOut,		//输出结果,注意需要有足够内存
                               peakselecttype_1 Mod,			//输入筛选模式
                               Int32 peaktrough_nearside,		//判定范围
                               double peaktrough_val,			//判定阈值
                               Bool circleMod);					//circleMod=TRUE表示为头尾相接的数组(value[0]与value[valuenum-1]是相邻的),circleMod=FALSE,表示不相邻


    //筛选峰峰值1
    Int8 Myselect_peaktrough_132FC(Peak_Trough32FC *peaktroughIn,		//输入波形
                                   Peak_Trough32FC *peaktroughOut,		//输出结果,注意需要有足够内存
                                   peakselecttype_1 Mod,				//输入筛选模式
                                   Int32 peaktrough_nearside,			//判定范围
                                   double peaktrough_val,				//判定阈值
                                   Bool circleMod);						//circleMod=TRUE表示为头尾相接的数组(value[0]与value[valuenum-1]是相邻的),circleMod=FALSE,表示不相邻


    //筛选峰峰值2
    Int8 Myselect_peaktrough_2(Peak_Trough *peaktroughIn,		//输入波形
                               Peak_Trough *peaktroughOut,		//输出结果,注意需要有足够内存
                               peakselecttype_2 Mod,			//输入筛选模式
                               Int32 peaktrough_val,			//判定阈值
                               Bool circleMod);					//circleMod=TRUE表示为头尾相接的数组(value[0]与value[valuenum-1]是相邻的),circleMod=FALSE,表示不相邻


    //筛选峰峰值2
    Int8 Myselect_peaktrough_232FC(Peak_Trough32FC *peaktroughIn,	//输入波形
                                   Peak_Trough32FC *peaktroughOut,	//输出结果,注意需要有足够内存
                                   peakselecttype_2 Mod,			//输入筛选模式
                                   float peaktrough_val,			//判定阈值
                                   Bool circleMod);					//circleMod=TRUE表示为头尾相接的数组(value[0]与value[valuenum-1]是相邻的),circleMod=FALSE,表示不相邻


    //波峰或波谷滤波
    Int8 Mylvbo1size_peaktrough(Peak_Trough *peaktroughIn,		//输入波形
                                Peak_Trough *peaktroughOut,		//输出结果,注意需要有足够内存
                                peaklvbotype Mod,				//Mod=滤波谷：一个波峰数据再它的相邻的左右两侧数据都是波谷时，这个波峰判断为噪声,数值变为相邻波谷数值小的数
                                                                //Mod=滤波峰：一个波谷数据再它的相邻的左右两侧数据都是波峰时，这个波谷判断为噪声,数值变为相邻波峰数值大的数
                                Bool circleMod);				//circleMod=TRUE表示为头尾相接的数组(value[0]与value[valuenum-1]是相邻的),circleMod=FALSE,表示不相邻


    //波峰或波谷滤波
    Int8 Mylvbo1size_peaktrough32FC(Peak_Trough32FC *peaktroughIn,		//输入波形
                                    Peak_Trough32FC *peaktroughOut,		//输出结果,注意需要有足够内存
                                    peaklvbotype Mod,					//Mod=滤波谷：一个波峰数据再它的相邻的左右两侧数据都是波谷时，这个波峰判断为噪声,数值变为相邻波谷数值小的数
                                                                        //Mod=滤波峰：一个波谷数据再它的相邻的左右两侧数据都是波峰时，这个波谷判断为噪声,数值变为相邻波峰数值大的数
                                    Bool circleMod);					//circleMod=TRUE表示为头尾相接的数组(value[0]与value[valuenum-1]是相邻的),circleMod=FALSE,表示不相邻


    //漫水填充波形
    Int8 Mywatershed_peaktrough(Peak_Trough *peaktroughIn,		//输入波形
                                watershed *datawaterIn_Out,		//输出结果,对应波形的下标位置存放着水的体积和水深,内存大小至少为sizeof(watershed)*peaktroughIn.valuenum
                                Int32 waterheight,				//填充高度
                                Int32 waterwidth,				//填充宽度限制
                                peakwatertype Mod,				//填充模式(填波谷/填波峰)
                                Bool circleMod);				//circleMod=TRUE表示为头尾相接的数组(value[0]与value[valuenum-1]是相邻的),circleMod=FALSE,表示不相邻


    //漫水填充波形
    Int8 Mywatershed_peaktrough32FC(Peak_Trough32FC *peaktroughIn,	//输入波形
                                    watershed32FC *datawaterIn_Out,	//输出结果,对应波形的下标位置存放着水的体积和水深,内存大小至少为sizeof(watershed32FC)*peaktroughIn.valuenum
                                    float waterheight,				//填充高度
                                    Int32 waterwidth,				//填充宽度限制
                                    peakwatertype Mod,				//填充模式(填波谷/填波峰)
                                    Bool circleMod);				//circleMod=TRUE表示为头尾相接的数组(value[0]与value[valuenum-1]是相邻的),circleMod=FALSE,表示不相邻



    /**************以下是其他返回值的函数********************************/
    //生成随机数
    Int32 My_rand(void);


    //更新随机数种子
    void My_srand(Uint32 seed);

    //获取Mat类型大小,返回值为类型大小
    Int32 getsizeof(type Mod);		//输入Mat类型

    //获取MyConect类型的内存大小
    Int32 getConectsize();

    //获取Hough类型的内存大小
    Int32 getHoughsize();

    //获取Verb类型的内存大小
    Int32 getVerbsizeof(Int32 Vernum);


    //获取Ver1i类型的内存大小
    Int32 getVer1isizeof(Int32 Vernum);


    //获取Ver2i类型的内存大小
    Int32 getVer2isizeof(Int32 Vernum);


    //获取Ver3i类型的内存大小
    Int32 getVer3isizeof(Int32 Vernum);


    //获取Ver2v类型的内存大小
    Int32 getVer2vsizeof(Int32 Vernum);


    //获取Ver3v类型的内存大小
    Int32 getVer3vsizeof(Int32 Vernum);

    //快速统计2进制数据中有几个1,返回1的个数
    Int32 get_1_num_in_bin(Uint32 n);

    //把matIn的图像类型、大小尺寸、有效区域信息拷贝到MatOut中,而不拷贝matIn的图像内存数据
    void getcloneinfo(Mat matIn,	//输入想要拷贝的Mat矩阵
                      Mat *matOut);	//输出想要拷贝过去的Mat矩阵,需要注意是否有足够空间

    //判断掩膜Mask尺寸，是否符合matIn图片，返回0为符合，非0为不符合
    Int8 judgeMaskSize(Mat matIn,	//输入要对比的Mat矩阵
                       Mat mask);	//输入要对比的mask矩阵


    //判断掩膜Mask尺寸，是否符合ImageConectIn连通区域,返回0为符合，非0为不符合
    Int8 judgeCMaskSize(MyConect *ImageConectIn,	//输入要对比的连通区域
                        Mat mask);					//输入要对比的mask矩阵


    //判断图像image尺寸，是否符合ImageConectIn连通区域,返回0为符合，非0为不符合
    Int8 judgeCImageSize(MyConect *ImageConectIn,	//输入要对比的连通区域
                         Mat image);				//输入要对比的矩阵

    //判断添加线的Mat矩阵的尺寸,是符合MyCountLine线堆,返回0为符合，非0为不符合
    Int8 judgeLMaskSize(MyCountLine *LineIn,		//输入要对比的线堆
                        Mat image);					//输入要对比的矩阵

    //判断matIn数据是否有非0点，返回为0表示有非零点，返回1表示无非0点,notzeroLoc为第一个非零点的坐标(只处理Mat的有效区域)
    Int8 judgeDataNotzero(Mat matIn,				//输入要判断的Mat矩阵
                          L_Point *notzeroLoc);		//第一个非零点的坐标(如果有非零点的话)

    //把霍夫平面的rho转换为xy平面的rho,返回xy平面的rho
    Int32 draw_houghrho_to_xyrho(Uint16 rhoIn,		//输入要转换的rho
                                 Uint32 nWidth,		//输入原图xy平面的宽
                                 Uint32 nHeight);	//输入原图xy平面的高


    //把xy平面的rho转换为霍夫平面的rho,返回霍夫平面的rho
    Uint16 draw_xyrho_to_houghrho(Int32 rhoIn,		//输入要转换的rho
                                  Uint32 nWidth,	//输入原图xy平面的宽
                                  Uint32 nHeight);	//输入原图xy平面的高

    /*******************************************************************/


    /*************以下是专门争对特定图像的算法函数你可能在某些单一工程中会使用到,函数运行成功返回0，否则返回非0*********************/

    //保留区域图像中的狭窄部位,其余赋值为0(只处理MyConect的有效区域),并同时输出Hough图
    Int8 MyGetthin(MyConect *ImageConectIn,		//输入区域
                   getthin Mod,					//Mod=THIN_X时为获取X轴方向狭窄部位,Mod=THIN_Y时为获取Y轴方向狭窄部位,Mod=THIN_45时为获取45度轴方向狭窄部位,Mod=THIN_135时为获取135度轴方向狭窄部位
                   Int32 value,					//狭窄部位的小于等于宽度/高度
                   Mat *matOut,					//输出处理结果,输出CV_8UC1型,需要注意是否有足够空间
                   Mat *HoughOut);				//输出Hough图,Mat类型为CV_16UC1,内存大小至少为sizeof(Uint16)*MHC_TETARANGE*MHC_ROLENGTH

    //保留区域图像中的狭窄部位,其余赋值为0(只处理MyConect的有效区域)
    Int8 MyGetthinNoHough(MyConect *ImageConectIn,		//输入区域
                          getthin Mod,					//Mod=THIN_X时为获取X轴方向狭窄部位,Mod=THIN_Y时为获取Y轴方向狭窄部位,Mod=THIN_45时为获取45度轴方向狭窄部位,Mod=THIN_135时为获取135度轴方向狭窄部位
                          Int32 value,					//狭窄部位的小于等于宽度/高度
                          Mat *matOut);					//输出处理结果,输出CV_8UC1型,需要注意是否有足够空间

    //保留区域图像中不为0的连续部位中的狭窄部位,其余赋值为0(只处理Mat矩阵的有效区域)
    Int8 MyGetlxthinNoHough(Mat matIn,					//输入要处理的Mat矩阵仅支持输入CV_8UC1,
                            getthin Mod,				//Mod=THIN_X时为获取X轴方向狭窄部位,Mod=THIN_Y时为获取Y轴方向狭窄部位,Mod=THIN_45时为获取45度轴方向狭窄部位,Mod=THIN_135时为获取135度轴方向狭窄部位
                            Int32 value,				//狭窄部位的小于等于宽度/高度
                            Int32 kongxi,				//连续部位允许断开的最大距离小于等于
                            Mat *matOut);				//输出处理结果,输出CV_8UC1型,需要注意是否有足够空间

    //保留区域图像中每行或每列不为0的像素点总个数小于阈值的部位,其余赋值为0(只处理Mat矩阵的有效区域)
    Int8 MyGetlathinNoHough(Mat matIn,					//输入要处理的Mat矩阵仅支持输入CV_8UC1,
                            getthin Mod,				//Mod=THIN_X时为获取X轴方向狭窄部位,Mod=THIN_Y时为获取Y轴方向狭窄部位,Mod=THIN_45时为获取45度轴方向狭窄部位,Mod=THIN_135时为获取135度轴方向狭窄部位
                            Int32 value,				//非零点总数的小于等于阈值
                            Mat *matOut);				//输出处理结果,输出CV_8UC1型,需要注意是否有足够空间

    //保留区域图像中每行或每列不为0的像素点起点和终点小于阈值的部位,其余赋值为0(只处理Mat矩阵的有效区域)
    Int8 MyGetlanothinNoHough(Mat matIn,				//输入要处理的Mat矩阵仅支持输入CV_8UC1,
                              getthin Mod,				//Mod=THIN_X时为获取X轴方向狭窄部位,Mod=THIN_Y时为获取Y轴方向狭窄部位,Mod=THIN_45时为获取45度轴方向狭窄部位,Mod=THIN_135时为获取135度轴方向狭窄部位
                              Int32 value,				//非零点总数的小于等于阈值
                              Mat *matOut);				//输出处理结果,输出CV_8UC1型,需要注意是否有足够空间

    //保留每一行/列打头的非0元素,其余赋值为0(只处理Mat矩阵的有效区域)
    Int8 MyGeteverfirstbry(Mat matIn,					//输入要处理的Mat矩阵仅支持输入CV_8UC1,
                           getever Mod,					//保留每一行/列打头的非0元素的模式
                           Int32 kongxi,				//连续部位允许断开的最大距离小于等于
                           Mat *matOut);				//输出处理结果,输出CV_8UC1型,需要注意是否有足够空间

    //保留每一行/列打头的非0元素,其余赋值为0(只处理Mat矩阵的有效区域)
    Int8 MyGeteverfirstdistancebry(Mat matIn,			//输入要处理的Mat矩阵仅支持输入CV_8UC1,
                                   getever Mod,			//保留每一行/列打头的非0元素的模式
                                   Int32 distance,		//从第一个非0元素开始往后保留的长度,最小值为1
                                   Mat *matOut);		//输出处理结果,输出CV_8UC1型,需要注意是否有足够空间

    //直线连通域与掩膜对比函数(只处理MyConect和Mat的有效区域)
    Int8 MyConectComperMat(MyConect *ImageConectIn,		//输入区域
                           Mat mask,					//输入掩膜,支持CV_8UC1型二值图,且与ImageConectIn等尺寸
                           Int32 Gapmax,				//输入重叠部分单块联通区域最长的线允许中间断开的最大距离
                           Int32 *mianji,				//输出重叠面积
                           Int32 *longsize);			//输出重叠部分单块联通区域最长的线

    //获取电机趋向函数,verIn的vec1为电机坐标X,Y,vec2为时间帧数t
    Int8 GetMotorLine(Ver2v verIn,				//输入缓存
                      Int32 time,				//输入时间帧数
                      linefittype Mod,			//拟合模式
                      L_Point32 *motorOut);		//计算得到电机趋向

    //流水循环图像(只处理Mat矩阵的有效区域)
    Int8 MyCirculation(Mat *matIn_Out,			//输入/输出Mat矩阵仅支持输入CV_8UC1,CV_16UC1类型
                       circulatype Mod,			//流水循环模式
                       Int32 size);				//流水步长

    //删去非横线形状的区域(删去在linewidth宽度以下,但直线高度差却在top_deep_hight以上的区域)
    Int8 MyDelJudgeNoLine(MyConect *ImageConectIn,		//输入区域
                          MyConect *ImageConectOut,		//输出删去后的结果
                          Int32 linewidth,				//直线宽度以下
                          Int32 top_deep_hight);		//直线高度差以上

    //测量图像的X轴方向的实际累积长度(只处理Mat矩阵的有效区域)
    Int8 MyGetXlong(Mat matIn,				//输入Mat矩阵仅支持输入CV_8UC1类型二值图
                    Int32 *XlongOut);		//输出X轴方向的实际累积长度

    //测量图像的Y轴方向的实际累积长度(只处理Mat矩阵的有效区域)
    Int8 MyGetYlong(Mat matIn,				//输入Mat矩阵仅支持输入CV_8UC1类型二值图
                    Int32 *YlongOut);		//输出Y轴方向的实际累积长度

    //在4倍的大图上用1倍的小图逐列归一化,(只处理matInSmall矩阵的有效区域,注意由于内存尺度不同,三个Mat参数之间互相不支持有同一地址)
    Int8 MyGetBigCannyforminCanny(Mat matInBig,		//输入大图Mat矩阵仅支持输入CV_8UC1类型
                                  Mat matInSmall,	//输入小图Mat矩阵仅支持输入CV_8UC1类型二值图
                                  Mat *matOut,		//输出结果
                                  Int32 size,		//上下膨胀宽度
                                  Int32 value);		//最小值与最大值相差大于该阈值才做归一化

    //在4倍的大图上用1倍的小图逐列归一化(只处理matInSmall矩阵的有效区域,注意由于内存尺度不同,三个Mat参数之间互相不支持有同一地址)
    Int8 MyGetBigCannyforminCanny_rowline(Mat matInBig,		//输入大图Mat矩阵仅支持输入CV_8UC1类型
                                          Mat matInSmall,	//输入小图Mat矩阵仅支持输入CV_8UC1类型二值图
                                          Mat *matOut,		//输出结果
                                          L_line lineIn,	//只对lineIn附近做区域归一化(小图上的line)
                                          Int32 lineInsize,	//做归一化的lineIn直线的宽度
                                          Int32 value);		//最小值与最大值相差大于该阈值才做归一化

    //在4倍的大图上用1倍的小图逐列归一化后做二值,并保留窄部区域(只处理matInSmall矩阵的有效区域,注意由于内存尺度不同,三个Mat参数之间互相不支持有同一地址)
    Int8 MyGetBigCannyforminCanny_rowline_bry(Mat matInBig,			//输入大图Mat矩阵仅支持输入CV_8UC1类型
                                              Mat matInSmall,		//输入小图Mat矩阵仅支持输入CV_8UC1类型二值图
                                              Mat *matOut,			//输出结果
                                              L_line lineIn,		//只对lineIn附近做区域归一化(小图上的line)
                                              Int32 lineInsize,		//做归一化的lineIn直线的宽度
                                              Int32 value,			//最小值与最大值相差大于该阈值才做归一化
                                              Int32 bry_value,		//二值化阈值
                                              binarysidemod bry_mod,//二值化模式
                                              Int32 size_value,		//窄部区域阈值
                                              Int32 *leftOut,		//大图二值化图最左值
                                              Int32 *rightOut,		//大图二值化图最右值
                                              Int32 *topOut,		//大图二值化图最上值
                                              Int32 *bottomOut);	//大图二值化图最下值

    //在4倍的大图上用1倍的小图逐列归一化后做二值,并保留窄部区域,最后输出直线拟合结果(只处理matInSmall矩阵的有效区域,注意由于内存尺度不同,两个Mat参数之间互相不支持有同一地址)
    Int8 MyGetBigCannyforminCanny_rowline_outline(Mat matInBig,					//输入大图Mat矩阵仅支持输入CV_8UC1类型
                                                  Mat matInSmall,				//输入小图Mat矩阵仅支持输入CV_8UC1类型二值图
                                                  L_line *lineOut,				//输出拟合直线结果(大图上的line)
                                                  houghlineinfo *houghinfoOut,	//输出拟合的直线哈夫面信息结果(大图上的line)
                                                  sqaretype Mod,				//输入拟合模式
                                                  L_line lineIn,				//只对lineIn附近做区域归一化(小图上的line)
                                                  Int32 lineInsize,				//做归一化的lineIn直线的宽度
                                                  Int32 value,					//最小值与最大值相差大于该阈值才做归一化
                                                  Int32 bry_value,				//二值化阈值
                                                  binarysidemod bry_mod,		//二值化模式
                                                  Int32 size_value);			//窄部区域阈值

    //在4倍的大图上用1倍的小图逐行归一化(只处理matInSmall矩阵的有效区域,注意由于内存尺度不同,三个Mat参数之间互相不支持有同一地址)
    Int8 MyGetBigCannyforminCanny_line(Mat matInBig,		//输入大图Mat矩阵仅支持输入CV_8UC1类型
                                       Mat matInSmall,		//输入小图Mat矩阵仅支持输入CV_8UC1类型二值图
                                       Mat *matOut,			//输出结果
                                       L_line lineIn,		//只对lineIn附近做区域归一化(小图上的line)
                                       Int32 lineInsize,	//做归一化的lineIn直线的宽度
                                       Int32 value);		//最小值与最大值相差大于该阈值才做归一化

    //在4倍的大图上用1倍的小图逐行归一化后做二值,并保留窄部区域(只处理matInSmall矩阵的有效区域,注意由于内存尺度不同,三个Mat参数之间互相不支持有同一地址)
    Int8 MyGetBigCannyforminCanny_line_bry(Mat matInBig,			//输入大图Mat矩阵仅支持输入CV_8UC1类型
                                           Mat matInSmall,			//输入小图Mat矩阵仅支持输入CV_8UC1类型二值图
                                           Mat *matOut,				//输出结果
                                           L_line lineIn,			//只对lineIn附近做区域归一化(小图上的line)
                                           Int32 lineInsize,		//做归一化的lineIn直线的宽度
                                           Int32 value,				//最小值与最大值相差大于该阈值才做归一化
                                           Int32 bry_value,			//二值化阈值
                                           binarysidemod bry_mod,	//二值化模式
                                           Int32 size_value,		//窄部区域阈值
                                           Int32 *leftOut,			//大图二值化图最左值
                                           Int32 *rightOut,			//大图二值化图最右值
                                           Int32 *topOut,			//大图二值化图最上值
                                           Int32 *bottomOut);		//大图二值化图最下值

    //在4倍的大图上用1倍的小图逐行归一化后做二值,并保留窄部区域,最后输出直线拟合结果(只处理matInSmall矩阵的有效区域,注意由于内存尺度不同,两个Mat参数之间互相不支持有同一地址)
    Int8 MyGetBigCannyforminCanny_line_outline(Mat matInBig,				//输入大图Mat矩阵仅支持输入CV_8UC1类型
                                               Mat matInSmall,				//输入小图Mat矩阵仅支持输入CV_8UC1类型二值图
                                               L_line *lineOut,				//输出拟合直线结果(大图上的line)
                                               houghlineinfo *houghinfoOut,	//输出拟合的直线哈夫面信息结果(大图上的line)
                                               sqaretype Mod,				//输入拟合模式
                                               L_line lineIn,				//只对lineIn附近做区域归一化(小图上的line)
                                               Int32 lineInsize,			//做归一化的lineIn直线的宽度
                                               Int32 value,					//最小值与最大值相差大于该阈值才做归一化
                                               Int32 bry_value,				//二值化阈值
                                               binarysidemod bry_mod,		//二值化模式
                                               Int32 size_value);			//窄部区域阈值

}

#endif // MYHALCV2_H
