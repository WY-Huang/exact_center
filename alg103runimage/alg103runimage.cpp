#include "myhalcv2.h"
#define QUICK_TRANSMIT    //快速传输

// int alg103_runimage(cv::Mat &cvimgIn, std::vector <cv::Point2f> &pointcloud,std::vector <cv::Point2f> &namepoint,
//                     bool &solderjoints, int step);

// 原图处理获取中心线，step取值（1-9， ！0）查看各处理步骤的效果
int alg103_runimage(cv::Mat &cvimgIn,
                    std::vector <cv::Point2f> &pointcloud,// 中心线坐标{"(x, y)", ...}
                    std::vector <cv::Point2f> &namepoint,// 中心线起点和终点位置坐标
                    bool &solderjoints,// 是否焊点
                    int step)    //输出中间处理步骤的信息
{
    Uint8 bryvalue;
    Int32 i32_bryvalue;
    Int32 i,j,n,t;
    Myhalcv2::Mat imageIn;
    Myhalcv2::Mat imageGasu;
    Myhalcv2::Mat imageBry;
    Myhalcv2::Mat m16_filterIma;
    Myhalcv2::Mat m_brygujia;
    Myhalcv2::Mat m_filter;
    Myhalcv2::Mat imageGasupain;
    Myhalcv2::MyConect ImageConect,ImageConectlong;
    Int32 nWidth=cvimgIn.cols;	//输入图像宽,1024
    Int32 nHeight=cvimgIn.rows;	//输入图像高,1536
    Uint8 filterdata[25]={0,0,0,0,0,
                          0,0,0,0,0,
                          1,1,1,1,1,
                          0,0,0,0,0,
                          0,0,0,0,0};

    Myhalcv2::L_Point32 f_center={-1,-1};
    Int32 X_Linestarty=0;
    Int32 X_Lineendy=0;
    cv::Point2f cv_point;

    char *cv8uc1_Imagebuff_image;
    char *cv8uc1_Imagebuff1;
    char *cv8uc1_Imagebuff2;
    char *cv8uc1_Imagebuff3;
    char *cv8uc1_Imagebuff4;
    char *cv8uc1_Imagebuff5;
    char *cv8uc1_Imagebuff6;
    char *cv8uc1_Imagebuff7;
    char *cv8uc1_Imagebuff8;
    char *cv8uc1_Imagebuff9;

    Int32 *X_line;
    float *f_line;
    Uint8 *X_lineMark;
    Int32 *X_linedif32,*niheX,*niheY;
    Myhalcv2::MyConect ImageConectlongPX,Imageheadline;

    Int32 firstsearch;
    Int32 firstsearch_stx,firstsearch_sty,firstsearch_edx,firstsearch_edy;
    Int32 jishuST_x,jishuST_y,jishuED_x,jishuED_y,jishuNum;
    Int32 firstdimian;
    Int32 fuzhuxielv,b_fuzhuxielv,jishuxielv;

    Myhalcv2::L_Point32 fuzhufindST,fuzhufindED;//结果线2拟合区域,(上方)

//
    static int oldwidth=0, oldHeight=0;

  // 申请内存空间
  if(oldwidth!=cvimgIn.cols||oldHeight!=cvimgIn.rows)
  {
    // 释放内存
    if(oldwidth!=0||oldHeight!=0)
    {
      Myhalcv2::MyhalcvMemFree();
      delete [] cv8uc1_Imagebuff_image;
      delete [] cv8uc1_Imagebuff1;
      delete [] cv8uc1_Imagebuff2;
      delete [] cv8uc1_Imagebuff3;
      delete [] cv8uc1_Imagebuff4;
      delete [] cv8uc1_Imagebuff5;
      delete [] cv8uc1_Imagebuff6;
      delete [] cv8uc1_Imagebuff7;
      delete [] cv8uc1_Imagebuff8;
      delete [] X_line;
      delete [] X_lineMark;
      delete [] X_linedif32;
      delete [] niheX;
      delete [] niheY;
      delete [] f_line;
    }
    oldwidth=cvimgIn.cols;// 1536
    oldHeight=cvimgIn.rows;// 1024
    Myhalcv2::MyhalcvMemInit(oldHeight,oldwidth); // 内存初始化

    cv8uc1_Imagebuff_image=new char [oldwidth*oldHeight*4];
    cv8uc1_Imagebuff1=new char [oldwidth*oldHeight];
    cv8uc1_Imagebuff2=new char [Myhalcv2::getHoughsize()];
    cv8uc1_Imagebuff3=new char [Myhalcv2::getConectsize()*oldwidth*oldHeight];
    cv8uc1_Imagebuff4=new char [oldwidth*oldHeight];
    cv8uc1_Imagebuff5=new char [oldwidth*oldHeight];
    cv8uc1_Imagebuff6=new char [oldwidth*oldHeight*2];
    cv8uc1_Imagebuff7=new char [oldwidth*oldHeight];
    cv8uc1_Imagebuff8=new char [oldwidth*oldHeight];
    cv8uc1_Imagebuff9=new char [oldwidth*oldHeight];

    Int32 bigsize;//1536
    bigsize=oldwidth>oldHeight?oldwidth:oldHeight;
    X_line=new Int32 [bigsize];
    f_line=new float [bigsize];
    X_lineMark=new Uint8 [bigsize*4];
    X_linedif32=new Int32 [bigsize];
    niheX=new Int32 [bigsize];
    niheY=new Int32 [bigsize];
  }

/*********************/
    //算法参数
    Int32 pingjun=15;//15;
    Int32 gujiaerzhi=160;//160;
    Int32 widthliantongdis=5;//5;
    Int32 highliantongdis=5;//5;
    Int32 jiguanglong=5;//5;//激光长度
    Int32 jiguangkuandu=10;//10;//激光宽度
    Int32 jiguangduibidu=5;//5;

    if(step==2)
    {
      return 0;
    }
    imageIn=Myhalcv2::MatCreat(nWidth,nHeight,Myhalcv2::CCV_8UC1,0);// 创建空图像矩阵

    Myhalcv2::CvMatToMat(cvimgIn,&imageIn,cv8uc1_Imagebuff_image);// 将cvimage转为MATimage

    // imageGasu=Myhalcv2::MatCreat(nWidth,nHeight,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff5);
    imageBry=Myhalcv2::MatCreat(nWidth,nHeight,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff4);
    m_brygujia=Myhalcv2::MatCreat(nWidth,nHeight,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff7);
    m16_filterIma=Myhalcv2::MatCreat(nWidth,nHeight,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff6);
    
    // Myhalcv2::Mygausspyramid_2levl(imageIn,&imageGasu);// 下采样为原图的1/16大小，高宽各1/4
    imageGasu = Myhalcv2::MatCreatClone(imageIn,cv8uc1_Imagebuff9); // 复制一份imageGasu

    if(step==3)// 输出高斯金字塔处理后的图
    {
        Myhalcv2::MatToCvMat(imageGasu,&cvimgIn);
        cv::imshow("imageGasu", cvimgIn);
        cv::imwrite("30imageGasu.jpg", cvimgIn);
        cv::waitKey(0);
        return 0;
    }

    if(step!=0)
    {
        imageGasupain=Myhalcv2::MatCreatClone(imageGasu,cv8uc1_Imagebuff8); // 复制一份imageGasu
    }

    Myhalcv2::Mybinaryval(imageGasu,&bryvalue,Myhalcv2::MHC_BARINYVAL_MEAN);// 统计全图计算二值化阈值

    i32_bryvalue=(Int32)bryvalue+pingjun;//求平均值二值化阈值
    Myhalcv2::Mybinary(imageGasu,&imageBry,Myhalcv2::MHC_BARINY_VALUE_IMG,255,i32_bryvalue,0);// 高斯图二值化
    if(step==4)
    {
        Myhalcv2::MatToCvMat(imageBry,&cvimgIn);
        cv::imshow("imageBry", cvimgIn);
        cv::imwrite("40imageBry.jpg", cvimgIn);
        cv::waitKey(0);
        return 0;
    }
    // m_brygujia=Myhalcv2::MatCreatzero(nHeight/4,nWidth/4,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff7);
    m_brygujia=Myhalcv2::MatCreatzero(nHeight,nWidth,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff7);
    Myhalcv2::Mynormalize_lineXY(imageGasu,&m_brygujia,jiguangduibidu);// 逐行归一化

    if(step==5)
    {
        Myhalcv2::MatToCvMat(m_brygujia,&cvimgIn);
        cv::imshow("m_brygujia", cvimgIn);
        cv::imwrite("50m_brygujia.jpg", cvimgIn);
        cv::waitKey(0);
        return 0;
    }

    i32_bryvalue=gujiaerzhi;
    Myhalcv2::Mybinary(m_brygujia,&m_brygujia,Myhalcv2::MHC_BARINY_VALUE_IMG,255,i32_bryvalue,0);// 骨架图二值化

    if(step==6)
    {
        Myhalcv2::Mymat_to_binself(&m_brygujia,255);
        Myhalcv2::MatToCvMat(m_brygujia,&cvimgIn);
        cv::imshow("m_brygujia", cvimgIn);
        cv::imwrite("60m_brygujia.jpg", cvimgIn);
        cv::waitKey(0);
        return 0;
    }

    Myhalcv2::Myconnection2(m_brygujia,&ImageConect,jiguanglong,widthliantongdis,highliantongdis,Myhalcv2::MHC_MORPH_RECT,
                            Myhalcv2::MHC_8LT,cv8uc1_Imagebuff3);//创建8联通区域ImageConect,最小面积120,两区域距离小于2认为同一区域
    Myhalcv2::Myselect_shape(&ImageConect,&ImageConectlong,Myhalcv2::MHC_CONNECT_WIDTH_HEIGHT,jiguanglong,
                            MAX(ImageConect.nWidth,ImageConect.nHeight));//在ImageConect中筛选出高度大于50的联通域
    if(ImageConectlong.AllMarkPointCount==0)
    {
    #ifdef QUICK_TRANSMIT
        Myhalcv2::MatToCvMat(imageGasu,&cvimgIn);
    #endif
        return 1;
    }
    Myhalcv2::MyGetthinNoHough(&ImageConectlong,Myhalcv2::THIN_X,jiguangkuandu,&m_brygujia);
    if(step==7)
    {
        Myhalcv2::MatToCvMat(m_brygujia,&cvimgIn);
        cv::imshow("m_brygujia", cvimgIn);
        cv::imwrite("70m_brygujia.jpg", cvimgIn);
        cv::waitKey(0);
        return 0;
    }

    Myhalcv2::Mydilation_circle2(m_brygujia,&imageBry,2,0,Myhalcv2::MHC_MORPH_RECT);

    /***********************/
    //以下的图像几乎都是完美图像,需要检测出结果
    //以下对高斯图做卷积
    // m16_filterIma=Myhalcv2::MatCreatzero(nHeight/4,nWidth/4,Myhalcv2::CCV_16UC1,cv8uc1_Imagebuff6);
    m16_filterIma=Myhalcv2::MatCreatzero(nHeight,nWidth,Myhalcv2::CCV_16UC1,cv8uc1_Imagebuff6);
    m_filter=Myhalcv2::MatCreat(5,5,Myhalcv2::CCV_8UC1,filterdata);
    Myhalcv2::Myfilter(imageGasu,m_filter,&m16_filterIma,Myhalcv2::CCV_16UC1,0,f_center);
    // memset(X_line,0,sizeof(Int32)*nHeight/4);// 初始化为0
    // memset(X_lineMark,0,nHeight/4);
    memset(X_line,0,sizeof(Int32)*nHeight);// 初始化为0
    memset(X_lineMark,0,nHeight);
    X_Linestarty=0;
    X_Lineendy=0;

    //以下取出二值图结果中每行卷积最大值
    // m_brygujia=Myhalcv2::MatCreatzero(nHeight/4,nWidth/4,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff7);
    m_brygujia=Myhalcv2::MatCreatzero(nHeight,nWidth,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff7);
    for(j=m16_filterIma.starty;j<m16_filterIma.starty+m16_filterIma.height;j++)
    {
        Uint16 max=0;
        Uint16 maxX=m16_filterIma.startx;
        Uint16 maxXn=0;
        for(i=m16_filterIma.startx;i<m16_filterIma.startx+m16_filterIma.width;i++)
        {
            if(imageBry.ptr_uchar[j*imageBry.nWidth+i]!=0)
            {
                if(max<m16_filterIma.ptr_ushort[j*m16_filterIma.nWidth+i])
                {
                    max=m16_filterIma.ptr_ushort[j*m16_filterIma.nWidth+i];
                    maxXn=1;
                    maxX=i;
                }
                else if(max==m16_filterIma.ptr_ushort[j*m16_filterIma.nWidth+i])
                {
                    maxXn++;
                    maxX=i+maxX;
                }
            }
        }
        if(maxXn!=0)
        {
            X_line[j]=(maxX<<1)/maxXn;
            if(X_Linestarty==0)
            {
                X_Linestarty=j;//骨架起点
            }
            X_Lineendy=j;//骨架终点
            m_brygujia.data[j*imageGasu.nWidth+(X_line[j]>>1)]=255;
        }

        if(step==8)
        {
            if(X_line[j]!=0&&maxX!=imageBry.startx)
            {
                imageGasupain.data[j*imageGasu.nWidth+(X_line[j]>>1)]=0;
            }
        }
    }
    if(step==8)
    {
        Myhalcv2::MatToCvMat(imageGasupain,&cvimgIn);
        cv::imshow("imageGasupain", cvimgIn);
        cv::imwrite("80imageGasupain.jpg", cvimgIn);
        cv::waitKey(0);
        return 0;
    }

    if(X_Lineendy==0)//没找到骨架
    {
    #ifdef QUICK_TRANSMIT
        Myhalcv2::MatToCvMat(imageGasu,&cvimgIn);
    #endif
        return 1;
    }
    Myhalcv2::Myconnection(m_brygujia,&ImageConect,jiguanglong,1,Myhalcv2::MHC_8LT,cv8uc1_Imagebuff3);//先去掉离散点
    if(ImageConect.AllMarkPointCount==0)
    {
    #ifdef QUICK_TRANSMIT
        Myhalcv2::MatToCvMat(imageGasu,&cvimgIn);
    #endif
        return 1;
    }
    for(j=0;j<ImageConect.AllMarkPointCount;j++)
    {
        for(i=0;i<ImageConect.AllMarkPoint[j].PointArea;i++)
        {
            Int32 y=ImageConect.AllMarkPoint[j].point[i].y;
            X_lineMark[y]=1;
        }
    }
    // Myhalcv2::Myfixdata(X_line,X_lineMark,nHeight/4);//修复空的线
    // m_brygujia=Myhalcv2::MatCreatzero(nHeight/4,nWidth/4,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff7);
    Myhalcv2::Myfixdata(X_line,X_lineMark,nHeight);//修复空的线
    m_brygujia=Myhalcv2::MatCreatzero(nHeight,nWidth,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff7);
    for(j=X_Linestarty;j<=X_Lineendy;j++)
    {
        m_brygujia.data[j*m_brygujia.nWidth+(X_line[j]>>1)]=255;
    }
    if(step==9)
    {
        Myhalcv2::MatToCvMat(m_brygujia,&cvimgIn);
        cv::imshow("m_brygujia", cvimgIn);
        cv::imwrite("90m_brygujia.jpg", cvimgIn);
        cv::waitKey(0);
        return 0;
    }
/***************************************/
    // Myhalcv2::Myresizefix2bitdata_4fSize(X_line,X_lineMark,f_line,nHeight/4);
    Myhalcv2::Myresizefix2bitdata_4fSize(X_line,X_lineMark,f_line,nHeight);
    for (int i = 0; i <1538; i++)
    {
        std::cout << X_line[i] << std::endl;
    }
    // std::cout << f_line[1536] << std::endl;
    for(i=0;i<nHeight;i++)
    {
        // Int32 y=(Int32)(((float)i/4)+0.5);
        // Int32 x=(Int32)(f_line[i]/4+0.5);
        Int32 y=(Int32)(((float)i)+0.5);
        Int32 x=(Int32)(f_line[i]+0.5);
        if(x>=0&&x<imageBry.nWidth&&y>=0&&y<imageBry.nHeight)
        {
            if(imageBry.data[y*imageBry.nWidth+x]==0)
            {
                f_line[i]=-1;// 空点设为-1
            }
        }
        cv::Point2f point(f_line[i],i);
        pointcloud.push_back(point);
        // std::cout << pointcloud[i] << std::endl;
    }
    // 绘制中心线图像
    if(step==1)
    {
        Myhalcv2::MatToCvMat(imageGasu,&cvimgIn);
        if(cvimgIn.type()==CV_8UC1)
            cv::cvtColor(cvimgIn,cvimgIn,cv::COLOR_GRAY2BGR);
        for(j=0;j<nHeight;j++)
        {
            if(f_line[j]>=0)
            {
                // Int32 di=(f_line[j]/4+0.5);
                // Int32 dj=j/4;
                Int32 di=(f_line[j]+0.5);
                Int32 dj=j;
                if(di>=0&&di<imageGasu.nWidth)
                {
                    cvimgIn.data[dj*imageGasu.nWidth*3+di*3]=255;
                    cvimgIn.data[dj*imageGasu.nWidth*3+di*3+1]=0;
                    cvimgIn.data[dj*imageGasu.nWidth*3+di*3+2]=0;
                }
            }
        }
    }
    // std::cout << j << std::endl;
    // cv::imshow("centerline", cvimgIn);
    // cv::imwrite("10centerline.jpg", cvimgIn);
    // cv::waitKey(0);

    solderjoints=false;
    // 第一个点的坐标
    cv_point.x=pointcloud[0].x;
    cv_point.y=pointcloud[0].y;
    namepoint.push_back(cv_point); 
    // 最后一个点的坐标
    cv_point.x=pointcloud[pointcloud.size()-1].x;
    cv_point.y=pointcloud[pointcloud.size()-1].y;
    namepoint.push_back(cv_point);  

    std::cout << namepoint[0] << "\n" << namepoint[1] << std::endl;
    return 0;
}

// 灰度质心法提取中心线
int alg103_graycentroid(cv::Mat &cvimgIn,
                        std::vector <cv::Point2f> &pointcloud,// 中心线坐标{"(x, y)", ...}
                        std::vector <cv::Point2f> &namepoint,// 中心线起点和终点位置坐标
                        bool &solderjoints,// 是否焊点
                        int step)    //输出中间处理步骤的信息
{
    cv::Mat grayimg;
    cv::GaussianBlur(cvimgIn, grayimg, cv::Size(0, 0), 6, 6);
    
    for (int i = 0;i < grayimg.rows;i++) 
    {
        float sum_value = 0;
        float sum_valuecoor = 0;
        std::vector<float> current_value;
        std::vector<float> current_coordinat;
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
        float x_centroid = sum_valuecoor / sum_value;

        cv::Point2f point(x_centroid, i);
        pointcloud.push_back(point);
        
        current_value.clear();
        current_coordinat.clear();
    }
    if(step==1)
    {
        if(cvimgIn.type()==CV_8UC1)
            cv::cvtColor(cvimgIn,cvimgIn,cv::COLOR_GRAY2BGR);

        for (int i=0; i<pointcloud.size(); i++)
        {
            cv::circle(cvimgIn, cv::Point(round(pointcloud[i].x), i), 0, cv::Scalar(0, 0, 255), -1, 8);
        }
    }

    solderjoints=false;
    // 第一个点的坐标
    cv::Point2f cv_point;
    cv_point.x=pointcloud[0].x;
    cv_point.y=pointcloud[0].y;
    namepoint.push_back(cv_point); 
    // 最后一个点的坐标
    cv_point.x=pointcloud[pointcloud.size()-1].x;
    cv_point.y=pointcloud[pointcloud.size()-1].y;
    namepoint.push_back(cv_point);  

    return 0;
}


int main()
{
    cv::Mat srcimg0 = cv::imread("/home/wanyel/vs_code/exact_center/srcImg/bmp/test2r.jpg");
    cv::Mat grayimg;
    cv::cvtColor(srcimg0, grayimg, cv::COLOR_BGR2GRAY);

    std::vector <cv::Point2f> pointcloud0;
    std::vector <cv::Point2f> namepoint0;
    bool solderjoints0;

    // 20-30ms
    clock_t begin, end;
    begin = clock();

    alg103_graycentroid(grayimg, pointcloud0, namepoint0, solderjoints0, 1);

    end = clock();
    std::cout << "alg103runimage costs:" << double(end - begin) / 1000 << "ms" << std::endl;

    for (int i=0; i<pointcloud0.size(); i++)
    {
        cv::circle(srcimg0, cv::Point(round(pointcloud0[i].x), i), 0, cv::Scalar(0, 0, 255), -1, 8);
        // std::cout << pointcloud0[i].x << "\t" << round(pointcloud0[i].x) << std::endl;
    }
    cv::imshow("centerline", srcimg0);
    // cv::imwrite("alg103_test2r_round.jpg", srcimg0);
    cv::waitKey(0);
    return 0;
}