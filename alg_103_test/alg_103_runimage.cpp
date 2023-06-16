#include "myhalcv2.h"
#define QUICK_TRANSMIT    //快速传输


class Targetpoint
{
public:
  cv::Point2f pointf;
  std::string name;
};


int alg103_runimage(cv::Mat &cvimgIn,
                    std::vector <cv::Point2f> &pointcloud,
                    std::vector <Targetpoint> &namepoint,
                    bool &solderjoints,
                    int step)    //输出结果点信息
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
    Myhalcv2::Mat m_tempmatIn;
    Myhalcv2::MyConect ImageConect,ImageConectlong;

    Int32 nWidth=cvimgIn.cols;	//输入图像宽
    Int32 nHeight=cvimgIn.rows;	//输入图像高
    Uint8 filterdata[25]={0,0,0,0,0,
                          0,0,0,0,0,
                          1,1,1,1,1,
                          0,0,0,0,0,
                          0,0,0,0,0};

    Myhalcv2::L_Point32 f_center={-1,-1};
    Int32 X_Linestarty=0;
    Int32 X_Lineendy=0;
    cv::Point2f cv_point;
    Int32 jiguangTop,jiguangDeep,jiguangLeft,jiguangRight;
    Int32 nstarti,nendi,nstartj,nendj;
    Targetpoint targetpoint;

    // add start
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
  // add end


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
    imageIn=Myhalcv2::MatCreat(nWidth,nHeight,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff_image);
    Myhalcv2::CvMatToMat(cvimgIn,&imageIn,cv8uc1_Imagebuff_image);
    imageGasu=Myhalcv2::MatCreat(nWidth,nHeight,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff5);
    imageBry=Myhalcv2::MatCreat(nWidth,nHeight,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff4);
    m_brygujia=Myhalcv2::MatCreat(nWidth,nHeight,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff7);
    m16_filterIma=Myhalcv2::MatCreat(nWidth,nHeight,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff6);
    
    Myhalcv2::Mygausspyramid_2levl(imageIn,&imageGasu);

    if(step==3)
    {
        Myhalcv2::MatToCvMat(imageGasu,&cvimgIn);
        return 0;
    }

    if(step!=0)
    {
        imageGasupain=Myhalcv2::MatCreatClone(imageGasu,cv8uc1_Imagebuff8);
    }

    Myhalcv2::Mybinaryval(imageGasu,&bryvalue,Myhalcv2::MHC_BARINYVAL_MEAN);

    i32_bryvalue=(Int32)bryvalue+pingjun;//求平均值二值化阈值
    Myhalcv2::Mybinary(imageGasu,&imageBry,Myhalcv2::MHC_BARINY_VALUE_IMG,255,i32_bryvalue,0);
    if(step==4)
    {
        Myhalcv2::MatToCvMat(imageBry,&cvimgIn);
        return 0;
    }
    m_brygujia=Myhalcv2::MatCreatzero(nHeight/4,nWidth/4,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff7);
    Myhalcv2::Mynormalize_lineXY(imageGasu,&m_brygujia,jiguangduibidu);

    if(step==5)
    {
        Myhalcv2::MatToCvMat(m_brygujia,&cvimgIn);
        return 0;
    }

    i32_bryvalue=gujiaerzhi;
    Myhalcv2::Mybinary(m_brygujia,&m_brygujia,Myhalcv2::MHC_BARINY_VALUE_IMG,255,i32_bryvalue,0);

    if(step==6)
    {
        Myhalcv2::Mymat_to_binself(&m_brygujia,255);
        Myhalcv2::MatToCvMat(m_brygujia,&cvimgIn);
        return 0;
    }

    Myhalcv2::Myconnection2(m_brygujia,&ImageConect,jiguanglong,widthliantongdis,highliantongdis,Myhalcv2::MHC_MORPH_RECT,Myhalcv2::MHC_8LT,cv8uc1_Imagebuff3);//创建8联通区域ImageConect,最小面积120,两区域距离小于2认为同一区域
    Myhalcv2::Myselect_shape(&ImageConect,&ImageConectlong,Myhalcv2::MHC_CONNECT_WIDTH_HEIGHT,jiguanglong,MAX(ImageConect.nWidth,ImageConect.nHeight));//在ImageConect中筛选出高度大于50的联通域
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
        return 0;
    }
    Myhalcv2::Mysmallest_rectangle(&ImageConectlong,&jiguangLeft,&jiguangRight,&jiguangTop,&jiguangDeep);

    Myhalcv2::Mydilation_circle2(m_brygujia,&imageBry,2,0,Myhalcv2::MHC_MORPH_RECT);

    nstartj=MAX(jiguangTop*4,0);
    nendj=MIN(jiguangDeep*4,nHeight-1);
    nstarti=MAX(jiguangLeft*4-30,0);
    nendi=MIN(jiguangRight*4+30,nWidth-1);

    Myhalcv2::MyCutRoi(imageIn,&m_tempmatIn,Myhalcv2::MHC_CUT_NOTCOPY,nstarti,nstartj,nendi-nstarti+1,nendj-nstartj+1);

    for(j=m_tempmatIn.starty;j<m_tempmatIn.starty+m_tempmatIn.height;j++)
    {
        Int32 sum_valuecoor=0;
        Int32 sum_value=0;

        for(i=m_tempmatIn.startx;i<m_tempmatIn.startx+m_tempmatIn.width;i++)
        {
            Int32 di=i>>2;
            Int32 dj=j>>2;
            if(imageBry.data[dj*imageBry.nWidth+di]!=0)
            {
                sum_valuecoor=sum_valuecoor+(Int32)imageIn.data[j*imageIn.nWidth+i]*i;
                sum_value=sum_value+imageIn.data[j*imageIn.nWidth+i];
            }
        }
        if(sum_value!=0)
        {
            f_line[j]=(float)sum_valuecoor/sum_value;
            if(X_Linestarty==0)
            {
                X_Linestarty=j;//骨架起点
            }
            X_Lineendy=j;//骨架终点
            X_lineMark[j]=1;
        }
        if(step==8)
        {
            if(sum_value!=0)
            {
                imageGasupain.data[j*imageGasupain.nWidth+X_line[j]]=128;
            }
        }
    }
    if(step==8)
    {
        Myhalcv2::MatToCvMat(imageGasupain,&cvimgIn);
        return 0;
    }
    /***********************/

    /*
    //以下的图像几乎都是完美图像,需要检测出结果
    //以下对高斯图做卷积
    m16_filterIma=Myhalcv2::MatCreatzero(nHeight/4,nWidth/4,Myhalcv2::CCV_16UC1,cv8uc1_Imagebuff6);
    m_filter=Myhalcv2::MatCreat(5,5,Myhalcv2::CCV_8UC1,filterdata);
    Myhalcv2::Myfilter(imageGasu,m_filter,&m16_filterIma,Myhalcv2::CCV_16UC1,0,f_center);
    memset(X_line,0,sizeof(Int32)*nHeight/4);
    memset(X_lineMark,0,nHeight/4);
    X_Linestarty=0;
    X_Lineendy=0;
    //以下取出二值图结果中每行卷积最大值
    m_brygujia=Myhalcv2::MatCreatzero(nHeight/4,nWidth/4,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff7);
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
    Myhalcv2::Myfixdata(X_line,X_lineMark,nHeight/4);//修复空的线
    m_brygujia=Myhalcv2::MatCreatzero(nHeight/4,nWidth/4,Myhalcv2::CCV_8UC1,cv8uc1_Imagebuff7);
    for(j=X_Linestarty;j<=X_Lineendy;j++)
    {
        m_brygujia.data[j*m_brygujia.nWidth+(X_line[j]>>1)]=255;
    }
    if(step==9)
    {
        Myhalcv2::MatToCvMat(m_brygujia,&cvimgIn);
        return 0;
    }
    */
/***************************************/
//  Myhalcv2::Myresizefix2bitdata_4fSize(X_line,X_lineMark,f_line,nHeight/4);
    for(i=0;i<nHeight;i++)
    {
        Int32 y=(Int32)(((float)i/4)+0.5);
        Int32 x=(Int32)(f_line[i]/4+0.5);
        if(x>=0&&x<imageBry.nWidth&&y>=0&&y<imageBry.nHeight)
        {
            if(imageBry.data[y*imageBry.nWidth+x]==0)
            {
                f_line[i]=-1;
            }
        }
        cv::Point2f point(f_line[i],i);
        pointcloud.push_back(point);
    }
    if(step==1)
    {
        Myhalcv2::MatToCvMat(imageGasu,&cvimgIn);
        if(cvimgIn.type()==CV_8UC1)
            cv::cvtColor(cvimgIn,cvimgIn,cv::COLOR_GRAY2BGR);
        for(j=0;j<nHeight;j++)
        {
            if(f_line[j]>=0)
            {
                Int32 di=(f_line[j]/4+0.5);
                Int32 dj=j/4;
                if(di>=0&&di<imageGasu.nWidth)
                {
                    cvimgIn.data[dj*imageGasu.nWidth*3+di*3]=255;
                    cvimgIn.data[dj*imageGasu.nWidth*3+di*3+1]=0;
                    cvimgIn.data[dj*imageGasu.nWidth*3+di*3+2]=0;
                }
            }
        }
    }
    solderjoints=false;
    cv_point.x=pointcloud[0].x;
    cv_point.y=pointcloud[0].y;
    targetpoint.pointf=cv_point;
    targetpoint.name="point_0";
    namepoint.push_back(targetpoint);  
    cv_point.x=0;
    cv_point.y=0;
    targetpoint.pointf=cv_point;
    targetpoint.name="normal";
    namepoint.push_back(targetpoint);  
    cv_point.x=pointcloud[pointcloud.size()-1].x;
    cv_point.y=pointcloud[pointcloud.size()-1].y;
    targetpoint.pointf=cv_point;
    targetpoint.name="point_1";
    namepoint.push_back(targetpoint);  

    return 0;
}

int main()
{
    cv::Mat srcimg0 = cv::imread("/home/wanyel/vs_code/exact_center/allData/srcImg/bmp/test2r.jpg");
    cv::Mat grayimg;
    cv::cvtColor(srcimg0, grayimg, cv::COLOR_BGR2GRAY);

    std::vector <cv::Point2f> pointcloud0;
    std::vector <Targetpoint> namepoint0;
    bool solderjoints0;

    // 20-30ms
    clock_t begin, end;
    begin = clock();

    alg103_runimage(grayimg, pointcloud0, namepoint0, solderjoints0, 1);

    end = clock();
    std::cout << "alg103runimage costs:" << double(end - begin) / 1000 << "ms" << std::endl;

    for (int i=0; i<pointcloud0.size(); i++)
    {
        cv::circle(srcimg0, cv::Point(round(pointcloud0[i].x), i), 0, cv::Scalar(0, 0, 255), -1, 8);
        // std::cout << pointcloud0[i].x << "\t" << round(pointcloud0[i].x) << std::endl;
    }

    cv::Mat rotatedImage;
    cv::rotate(srcimg0, rotatedImage, cv::ROTATE_90_COUNTERCLOCKWISE);
    cv::imshow("centerline", rotatedImage);
    // cv::imwrite("alg103_test2r_round.jpg", srcimg0);
    cv::waitKey(0);
    return 0;
}