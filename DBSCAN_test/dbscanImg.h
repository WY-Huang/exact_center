#include <cmath>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;

// 样本点的类
class samp_point
{
public:
    float x;    //x坐标
    float y;    //y坐标
    float z;    //像素值
    int cluster;    //簇的序号，为-1则表示不属于任何簇
    int pointType;   //1 噪点 2 边界点 3 核心点 
    int visited;     // 1 - 已访问   0 - 未访问
    int pts;        //该点周围邻域内的点数

    samp_point()     //析构函数初始化
    {
        cluster = -1;
        pointType = 1;
        visited = 0;
        pts = 0;
    }
};


// 领域内点的类
class neps_point
{
public:
    samp_point a;    //点
    int index;  //点在原点集中的索引
};


// 记录每一个邻域点所属于的核心点的类：
class neps_list
{
public:
    int c_idx;    //该邻域内点所属核心点的索引
    int n_idx;    //该邻域内点的索引
};


float squareDistance(samp_point a, samp_point b);   // 求曼哈顿距离代码
vector<samp_point> load_mat_to_array(cv::Mat img);  // 将Mat矩阵载入samp_point数组中的代码
void show_cluster_img(vector<vector<samp_point>> Clusters, cv::Mat &img, int row, int col); // 把灰度图中的所有像素点分类之后，给每一类像素点进行着色的代码，其中每一类都着不同的颜色
vector<vector<samp_point>> Dbscan(vector<samp_point> &p, float Eps, int MinPts);    // Dbscan算法实现