#include "dbscanImg.h"


// 求曼哈顿距离
float squareDistance(samp_point a, samp_point b)
{
    return ((abs(a.x - b.x) + abs(a.y - b.y)) + abs(a.z-b.z));
}

// 将Mat矩阵载入samp_point数组
vector<samp_point> load_mat_to_array(cv::Mat img)
{
    vector<samp_point> p;
    samp_point tmp;
    for(int i = 0; i < img.rows; i++)
    {
        for(int j = 0; j < img.cols; j++)
        {
            tmp.x = (float)j;   // x坐标
            tmp.y = (float)i;   // y坐标
            tmp.z = (float)img.ptr<uchar>(i)[j];   // 像素值z
            p.push_back(tmp);
        }
    }
    return p;
}

// 把灰度图中的所有像素点分类之后，给每一类像素点进行着色的代码，其中每一类都着不同的颜色
#define randomInt(a, b) (rand() % (b - a + 1) + a)

void show_cluster_img(vector<vector<samp_point>> Clusters, cv::Mat &img, int row, int col)
{
    cv::Mat img_tmp = cv::Mat::zeros(row, col, CV_8UC3);
 
    srand((unsigned)time(NULL));
    for(int i = 0; i < Clusters.size(); i++)   //打印分类结果
    {
        int r = randomInt(0, 255);
        int b = randomInt(0, 255);
        int g = randomInt(0, 255);
        std::cout << "color: " << "r=" << r << "\tb=" << b << "\tg=" << g << std::endl;
        for(int j = 0; j < Clusters[i].size(); j++)
        {
            int x = (int)Clusters[i][j].x;
            int y = (int)Clusters[i][j].y;
            
            if(x >= 0 && x < col && y >= 0 && y < row)
            {
                img_tmp.at<cv::Vec3b>(y, x)[0] = r;
                img_tmp.at<cv::Vec3b>(y, x)[1] = b;
                img_tmp.at<cv::Vec3b>(y, x)[2] = g;
            }
        }
    
    }
 
    img_tmp.copyTo(img);

}

// Dbscan算法实现
vector<vector<samp_point>> Dbscan(vector<samp_point> &p, float Eps, int MinPts)
{
  vector<vector<neps_list>> c_p(p.size());   //记录每一个点的邻域点集
  for(int i=0; i < p.size(); i++)
  {
    for(int j=i; j < p.size(); j++)
    {
      if(squareDistance(p[i], p[j]) < Eps)
      {
        p[i].pts++;    //计数邻域内的点数
        neps_list t;
        t.c_idx = i;
        t.n_idx = j;
        c_p[i].push_back(t);   //将点j加入点i的邻域点集中
 
 
        if(i != j)   
        {
          p[j].pts++;
          t.c_idx = j;
          t.n_idx = i;
          c_p[j].push_back(t);    //将点i加入点j的邻域点集中
        }
      }
    }
  }
 
 
  
  for(int i = 0; i < p.size(); i++)   //判断邻域内的点数是否达到MinPts，达到则认为是核心点
  {
    if(p[i].pts >= MinPts)    //如果索引i的点是核心点，则标记其邻域所有点
    {
      p[i].pointType = 3;    //标记核心点
    }
  }
 
 
  vector<vector<samp_point>> Clusters;    //簇集合
 
 
  int len = p.size();
 
 
  int cluster_num = -1;    //簇号初始化为-1
 
 
  for(int i = 0; i < len; i++)   //循环遍历每一个点
  {
    if(p[i].visited == 1)    //如果当前点已经被访问，则跳过
      continue;
 
 
    p[i].visited = 1;     //如果当前点未被访问，则标记为已访问
 
 
    if(p[i].pointType == 3)    //如果当前点为核心点
    {
      vector<samp_point> C;   //新建一个簇
      cluster_num++;   //簇序号加1
      C.push_back(p[i]);   //将当前核心点加入到新建的簇中
      p[i].cluster = cluster_num;   //将当前的簇序号赋值给该点的所属簇序号
      
      vector<neps_point> N;  //求当前核心点的邻域点集合
      for(int k = 0; k < c_p[i].size(); k++)
      {
        neps_point tt;
        tt.a = p[c_p[i][k].n_idx];
        tt.index = c_p[i][k].n_idx;
        N.push_back(tt);
      }
 
 
      for(int j = 0; j < N.size(); j++)   //遍历邻域点集合中的所有点
      {
        if(p[N[j].index].visited == 0)   //通过index访问原点集中的当前邻域点，如果未被访问，则往下执行
        {
          p[N[j].index].visited = 1;   //在原点集中标记该点为已访问
          N[j].a.visited = 1;         //同时在邻域点集中标记该点为已访问
 
 
          if(p[N[j].index].pointType == 3)
          {
            for(int k = 0; k < c_p[N[j].index].size(); k++)
            {
              neps_point tt;
              tt.a = p[c_p[N[j].index][k].n_idx];
              tt.index = c_p[N[j].index][k].n_idx;
              N.push_back(tt);
            }
          }
 
 
          if(N[j].a.cluster == -1)   //如果当前遍历点尚未加入任何簇
          {
            C.push_back(p[N[j].index]);    //将该点加入新建的簇中
            N[j].a.cluster = cluster_num;   //将当前的簇序号赋值给该点的所属簇序号
            p[N[j].index].cluster = cluster_num;   //将当前的簇序号赋值给该点的所属簇序号
          }
        }
      }
      Clusters.push_back(C);    //将新建的簇加入簇集合中
      printf("Clusters.size() = %zu\n", Clusters.size());
    }
 
  }
 
 
  return Clusters;
}