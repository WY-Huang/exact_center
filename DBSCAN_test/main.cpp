#include "dbscan.h"
#include "dbscanImg.h"

void test_dbscan()
{
	vector<point> dataset;
	// dataset[0].x = 1;
	// dataset[0].y = 2;
	// dataset[1].x = 2;
	// dataset[1].y = 2;
	// dataset[2].x = 2;
	// dataset[2].y = 3;
	// dataset[3].x = 8;
	// dataset[3].y = 7;
	// dataset[4].x = 8;
	// dataset[4].y = 8;
	// dataset[5].x = 25;
	// dataset[5].y = 80;

	point p1(1, 2, 0);
	dataset.push_back(p1);
	point p2(2, 2, 1);
	dataset.push_back(p2);
	point p3(2, 3, 2);
	dataset.push_back(p3);
	point p4(8, 7, 3);
	dataset.push_back(p4);
	point p5(8, 8, 4);
	dataset.push_back(p5);
	point p6(25, 80, 5);
	dataset.push_back(p6);
	DBSCAN(dataset, 3, 2);
	
	for (int i = 0; i < 6; i++) {
		cout << "第" << i + 1 << "个数据: " << dataset[i].x << "," << dataset[i].y << "," << dataset[i].cluster << "\n";//cluster为聚类结果的标签，从0开始，eg:0,1,2,3,4,5
	}
}

void test_dbscanImg()
{
	cv::Mat img = cv::imread("/home/wanyel/vs_code/exact_center/transparency_test/test_img/NBU_20230720_location/ROI/2023_07_20_15_29_01_307_ROI_.bmp", cv::IMREAD_GRAYSCALE);
	cv::resize(img, img, cv::Size(round(img.cols*0.5), round(img.rows*0.5)), 2);
	cv::imshow("img", img);
	
	vector<samp_point> p = load_mat_to_array(img);
	
	printf("p.size() = %zu\n", p.size());
	
	vector<vector<samp_point>> Clusters = Dbscan(p, 8, 10);
	
	cv::Mat rst;
	show_cluster_img(Clusters, rst, img.rows, img.cols);  //显示分类结果
	
	cv::imshow("rst", rst);
	cv::waitKey(0);

}

int main(int argc, char** argv) 
{
	/* ============== 算法耗时分析 ========================== */
	clock_t begin, end;
	begin = clock();

	/* ===== 测试第一种dbscan算法 ======= */
	// test_dbscan();

	/* ===== 测试第二种dbscanImg算法 ======= */
	// time cost 249956 ms, 耗时太久
	test_dbscanImg();
	
	end = clock();
	std::cout << "algorithm costs:" << double(end - begin) / 1000 << "ms" << std::endl;


	return 0;
}
