#include "dbscan.h"


int main(int argc, char** argv) {
	vector<point> dataset;
	//dataset[0].x = 1;
	//dataset[0].y = 2;
	//dataset[1].x = 2;
	//dataset[1].y = 2;
	//dataset[2].x = 2;
	//dataset[2].y = 3;
	//dataset[3].x = 8;
	//dataset[3].y = 7;
	//dataset[4].x = 8;
	//dataset[4].y = 8;
	//dataset[5].x = 25;
	//dataset[5].y = 80;
	point p1(1,2,0);
	dataset.push_back(p1);
	point p2(2, 2, 1);
	dataset.push_back(p2);
	point p3(2, 3, 2);
	dataset.push_back(p3);
	point p4(8,7, 3);
	dataset.push_back(p4);
	point p5(8, 8, 4);
	dataset.push_back(p5);
	point p6(25, 80, 5);
	dataset.push_back(p6);
	DBSCAN(dataset, 3, 2);
	
	for (int i = 0; i < 6; i++) {
		cout << "第" << i + 1 << "个数据: " << dataset[i].x << "," << dataset[i].y << "," << dataset[i].cluster << "\n";//cluster为聚类结果的标签，从0开始，eg:0,1,2,3,4,5
	}
	return 0;
}
