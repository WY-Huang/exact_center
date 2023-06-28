/*
暂未测试成功（20230628）
*/

// /home/wanyel/TPSoftware/vcpkg/installed/x64-linux/include/matplotlibcpp.h
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;
int main() {
    plt::plot({1,3,2,4});
    plt::show();
}