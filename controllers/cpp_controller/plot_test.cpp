#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
int main() {
    plt::plot({1,3,2,4});
    plt::show();
}

// g++ plot_test.cpp -std=c++11 -I/usr/include/python3.12 -lpython3.12 -I/home/lazar/.local/share/pipx/venvs/numpy/lib/python3.12/site-packages/numpy/_core/include 