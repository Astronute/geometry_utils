#include <iostream>
#include <set>
#include <deque>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <cmath>
#include <algorithm>
#include "geometry_utils.h"
//#include "intersecter.h"



void main() {
    // region test
    std::vector<Eigen::Vector2d> region_0{ Eigen::Vector2d(0, 0),
                                       Eigen::Vector2d(4, 0),
                                       Eigen::Vector2d(4, 4),
                                       Eigen::Vector2d(0, 3) };
    std::vector<Eigen::Vector2d> region_1{ Eigen::Vector2d(1, 0),
                                           Eigen::Vector2d(2, 0),
                                           Eigen::Vector2d(2, 10),
                                           Eigen::Vector2d(1, 10) };


    GeometryUtils GU;
    std::vector<std::vector<Eigen::Vector2d>> polygons = GU.calc_AnotB(region_0, region_1);
    for (int i = 0; i < polygons.size(); ++i) {
        for (auto p : polygons[i]) {
            std::cout << "polygon " << i << ": " << p.transpose() << std::endl;
        }
        std::cout << std::endl;
    }
}