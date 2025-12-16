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
    //std::vector<Eigen::Vector2d> region_0{ Eigen::Vector2d(0, 0),
    //                                   Eigen::Vector2d(10, 0),
    //                                   Eigen::Vector2d(10, 7),
    //                                   Eigen::Vector2d(0, 7)
    //};

    //std::vector<Eigen::Vector2d> region_1{ Eigen::Vector2d(1, 1),
    //                                   Eigen::Vector2d(2, 1),
    //                                   Eigen::Vector2d(10, 7),
    //                                   Eigen::Vector2d(1, 2)
    //};

    std::vector<Eigen::Vector2d> region_0{ Eigen::Vector2d(16.366, 13.9891),
                                   Eigen::Vector2d(16.2736, 13.7581),
                                   Eigen::Vector2d(16.6533,  13.948)
    };

    std::vector<Eigen::Vector2d> region_1{ Eigen::Vector2d(6.80226, -1.03985),
                                           Eigen::Vector2d(22.0138,  1.4954),
                                           Eigen::Vector2d(44.6356, -1.01814),
                                           Eigen::Vector2d(25.9256, 5.21851),
                                           Eigen::Vector2d(32.5322, 11.8251) };


    GeometryUtils GU;
    std::vector<std::vector<Eigen::Vector2d>> polygons = GU.calc_AnotB(region_0, region_1);
    for (int i = 0; i < polygons.size(); ++i) {
        GU.sort_polygon_vertices_ccw(polygons[i]);
        for (auto p : polygons[i]) {
            std::cout << "polygon " << i << ": " << p.transpose() << std::endl;
        }
        std::cout << std::endl;
    }
}