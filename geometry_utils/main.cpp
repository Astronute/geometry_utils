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

    std::vector<Eigen::Vector2d> region_0{ Eigen::Vector2d(42.3596, 16.3366),
                                   Eigen::Vector2d(12.5464, 20.5956),
                                   Eigen::Vector2d(5.08082, 1.93173),
                                   Eigen::Vector2d(26.7527, -8.90419),
                                   Eigen::Vector2d(19.6569, -16),
                                   Eigen::Vector2d(33.5279, -16),
                                   Eigen::Vector2d(30.3167, -9.57771),
                                   Eigen::Vector2d(45.3167, 10.4223)
    };

    std::vector<Eigen::Vector2d> region_1{ Eigen::Vector2d(10,  0),
                                           Eigen::Vector2d(22,  2),
                                           Eigen::Vector2d(40,  0),
                                           Eigen::Vector2d(25,  5),
                                           Eigen::Vector2d(30, 10) };


    GeometryUtils GU;
    std::cout << GU.pointInPolygon(Eigen::Vector2d(31, 3), region_1) << std::endl;
    //std::vector<std::vector<Eigen::Vector2d>> polygons = GU.calc_AnotB(region_0, region_1);
    //for (int i = 0; i < polygons.size(); ++i) {
    //    GU.sort_polygon_vertices_ccw(polygons[i]);
    //    for (auto p : polygons[i]) {
    //        std::cout << "polygon " << i << ": " << p.transpose() << std::endl;
    //    }
    //    std::cout << std::endl;
    //}
}