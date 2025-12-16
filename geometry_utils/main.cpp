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

    std::vector<Eigen::Vector2d> region_0{ Eigen::Vector2d(31.2846, 13.5522),
                                   Eigen::Vector2d(29.8729, 38.5123),
                                   Eigen::Vector2d(21.7038, 42.0352),
                                   Eigen::Vector2d(-6.06454,  24.6991),
                                   Eigen::Vector2d(1.32079, 2.37531),
                                   Eigen::Vector2d(23.9684, 7.83008),
                                   Eigen::Vector2d(22.9373, -4.21442),
                                   Eigen::Vector2d(39.7145, 9.91682)
    };

    std::vector<Eigen::Vector2d> region_1{ Eigen::Vector2d(7.64842, 6.44218),
                                           Eigen::Vector2d(15.5381, 15.7025),
                                           Eigen::Vector2d(30.5937, 25.7687),
                                           Eigen::Vector2d(15.9, 19.9297),
                                           Eigen::Vector2d(16.5031,  26.975) };


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