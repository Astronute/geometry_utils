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
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"


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

    std::vector<cv::Point> cv_region{ cv::Point(6.80226, -1.03985),
                                        cv::Point(22.0138,  1.4954),
                                        cv::Point(44.6356, -1.01814),
                                        cv::Point(25.9256, 5.21851),
                                        cv::Point(32.5322, 11.8251) };

    int length = 2 * region_1.size();
    double* fast_region = new double[length];
    for (int i = 0; i < region_1.size(); ++i) {
        fast_region[2 * i] = region_1[i](0);
        fast_region[2 * i + 1] = region_1[i](1);
    }

    GeometryUtils GU;
    std::cout << GU.pointInPolygon(22.0138, 1.4954, fast_region, length, true) << std::endl;
    //std::vector<std::vector<Eigen::Vector2d>> polygons = GU.calc_AnotB(region_0, region_1);
    //for (int i = 0; i < polygons.size(); ++i) {
    //    GU.sort_polygon_vertices_ccw(polygons[i]);
    //    for (auto p : polygons[i]) {
    //        std::cout << "polygon " << i << ": " << p.transpose() << std::endl;
    //    }
    //    std::cout << std::endl;
    //}
    delete[] fast_region;
}