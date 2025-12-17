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
    std::vector<GU::Point> region_0{ GU::Point(0, 0),
                                       GU::Point(10, 0),
                                       GU::Point(10, 7),
                                       GU::Point(0, 7)
    };

    std::vector<GU::Point> region_1{ GU::Point(0, 0),
                                       GU::Point(2, 1),
                                       GU::Point(10, 7),
                                       GU::Point(1, 2)
    };

    //std::vector<GU::Point> region_0{ GU::Point(16.366, 13.9891),
    //                               GU::Point(16.2736, 13.7581),
    //                               GU::Point(16.6533,  13.948)
    //};

    //std::vector<GU::Point> region_1{ GU::Point(6.80226, -1.03985),
    //                                GU::Point(22.0138,  1.4954),
    //                                GU::Point(44.6356, -1.01814),
    //                                GU::Point(25.9256, 5.21851),
    //                                GU::Point(32.5322, 11.8251) };

    //std::vector<cv::Point> cv_region{ cv::Point(6.80226, -1.03985),
    //                                    cv::Point(22.0138,  1.4954),
    //                                    cv::Point(44.6356, -1.01814),
    //                                    cv::Point(25.9256, 5.21851),
    //                                    cv::Point(32.5322, 11.8251) };

    int length = 2 * region_1.size();
    double* fast_region = new double[length];
    for (int i = 0; i < region_1.size(); ++i) {
        fast_region[2 * i] = region_1[i].x;
        fast_region[2 * i + 1] = region_1[i].y;
    }

    GeometryUtils GU;
    //std::vector<std::vector<GU::Point>> polygons = GU.calc_AnotB(region_0, region_1);
    //for (int i = 0; i < polygons.size(); ++i) {
    //    GU.sort_polygon_vertices_ccw(polygons[i]);
    //    for (auto p : polygons[i]) {
    //        std::cout << "polygon " << i << ": " << p << std::endl;
    //    }
    //    std::cout << std::endl;
    //}

    std::vector<GU::Point> polygon = GU.inflatePolygon(region_0, 0.5);
    for (int i = 0; i < polygon.size(); ++i) {
        std::cout << polygon[i] << std::endl;
    }


    delete[] fast_region;
}