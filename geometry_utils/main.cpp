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
#include "polygon_offset.h"
//#include "intersecter.h"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"


void main() {
    // region test
    //std::vector<GU::Point> region_0{ GU::Point(0, 0),
    //                                   GU::Point(10, 0),
    //                                   GU::Point(10, 7),
    //                                   GU::Point(0, 7)
    //};

    //std::vector<GU::Point> region_1{ GU::Point(0, 0),
    //                                   GU::Point(10, 0),
    //                                   GU::Point(10, 7),
    //                                   GU::Point(10, 9),
    //                                   GU::Point(0, 5),
    //                                    GU::Point(0, 5.01)
    //};

    //std::vector<GU::Point> region_0{ GU::Point(-104.297369, 29.360241),
    //                                GU::Point(-46.385867, 66.161903),
    //                                GU::Point(14.327804, 34.030510),
    //                                GU::Point(9.097088, -27.804012),
    //                                GU::Point(-60.583526, -49.100576)
    //                                };
    std::vector<GU::Point> region_1{ GU::Point(-87.516927557055908, 46.705449594072263),
                                    GU::Point(-42.083757051684763, 46.705449594072263),
                                    GU::Point(12.005232420174487, 26.017181756584385),
                                    GU::Point(8.5384297430614993, -30.371581978777002),
                                    GU::Point(-47.718026933643579, -69.619465355128753)
    };
    //std::vector<cv::Point> cv_region{ cv::Point(6.80226, -1.03985),
    //                                    cv::Point(22.0138,  1.4954),
    //                                    cv::Point(44.6356, -1.01814),
    //                                    cv::Point(25.9256, 5.21851),
    //                                    cv::Point(32.5322, 11.8251) };

    GeometryUtils GU;

    int length = 2 * region_1.size();
    double* fast_region = new double[length];
    for (int i = 0; i < region_1.size(); ++i) {
        fast_region[2 * i] = region_1[i].x;
        fast_region[2 * i + 1] = region_1[i].y;
    }
    
    //std::vector<std::vector<GU::Point>> polygons = GU.calc_AnotB(region_0, region_1);
    //for (int i = 0; i < polygons.size(); ++i) {
    //    GU.sort_polygon_vertices_ccw(polygons[i]);
    //    for (auto p : polygons[i]) {
    //        std::cout << "polygon " << i << ": " << p << std::endl;
    //    }
    //    std::cout << std::endl;
    //}
    PolygonOffset po;
    std::vector<GU::Point> target_polygon = GU.polygonFilter(region_1, 0.1);
    std::vector<std::vector<GU::Point>> rings = po.inflatePolygon(target_polygon, 1.0);
    for (int i = 0; i < rings.size(); ++i) {
        std::cout << "ring : " << i << std::endl;
        for (int j = 0; j < rings[i].size(); ++j) {
            std::cout << rings[i][j] << std::endl;
        }
    }


    delete[] fast_region;
}