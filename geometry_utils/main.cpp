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
//#include "opencv2/core.hpp"
//#include "opencv2/imgproc.hpp"
//#include "opencv2/highgui.hpp"
//#include "opencv2/videoio.hpp"


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

    std::vector<GU::Point> region_0{ GU::Point(43.116028440043323, 19.843321803805768),
                                    GU::Point(38.73416055080979, 25.455209329963878),
                                    GU::Point(7.361910439408538, 23.630335169781997),
                                    GU::Point(6.9659761518011978, 21.460745702264255),
                                    GU::Point(44.902663113649666, 21.460745702264255),
                                    GU::Point(44.902663113649673, 17.578843467967129)
                                    };
    std::vector<GU::Point> region_1{ GU::Point(6.1006657784124165, -1.7133066920493876),
                                    GU::Point(42.902663113649666, -1.7133066920493876),
                                    GU::Point(42.902663113649666, 19.460745702264255),
                                    GU::Point(6.1006657784124165, 19.460745702264255)
    };
    //std::vector<cv::Point> cv_region{ cv::Point(6.80226, -1.03985),
    //                                    cv::Point(22.0138,  1.4954),
    //                                    cv::Point(44.6356, -1.01814),
    //                                    cv::Point(25.9256, 5.21851),
    //                                    cv::Point(32.5322, 11.8251) };

    GeometryUtils gu_;

    GU::Line line = GU::Line(GU::Point(0, 0), GU::Point(5, 5));
    double dis = gu_.calc_pointSegmentDistance(GU::Point(0, 5), line, true);
    std::cout << dis << std::endl;
}