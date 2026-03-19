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

using namespace GU;

void main() {
    // region test
    //std::vector<GU::Point> region_0{ GU::Point(0, 0),
    //                                   GU::Point(2, 2),
    //                                   GU::Point(4, 0),
    //                                   GU::Point(6, 2),
    //                                   GU::Point(4, 1),
    //                                   GU::Point(2, 4)
    //};

    //std::vector<GU::Point> region_1{ GU::Point(0, 0),
    //                                   GU::Point(10, 0),
    //                                   GU::Point(10, 7),
    //                                   GU::Point(10, 9),
    //                                   GU::Point(0, 5),
    //                                    GU::Point(0, 5.01)
    //};

    std::vector<GU::Point> region_0{ GU::Point(-16.75037550195249, -18.257535480561696),
                                    GU::Point(-12.042743982648728, -17.689368835927834),
                                    GU::Point(-12.736123621650718, -23.210264776075398)
                                    };
    //std::vector<GU::Point> region_1{ GU::Point(-26.688798, -17.568103),
    //                                GU::Point(-22.532573, -2.074950),
    //                                GU::Point(-39.030405, -6.379716),
    //                                GU::Point(-39.925187, -18.238896),
    //                                GU::Point(-26.688798, -17.568103)
    //};
    //std::vector<cv::Point> cv_region{ cv::Point(6.80226, -1.03985),
    //                                    cv::Point(22.0138,  1.4954),
    //                                    cv::Point(44.6356, -1.01814),
    //                                    cv::Point(25.9256, 5.21851),
    //                                    cv::Point(32.5322, 11.8251) };

    bool cc = GU::calc_line_cross_polygon(GU::Line(GU::Point(-18.291195987584452, -17.994333503608065), GU::Point(-14.982549680277309, -20.438654901428244)), region_0);
    std::cout << static_cast<int>(cc) << std::endl;
}