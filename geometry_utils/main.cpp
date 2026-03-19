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

    std::vector<GU::Point> region_0{ GU::Point(0, 0),
                                    GU::Point(2, 0),
                                    GU::Point(4, 4),
                                    GU::Point(6, 0),
                                    GU::Point(8, 0),
                                    GU::Point(4, 8)
                                    };

    bool cc = GU::calc_line_cross_polygon(GU::Line(GU::Point(2, 4), GU::Point(6, 4)), region_0);
    std::cout << static_cast<int>(cc) << std::endl;
}