#ifndef __GEOMETRY_UTILS_H__
#define __GEOMETRY_UTILS_H__

#include <iostream>
#include <cfloat>
#include <limits>
#include <memory>
#include <vector>
#include <deque>
#include <set>
#include "Eigen/Dense"
#include "geometry_types.h"
#include "intersecter.h"
#include "selector.h"

class GeometryUtils{
public:

    GeometryUtils();
    ~GeometryUtils();

    GU::Node* upgradePolygon(const std::vector<GU::Point> &p);

    void printPolygon(GU::Node* root);

    GU::Node* nextNonIntersection(GU::Node* n);

    double pointInPolygon(const GU::Point& p, GU::Node* root);

    double pointInPolygon(const GU::Point& p, const std::vector<GU::Point>& polygon, bool measureDist);

    double pointInPolygon(const double x, const double y, const double* polygon, int length, bool measureDist);

    bool isIntersection(const GU::Line& lineA, const GU::Line& lineB);

    GU::Intersection calc_linesIntersect(const GU::Line& lineA, const GU::Line& lineB);

    template<typename VectorType>
    std::vector<VectorType> simplifyCurve(const std::vector<VectorType>& curve, double epsilon);

    bool calc_line_cross_polygon(const GU::Line& line, const std::vector<GU::Point>& polygon);
    
    void sort_polygon_vertices_ccw(std::vector<GU::Point>& boundary);

    std::vector<std::vector<GU::Point>> calc_AnotB(const std::vector<GU::Point>& region_0, const std::vector<GU::Point>& region_1);

private:
    std::vector<GU::Node*> all_polygon_node_;
    std::vector<Segment*> all_segment_ptr_;
};




#endif