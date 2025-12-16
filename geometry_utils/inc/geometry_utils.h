#ifndef __GEOMETRY_UTILS_H__
#define __GEOMETRY_UTILS_H__

#include <iostream>
#include <cfloat>
#include <limits>
#include <memory>
#include <vector>
#include <deque>
#include "Eigen/Dense"
#include "geometry_types.h"
#include "intersecter.h"
#include "selector.h"

class GeometryUtils{
public:

    GeometryUtils();
    ~GeometryUtils();

    GU::Node* upgradePolygon(const std::vector<Eigen::Vector2d> &p);

    void printPolygon(GU::Node* root);

    GU::Node* nextNonIntersection(GU::Node* n);

    double pointInPolygon(const Eigen::Vector2d& p, GU::Node* root);

    double pointInPolygon(const Eigen::Vector2d& p, const std::vector<Eigen::Vector2d>& polygon, bool measureDist);

    double pointInPolygon(const int x, const int y, const double* polygon, int length, bool measureDist);

    GU::Intersection calc_linesIntersect(const Eigen::Vector2d &a0, const Eigen::Vector2d &a1, const Eigen::Vector2d &b0, const Eigen::Vector2d &b1);

    template<typename VectorType>
    std::vector<VectorType> simplifyCurve(const std::vector<VectorType>& curve, double epsilon);

    bool calc_line_cross_polygon(const Eigen::Vector2d& lstart, const Eigen::Vector2d& lend, const std::vector<Eigen::Vector2d>& polygon);
    
    void sort_polygon_vertices_ccw(std::vector<Eigen::Vector2d>& boundary);

    std::vector<std::vector<Eigen::Vector2d>> calc_AnotB(const std::vector<Eigen::Vector2d>& region_0, const std::vector<Eigen::Vector2d>& region_1);

private:
    std::vector<GU::Node*> all_polygon_node_;
    std::vector<Segment*> all_segment_ptr_;
};




#endif