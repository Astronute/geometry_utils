#ifndef __GEOMETRY_UTILS_H__
#define __GEOMETRY_UTILS_H__

#include <iostream>
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

    Node* upgradePolygon(const std::vector<Eigen::Vector2d> &p);

    void printPolygon(Node* root);

    Node* nextNonIntersection(Node* n);

    double pointInPolygon(const Eigen::Vector2d& p, Node* root);

    Intersection calc_linesIntersect(const Eigen::Vector2d &a0, const Eigen::Vector2d &a1, const Eigen::Vector2d &b0, const Eigen::Vector2d &b1);
    
    void sort_polygon_vertices_ccw(std::vector<Eigen::Vector2d>& boundary);

    std::vector<std::vector<Eigen::Vector2d>> calc_AnotB(const std::vector<Eigen::Vector2d>& region_0, const std::vector<Eigen::Vector2d>& region_1);

private:
    std::vector<Node*> all_polygon_node_;
    std::vector<Segment*> all_segment_ptr_;
};




#endif