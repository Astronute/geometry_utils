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

#ifdef _DEBUG
#define DEBUG_PRINT(...) std::cout << "[DEBUG] " << __VA_ARGS__ << std::endl
#define DEBUG_PRINT_OLDLINE(...) std::cout << "[DEBUG] " << __VA_ARGS__
#define DEBUG_PRINT_NEWLINE() std::cout << std::endl

#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINT_OLDLINE(...)
#define DEBUG_PRINT_NEWLINE(...)
#endif

class GeometryUtils{
public:

    GeometryUtils();
    ~GeometryUtils();

    GU::Node* upgradePolygon(const std::vector<GU::Point> &p);

    void printPolygon(GU::Node* root);

    GU::Node* nextNonIntersection(GU::Node* n);

    // 判断线段是否相交
    bool isIntersection(const GU::Line& lineA, const GU::Line& lineB);

    // 计算线段交点
    GU::Intersection calc_linesIntersect(const GU::Line& lineA, const GU::Line& lineB);

    // 计算点到线段/直线的最短距离
    double calc_pointSegmentDistance(const double x, const double y, const GU::Line& line, bool measureLine);

    inline double calc_pointSegmentDistance(const GU::Point p, const GU::Line& line, bool measureLine) {
        return calc_pointSegmentDistance(p(0), p(1), line, measureLine);
    }

    // 计算点到路径曲线的最近

    // 多边形顶点逆时针排序
    void sort_polygon_vertices_ccw(std::vector<GU::Point>& boundary);

    // 点到多边形的距离
    double pointInPolygon(const GU::Point& p, GU::Node* root);

    double pointInPolygon(const GU::Point& p, const std::vector<GU::Point>& polygon, bool measureDist);

    double pointInPolygon(const double x, const double y, const double* polygon, int length, bool measureDist);

    // 计算多边形自交点
    std::vector<GU::Intersection> calc_geometryIntersection(const std::vector<GU::Point>& region);

    // 计算多边形面积
    double polygonArea(const std::vector<GU::Point>& region);

    // 自交多边形过滤
    std::vector<GU::Point> polygonFilter(const std::vector<GU::Point>& polygon, double gap);

    // 判断线段是否穿过多边形的边
    bool calc_line_cross_polygon(const GU::Line& line, const std::vector<GU::Point>& polygon);

    // 多边形布尔运算
    std::vector<std::vector<GU::Point>> calc_AnotB(const std::vector<GU::Point>& region_0, const std::vector<GU::Point>& region_1);

    // 曲线抽稀
    std::vector<GU::Point> simplifyCurve(const std::vector<GU::Point>& curve, double epsilon);

private:
    std::vector<GU::Node*> all_polygon_node_;
    std::vector<Segment*> all_segment_ptr_;
};




#endif