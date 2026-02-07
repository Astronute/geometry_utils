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

namespace GU {
    class GeometryUtils {
    public:

        GeometryUtils();
        ~GeometryUtils();

        GU::Node* upgradePolygon(const std::vector<GU::Point>& p);

        void printPolygon(GU::Node* root);

        GU::Node* nextNonIntersection(GU::Node* n);

        // 计算点到路径曲线的最近

    private:
        std::vector<GU::Node*> all_polygon_node_;
        std::vector<Segment*> all_segment_ptr_;
    };



    // 判断线段是否相交
    bool isIntersection(const Line& lineA, const Line& lineB);

    // 计算线段交点
    Intersection calc_linesIntersect(const Line& lineA, const Line& lineB);

    // 计算点到线段/直线的最短距离
    double calc_pointSegmentDistance(const double x, const double y, const Line& line, bool measureLine);

    inline double calc_pointSegmentDistance(const Point p, const Line& line, bool measureLine) {
        return calc_pointSegmentDistance(p(0), p(1), line, measureLine);
    }

    // 多边形顶点逆时针排序
    void sort_polygon_vertices_ccw(std::vector<Point>& boundary);

    // 点到多边形的距离
    double pointInPolygon(const Point& p, Node* root);

    double pointInPolygon(const Point& p, const std::vector<Point>& polygon, bool measureDist);

    double pointInPolygon(const double x, const double y, const double* polygon, int length, bool measureDist);

    // 计算多边形自交点
    std::vector<Intersection> calc_geometryIntersection(const std::vector<Point>& region);

    // 计算多边形面积
    double polygonArea(const std::vector<Point>& region);

    // 判断线段是否穿过多边形的边
    bool calc_line_cross_polygon(const Line& line, const std::vector<Point>& polygon);

    // 多边形布尔运算
    std::vector<std::vector<Point>> calc_AnotB(const std::vector<Point>& region_0, const std::vector<Point>& region_1);

    // 曲线抽稀
    std::vector<Point> simplifyCurve(const std::vector<Point>& curve, double epsilon);

}



#endif