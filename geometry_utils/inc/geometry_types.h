#pragma once
#include <stdexcept>
#include <iostream>
#include <limits>
#include <cfloat>
#include <cmath>

namespace GU {
    struct Point {
        Point() : x(0.0), y(0.0) {}
        Point(double x_, double y_) : x(x_), y(y_) {}

        double operator()(int index) const {
            if (index == 0) {
                return x;
            }
            else if (index == 1) {
                return y;
            }
            else {
                throw std::out_of_range("Vector2d index out of range! Only 0 (x) or 1 (y) are allowed.");
            }
        }

        friend std::ostream& operator<<(std::ostream& os, const Point& p);

        double x, y;
    };

    inline std::ostream& operator<<(std::ostream& os, const Point& vec) {
        os << "(" << vec.x << ", " << vec.y << ")";
        return os;
    }

    struct Node {
        Point p;
        bool is_intersection = false;
        Node* next = nullptr;
        Node* prev = nullptr;
        Node* friend_n = nullptr; // 同交点的另一个多边形节点
        double dist; // alongA
    };

    class Vector2d {
    public:
        Vector2d() {
        }
        Vector2d(double x_, double y_) {
            x = x_; y = y_;
        }
        Vector2d(const Point &p) {
            x = p.x; y = p.y;
        }

        double operator()(int index) const {
            if (index == 0) {
                return x;
            }
            else if (index == 1) {
                return y;
            }
            else {
                throw std::out_of_range("Vector2d index out of range! Only 0 (x) or 1 (y) are allowed.");
            }
        }

        Vector2d operator-(const Vector2d& other) const {
            return Vector2d(this->x - other.x, this->y - other.y);
        }

        Vector2d operator+(const Vector2d& other) const {
            return Vector2d(this->x + other.x, this->y + other.y);
        }

        Vector2d operator*(double val) const {
            return Vector2d(this->x * val, this->y * val);
        }

        Vector2d operator/(double val) const {
            if (std::fabs(val) < 1e-10) {
                throw std::invalid_argument("Vector2d::operator/ - division by zero (or near-zero scalar)!");
            }
            return Vector2d(this->x/val, this->y/val);
        }

        friend std::ostream& operator<<(std::ostream& os, const Point& p);

        double norm() {
            return std::sqrt(x*x+y*y);
        }

        double dot(const Vector2d& vec) {
            return this->x * vec(0) + this->y * vec(1);
        }

        double cross(const Vector2d& vec) {
            return this->x * vec(1) - this->y * vec(0);
        }

        Vector2d normalized() {
            const double norm_ = norm();
            if (std::fabs(norm_) < 1e-10) {
                throw std::invalid_argument("Vector2d::normalized() - zero vector cannot be normalized!");
            }
            return Vector2d(this->x/norm_, this->y/norm_);
        }

        double x, y;
    };

    inline std::ostream& operator<<(std::ostream& os, const Vector2d& vec) {
        os << "(" << vec.x << ", " << vec.y << ")";
        return os;
    }

    struct Intersection {
        Point p;
        double cross;
        double isParallel = false;
        double alongA = FLT_MAX;
        double alongB = FLT_MAX;
    };

    struct Line {
        Line() {
        
        }
        Line(double x1, double y1, double x2, double y2) {
            startX = x1; startY = y1;
            endX = x2; endY = y2;
        }
        Line(Point p1, Point p2) {
            startX = p1.x; startY = p1.y;
            endX = p2.x; endY = p2.y;
        }
        Line(Vector2d v1, Vector2d v2) {
            startX = v1.x; startY = v1.y;
            endX = v2.x; endY = v2.y;
        }

        double startX, startY, endX, endY;
        bool valid;
    };

    struct MatchPoint {

        MatchPoint(int idx, bool mh, bool ms) {
            index = idx;
            matchs_chain_head = mh;
            matches_seg_start = ms;
        }

        int index = -1;
        bool matchs_chain_head = false; // 匹配到链的头部/尾部
        bool matches_seg_start = false; // 匹配到起点/终点
    };
}
