#pragma once

#include <limits>
#include "Eigen/Dense"

namespace GU {
    struct Node {
        Eigen::Vector2d p;
        bool is_intersection = false;
        Node* next = nullptr;
        Node* prev = nullptr;
        Node* friend_n = nullptr; // 同交点的另一个多边形节点
        double dist; // alongA
    };

    struct Intersection {
        Eigen::Vector2d p;
        double cross;
        double isParallel = false;
        double alongA = FLT_MAX;
        double alongB = FLT_MAX;
    };

    struct Line {

        Line(double x1, double y1, double x2, double y2) {
            startX = x1; startY = y1;
            endX = x2; endY = y2;
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
