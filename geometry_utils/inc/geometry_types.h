#pragma once

#include <limits>
#include "Eigen/Dense"

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
    double alongA = std::numeric_limits<double>::max();
    double alongB = std::numeric_limits<double>::max();
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