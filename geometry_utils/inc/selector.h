#pragma once

#include "vector"
#include "deque"
#include "geometry_types.h"
#include "linked_list.h"
#include "Eigen/Dense"

class SegmentSelector {
public:
    SegmentSelector() {
    }
    ~SegmentSelector() {
    }

    std::vector<Segment> select(const Eigen::Matrix4i& selector, std::vector<Segment*> segments);

    std::vector<std::deque<Eigen::Vector2d>> segmentChain(const std::vector<Segment>& segments);

    Eigen::Matrix4i difference() {
        Eigen::Matrix4i select;
        select << 0, 0, 0, 0,
            2, 0, 2, 0,
            1, 1, 0, 0,
            0, 1, 2, 0;
        return select;
    }
    Eigen::Matrix4i difference2() {
        Eigen::Matrix4i select;
        select << 0, 0, 0, 0,
            2, 0, 2, 0,
            1, 1, 0, 0,
            0, 0, 0, 0;
        return select;
    }
};