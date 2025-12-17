#include "polygon_offset.h"


PolygonOffset::PolygonOffset() {}

PolygonOffset::~PolygonOffset() {}

bool PolygonOffset::pointsSame(const GU::Point& a, const GU::Point& b) {
    return std::fabs(a(0) - b(0)) < EPS && std::fabs(a(1) - b(1)) < EPS;
}

// 计算自交点
std::unordered_map<int, std::vector<GU::Point>> PolygonOffset::getIntersectionPoints(const std::vector<GU::Point>& polygon) {
    std::unordered_map<int, std::vector<GU::Point>> intersection_map; // 顶点x：交点1、交点2...
    GeometryUtils gu;

    int len = polygon.size();
    for (int i = 0; i < len; ++i) {
        GU::Line line = GU::Line(polygon[i], polygon[(i + 1) % len]);
        for (int j = 0; j < len; ++j) {
            if (std::abs(i - j) > 1) {
                GU::Line line_a = GU::Line(polygon[j], polygon[(j + 1) % len]);
                if (gu.isIntersection(line, line_a)) {
                    GU::Intersection inc = gu.calc_linesIntersect(line, line_a);
                    intersection_map[i].push_back(inc.p);
                    intersection_map[j].push_back(inc.p);
                }
            }
        }
    }

    // 去除重复点
    for (int idx = 0; idx < len; ++idx) {
        for (int i = 0; i < intersection_map[idx].size(); ++i) {
            for (int j = i + 1; j < intersection_map[idx].size(); ++j) {
                if (pointsSame(intersection_map[idx][i], intersection_map[idx][j])) {
                    auto it = intersection_map[idx].begin();
                    intersection_map[idx].erase(it + j);
                }
            }
        }
    }
    // 点排序
    for (int idx = 0; idx < len; ++idx) {
        for (int i = 0; i < intersection_map[idx].size(); ++i) {
            auto cmp = [&](GU::Point a, GU::Point b) {
                double d1 = (a.x - polygon[idx].x) * (a.x - polygon[idx].x) + (a.y - polygon[idx].y) * (a.y - polygon[idx].y);
                double d2 = (b.x - polygon[idx].x) * (b.x - polygon[idx].x) + (b.y - polygon[idx].y) * (b.y - polygon[idx].y);
                return d1 < d2;
                };

            std::sort(intersection_map[idx].begin(), intersection_map[idx].end(), cmp);
        }
    }
    // for(int idx=0; idx<len; ++idx){ // debug
    //     std::cout << " idx: " << idx << std::endl;
    //     for(auto p: intersection_map[idx]){
    //         std::cout << p.transpose() << std::endl;
    //     }
    // }

    return intersection_map;
}

std::vector<GU::Point> PolygonOffset::inflatePolygon(const std::vector<GU::Point>& polygon, const double offset) {
    std::vector<GU::Point> valid_point;
    std::vector<GU::Vector2d> boundary;
    GeometryUtils gu;
    for (auto p : polygon) {
        boundary.push_back(GU::Vector2d(p));
    }

    // 计算顶点偏移
    int N = boundary.size();
    std::vector<GU::Vector2d> off_points;
    for (int i = 0; i < N; ++i) {
        GU::Vector2d left_evec = (boundary[((i - 1) + N) % N] - boundary[i]).normalized();
        GU::Vector2d right_evec = (boundary[(i + 1) % N] - boundary[i]).normalized();
        double cos_theta = left_evec.dot(right_evec);
        double half_sin_theta = std::sqrt((1 - cos_theta) / 2.0);
        GU::Vector2d offset_pos = (left_evec + right_evec).normalized() * (offset / half_sin_theta) + boundary[i];
        GU::Vector2d left_rvec = (boundary[i] - boundary[((i - 1) + N) % N]).normalized();
        GU::Vector2d right_rvec = (boundary[(i + 1) % N] - boundary[i]).normalized();
        double cross_product = left_rvec(0) * right_rvec(1) - left_rvec(1) * right_rvec(0);
        if (cross_product < 0) {
            offset_pos = (left_evec + right_evec).normalized() * (-1.0 * offset / half_sin_theta) + GU::Vector2d(boundary[i].x, boundary[i].y);
        }
        off_points.push_back(offset_pos);

    }

    // 标记无效边
    std::vector<GU::Line> off_edges;
    std::vector<bool> invalid_edge_mask(N, false);
    int invalid_count = 0;
    for (int i = 0; i < off_points.size(); ++i) {
        GU::Vector2d ori_vec = (boundary[(i + 1) % N] - boundary[i]).normalized();
        GU::Vector2d off_vec = (off_points[(i + 1) % N] - off_points[i]).normalized();
        double dot_vec = ori_vec.dot(off_vec);
        if (dot_vec < 0) {
            invalid_edge_mask[i] = true;
            invalid_count++;
        }
        off_edges.push_back(GU::Line(off_points[i](0), off_points[i](1), off_points[(i + 1) % N](0), off_points[(i + 1) % N](1)));
    }

    if (N - invalid_count < 2) { //(有效边不足两条则return)
        return valid_point;
    }

    // 处理无效边 情况1
    for (int i = 0; i < invalid_edge_mask.size(); ++i) {
        GU::Line invalid_edge;
        if (invalid_edge_mask[i] && !invalid_edge_mask[((i - 1) + N) % N] && !invalid_edge_mask[(i + 1) % N]) {
            std::cout << " process case1: " << std::endl;
            GU::Intersection inc = gu.calc_linesIntersect(off_edges[((i - 1) + N) % N], off_edges[(i + 1) % N]);
            off_edges[((i - 1) + N) % N] = GU::Line(off_edges[((i - 1) + N) % N].startX, off_edges[((i - 1) + N) % N].startY, inc.p.x, inc.p.y);
            off_edges[(i + 1) % N] = GU::Line(inc.p.x, inc.p.y, off_edges[(i + 1) % N].endX, off_edges[(i + 1) % N].endY);
        }
    }

    // 处理无效边 情况2
    {
        int mask_len = invalid_edge_mask.size();
        std::vector<bool> visited(mask_len, false);
        for (int i = 0; i < invalid_edge_mask.size(); ++i) {
            std::set<int> invalid_index;
            if (!invalid_edge_mask[i]) {
                valid_point.push_back(GU::Point(off_edges[i].startX, off_edges[i].startY));
            }
            if (invalid_edge_mask[i] && !visited[i]) {
                visited[i] = true;
                invalid_index.insert(i);
                int left = i - 1;
                int l_index = (left + mask_len) % mask_len;
                while (invalid_edge_mask[l_index]) {
                    visited[l_index] = true;
                    invalid_index.insert(l_index);
                    l_index = (--left + mask_len) % mask_len;
                }

                int right = i + 1;
                int r_index = right % mask_len;
                while (invalid_edge_mask[r_index]) {
                    if (invalid_edge_mask[r_index]) {
                        visited[r_index] = true;
                        invalid_index.insert(r_index);
                        r_index = (++right) % mask_len;
                    }
                }

                if (invalid_index.size() > 1) {
                    std::cout << " process case2, num: " << invalid_index.size() << std::endl;
                    for (int idx : invalid_index) {
                        // 计算无效边偏移
                        GU::Vector2d p0_vec = boundary[idx];
                        GU::Vector2d p1_vec = boundary[(idx + 1) % N];
                        GU::Vector2d T_vec = (p1_vec - p0_vec).normalized();
                        GU::Vector2d N_vec = GU::Vector2d(-1.0 * T_vec(1), T_vec(0)).normalized();
                        GU::Vector2d new_p0_vec = p0_vec + N_vec * offset;
                        GU::Vector2d new_p1_vec = p1_vec + N_vec * offset;
                        GU::Line new_off_line = GU::Line(new_p0_vec, new_p1_vec);
                        // 计算与左右有效边的交点
                        GU::Intersection left_inc, right_inc;
                        if (gu.isIntersection(new_off_line, off_edges[l_index])) {
                            left_inc = gu.calc_linesIntersect(new_off_line, off_edges[l_index]);
                            off_edges[l_index] = GU::Line(GU::Point(off_edges[l_index].startX, off_edges[l_index].startY), left_inc.p);
                        }
                        if (gu.isIntersection(new_off_line, off_edges[r_index])) {
                            right_inc = gu.calc_linesIntersect(new_off_line, off_edges[r_index]);
                            off_edges[r_index] = GU::Line(right_inc.p, GU::Point(off_edges[r_index].endX, off_edges[r_index].endY));
                        }
                    }
                    valid_point.push_back(GU::Point(off_edges[l_index].endX, off_edges[l_index].endY));
                }
            }
        }
    }

    return valid_point;
}

std::vector<std::vector<GU::Point>> PolygonOffset::processRing(const std::vector<GU::Point>& ring) {
    std::vector<std::vector<GU::Point>> all_rings;
    std::vector<std::vector<GU::Point>> valid_rings;

    // 获取自交点
    std::unordered_map<int, std::vector<GU::Point>> insection_map = getIntersectionPoints(ring);
    std::vector<GU::Point> all_points;
    std::vector<bool> isIntersection;
    for (int i = 0; i < ring.size(); ++i) {
        all_points.push_back(ring[i]);
        isIntersection.push_back(false);
        if (!insection_map[i].empty()) {
            for (auto p : insection_map[i]) {
                all_points.push_back(p);
                isIntersection.push_back(true);
            }
        }
    }

    int len = all_points.size();
    std::vector<bool> visited(all_points.size(), false);
    std::queue<std::pair<int, GU::Point>> searchlist;
    searchlist.push(std::make_pair(0, all_points[0]));

    while (!searchlist.empty()) {
        auto search_info = searchlist.front();
        int search_idx = search_info.first;
        GU::Point startPoint = search_info.second;
        searchlist.pop();

        std::vector<GU::Point> new_ring;
        new_ring.push_back(search_info.second); // from queue

        int search_next_idx = (search_idx + 1) % len;
        GU::Point nextPoint = all_points[search_next_idx];
        while (!pointsSame(nextPoint, startPoint)) {
            if (!isIntersection[search_next_idx]) {
                new_ring.push_back(nextPoint);
                visited[search_next_idx] = true;
                search_next_idx = (search_next_idx + 1) % len;
                nextPoint = all_points[search_next_idx];
            }
            else {
                searchlist.push(std::make_pair(search_next_idx, all_points[search_next_idx]));
                for (int i = 0; i < len; ++i) {
                    if (pointsSame(nextPoint, all_points[i]) && search_next_idx != i) {
                        new_ring.push_back(all_points[i]);
                        visited[i] = true;
                        search_next_idx = (i + 1) % len;
                        nextPoint = all_points[search_next_idx];
                        break;
                    }
                }
            }

            nextPoint = all_points[search_next_idx];
        }
        all_rings.push_back(new_ring);
    }

    // std::cout << " ring num: " << all_rings.size() << std::endl;
    // 去无效环
    for (auto r : all_rings) {
        double area = 0.0;
        for (int i = 0; i < r.size(); ++i) {
            area += r[i](0) * r[(i + 1) % r.size()](1) - r[i](1) * r[(i + 1) % r.size()](0);
        }
        if (area > 0) {
            valid_rings.push_back(r);
        }
        std::cout << " ring area: " << area << std::endl;
    }

    return valid_rings;
}