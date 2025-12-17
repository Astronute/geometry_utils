#include "polygon_offset.h"


PolygonOffset::PolygonOffset() {}

PolygonOffset::~PolygonOffset() {}

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
