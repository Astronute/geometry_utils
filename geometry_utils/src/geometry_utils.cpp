#include "geometry_utils.h"


std::string FillInfoToString(const FillState& seg) {
    switch (seg) {
    case FillState::Unknown: return "Unknown";
    case FillState::Filled: return "Filled";
    case FillState::Empty: return "Empty";
    default: return "Invalid";
    }
}

GeometryUtils::GeometryUtils(){

}

GeometryUtils::~GeometryUtils() {
    for (auto node : all_polygon_node_) {
        delete node;
    }
    all_polygon_node_.clear();
    for (auto node : all_segment_ptr_) {
        delete node;
    }
    all_segment_ptr_.clear();
}

GU::Node* GeometryUtils::upgradePolygon(const std::vector<Eigen::Vector2d> &p){
    GU::Node* root = nullptr;
    for(int i=0; i<p.size(); ++i){
        GU::Node* node = new GU::Node();
        all_polygon_node_.push_back(node);
        node->p = p[i];
        if(root == nullptr){
            node->next = node;
            node->prev = node;
            root = node;
        }
        else{
            GU::Node* last_node = root->prev;
            last_node->next = node;
            root->prev = node;
            node->prev = last_node;
            node->next = root;
        }
    }

    return root;
}

void GeometryUtils::printPolygon(GU::Node* root) {
    GU::Node* p = root;
    do {
        std::cout << p->p.transpose() << std::endl;
        p = p->next;
    } while (p!=root);
}

GU::Intersection GeometryUtils::calc_linesIntersect(const GU::Line& lineA, const GU::Line& lineB) {
    double dx1 = lineA.endX - lineA.startX;
    double dy1 = lineA.endY - lineA.startY;
    double dx2 = lineB.endX - lineB.startX;
    double dy2 = lineB.endY - lineB.startY;

    double cross = dx1 * dy2 - dy1 * dx2;

    GU::Intersection inc;
    inc.cross = cross;
    if (std::fabs(cross) < 1e-10) {
        std::cout << " GeometryUtils calc_linesIntersect: || " << std::endl;
        inc.isParallel = true;
        return inc;
    }

    Eigen::Matrix2d A(2, 2);
    Eigen::Vector2d B(2);
    A << dx1, -1.0 * dx2,
        dy1, -1.0 * dy2;
    B << lineB.startX - lineA.startX,
        lineB.startY - lineA.startY;
    Eigen::Vector2d C = A.partialPivLu().solve(B); // colPivHouseholderQr()
    inc.alongA = C(0);
    inc.alongB = C(1);
    inc.p << lineA.startX + inc.alongA * dx1,
        lineA.startY + inc.alongA * dy1;

    return inc;
}


GU::Node* GeometryUtils::nextNonIntersection(GU::Node* n){
    GU::Node* node = n;
    do{
        node = node->next;
    }while(n->is_intersection);

    return node;
}


double GeometryUtils::pointInPolygon(const Eigen::Vector2d &p, GU::Node* root){
    double res = 0;
    int counter = 0;
    GU::Node* Pcur = root;
    do{
        GU::Node* Pnext = Pcur->next;
        if((p(1)>= Pcur->p(1)&&p(1)>= Pnext->p(1)) || (p(1)< Pcur->p(1)&&p(1)< Pnext->p(1)) || (p(0)> Pcur->p(0)&&p(0)> Pnext->p(0))){
            if (p(1) == Pcur->p(1) && p(1) == Pnext->p(1) && ((p(0) >= Pcur->p(0) && p(0) <= Pnext->p(0))|| (p(0) <= Pcur->p(0) && p(0) >= Pnext->p(0)))) {
                return 0;
            }
            if (p(1) == Pcur->p(1) && p(0) == Pcur->p(0)) {
                return 0;
            }
            Pcur = Pnext;
            continue;
        }
        Eigen::Vector2d v0_v1 = Pnext->p - Pcur->p;
        Eigen::Vector2d v0_p = p - Pcur->p;
        double cross = v0_v1(0)*v0_p(1) - v0_v1(1)*v0_p(0);
        if(cross==0){
            return 0;
        }
        if(Pnext->p(1)< Pcur->p(1)){
            cross = -cross;
        }
        counter += cross > 0;
        Pcur = Pnext;
    } while (Pcur != root);

    res = counter%2==0?-1:1;

    return res;
}

double GeometryUtils::pointInPolygon(const GU::Point& p, const std::vector<GU::Point>& polygon, bool measureDist) {
    double res = 0;
    int counter = 0;
    int len = polygon.size();

    double min_dist_num = FLT_MAX;
    double min_dist_denom = 1;

    GU::Point v0, v = polygon[len - 1];

    if (!measureDist) {
        for (int i = 0; i < len; ++i) {
            double dx, dy, dx1, dy1;

            v0 = v;
            v = polygon[i];
            dx = v.x - v0.x;
            dy = v.y - v0.y;
            dx1 = p.x - v0.x;
            dy1 = p.y - v0.y;
            if ((v0.y <= p.y && v.y <= p.y) || (v0.y > p.y && v.y > p.y) || (v0.x < p.x && v.x < p.x)) {
                if (p.y == v.y && (p.x == v.x || (p.y == v0.y &&
                    ((v0.x <= p.x && p.x <= v.x) || (v.x <= p.x && p.x <= v0.x))))) {
                    return 0;
                }
                continue;
            }

            double cross = dx * dy1 - dy * dx1;
            if (cross == 0) {
                return 0;
            }
            if (v.y < v0.y) {
                cross = -cross;
            }
            counter += cross > 0;
        }
        res = counter % 2 == 0 ? -1 : 1;
    }
    else {
        for (int i = 0; i < len; ++i) {

            double dx, dy, dx1, dy1, dx2, dy2, dist_num, dist_denom = 1;
            
            v0 = v;
            v = polygon[i];
            dx = v.x - v0.x;
            dy = v.y - v0.y;
            dx1 = p.x - v0.x;
            dy1 = p.y - v0.y;
            dx2 = p.x - v.x;
            dy2 = p.y - v.y;
            if (dx1 * dx + dy1 * dy <= 0) {
                dist_num = dx1 * dx1 + dy1 * dy1;
            }
            else if (dx2 * dx + dy2 * dy >= 0) {
                dist_num = dx2 * dx2 + dy2 * dy2;
            }
            else {
                dist_num = (dy1 * dx - dx1 * dy);
                dist_num *= dist_num;
                dist_denom = dx * dx + dy * dy;
            }
            if (dist_num * min_dist_denom < min_dist_num * dist_denom) {
                min_dist_num = dist_num;
                min_dist_denom = dist_denom;
                if (min_dist_num == 0) {
                    break;
                }
            }
            if ((v0.y <= p.y && v.y <= p.y) || (v0.y > p.y && v.y > p.y) || (v0.x < p.x && v.x < p.x)) {
                continue;
            }
            dist_num = dy1 * dx - dx1 * dy;
            if (dy < 0) {
                dist_num = -dist_num;
            }
            counter += dist_num > 0;
        }
        res = std::sqrt(min_dist_num / min_dist_denom);
        if (counter % 2 == 0) {
            res = -res;
        }
    }

    return res;
}

double GeometryUtils::pointInPolygon(const double x, const double y, const double* polygon, int length, bool measureDist) {
    double res = 0;
    int counter = 0;
    int len = length / 2;

    double min_dist_num = FLT_MAX;
    double min_dist_denom = 1;

    double v0_x, v_x = polygon[length - 2];
    double v0_y, v_y = polygon[length - 1];

    if (!measureDist) {
        for (int i = 0; i < len; ++i) {
            double dx, dy, dx1, dy1, dx2, dy2, dist_num = 1;

            v0_x = v_x;
            v0_y = v_y;
            v_x = polygon[i * 2];
            v_y = polygon[i * 2 + 1];
            dx = v_x - v0_x;
            dy = v_y - v0_y;
            dx1 = x - v0_x;
            dy1 = y - v0_y;

            if ((v0_y <= y && v_y <= y) || (v0_y > y && v_y > y) || (v0_x < x && v_x < x)) {
                if (y == v_y && (x == v_x || (y == v0_y &&
                    ((v0_x <= x && x <= v_x) || (v_x <= x && x <= v0_x))))) {
                    return 0;
                }
                continue;
            }

            dist_num = dy1 * dx - dx1 * dy;
            if (dist_num == 0) {
                return 0;
            }
            if (v_y < v0_y) {
                dist_num = -dist_num;
            }
            counter += dist_num > 0;
        }
        res = counter % 2 == 0 ? -1 : 1;
    }
    else {
        for (int i = 0; i < len; ++i) {

            double dx, dy, dx1, dy1, dx2, dy2, dist_num, dist_denom = 1;

            v0_x = v_x;
            v0_y = v_y;
            v_x = polygon[i * 2];
            v_y = polygon[i * 2 + 1];
            dx = v_x - v0_x;
            dy = v_y - v0_y;
            dx1 = x - v0_x;
            dy1 = y - v0_y;
            dx2 = x - v_x;
            dy2 = y - v_y;
            if (dx1 * dx + dy1 * dy <= 0) {
                dist_num = dx1 * dx1 + dy1 * dy1;
            }
            else if (dx2 * dx + dy2 * dy >= 0) {
                dist_num = dx2 * dx2 + dy2 * dy2;
            }
            else {
                dist_num = (dy1 * dx - dx1 * dy);
                dist_num *= dist_num;
                dist_denom = dx * dx + dy * dy;
            }
            if (dist_num * min_dist_denom < min_dist_num * dist_denom) {
                min_dist_num = dist_num;
                min_dist_denom = dist_denom;
                if (min_dist_num == 0) {
                    break;
                }
            }
            if ((v0_y <= y && v_y <= y) || (v0_y > y && v_y > y) || (v0_x < x && v_x < x)) {
                continue;
            }
            dist_num = dy1 * dx - dx1 * dy;
            if (dy < 0) {
                dist_num = -dist_num;
            }
            counter += dist_num > 0;
        }
        res = std::sqrt(min_dist_num / min_dist_denom);
        if (counter % 2 == 0) {
            res = -res;
        }
    }

    return res;
}

template<typename VectorType>
std::vector<VectorType> GeometryUtils::simplifyCurve(const std::vector<VectorType>& curve, double epsilon) {
    // Douglas-Peuker
    std::vector<VectorType> dp_path;
    if (curve.size() < 3) {
        return curve;
    }

    VectorType start = curve[0];
    VectorType end = curve.back();
    VectorType vec_line = end - start;
    //if (vec_line.norm() < 1e-10) {
    //    return { start };
    //}

    int index_target = 0;
    bool get_index = false;
    double max_l = 0.0;
    for (int i = 1; i < curve.size() - 1; ++i) {
        VectorType vec_sp = curve[i] - start;
        double l = std::fabs(vec_sp(0) * vec_line(1) - vec_sp(1) * vec_line(0)) / vec_line.norm();
        if (l >= max_l && l > epsilon) {
            max_l = l;
            index_target = i;
            get_index = true;
        }
    }

    if (get_index) {
        std::vector<VectorType> left_segment(curve.begin(), curve.begin() + index_target + 1);
        std::vector<VectorType> right_segment(curve.begin() + index_target, curve.end());
        std::vector<VectorType> dp_path_l = simplifyCurve(left_segment, epsilon);
        std::vector<VectorType> dp_path_r = simplifyCurve(right_segment, epsilon);

        dp_path = dp_path_l;
        dp_path.insert(dp_path.end(), dp_path_r.begin() + 1, dp_path_r.end());
    }
    else {
        dp_path.push_back(start);
        dp_path.push_back(end);
    }
    return dp_path;
}

template std::vector<Eigen::Vector2d>
GeometryUtils::simplifyCurve<Eigen::Vector2d>(const std::vector<Eigen::Vector2d>& curve, double epsilon);

template std::vector<Eigen::Vector2i>
GeometryUtils::simplifyCurve<Eigen::Vector2i>(const std::vector<Eigen::Vector2i>& curve, double epsilon);

bool GeometryUtils::calc_line_cross_polygon(const GU::Line& line, const std::vector<Eigen::Vector2d>& polygon) {
    //if ((pointInPolygon(Eigen::Vector2d(line.startX, line.startY), polygon, false) == 1) || (pointInPolygon(Eigen::Vector2d(line.endX, line.endY), polygon, false) == 1)) {
    //    return true;
    //}

    //int len = polygon.size();
    //Eigen::Vector2d v0, v = polygon[len - 1];
    //for (int i = 0; i < len; ++i) {
    //    v0 = v;
    //    v = polygon[i];
    //    GU::Intersection inc = calc_linesIntersect(GU::Line(v0(0), v0(1), v(0), v(1)), line);
    //    if (!inc.isParallel&&(inc.alongA > 1e-10 && inc.alongA < 1 - 1e-10) && (inc.alongB > 1e-10 && inc.alongB < 1 - 1e-10)) {
    //        return true;
    //    }
    //}

    return false;
}

void GeometryUtils::sort_polygon_vertices_ccw(std::vector<Eigen::Vector2d>& boundary) {
    int len = boundary.size();

    double Sx2 = 0.0;
    for (int i = 0; i < boundary.size(); ++i) {
        Sx2 += (boundary[(i + 1) % len](0) - boundary[i](0)) * (boundary[(i + 1) % len](1) + boundary[i](1));
    }
    if (Sx2 > 0) {
        std::reverse(boundary.begin(), boundary.end());
    }

}

std::vector<Eigen::Vector2d> GeometryUtils::inflatePolygon(const std::vector<Eigen::Vector2d>& boundary, const double offset) {
    std::vector<Eigen::Vector2d> valid_point;
    // 计算顶点偏移
    int N = boundary.size();
    std::vector<Eigen::Vector2d> off_points;
    for (int i = 0; i < N; ++i) {
        Eigen::Vector2d left_evec = (boundary[((i - 1) + N) % N] - boundary[i]).normalized();
        Eigen::Vector2d right_evec = (boundary[(i + 1) % N] - boundary[i]).normalized();
        double cos_theta = left_evec.dot(right_evec);
        double half_sin_theta = std::sqrt((1 - cos_theta) / 2.0);
        Eigen::Vector2d offset_pos = (left_evec + right_evec).normalized() * (offset / half_sin_theta) + boundary[i];
        Eigen::Vector2d left_rvec = (boundary[i] - boundary[((i - 1) + N) % N]).normalized();
        Eigen::Vector2d right_rvec = (boundary[(i + 1) % N] - boundary[i]).normalized();
        double cross_product = left_rvec(0) * right_rvec(1) - left_rvec(1) * right_rvec(0);
        if (cross_product < 0) {
            offset_pos = (left_evec + right_evec).normalized() * (-1.0 * offset / half_sin_theta) + boundary[i];
        }
        off_points.push_back(offset_pos);

    }

    // 标记无效边
    std::vector<GU::Line> off_edges;
    std::vector<bool> invalid_edge_mask(N, false);
    int invalid_count = 0;
    for (int i = 0; i < off_points.size(); ++i) {
        Eigen::Vector2d ori_vec = (boundary[(i + 1) % N] - boundary[i]).normalized();
        Eigen::Vector2d off_vec = (off_points[(i + 1) % N] - off_points[i]).normalized();
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

    //// 处理无效边 情况1
    //for (int i = 0; i < invalid_edge_mask.size(); ++i) {
    //    GU::Line invalid_edge;
    //    if (invalid_edge_mask[i] && !invalid_edge_mask[((i - 1) + N) % N] && !invalid_edge_mask[(i + 1) % N]) {
    //        std::cout << " process case1: " << std::endl;
    //        Eigen::Vector2d inc;
    //        off_edges[((i - 1) + N) % N].calc_intersection(off_edges[(i + 1) % N], inc);
    //        off_edges[((i - 1) + N) % N] = SPline2D(off_edges[((i - 1) + N) % N].start_point, inc);
    //        off_edges[(i + 1) % N] = SPline2D(inc, off_edges[(i + 1) % N].end_point);
    //    }
    //}



    return valid_point;
}

std::vector<std::vector<Eigen::Vector2d>> GeometryUtils::calc_AnotB(const std::vector<Eigen::Vector2d>& region_0, const std::vector<Eigen::Vector2d>& region_1) {
    std::vector<std::vector<Eigen::Vector2d>> res;

    Intersecter intersecter;
    std::vector<Segment*> seg_primary, seg_secondary;
    intersecter.addRegion(region_0, true);
    std::vector<Segment*> res_A = intersecter.calculate(true);
    for (auto s : res_A) {
        Segment* seg = new Segment(s->start, s->end);
        seg->id = s->id;
        seg->myFill = s->myFill;
        seg->otherFill = s->otherFill;
        seg_primary.push_back(seg);
        all_segment_ptr_.push_back(seg);
        std::cout << "seg:" << s->start.transpose() << " -> " << s->end.transpose() <<
            " my:" << FillInfoToString(s->myFill.above) << ", " << FillInfoToString(s->myFill.below) << std::endl;
    }
    intersecter.reset();

    intersecter.addRegion(region_1, true);
    std::vector<Segment*> res_B = intersecter.calculate(true);
    for (auto s : res_B) {
        Segment* seg = new Segment(s->start, s->end);
        seg->id = s->id;
        seg->myFill = s->myFill;
        seg->otherFill = s->otherFill;
        seg_secondary.push_back(seg);
        all_segment_ptr_.push_back(seg);
        std::cout << "seg:" << s->start.transpose() << " -> " << s->end.transpose() <<
            " my:" << FillInfoToString(s->myFill.above) << ", " << FillInfoToString(s->myFill.below) << std::endl;
    }
    intersecter.reset();

    for (auto s : seg_primary) {
        intersecter.eventAddSegment(s, true);
    }
    for (auto s : seg_secondary) {
        intersecter.eventAddSegment(s, false);
    }
    std::vector<Segment*> res_C = intersecter.calculate(false);
    for (auto s : res_C) {
        std::cout << "seg:" << s->start.transpose() << " -> " << s->end.transpose() << std::endl;
    }

    SegmentSelector selctor;
    std::vector<Segment> sel_segments;
    if (intersecter.inc_count_ > 1) {
        sel_segments = selctor.select(selctor.difference(), res_C);
    }
    else {
        sel_segments = selctor.select(selctor.difference2(), res_C);
    }
    std::vector<std::deque<Eigen::Vector2d>> polygons = selctor.segmentChain(sel_segments);
    for (auto polygon : polygons) {
        if (polygon.size() > 2) {
            std::vector<Eigen::Vector2d> polygon_vector(polygon.begin(), polygon.end());
            res.push_back(polygon_vector);
        }
    }

    return res;
}


