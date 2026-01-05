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

// 将多边形顶点由数组表示转为单向链表表示
GU::Node* GeometryUtils::upgradePolygon(const std::vector<GU::Point> &p){
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
        std::cout << p->p << std::endl;
        p = p->next;
    } while (p!=root);
}

// 判断两线段是否相交（不包含相交于端点，线段平行重叠）
bool GeometryUtils::isIntersection(const GU::Line& lineA, const GU::Line& lineB) {
    double dx1, dy1, dx2, dy2;
    // vec line-AB:(dx1, dy1)
    dx1 = lineA.endX - lineA.startX;
    dy1 = lineA.endY - lineA.startY;
    // vec line-CD:(dx2, dy2)
    dx2 = lineB.endX - lineB.startX;
    dy2 = lineB.endY - lineB.startY;

    if (std::max(lineA.startX, lineA.endX) < std::min(lineB.startX, lineB.endX) || std::min(lineA.startX, lineA.endX) > std::max(lineB.startX, lineB.endX)) {
        return false;
    }
    if (std::max(lineA.startY, lineA.endY) < std::min(lineB.startY, lineB.endY) || std::min(lineA.startY, lineA.endY) > std::max(lineB.startY, lineB.endY)) {
        return false;
    }

    double cx1, cy1, cx2, cy2, cx3, cy3, cx4, cy4;
    // vec AC:(cx1, cy1)
    cx1 = lineB.startX - lineA.startX;
    cy1 = lineB.startY - lineA.startY;
    // vec AD:(cx2, cy2)
    cx2 = lineB.endX - lineA.startX;
    cy2 = lineB.endY - lineA.startY;
    // vec CA:(cx3, cy3)
    cx3 = lineA.startX - lineB.startX;
    cy3 = lineA.startY - lineB.startY;
    // vec CB:(cx4, cy4)
    cx4 = lineA.endX - lineB.startX;
    cy4 = lineA.endY - lineB.startY;

    if ((cx1*dy1-cy1*dx1) * (cx2 * dy1-cy2*dx1) >= 0 || (cx3*dy2-cy3*dx2) * (cx4*dy2-cy4*dx2) >= 0) {
        return false;
    }

    return true;
}

// 计算两线段的交点
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
    inc.p = GU::Point(lineA.startX + inc.alongA * dx1, lineA.startY + inc.alongA * dy1);

    return inc;
}

double GeometryUtils::calc_pointSegmentDistance(const double x, const double y, const GU::Line& line) {
    double dx1, dy1, dx2, dy2, dx3, dy3, dot1, dot2, ll, cross;
    dx1 = line.endX - line.startX;
    dy1 = line.endY - line.startY;
    dx2 = x - line.startX;
    dy2 = y - line.startY;
    dx3 = x - line.endX;
    dy3 = y - line.endY;
    dot1 = dx1 * dx2 + dy1 * dy2;
    dot2 = (-1.0 * dx1) * dx3 + (-1.0 * dy1) * dy3;
    ll = std::sqrt(dx1 * dx1 + dy1 * dy1);

    if (ll < 1e-10) {
        return std::sqrt(dx2 * dx2 + dy2 * dy2);
    }
    if (dot1 < 1e-10) {
        return std::sqrt(dx2 * dx2 + dy2 * dy2);
    }
    if (dot2 < 1e-10) {
        return std::sqrt(dx3 * dx3 + dy3 * dy3);
    }

    return std::fabs((dx1 * dy2 - dx2 * dy1) / ll);
}

GU::Node* GeometryUtils::nextNonIntersection(GU::Node* n){
    GU::Node* node = n;
    do{
        node = node->next;
    }while(n->is_intersection);

    return node;
}

// 计算点到多边形的距离（0:边上 >0:内 <0:外）
double GeometryUtils::pointInPolygon(const GU::Point&p, GU::Node* root){
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
        GU::Point v0_v1 = GU::Point(Pnext->p.x - Pcur->p.x, Pnext->p.y - Pcur->p.y);
        GU::Point v0_p = GU::Point(p.x - Pcur->p.x, p.y - Pcur->p.y);
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


// 曲线抽稀
std::vector<GU::Point> GeometryUtils::simplifyCurve(const std::vector<GU::Point>& curve, double epsilon) {
    // Douglas-Peuker
    std::vector<GU::Point> dp_path;
    if (curve.size() < 3) {
        return curve;
    }

    GU::Point start = curve[0];
    GU::Point end = curve.back();
    double dx1, dy1, norm1;
    // vec_line(dx1, dy1)
    dx1 = end.x - start.x;
    dy1 = end.y - start.y;
    norm1 = std::sqrt(dx1* dx1+ dy1* dy1);
    //if (vec_line.norm() < 1e-10) {
    //    return { start };
    //}

    int index_target = 0;
    bool get_index = false;
    double max_l = 0.0;
    for (int i = 1; i < curve.size() - 1; ++i) {
        double dx2, dy2;
        // vec_sp(dx2, dy2)
        dx2 = curve[i].x - start.x;
        dy2 = curve[i].y - start.y;
        double l = std::fabs(dx2 * dy1 - dy2 * dx1) / norm1;
        if (l >= max_l && l > epsilon) {
            max_l = l;
            index_target = i;
            get_index = true;
        }
    }

    if (get_index) {
        std::vector<GU::Point> left_segment(curve.begin(), curve.begin() + index_target + 1);
        std::vector<GU::Point> right_segment(curve.begin() + index_target, curve.end());
        std::vector<GU::Point> dp_path_l = simplifyCurve(left_segment, epsilon);
        std::vector<GU::Point> dp_path_r = simplifyCurve(right_segment, epsilon);

        dp_path = dp_path_l;
        dp_path.insert(dp_path.end(), dp_path_r.begin() + 1, dp_path_r.end());
    }
    else {
        dp_path.push_back(start);
        dp_path.push_back(end);
    }
    return dp_path;
}

//template std::vector<Eigen::Vector2d>
//GeometryUtils::simplifyCurve<Eigen::Vector2d>(const std::vector<Eigen::Vector2d>& curve, double epsilon);
//
//template std::vector<Eigen::Vector2i>
//GeometryUtils::simplifyCurve<Eigen::Vector2i>(const std::vector<Eigen::Vector2i>& curve, double epsilon);

// 判断线段是否与多边形有交点（不包括整条线段与边重叠）
bool GeometryUtils::calc_line_cross_polygon(const GU::Line& line, const std::vector<GU::Point>& polygon) {
    int len = polygon.size();
    if (len < 3) {
        return false;
    }
    double s_in = pointInPolygon(GU::Point(line.startX, line.startY), polygon, false);
    double end_in = pointInPolygon(GU::Point(line.endX, line.endY), polygon, false);
    if (s_in* end_in==-1) {
        return true;
    }

    
    GU::Point v0, v = polygon[len - 1];
    for (int i = 0; i < len; ++i) {
        v0 = v;
        v = polygon[i];
        GU::Intersection inc = calc_linesIntersect(GU::Line(v0.x, v0.y, v.x, v.y), line);
        if (!inc.isParallel&&(inc.alongA > 1e-10 && inc.alongA < 1 - 1e-10) && (inc.alongB > 1e-10 && inc.alongB < 1 - 1e-10)) {
            return true;
        }
    }

    return false;
}

// 将多边形顶点按照逆时针排列
void GeometryUtils::sort_polygon_vertices_ccw(std::vector<GU::Point>& boundary) {
    int len = boundary.size();

    double Sx2 = 0.0;
    for (int i = 0; i < boundary.size(); ++i) {
        Sx2 += (boundary[(i + 1) % len](0) - boundary[i](0)) * (boundary[(i + 1) % len](1) + boundary[i](1));
    }
    if (Sx2 > 0) {
        std::reverse(boundary.begin(), boundary.end());
    }

}

// 计算多边形面积
double GeometryUtils::polygonArea(const std::vector<GU::Point>& region) {
    int len = region.size();

    double Sx2 = 0.0;
    for (int i = 0; i < region.size(); ++i) {
        Sx2 += (region[(i + 1) % len](0) - region[i](0)) * (region[(i + 1) % len](1) + region[i](1));
        //std::cout << Sx2 << std::endl;
    }
    return Sx2 / 2.0;
}

std::vector<GU::Point> GeometryUtils::polygonFilter(const std::vector<GU::Point>& polygon, double gap) {
    std::vector<GU::Point> filter_vertex;
    
    int len = polygon.size();
    if (len < 3) {
        std::cout << " Invalid polygon: fewer than 3 vertices " << std::endl;
        return filter_vertex;
    }
    // |p1-p2|^2
    auto norm2 = [](const GU::Point& p1, const GU::Point& p2) -> double {
        return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
        };
    // |p1-p2|
    auto norm = [](const GU::Point& p1, const GU::Point& p2) -> double {
        return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
        };


    GU::Node* root = upgradePolygon(polygon);
    GU::Node* cur = root;
    double dx1, dy1, dx2, dy2, cross;
    do {
        cur = cur->next;
        dx1 = cur->prev->p.x - cur->p.x; dy1 = cur->prev->p.y - cur->p.y;
        dx2 = cur->next->p.x - cur->p.x; dy2 = cur->next->p.y - cur->p.y;
        cross = dx1 * dy2 - dy1 * dx2;
        if (std::fabs(cross) < 1e-7 || norm(cur->p, cur->prev->p) < gap) {
            cur->prev->next = cur->next;
            cur->next->prev = cur->prev;
        }
    } while (cur != root);

    GU::Node* p_vertex = root;
    do {
        filter_vertex.push_back(p_vertex->p);
        p_vertex = p_vertex->next;
    } while (p_vertex != root);
    for (auto node : all_polygon_node_) {
        delete node;
    }
    all_polygon_node_.clear();

    if (filter_vertex.size() < 3) {
        std::cout << " Invalid polygon: fewer than 3 vertices " << std::endl;
        filter_vertex.clear();
        return filter_vertex;
    }

    if (calc_geometryIntersection(filter_vertex).size() > 0) {
        std::cout << " Invalid polygon: self intersection " << std::endl;
        filter_vertex.clear();
        return filter_vertex;
    }
    double area = polygonArea(filter_vertex);
    if (area > 0) {
        std::reverse(filter_vertex.begin(), filter_vertex.end());
    }
    std::cout << "area: " << area << std::endl;

    return filter_vertex;
}

// 计算多边形自交点
std::vector<GU::Intersection> GeometryUtils::calc_geometryIntersection(const std::vector<GU::Point>& region) {

    Intersecter intersecter;
    intersecter.addRegion(region, true);

    return intersecter.calcIntersect();
}

// 多边形布尔运算：A - B
std::vector<std::vector<GU::Point>> GeometryUtils::calc_AnotB(const std::vector<GU::Point>& region_0, const std::vector<GU::Point>& region_1) {
    std::vector<std::vector<GU::Point>> res;

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
        DEBUG_PRINT("seg:" << s->start << " -> " << s->end <<
            " my:" << FillInfoToString(s->myFill.above) << ", " << FillInfoToString(s->myFill.below));
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
        DEBUG_PRINT("seg:" << s->start << " -> " << s->end <<
            " my:" << FillInfoToString(s->myFill.above) << ", " << FillInfoToString(s->myFill.below));
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
        DEBUG_PRINT("seg:" << s->start << " -> " << s->end);
    }

    SegmentSelector selctor;
    std::vector<Segment> sel_segments;
    if (intersecter.inc_count_ > 1) {
        sel_segments = selctor.select(selctor.difference(), res_C);
    }
    else {
        sel_segments = selctor.select(selctor.difference2(), res_C);
    }
    std::vector<std::deque<GU::Point>> polygons = selctor.segmentChain(sel_segments);
    for (auto polygon : polygons) {
        if (polygon.size() > 2) {
            std::vector<GU::Point> polygon_vector(polygon.begin(), polygon.end());
            res.push_back(polygon_vector);
        }
    }

    return res;
}


