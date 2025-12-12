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


GU::Intersection GeometryUtils::calc_linesIntersect(const Eigen::Vector2d &a0, const Eigen::Vector2d &a1, const Eigen::Vector2d &b0, const Eigen::Vector2d &b1){
    Eigen::Vector2d A_vec = a1 - a0;
    Eigen::Vector2d B_vec = b1 - b0;

    double cross = A_vec(0)*B_vec(1) - A_vec(1)*B_vec(0);
    GU::Intersection inc;
    inc.cross = cross;
    if(cross == 0){
        std::cout << " GeometryUtils calc_linesIntersect: || " << std::endl;
        return inc;
    }

    Eigen::Matrix2d A(2, 2);
    Eigen::Vector2d B(2);
    A << A_vec(0), -1.0*(B_vec(0)),
         A_vec(1), -1.0*(B_vec(1));
    B << b0(0) - a0(0),
         b0(1) - a0(1);
    Eigen::Vector2d C = A.partialPivLu().solve(B); // colPivHouseholderQr()
    inc.alongA = C(0);
    inc.alongB = C(1);
    inc.p << a0(0) + inc.alongA*(A_vec(0)),
             a0(1) + inc.alongA*(A_vec(1));
    
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
    std::vector<Segment> sel_segments = selctor.select(selctor.difference(), res_C);
    std::vector<std::deque<Eigen::Vector2d>> polygons = selctor.segmentChain(sel_segments);
    for (auto polygon : polygons) {
        std::vector<Eigen::Vector2d> polygon_vector(polygon.begin(), polygon.end());
        res.push_back(polygon_vector);
    }

    return res;
}


