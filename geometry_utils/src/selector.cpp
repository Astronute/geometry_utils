#include "selector.h"


std::vector<Segment> SegmentSelector::select(const Eigen::Matrix4i& selector, std::vector<Segment*> segments) {
    std::vector<Segment> res;
    for (int i = 0; i < segments.size(); ++i) {
        int idx = (segments[i]->myFill.above == FillState::Filled ? 8 : 0) +
            (segments[i]->myFill.below == FillState::Filled ? 4 : 0) +
            (segments[i]->otherFill.above == FillState::Filled ? 2 : 0) +
            (segments[i]->otherFill.below == FillState::Filled ? 1 : 0);
        int fillinfo = selector(idx / 4, idx % 4);
        if (fillinfo != 0) {
            Segment seg = Segment(segments[i]->start, segments[i]->end);
            if (fillinfo == 1) {
                seg.myFill.above = FillState::Filled;
                seg.myFill.below = FillState::Empty;
            }
            else if (fillinfo == 2) {
                seg.myFill.above = FillState::Empty;
                seg.myFill.below = FillState::Filled;
            }
            res.push_back(seg);
        }
    }
    return res;
}


std::vector<std::deque<GU::Point>> SegmentSelector::segmentChain(const std::vector<Segment>& segments) {
    std::vector<std::deque<GU::Point>> polygons;
    std::vector<std::deque<GU::Point>> chains;

    // |p1-p2|^2
    auto norm2 = [](const GU::Point& p1, const GU::Point& p2) -> double {
        return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
        };
    // |p1-p2|
    auto norm = [](const GU::Point& p1, const GU::Point& p2) -> double {
        return std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
        };
    // (p1-p2)归一化
    auto normalized = [norm](const GU::Point& p1, const GU::Point& p2) ->GU::Point {
        double L = norm(p1, p2);
        if (L < 1e-10) {
            std::cout << " segmentChain error: zero-length segment detected " << std::endl;
            return GU::Point(FLT_MAX, FLT_MAX);
        }
        return GU::Point((p1.x - p2.x)/L, (p1.y - p2.y)/L);
        };


    for (auto seg : segments) {
        GU::Point seg_start = seg.start;
        GU::Point seg_end = seg.end;
        if (norm2(seg_start, seg_end) < 1e-10) {
            std::cout << " segmentChain error: zero-length segment detected " << std::endl;
            return polygons;
        }

        std::shared_ptr<GU::MatchPoint> first_match = std::make_shared<GU::MatchPoint>(0, false, false);
        std::shared_ptr<GU::MatchPoint> second_match = std::make_shared<GU::MatchPoint>(0, false, false);
        std::shared_ptr<GU::MatchPoint> next_match = first_match;
        auto setMatch = [&](int index, bool m_chain_head_start, bool m_seg_start) {
            next_match->index = index;
            next_match->matchs_chain_head = m_chain_head_start;
            next_match->matches_seg_start = m_seg_start;
            if (next_match == first_match) {
                next_match = second_match; // first_match值填充并将next_match指向切换到second_match
                return false;
            }
            next_match = nullptr;
            return true;
            };

        for (int i = 0; i < chains.size(); ++i) {
            GU::Point chain_head = chains[i].front();
            GU::Point chain_tail = chains[i].back();
            if (norm(chain_head, seg_start) < 1e-10) {
                // 线段起点与链头匹配
                if (setMatch(i, true, true)) {
                    break;
                }
            }
            else if (norm(chain_head, seg_end) < 1e-10) {
                if (setMatch(i, true, false)) {
                    break;
                }
            }
            else if (norm(chain_tail, seg_start) < 1e-10) {
                if (setMatch(i, false, true)) {
                    break;
                }
            }
            else if (norm(chain_tail, seg_end) < 1e-10) {
                if (setMatch(i, false, false)) {
                    break;
                }
            }
        }

        // 没有匹配到链
        if (next_match == first_match) {
            std::deque<GU::Point> chain{ seg_start , seg_end };
            chains.push_back(chain);
            continue;
        }
        // 匹配到一条链
        if (next_match == second_match) {
            int index = first_match->index;
            GU::Point pt = first_match->matches_seg_start ? seg_end : seg_start;
            bool addToHead = first_match->matchs_chain_head;

            GU::Point grow = addToHead ? chains[index][0] : chains[index][chains[index].size() - 1]; // 前插/后插位置的元素
            GU::Point grow2 = addToHead ? chains[index][1] : chains[index][chains[index].size() - 2];
            GU::Point oppo = addToHead ? chains[index][chains[index].size() - 1] : chains[index][0];
            GU::Point oppo2 = addToHead ? chains[index][chains[index].size() - 2] : chains[index][1];

            GU::Point vec_c = normalized(grow2, grow);
            GU::Point vec_p = normalized(grow, pt);
            double cross1 = vec_c(0) * vec_p(1) - vec_c(1) * vec_p(0);
            std::cout << "cross : " << cross1 << std::endl;
            // 线段匹配点与链的头/尾共线，删除头尾点
            if (std::fabs(cross1) < 1e-1) {
                if (addToHead) {
                    chains[index].pop_front();
                }
                else {
                    chains[index].pop_back();
                }
                grow = grow2;
            }
            // 线段匹配点和链上匹配点的另一头相接(多边形闭合)
            if (norm(oppo, pt) < 1e-10) {
                GU::Point vec_c = normalized(oppo2, oppo);
                GU::Point vec_p = normalized(grow, oppo);
                double cross2 = vec_c(0) * vec_p(1) - vec_c(1) * vec_p(0);
                if (std::fabs(cross2) < 1e-10) {
                    if (addToHead) {
                        chains[index].pop_back();
                    }
                    else {
                        chains[index].pop_front();
                    }
                }
                polygons.push_back(chains[index]);
                chains.erase(chains.begin() + index);
                continue;
            }
            // 匹配到的链未闭合
            if (addToHead) {
                chains[index].push_front(pt);
            }
            else {
                chains[index].push_back(pt);
            }
            continue;
        }

        // 匹配到两条链
        int first_chain_idx = first_match->index;
        int second_chain_idx = second_match->index;
        bool reverse_first = chains[first_chain_idx].size() < chains[second_chain_idx].size();
        if (first_match->matchs_chain_head) {
            if (second_match->matchs_chain_head) {
                if (reverse_first) {
                    std::reverse(chains[first_chain_idx].begin(), chains[first_chain_idx].end());
                    chains[first_chain_idx].insert(chains[first_chain_idx].end(), chains[second_chain_idx].begin(), chains[second_chain_idx].end());
                    chains.erase(chains.begin() + second_chain_idx);
                }
                else {
                    std::reverse(chains[second_chain_idx].begin(), chains[second_chain_idx].end());
                    chains[second_chain_idx].insert(chains[second_chain_idx].end(), chains[first_chain_idx].begin(), chains[first_chain_idx].end());
                    chains.erase(chains.begin() + first_chain_idx);
                }
            }
            else {
                chains[second_chain_idx].insert(chains[second_chain_idx].end(), chains[first_chain_idx].begin(), chains[first_chain_idx].end());
                chains.erase(chains.begin() + first_chain_idx);
            }
        }
        else {
            if (second_match->matchs_chain_head) {
                chains[first_chain_idx].insert(chains[first_chain_idx].end(), chains[second_chain_idx].begin(), chains[second_chain_idx].end());
                chains.erase(chains.begin() + second_chain_idx);
            }
            else {
                if (reverse_first) {
                    std::reverse(chains[first_chain_idx].begin(), chains[first_chain_idx].end());
                    chains[second_chain_idx].insert(chains[second_chain_idx].end(), chains[first_chain_idx].begin(), chains[first_chain_idx].end());
                    chains.erase(chains.begin() + first_chain_idx);
                }
                else {
                    std::reverse(chains[second_chain_idx].begin(), chains[second_chain_idx].end());
                    chains[first_chain_idx].insert(chains[first_chain_idx].end(), chains[second_chain_idx].begin(), chains[second_chain_idx].end());
                    chains.erase(chains.begin() + second_chain_idx);
                }
            }
        }
    }

    return polygons;
}