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


std::vector<std::deque<Eigen::Vector2d>> SegmentSelector::segmentChain(const std::vector<Segment>& segments) {
    std::vector<std::deque<Eigen::Vector2d>> polygons;
    std::vector<std::deque<Eigen::Vector2d>> chains;

    for (auto seg : segments) {
        Eigen::Vector2d seg_start = seg.start;
        Eigen::Vector2d seg_end = seg.end;
        std::cout << "chain start: " << seg_start.transpose() << " -> " << seg_end.transpose() << std::endl;
        if ((seg_start - seg_end).norm() < 1e-10) {
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
            Eigen::Vector2d chain_head = chains[i].front();
            Eigen::Vector2d chain_tail = chains[i].back();
            if ((chain_head - seg_start).norm() < 1e-10) {
                // 线段起点与链头匹配
                if (setMatch(i, true, true)) {
                    break;
                }
            }
            else if ((chain_head - seg_end).norm() < 1e-10) {
                if (setMatch(i, true, false)) {
                    break;
                }
            }
            else if ((chain_tail - seg_start).norm() < 1e-10) {
                if (setMatch(i, false, true)) {
                    break;
                }
            }
            else if ((chain_tail - seg_end).norm() < 1e-10) {
                if (setMatch(i, false, false)) {
                    break;
                }
            }
        }

        // 没有匹配到链
        if (next_match == first_match) {
            std::deque<Eigen::Vector2d> chain{ seg_start , seg_end };
            chains.push_back(chain);
            continue;
        }
        // 匹配到一条链
        if (next_match == second_match) {
            int index = first_match->index;
            Eigen::Vector2d pt = first_match->matches_seg_start ? seg_end : seg_start;
            bool addToHead = first_match->matchs_chain_head;

            Eigen::Vector2d grow = addToHead ? chains[index][0] : chains[index][chains[index].size() - 1]; // 前插/后插位置的元素
            Eigen::Vector2d grow2 = addToHead ? chains[index][1] : chains[index][chains[index].size() - 2];
            Eigen::Vector2d oppo = addToHead ? chains[index][chains[index].size() - 1] : chains[index][0];
            Eigen::Vector2d oppo2 = addToHead ? chains[index][chains[index].size() - 2] : chains[index][1];

            Eigen::Vector2d vec_c = (grow2 - grow).normalized();
            Eigen::Vector2d vec_p = (grow - pt).normalized();
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
            if ((oppo - pt).norm() < 1e-10) {
                Eigen::Vector2d vec_c = (oppo2 - oppo).normalized();
                Eigen::Vector2d vec_p = (grow - oppo).normalized();
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