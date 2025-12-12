#include "intersecter.h"


Intersecter::Intersecter(){
	
}

Intersecter::~Intersecter(){
	reset();
}

bool Intersecter::pointsSame(const Eigen::Vector2d& a, const Eigen::Vector2d& b) {
	return std::fabs(a(0) - b(0)) < 1e-10 && std::fabs(a(1) - b(1)) < 1e-10;
}

// 判断点p是否在线段left,right之间
bool Intersecter::pointBetween(const Eigen::Vector2d& p, const Eigen::Vector2d& left, const Eigen::Vector2d& right) {
	Eigen::Vector2d line_vec = right - left;
	Eigen::Vector2d vec = p - left;

	double dot = line_vec.dot(vec);
	if (dot < 1e-10) {
		return false;
	}

	double sqlen = line_vec.dot(line_vec);
	if (dot - sqlen > -1e-10) {
		return false;
	}
	return true;
}

// 0:相等 -1:p1在p2之前 1:p1在p2之后
int Intersecter::pointsCompare(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
	if (std::fabs(p1(0) - p2(0)) < 1e-10) {
		return std::fabs(p1(1) - p2(1)) < 1e-10 ? 0 : p1(1) < p2(1) ? -1 : 1;
	}
	return p1(0) < p2(0) ? -1 : 1;
}

// p1_isstart:是否起点 p1_0:事件点 p1_1:事件点所在段终点
// p2_isstart:是否起点 p2_0:事件点 p2_1:事件点所在段终点
int Intersecter::eventsCompare(const bool& p1_isstart, const Eigen::Vector2d& p1_0, const Eigen::Vector2d& p1_1, 
	const bool& p2_isstart, const Eigen::Vector2d& p2_0, const Eigen::Vector2d& p2_1) {
	// 按事件点坐标排序
	int comp = pointsCompare(p1_0, p2_0);
	if (comp != 0) {
		return comp;
	}
	// 事件点坐标相同，所在段另一个坐标相同
	if (pointsCompare(p1_1, p2_1) == 0) {
		return 0;
	}
	// 事件点坐标相同，优先处理终点事件
	if (p1_isstart != p2_isstart) {
		return p1_isstart ? 1 : -1;
	}
	// 事件点坐标相同，类型相同，优先处理线段位于下方的事件点
	if (p1_isstart) {
		Eigen::Vector2d vec1 = p1_1 - p1_0;
		Eigen::Vector2d vec2 = p2_1 - p2_0;
		return vec2(0) * vec1(1) - vec2(1) * vec1(0) >= 0 ? 1 : -1;
	}
	else {
		Eigen::Vector2d vec1 = p1_1 - p2_1;
		Eigen::Vector2d vec2 = p2_0 - p2_1;
		return vec2(0) * vec1(1) - vec2(1) * vec1(0) >= 0 ? 1 : -1;
	}
}

// ev1: 找出事件点下方的状态点对应的事件点ev2
// return 1:ev1的起点在ev2线段上方/ev1起点与ev2线段共线且ev1终点在ev2线段上方/ev1线段与ev2线段共线
// return -1:ev1起点在ev2线段下方/ev1起点与ev2线段共线且ev1终点在ev2线段下方
int Intersecter::statusCompare(EventNode* ev1, EventNode* ev2) {
	if (!ev1 || !ev2 || !ev1->seg || !ev2->seg) {
		throw std::invalid_argument("statusCompare: null pointer");
	}

	// 求局部上下关系
	Eigen::Vector2d ev2_vec = ev2->seg->end - ev2->seg->start;

	Eigen::Vector2d vec_ss = ev1->seg->start - ev2->seg->start;
	Eigen::Vector2d vec_se = ev1->seg->end - ev2->seg->start;
	double cross1 = ev2_vec(0) * vec_ss(1) - ev2_vec(1) * vec_ss(0);
	double cross2 = ev2_vec(0) * vec_se(1) - ev2_vec(1) * vec_se(0);
	if (std::fabs(cross1) < 1e-10) { // 共线
		if (std::fabs(cross2) < 1e-10) {
			return 1;
		}
	}
	// 不共线通过与扫描线交点判断局部上下关系
	double inc_t = (ev1->seg->start(0) - ev2->seg->start(0)) / (ev2->seg->end(0) - ev2->seg->start(0));
	double inc_y = ev2->seg->start(1) + inc_t * (ev2->seg->end(1) - ev2->seg->start(1));

	return inc_y > ev1->seg->start(1) ? -1 : 1;
}

void Intersecter::eventAdd(EventNode* const root, EventNode* const node, const Eigen::Vector2d other_pt) {
	EventNode* last = root;
	EventNode* here = root->next;
	while (here != nullptr) {
		int comp = eventsCompare(node->isStart, node->pos, other_pt, here->isStart, here->pos, here->other->pos);
		if (comp == -1) {
			node->next = here;
			node->prev = here->prev;
			here->prev->next = node;
			here->prev = node;
			return ;
		}
		last = here;
		here = here->next;
	}
	last->next = node;
	node->prev = last;
	node->next = nullptr;
}

GU::Intersection Intersecter::linesIntersection(const Eigen::Vector2d& a0, const Eigen::Vector2d& a1, const Eigen::Vector2d& b0, const Eigen::Vector2d& b1) {
	Eigen::Vector2d A_vec = a1 - a0;
	Eigen::Vector2d B_vec = b1 - b0;

	GU::Intersection inc;
	double cross_ab = A_vec(0) * B_vec(1) - A_vec(1) * B_vec(0);
	inc.cross = cross_ab;
	if (std::fabs(cross_ab) < 1e-10) {
		return inc;
	}
	Eigen::Matrix2d A(2, 2);
	Eigen::Vector2d B(2);
	A << A_vec(0), -1.0 * (B_vec(0)),
		A_vec(1), -1.0 * (B_vec(1));
	B << b0(0) - a0(0),
		b0(1) - a0(1);
	Eigen::Vector2d C = A.partialPivLu().solve(B); // colPivHouseholderQr()
	inc.alongA = C(0);
	inc.alongB = C(1);
	inc.p << a0(0) + inc.alongA * (A_vec(0)),
		a0(1) + inc.alongA * (A_vec(1));

	return inc;
}

// 当前 ev1(a1-a2), above/below: ev2(b1-b2)
// return: 
// nullptr: 平行不共线/首尾相接/不平行相交 （没有交点）
// ev2: 共线完全重叠/共线有重叠且起点对齐 切割后的ev2(ev2与ev1的重叠部分)
EventNode* Intersecter::eventsIntersection(EventNode* ev1, EventNode* ev2) {
	GU::Intersection inc = linesIntersection(ev1->seg->start, ev1->seg->end, ev2->seg->start, ev2->seg->end);

	// 处理平行线
	if (std::fabs(inc.cross) < 1e-10) {	
		Eigen::Vector2d ev1_seg_vec = ev1->seg->end - ev1->seg->start;
		Eigen::Vector2d vec = ev1->seg->end - ev2->seg->start;
		double cross = ev1_seg_vec(0) * vec(1) - ev1_seg_vec(1) * vec(0);
		// 平行不共线
		if (!(std::fabs(cross) < 1e-10)) { 
			return nullptr;
		}
		// 共线首位相接
		if (pointsSame(ev1->seg->start, ev2->seg->end) || pointsSame(ev1->seg->end, ev2->seg->start)) {
			return nullptr;
		}

		bool a1_equ_b1 = pointsSame(ev1->seg->start, ev2->seg->start);
		bool a2_equ_b2 = pointsSame(ev1->seg->end, ev2->seg->end);
		// 共线重叠
		if (a1_equ_b1 && a2_equ_b2) {
			return ev2;
		}
		bool a1_between = !a1_equ_b1 && pointBetween(ev1->seg->start, ev2->seg->start, ev2->seg->end);
		bool a2_between = !a2_equ_b2 && pointBetween(ev1->seg->end, ev2->seg->start, ev2->seg->end);

		// 共线有重叠段且起点对齐
		if (a1_equ_b1) {
			if (a2_between) {
				//  ev2(b1)----------(b2)
				//  ev1(a1)---(a2)
				// 增加a2---(b2)线段端点事件，返回重叠段(b1)---(a2)
				Segment* new_seg = new Segment(ev1->seg->end, ev2->seg->end);
				new_seg->myFill = ev2->seg->myFill;
				//new_seg.otherFill = nullptr;
				all_seg_node.push_back(new_seg);

				event_list.detach(ev2->other); // other从双向链表中断开链接
				ev2->seg->end = ev1->seg->end;
				ev2->other->pos = ev1->seg->end; // 更新other坐标
				eventAdd(event_list.root, ev2->other, ev2->pos); // other重新建立链接

				// new event
				eventAddSegment(new_seg, ev2->primary);
			}
			else {
				//  ev2(b1)---(b2)
				//  ev1(a1)----------(a2)
				Segment* new_seg = new Segment(ev2->seg->end, ev1->seg->end);
				new_seg->myFill = ev1->seg->myFill;
				//new_seg.otherFill = nullptr;
				all_seg_node.push_back(new_seg);

				event_list.detach(ev1->other);
				ev1->seg->end = ev2->seg->end;
				ev1->other->pos = ev2->seg->end;
				eventAdd(event_list.root, ev1->other, ev1->pos);

				eventAddSegment(new_seg, ev1->primary);
			}
			return ev2; // 这里返回拆分后的线段是因为返回后需要删除重复线段，留下切割后的两段线段
		}
		else if (a1_between) {
			if (!a2_equ_b2) {
				if (a2_between) {
					//  ev2(b1)-----------------(b2)
					//  ev1       (a1)---(a2)
					Segment* new_seg = new Segment(ev1->seg->end, ev2->seg->end);
					new_seg->myFill = ev2->seg->myFill;
					//new_seg.otherFill = nullptr;
					all_seg_node.push_back(new_seg);

					event_list.detach(ev2->other);
					ev2->seg->end = ev1->seg->end;
					ev2->other->pos = ev1->seg->end;
					eventAdd(event_list.root, ev2->other, ev2->pos);

					eventAddSegment(new_seg, ev2->primary);
				}
				else {
					//  ev2(b1)----------(b2)
					//  ev1       (a1)----------(a2)
					Segment* new_seg = new Segment(ev2->seg->end, ev1->seg->end);
					new_seg->myFill = ev1->seg->myFill;
					//new_seg.otherFill = nullptr;
					all_seg_node.push_back(new_seg);

					event_list.detach(ev1->other);
					ev1->seg->end = ev2->seg->end;
					ev1->other->pos = ev2->seg->end;
					eventAdd(event_list.root, ev1->other, ev1->pos);

					eventAddSegment(new_seg, ev1->primary);
				}
			}
			//  ev2(b1)----------(b2)
			//  ev1       (a1)---(a2)
			Segment* new_seg = new Segment(ev1->seg->start, ev2->seg->end);
			new_seg->myFill = ev2->seg->myFill;
			//new_seg.otherFill = nullptr;
			all_seg_node.push_back(new_seg);

			event_list.detach(ev2->other);
			ev2->seg->end = ev1->seg->start;
			ev2->other->pos = ev1->seg->start;
			eventAdd(event_list.root, ev2->other, ev2->pos);

			eventAddSegment(new_seg, ev2->primary);
		}
	}
	else {
		// 将ev1断开
		if (inc.alongA >= 1e-10 && inc.alongA - 1 <= -1e-10) {
			std::cout << "                                                               inc: " << inc.p.transpose() << std::endl;
			if (inc.alongB > -1e-10 && inc.alongB < 1e-10) {
				// 交点在A中间B起点 A:event line B:below
				Segment* new_seg = new Segment(ev2->seg->start, ev1->seg->end);
				new_seg->myFill = ev1->seg->myFill;
				//new_seg.otherFill = nullptr;
				all_seg_node.push_back(new_seg);

				event_list.detach(ev1->other);
				ev1->seg->end = ev2->seg->start;
				ev1->other->pos = ev2->seg->start;
				eventAdd(event_list.root, ev1->other, ev1->pos);
				eventAddSegment(new_seg, ev1->primary);
			}
			else if (inc.alongB >= 1e-10 && inc.alongB <= 1-1e-10) {
				// 交点在A中间B中间，拆分事件线段，更新事件点other
				Segment* new_seg = new Segment(inc.p, ev1->seg->end);
				new_seg->myFill = ev1->seg->myFill;
				//new_seg.otherFill = nullptr;
				all_seg_node.push_back(new_seg);

				// 更新事件点终点
				event_list.detach(ev1->other);
				ev1->seg->end = inc.p;
				ev1->other->pos = inc.p;
				// 新建终点事件
				eventAdd(event_list.root, ev1->other, ev1->pos);
				// 新建多余线段事件
				eventAddSegment(new_seg, ev1->primary);
			}
			else if (inc.alongB > 1-1e-10 && inc.alongB <1+1e-10) {
				// 交点在A中间B终点 A:event line B:above/below
				Segment* new_seg = new Segment(ev2->seg->end, ev1->seg->end);
				new_seg->myFill = ev1->seg->myFill;
				//new_seg.otherFill = nullptr;
				all_seg_node.push_back(new_seg);

				event_list.detach(ev1->other);
				ev1->seg->end = ev2->seg->end;
				ev1->other->pos = ev2->seg->end;
				eventAdd(event_list.root, ev1->other, ev1->pos);
				eventAddSegment(new_seg, ev1->primary);
			}
		}
		// 将B断开
		if (inc.alongB >= 1e-10 && inc.alongB - 1 < -1e-10) {
			if (inc.alongA > -1e-10 && inc.alongA < 1e-10) {
				// 交点在B中间A的起点 A:event B:above
				Segment* new_seg = new Segment(ev1->seg->start, ev2->seg->end);
				new_seg->myFill = ev2->seg->myFill;
				//new_seg.otherFill = nullptr;
				all_seg_node.push_back(new_seg);

				event_list.detach(ev2->other);
				ev2->seg->end = ev1->seg->start;
				ev2->other->pos = ev1->seg->start;
				eventAdd(event_list.root, ev2->other, ev2->pos);
				eventAddSegment(new_seg, ev2->primary);
			}
			else if (inc.alongA >= 1e-10 && inc.alongA <= 1-1e-10) {
				// 交点在B中间A中间 拆分状态线段，更新状态线段other事件点，新增事件线段
				Segment* new_seg = new Segment(inc.p, ev2->seg->end);
				new_seg->myFill = ev2->seg->myFill;
				//new_seg.otherFill = nullptr;
				all_seg_node.push_back(new_seg);

				event_list.detach(ev2->other);
				ev2->seg->end = inc.p;
				ev2->other->pos = inc.p;
				eventAdd(event_list.root, ev2->other, ev2->pos);
				eventAddSegment(new_seg, ev2->primary);
			}
			else if (inc.alongA > 1-1e-10 && inc.alongA < 1+1e-10) {
				// 交点在B中间A的终点 A:event B:above/below
				Segment* new_seg = new Segment(ev1->seg->end, ev2->seg->end);
				new_seg->myFill = ev2->seg->myFill;
				//new_seg.otherFill = nullptr;
				all_seg_node.push_back(new_seg);

				event_list.detach(ev2->other);
				ev2->seg->end = ev1->seg->end;
				ev2->other->pos = ev1->seg->end;
				eventAdd(event_list.root, ev2->other, ev2->pos);
				eventAddSegment(new_seg, ev2->primary);
			}

		}
	}

	return nullptr;
}

void Intersecter::eventAddSegment(Segment* const seg, const bool &primary) {
	// add seg start
	EventNode* startnode = new EventNode();
	startnode->isStart = true;
	startnode->pos = seg->start;
	startnode->seg = seg;
	startnode->primary = primary;
	eventAdd(event_list.root, startnode, seg->end);
	all_event_node.push_back(startnode);

	// add seg end
	EventNode* endnode = new EventNode();
	startnode->other = endnode;
	endnode->isStart = false;
	endnode->pos = seg->end;
	endnode->seg = seg;
	endnode->primary = primary;
	endnode->other = startnode;
	eventAdd(event_list.root, endnode, startnode->pos);
	all_event_node.push_back(endnode);
}

void Intersecter::addRegion(const std::vector<Eigen::Vector2d>& region, const bool& primary) {
	Eigen::Vector2d pt1;
	Eigen::Vector2d pt2 = region[region.size()-1];
	for (int i = 0; i < region.size(); ++i) {
		// 段
		pt1 = pt2;
		pt2 = region[i];
		Segment* seg;
		if (pt1(0) == pt2(0)) {
			if (pt1(1) < pt2(1)) {
				seg = new Segment(pt1, pt2);
			}
			else {
				seg = new Segment(pt2, pt1);
			}
		}
		else if (pt1(0) < pt2(0)) {
			seg = new Segment(pt1, pt2);
		}
		else {
			seg = new Segment(pt2, pt1);
		}
		// 将段转为开始事件和结束事件
		eventAddSegment(seg, primary);
		all_seg_node.push_back(seg);
	}
}

EventNode* Intersecter::checkBothIntersections(EventNode* ev, EventNode* above, EventNode* below) {
	if (above) {
		std::cout << "above: " << above->pos.transpose() << " -> " << above->other->pos.transpose() << std::endl;
		EventNode* eve = eventsIntersection(ev, above);
		if (eve) {
			std::cout << "                                             Overlapping ev: " << eve->pos.transpose() << " -> " << eve->other->pos.transpose() << std::endl;
			return eve;
		}
	}
	if (below) {
		std::cout << "below: " << below->pos.transpose() << " -> " << below->other->pos.transpose() << std::endl;
		return eventsIntersection(ev, below);
	}
	return nullptr;
}

std::vector<Segment*> Intersecter::calculate(const bool& selfIntersection) {

	// 状态列表中只会插入起点事件点
	StatusList status_list = StatusList();
	std::vector<Segment*> segments;
	int iter_num = 0;
	while (!event_list.isEmpty()) {
		EventNode* ev = event_list.getHead();
		std::cout << "iter " << iter_num << std::endl;
		std::cout << "cur-----event list-----" << std::endl;
		std::cout << ">>";
		event_list.printList();
		std::cout << "-----------------------" << std::endl;
		iter_num++;
		if (ev->isStart) {
			// ev: 线段起点事件
			// 1、从状态集中搜索事件点上方和下方的事件点(空间位置,下方包括共线)
			StatusNode* prev = status_list.root;
			StatusNode* here = status_list.root->next;
			while (here != nullptr) {
				if (statusCompare(ev, here->ev) == 1) {
					break;
				}
				prev = here;
				here = here->next;
			}
			EventNode* above = prev == status_list.root ? nullptr : prev->ev;
			EventNode* below = here != nullptr ? here->ev : nullptr;

			// 2、计算事件点(起点)所在线段与上下状态线段的交点(空间位置)
			EventNode* eve = checkBothIntersections(ev, above, below);
			if (eve) {
				std::cout << "                                             Overlapping ev: " << eve->pos.transpose() << " -> " << eve->other->pos.transpose() << std::endl;
				if (!selfIntersection) {
					eve->seg->otherFill = ev->seg->myFill;
				}
				event_list.detach(ev->other);
				event_list.detach(ev);
			}

			if (event_list.getHead() != ev) {
				// something was inserted before us in the event queue, so loop back around and
				// process it before continuing
				std::cout << " ---------something was inserted before event_list---------- " << std::endl;
				std::cout << "------event list-------" << std::endl;
				event_list.printList();
				std::cout << "-----------------------" << std::endl;
				std::cout << "update   status list  " << std::endl;
				status_list.printList();
				std::cout << "-----------------------" << std::endl;
				std::cout << std::endl;
				continue;
			}

			// calculate fill flags 
			if (selfIntersection) {
				bool toggle;
				if (ev->seg->myFill.below == FillState::Unknown) {
					toggle = true;
				}
				else {
					toggle = ev->seg->myFill.above != ev->seg->myFill.below;
				}
				
				if (!below) {
					ev->seg->myFill.below = FillState::Empty;
				}
				else {
					ev->seg->myFill.below = below->seg->myFill.above;
				}
				if (toggle) {
					ev->seg->myFill.above = ev->seg->myFill.below == FillState::Empty ? FillState::Filled : ev->seg->myFill.below == FillState::Filled ? FillState::Empty : ev->seg->myFill.above;
				}
				else {
					ev->seg->myFill.above = ev->seg->myFill.below;
				}
			}
			else {
				if (ev->seg->otherFill.above == FillState::Unknown || ev->seg->otherFill.below == FillState::Unknown) {
					if (!below) {
						// 事件线段下方没有线段
						ev->seg->otherFill.above = FillState::Empty;
						ev->seg->otherFill.below = FillState::Empty;
					}
					else {
						if (ev->primary == below->primary) {
							ev->seg->otherFill.above = below->seg->otherFill.above;
							ev->seg->otherFill.below = below->seg->otherFill.above;
						}
						else {
							ev->seg->otherFill.above = below->seg->myFill.above;
							ev->seg->otherFill.below = below->seg->myFill.above;
						}
					}
				}
			}

			// 根据当前事件点创建状态点并接入状态链表
			StatusNode* new_stat_node = new StatusNode();
			new_stat_node->ev = ev; // 状态点对应的事件点
			new_stat_node->prev = prev; // 上一个状态点
			new_stat_node->next = here; // 下一个状态点
			prev->next = new_stat_node; // 更新上一状态点(上一状态点next指向当前新建的状态点)
			if (here != nullptr) {
				here->prev = new_stat_node;
			}
			ev->other->status = new_stat_node; // 终点事件点的状态=随着起点事件新建的状态点(方便后续状态点删除操作)
			all_status_node.push_back(new_stat_node);
		}
		else {
			// ev: 线段终点事件
			std::cout << "end ev :" << ev->pos.transpose() << " -> " << ev->other->pos.transpose() << std::endl;
			StatusNode* st = ev->status;
			if (st == nullptr) {
				reset();
				throw std::runtime_error("PolyBool: Zero-length segment detected; your epsilon is probably too small or too large");
			}

			if (status_list.exists(st->prev) && (status_list.exists(st->next))) {
				eventsIntersection(st->prev->ev, st->next->ev);
			}

			// myFill主多边形信息，otherFill次多边形信息
			if (!ev->primary) {
				FillInfo s = ev->seg->myFill;
				ev->seg->myFill = ev->seg->otherFill;
				ev->seg->otherFill = s;
			}
			segments.push_back(ev->seg);

			status_list.detach(st); // 状态删除
		}
		event_list.detach(event_list.getHead());

		// *****************debug*****************
		std::cout << std::endl;
		if (true) {
			//std::cout << "update   event list   " << std::endl;
			//event_list.printList();
			//std::cout << "-----------------------" << std::endl;
			std::cout << "update   status list  " << std::endl;
			status_list.printList();
			std::cout << "-----------------------" << std::endl;
		}
		std::cout << std::endl;
		std::cout << std::endl;
		// *****************debug*****************
	}
	
	return segments;
}

std::vector<Eigen::Vector2d> Intersecter::calc_intersections() {
	std::vector<Eigen::Vector2d> res;

	return res;
}

void Intersecter::reset() {
	for (auto node : all_event_node) {
		delete node;
	}
	all_event_node.clear();

	for (auto node : all_status_node) {
		delete node;
	}
	all_status_node.clear();

	for (auto node : all_seg_node) {
		delete node;
	}
	all_seg_node.clear();
}