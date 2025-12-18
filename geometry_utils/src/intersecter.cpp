#include "intersecter.h"
#include "Eigen/Dense"

Intersecter::Intersecter(){
	
}

Intersecter::~Intersecter(){
	reset();
}

bool Intersecter::pointsSame(const GU::Point& a, const GU::Point& b) {
	return std::fabs(a(0) - b(0)) < EPS && std::fabs(a(1) - b(1)) < EPS;
}

// 判断点p是否在线段left,right之间
bool Intersecter::pointBetween(const GU::Point& p, const GU::Point& left, const GU::Point& right) {
	// line_vec(dx1, dy1)
	// vec(dx2, dy2)
	double dx1, dy1, dx2, dy2;
	dx1 = right.x - left.x;
	dy1 = right.y - left.y;
	dx2 = p.x - left.x;
	dy2 = p.y - left.y;

	double dot = dx1 * dx2 + dy1 * dy2;
	if (dot < EPS) {
		return false;
	}

	double sqlen = dx1 * dx1 + dy1 * dy1;
	if (dot - sqlen > -EPS) {
		return false;
	}
	return true;
}

// 0:相等 -1:p1在p2之前 1:p1在p2之后
int Intersecter::pointsCompare(const GU::Point& p1, const GU::Point& p2) {
	if (std::fabs(p1(0) - p2(0)) < EPS) {
		return std::fabs(p1(1) - p2(1)) < EPS ? 0 : p1(1) < p2(1) ? -1 : 1;
	}
	return p1(0) < p2(0) ? -1 : 1;
}

// p1_isstart:是否起点 p1_0:事件点 p1_1:事件点所在段终点
// p2_isstart:是否起点 p2_0:事件点 p2_1:事件点所在段终点
int Intersecter::eventsCompare(const bool& p1_isstart, const GU::Point& p1_0, const GU::Point& p1_1,
	const bool& p2_isstart, const GU::Point& p2_0, const GU::Point& p2_1) {
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
	double dx1, dy1, dx2, dy2;
	if (p1_isstart) {
		// vec1(dx1, dy1)
		dx1 = p1_1.x - p1_0.x;
		dy1 = p1_1.y - p1_0.y;
		// vec2(dx2, dy2)
		dx2 = p2_1.x - p2_0.x;
		dy2 = p2_1.y - p2_0.y;
		// vec2 x vec1
		return dx2 * dy1 - dy2 * dx1 >= 0 ? 1 : -1;
	}
	else {
		// vec1(dx1, dy1)
		dx1 = p1_1.x - p2_1.x;
		dy1 = p1_1.y - p2_1.y;
		// vec2(dx2, dy2)
		dx2 = p2_0.x - p2_1.x;
		dy2 = p2_0.y - p2_1.y;
		// vec2 x vec1
		return dx2 * dy1 - dy2 * dx1 >= 0 ? 1 : -1;
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
	double dx1, dy1, dx2, dy2, dx3, dy3;
	// ev2_vec(dx1, dy1)
	dx1 = ev2->seg->end.x - ev2->seg->start.x;
	dy1 = ev2->seg->end.y - ev2->seg->start.y;
	// vec_ss(dx2, dy2)
	dx2 = ev1->seg->start.x - ev2->seg->start.x;
	dy2 = ev1->seg->start.y - ev2->seg->start.y;
	// vec_se(dx3, dy3)
	dx3 = ev1->seg->end.x - ev2->seg->start.x;
	dy3 = ev1->seg->end.y - ev2->seg->start.y;
	// cross1 = ev2_vec x vec_ss
	double cross1 = dx1 * dy2 - dy1 * dx2;
	// cross2 = ev2_vec x vec_se
	double cross2 = dx1 * dy3 - dy1 * dx3;
	if (std::fabs(cross1) < EPS) { // 共线
		if (std::fabs(cross2) < EPS) {
			return 1;
		}
	}
	// 不共线通过与扫描线交点判断局部上下关系
	double inc_t = (ev1->seg->start(0) - ev2->seg->start(0)) / (ev2->seg->end(0) - ev2->seg->start(0));
	double inc_y = ev2->seg->start(1) + inc_t * (ev2->seg->end(1) - ev2->seg->start(1));

	return inc_y > ev1->seg->start(1) ? -1 : 1;
}

void Intersecter::eventAdd(EventNode* const root, EventNode* const node, const GU::Point other_pt) {
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

GU::Intersection Intersecter::linesIntersection(const GU::Point& a0, const GU::Point& a1, const GU::Point& b0, const GU::Point& b1) {
	double dx1, dy1, dx2, dy2;
	// A_vec(dx1, dy1)
	dx1 = a1.x - a0.x;
	dy1 = a1.y - a0.y;
	// B_vec(dx2, dy2)
	dx2 = b1.x - b0.x;
	dy2 = b1.y - b0.y;

	GU::Intersection inc;
	double cross_ab = dx1 * dy2 - dy1 * dx2;
	inc.cross = cross_ab;
	if (std::fabs(cross_ab) < EPS) {
		inc.isParallel = true;
		return inc;
	}
	Eigen::Matrix2d A(2, 2);
	Eigen::Vector2d B(2);
	A << dx1, -1.0 * dx2,
		dy1, -1.0 * dy2;
	B << b0(0) - a0(0),
		b0(1) - a0(1);
	Eigen::Vector2d C = A.partialPivLu().solve(B); // colPivHouseholderQr()
	inc.alongA = C(0);
	inc.alongB = C(1);
	inc.p = GU::Point(a0(0) + inc.alongA * dx1, a0(1) + inc.alongA * dy1);
	return inc;
}

// 当前 ev1(a1-a2), above/below: ev2(b1-b2)
// return: 
// nullptr: 平行不共线/首尾相接/不平行相交 （没有交点）
// ev2: 共线完全重叠/共线有重叠且起点对齐 切割后的ev2(ev2与ev1的重叠部分)
EventNode* Intersecter::eventsIntersection(EventNode* ev1, EventNode* ev2) {
	GU::Intersection inc = linesIntersection(ev1->seg->start, ev1->seg->end, ev2->seg->start, ev2->seg->end);

	// 处理平行线
	if (inc.isParallel) {
		double dx1, dy1, dx2, dy2;
		// ev1_seg_vec(dx1, dy1)
		dx1 = ev1->seg->end.x - ev1->seg->start.x;
		dy1 = ev1->seg->end.y - ev1->seg->start.y;
		// vec(dx2, dy2)
		dx2 = ev1->seg->end.x - ev2->seg->start.x;
		dy2 = ev1->seg->end.y - ev2->seg->start.y;
		// ev1_seg_vec x vec
		double cross = dx1 * dy2 - dy1 * dx2;
		// 平行不共线
		if (!(std::fabs(cross) < EPS)) {
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
		bool inc_added = false;
		if (inc.alongA >= EPS && inc.alongA - 1 <= -EPS) {
			if (inc.alongB > -EPS && inc.alongB < EPS) {
				// 交点在A中间B起点 A:event line B:below
				DEBUG_PRINT("                                                               inc: " << inc.p);
				if (!inc_added) {
					intersections_.push_back(inc); inc_added = true;
				}
				inc_count_++;
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
			else if (inc.alongB >= EPS && inc.alongB <= 1- EPS) {
				DEBUG_PRINT("                                                               inc: " << inc.p);
				if (!inc_added) {
					intersections_.push_back(inc); inc_added = true;
				}
				inc_count_++;
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
			else if (inc.alongB > 1- EPS && inc.alongB <1+ EPS) {
				// 交点在A中间B终点 A:event line B:above/below
				DEBUG_PRINT("                                                               inc: " << inc.p);
				if (!inc_added) {
					intersections_.push_back(inc); inc_added = true;
				}
				inc_count_++;
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
		if (inc.alongB >= EPS && inc.alongB - 1 < -EPS) {
			if (inc.alongA > -EPS && inc.alongA < EPS) {
				// 交点在B中间A的起点 A:event B:above
				if (!inc_added) {
					intersections_.push_back(inc); inc_added = true;
				}
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
			else if (inc.alongA >= EPS && inc.alongA <= 1- EPS) {
				// 交点在B中间A中间 拆分状态线段，更新状态线段other事件点，新增事件线段
				if (!inc_added) {
					intersections_.push_back(inc); inc_added = true;
				}
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
			else if (inc.alongA > 1- EPS && inc.alongA < 1+ EPS) {
				// 交点在B中间A的终点 A:event B:above/below
				if (!inc_added) {
					intersections_.push_back(inc); inc_added = true;
				}
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

void Intersecter::addRegion(const std::vector<GU::Point>& region, const bool& primary) {
	GU::Point pt1;
	GU::Point pt2 = region[region.size()-1];
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
		DEBUG_PRINT("above: " << above->pos << " -> " << above->other->pos);
		EventNode* eve = eventsIntersection(ev, above);
		if (eve) {
			DEBUG_PRINT("                                             Overlapping ev: " << eve->pos << " -> " << eve->other->pos);
			return eve;
		}
	}
	if (below) {
		DEBUG_PRINT("below: " << below->pos << " -> " << below->other->pos);
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
		DEBUG_PRINT("iter " << iter_num);
		DEBUG_PRINT("cur-----event list-----");
		DEBUG_PRINT_OLDLINE(">>");
		event_list.printList();
		DEBUG_PRINT("-----------------------");
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
				DEBUG_PRINT("                                             Overlapping ev: " << eve->pos << " -> " << eve->other->pos);
				if (!selfIntersection) {
					eve->seg->otherFill = ev->seg->myFill;
				}
				event_list.detach(ev->other);
				event_list.detach(ev);
			}

			if (event_list.getHead() != ev) {
				// something was inserted before us in the event queue, so loop back around and
				// process it before continuing
				DEBUG_PRINT(" ---------something was inserted before event_list---------- ");
				DEBUG_PRINT("------event list-------");
				event_list.printList();
				DEBUG_PRINT("-----------------------");
				DEBUG_PRINT("update   status list  ");
				status_list.printList();
				DEBUG_PRINT("-----------------------");
				DEBUG_PRINT_NEWLINE();
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
			DEBUG_PRINT("end ev :" << ev->pos << " -> " << ev->other->pos);
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
		DEBUG_PRINT_NEWLINE();
		if (true) {
			//std::cout << "update   event list   " << std::endl;
			//event_list.printList();
			//std::cout << "-----------------------" << std::endl;
			DEBUG_PRINT("update   status list  ");
			status_list.printList();
			DEBUG_PRINT("-----------------------");
		}
		DEBUG_PRINT_NEWLINE();
		DEBUG_PRINT_NEWLINE();
		// *****************debug*****************
	}
	
	return segments;
}

std::vector<GU::Intersection> Intersecter::calcIntersect() {
	intersections_.clear();
	// 状态列表中只会插入起点事件点
	StatusList status_list = StatusList();
	std::vector<Segment*> segments;
	int iter_num = 0;
	while (!event_list.isEmpty()) {
		EventNode* ev = event_list.getHead();
		DEBUG_PRINT("iter " << iter_num);
		DEBUG_PRINT("cur-----event list-----");
		DEBUG_PRINT_OLDLINE(">>");
		event_list.printList();
		DEBUG_PRINT("-----------------------");
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
				DEBUG_PRINT("                                             Overlapping ev: " << eve->pos << " -> " << eve->other->pos);
				event_list.detach(ev->other);
				event_list.detach(ev);
			}

			if (event_list.getHead() != ev) {
				// something was inserted before us in the event queue, so loop back around and
				// process it before continuing
				DEBUG_PRINT(" ---------something was inserted before event_list---------- ");
				DEBUG_PRINT("------event list-------");
				event_list.printList();
				DEBUG_PRINT("-----------------------");
				DEBUG_PRINT("update   status list  ");
				status_list.printList();
				DEBUG_PRINT("-----------------------");
				DEBUG_PRINT_NEWLINE();
				continue;
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
			DEBUG_PRINT("end ev :" << ev->pos << " -> " << ev->other->pos);
			StatusNode* st = ev->status;
			if (st == nullptr) {
				reset();
				throw std::runtime_error("PolyBool: Zero-length segment detected; your epsilon is probably too small or too large");
			}

			if (status_list.exists(st->prev) && (status_list.exists(st->next))) {
				eventsIntersection(st->prev->ev, st->next->ev);
			}

			segments.push_back(ev->seg);

			status_list.detach(st); // 状态删除
		}
		event_list.detach(event_list.getHead());

		// *****************debug*****************
		DEBUG_PRINT_NEWLINE();
		if (true) {
			//std::cout << "update   event list   " << std::endl;
			//event_list.printList();
			//std::cout << "-----------------------" << std::endl;
			DEBUG_PRINT("update   status list  ");
			status_list.printList();
			DEBUG_PRINT("-----------------------");
		}
		DEBUG_PRINT_NEWLINE();
		DEBUG_PRINT_NEWLINE();
		// *****************debug*****************
	}
	return intersections_;
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
	inc_count_ = 0;
}