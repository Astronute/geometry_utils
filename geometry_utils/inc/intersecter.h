#ifndef __INTERSECTER_H__
#define __INTERSECTER_H__

#include <iostream>
#include <vector>
#include "linked_list.h"
#include "geometry_types.h"

class Intersecter
{
public:
	Intersecter();
	~Intersecter();

	bool pointsSame(const GU::Point& a, const GU::Point& b);

	bool pointBetween(const GU::Point& p, const GU::Point& left, const GU::Point& right);

	int pointsCompare(const GU::Point& p1, const GU::Point& p2);

	int eventsCompare(const bool& p1_isstart, const GU::Point& p1_0, const GU::Point& p1_1, const bool& p2_isstart, const GU::Point& p2_0, const GU::Point& p2_1);

	int statusCompare(EventNode* ev1, EventNode* ev2);

	EventNode* eventsIntersection(EventNode* ev1, EventNode* ev2);

	EventNode* checkBothIntersections(EventNode* ev, EventNode* above, EventNode* below);

	GU::Intersection linesIntersection(const GU::Point& a0, const GU::Point& a1, const GU::Point& b0, const GU::Point& b1);

	void addRegion(const std::vector<GU::Point> &region, const bool& primary);

	void eventAdd(EventNode* const root, EventNode* const node, const GU::Point other_pt);

	void eventAddSegment(Segment* const seg, const bool &primary);

	std::vector<Segment*> calculate(const bool& selfIntersection);

	void reset();

	EventList event_list = EventList();

	int inc_count_;

private:
	std::vector<EventNode*> all_event_node;
	std::vector<StatusNode*> all_status_node;
	std::vector<Segment*> all_seg_node;

	double EPS = 1e-3;

};

#endif // !__INTERSECTER_H__


