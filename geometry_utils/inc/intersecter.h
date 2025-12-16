#ifndef __INTERSECTER_H__
#define __INTERSECTER_H__

#include <iostream>
#include <vector>
#include "Eigen/Dense"
#include "linked_list.h"
#include "geometry_types.h"

class Intersecter
{
public:
	Intersecter();
	~Intersecter();

	bool pointsSame(const Eigen::Vector2d& a, const Eigen::Vector2d& b);

	bool pointBetween(const Eigen::Vector2d& p, const Eigen::Vector2d& left, const Eigen::Vector2d& right);

	int pointsCollinear(const Eigen::Vector2d& a, const Eigen::Vector2d& b);

	int pointsCompare(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2);

	int eventsCompare(const bool& p1_isstart, const Eigen::Vector2d& p1_0, const Eigen::Vector2d& p1_1, const bool& p2_isstart, const Eigen::Vector2d& p2_0, const Eigen::Vector2d& p2_1);

	int statusCompare(EventNode* ev1, EventNode* ev2);

	EventNode* eventsIntersection(EventNode* ev1, EventNode* ev2);

	EventNode* checkBothIntersections(EventNode* ev, EventNode* above, EventNode* below);

	GU::Intersection linesIntersection(const Eigen::Vector2d& a0, const Eigen::Vector2d& a1, const Eigen::Vector2d& b0, const Eigen::Vector2d& b1);

	void addRegion(const std::vector<Eigen::Vector2d> &region, const bool& primary);

	void eventAdd(EventNode* const root, EventNode* const node, const Eigen::Vector2d other_pt);

	void eventAddSegment(Segment* const seg, const bool &primary);

	std::vector<Eigen::Vector2d> calc_intersections();

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


