#pragma once

#include <iostream>
#include <vector>
#include <queue>
#include <utility>
#include <unordered_map>
#include "geometry_types.h"
#include "geometry_utils.h"

class PolygonOffset {
public:
	PolygonOffset();
	~PolygonOffset();

	std::vector<std::vector<GU::Point>> inflatePolygon(const std::vector<GU::Point>& polygon, const double offset);

	std::vector<GU::Point> inflateLines(const std::vector<GU::Point>& polygon, const double offset);

	std::vector<std::vector<GU::Point>> processRing(const std::vector<GU::Point>& ring);

	std::unordered_map<int, std::vector<GU::Point>> getIntersectionPoints(const std::vector<GU::Point>& polygon);



	bool pointsSame(const GU::Point& a, const GU::Point& b);

	double EPS = 1e-7;

};