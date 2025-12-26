#pragma once

#include <iostream>
#include <vector>
#include <queue>
#include <utility>
#include <unordered_map>
#include "geometry_types.h"
#include "geometry_utils.h"

#ifdef _DEBUG
#define DEBUG_PRINT(...) std::cout << "[DEBUG] " << __VA_ARGS__ << std::endl
#define DEBUG_PRINT_OLDLINE(...) std::cout << "[DEBUG] " << __VA_ARGS__
#define DEBUG_PRINT_NEWLINE() std::cout << std::endl

#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINT_OLDLINE(...)
#define DEBUG_PRINT_NEWLINE(...)
#endif

//enum PLType {
//	TIP, // 相交
//	PFIP, // 延长线相交，交点在正方向
//	NFIP  // 延长线相交，交点在负方向
//};

class PolygonOffset {
public:
	PolygonOffset();
	~PolygonOffset();

	std::vector<std::vector<GU::Point>> inflatePolygon(const std::vector<GU::Point>& polygon, const double offset);

	std::vector<GU::Point> inflateLines(const std::vector<GU::Point>& polygon, const double offset);

	std::vector<std::vector<GU::Point>> processRing(const std::vector<GU::Point>& ring);

	std::unordered_map<int, std::vector<GU::Point>> getIntersectionPoints(const std::vector<GU::Point>& polygon);

	std::vector<GU::Point> testOffsetPolygon(const std::vector<GU::Point>& polygon, const double offset);

	bool pointsSame(const GU::Point& a, const GU::Point& b);

	double EPS = 1e-7;

};