#pragma once

#include <iostream>
#include <vector>
#include "geometry_types.h"
#include "geometry_utils.h"

class PolygonOffset {
public:
	PolygonOffset();
	~PolygonOffset();

	std::vector<GU::Point> inflatePolygon(const std::vector<GU::Point>& polygon, const double offset);

};