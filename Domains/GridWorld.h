#pragma once
#include "../Math/easymath.h"
#include <set>

class GridWorld // NOTE: this is a world type, not a domain
{
public:
	// Generation functions
	GridWorld();
	~GridWorld();

	const int xsize;
	const int ysize;

	std::vector<easymath::XY> get_unique_positions(size_t n);
	void randomize_positions(std::vector<easymath::XY>& r);
};