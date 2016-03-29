#include "GridWorld.h"

using namespace std;
using namespace easymath;

GridWorld::GridWorld(void) :
	xsize(100),
	ysize(100)
{
}

vector<XY> GridWorld::get_unique_positions(size_t n) {	 // was generatestaticroverpositions
	set<XY> positions;
	while (positions.size() < n) {
		positions.insert(XY(rand(0, xsize), rand(0, ysize)));
	}
	vector<XY> position_v(n);
	copy(positions.begin(), positions.end(), position_v.begin());
	return position_v;
}

void GridWorld::randomize_positions(std::vector<XY> &r) {	 // was generatestaticroverpositions
	set<XY> positions;
	size_t n = r.size();
	while (positions.size() < n) {
		positions.insert(XY(rand(0, xsize), rand(0, ysize)));
	}
	vector<XY> position_v(n);
	for (XY &i : r) {
		i = *positions.begin();
		positions.erase(positions.begin());
	}
}

GridWorld::~GridWorld(void)
{
}