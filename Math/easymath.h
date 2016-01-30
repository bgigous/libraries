#pragma once

#include "MatrixTypes.h"

namespace easymath{
	class XY: public std::pair<double,double>{
	public:
		XY(const double &x, const double &y):
			pair<double, double>(x,y),
			x(x),y(y){};

		XY(){};
		~XY(){};
		double x,y;
		XY friend operator-(const XY &lhs, const XY &rhs){
			return XY(lhs.x-rhs.x, lhs.y-rhs.y);
		}
		friend bool operator<(const XY &lhs, const XY &rhs){
			if (lhs.x!=rhs.x) return lhs.x<rhs.x;
			return lhs.y<rhs.y; // if same, sort by y values
		}
		friend bool operator==(const XY &lhs, const XY &rhs){
			return lhs.x==rhs.x && lhs.y==rhs.y;
		}
	};

	double manhattan_distance(const XY &p1, const XY &p2);
	double distance(const XY &p1, const XY &p2);
	void discretizeSegment(std::vector<XY>::iterator iter_begin, std::vector<XY> myvec, int n_even_segments);
	int cardinalDirection(const XY &dx_dy);
	double cross(const XY &U, const XY &V);

	typedef std::pair<XY,XY> line_segment;

	bool intersects(line_segment edge1, line_segment edge2);
}

