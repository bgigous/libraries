#pragma once

#include "MatrixTypes.h"
#include <queue>
#include <utility>


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
		friend XY operator*(const XY &lhs,double rhs){
			return XY(lhs.x*rhs,lhs.y*rhs);
		}
		friend double operator*(const XY &U, const XY &V){ // dot product
			return U.x*V.x+U.y*V.y;
		}
		friend XY operator+(const XY &lhs, const XY &rhs){
			return XY(lhs.x+rhs.x,lhs.y+rhs.y);
		}
		void operator=(const XY &rhs) {
			x = rhs.x;
			y = rhs.y;
		}
	};

	void clear(std::queue<XY> &q);

	double manhattan_distance(const XY &p1, const XY &p2);
	double euclidean_distance(const XY &p1, const XY &p2);
	void discretize_segment(std::vector<XY>::iterator iter_begin, std::vector<XY> myvec, int n_even_segments);
	int cardinal_direction(const XY &dx_dy);
	double cross(const XY &U, const XY &V);

	typedef std::pair<XY,XY> line_segment;

	bool intersects_in_center(line_segment edge1, line_segment edge2);

	template <class STL_Container, class UnaryPredicate>
	void remove_erase_if(STL_Container stl, UnaryPredicate pred){
		auto it = stl.begin();
		while (it!=stl.end()){
			if (!pred(*it)){
				stl.erase(it++);
			} else {
				it++;
			}
		}
	}

	double rand(double low, double high); // random with bounds
	double erfc(double x);
}
