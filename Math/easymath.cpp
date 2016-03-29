#include "easymath.h"

namespace easymath{

	double manhattan_distance(const XY &p1, const XY &p2){
		XY diff = p1-p2;
		return abs(diff.x)+abs(diff.y);
	}

	int cardinal_direction(const XY &dx_dy){
		if (dx_dy.y>=0){ // Going up
			if (dx_dy.x>=0) return 0; // up-right
			else return 1; // up-left
		} else {
			if (dx_dy.x>=0) return 2; // down-right
			else return 3; // down-left
		}
	}

	double euclidean_distance(const XY &p1, const XY &p2){
		double dx = p1.x-p2.x;
		double dy = p1.y-p2.y;
		return sqrt(dx*dx+dy*dy);
	}

	int bin(double &n, matrix1d& bounds) {
		for (size_t i = 0; i < bounds.size()-1; i++)
			if (n < bounds[i + 1])
				return i;
		return bounds.size();
	}

	double rand(double low, double high) {
		double r = double(std::rand()) / double(RAND_MAX);
		return r*(high - low) + low;
	}

	double cross(const XY &U, const XY &V){
		return U.x*V.y-U.y*V.x;
	}


	bool intersects_in_center(line_segment edge1, line_segment edge2){
		// Detects whether line intersects, but not at origin
		XY p = edge1.first;
		XY q = edge2.first;
		XY r = edge1.second - edge1.first;
		XY s = edge2.second - edge2.first;
		XY qpdiff = q-p;
		double rscross = cross(r,s);
		double t = cross((qpdiff),s)/rscross;
		double u = cross((qpdiff),r)/rscross;

		if (rscross==0){
			if (cross(qpdiff,r)==0){
				return true; // collinear
			} else return false; // parallel non-intersecting
		} else if (0<t && t<1 && 0<u && u<1){ // if you care about origins, <= rather than <
			return true; // intersects at p+tr = q+us
		} else {
			return false; // not parallel, don't inersect
		}
	}

	void clear(std::queue<XY> &q){
		std::queue<XY> empty;
		std::swap(q,empty);
	}

	double erfc(double x)
	{

		// constants
		double a1 =  0.254829592;
		double a2 = -0.284496736;
		double a3 =  1.421413741;
		double a4 = -1.453152027;
		double a5 =  1.061405429;
		double p  =  0.3275911;

		// Save the sign of x
		int sign = 1;
		if (x < 0)
			sign = -1;
		x = fabs(x);

		// A&S formula 7.1.26
		double t = 1.0/(1.0 + p*x);
		double y = 1.0 - (((((a5*t + a4)*t) + a3)*t + a2)*t + a1)*t*exp(-x*x);

		return 1- sign*y;
	} 
}