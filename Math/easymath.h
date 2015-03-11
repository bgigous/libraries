#pragma once

#include <vector>
#include <algorithm>
#include <queue>
#include <functional>
#include <utility>
#include <set>
#include <list>

#define COIN (double(rand())/double(RAND_MAX)) // bounded rand between 0 and 1
#define COIN_FLOOR0 (double(rand())/double(RAND_MAX+1)) // guaranteed floor 0 (for indices)

namespace easymath{
	class XY{
	public:
		XY(double x, double y):x(x),y(y){};
		XY(){};
		~XY(){};
		double x,y;
		XY & operator-(const XY &other){
			return XY(x-other.x, y-other.y);
		}
		friend bool operator<(const XY &lhs, const XY &rhs){
			if (lhs.x!=rhs.x) return lhs.x<rhs.x;
			return lhs.y<rhs.y; // if same, sort by y values
		}
		friend bool operator==(const XY &lhs, const XY &rhs){
			return lhs.x==rhs.x && lhs.y==rhs.y;
		}
	};

	typedef std::pair<double,int> P;
	//typedef std::vector<P> DistIDPairVector; 
	typedef std::list<P> DistIDPairList;
	
	std::vector<double> mean2(std::vector<std::vector<double> > myVector);
	std::vector<double> mean1(std::vector<std::vector<double> > myVector);
	double mean(std::vector<double> myVector);
	std::vector<double> sum(std::vector<std::vector<double> > myVector);
	double sum(std::vector<double> myVector);
	double scaleValue01(double val, double min, double max);
	double boundedRand(double min, double max);
	int getMaxIndex(std::vector<double> myvector);
	std::set<XY> getNUniquePositions(int N, double xbound, double ybound=-1);
	double distance(XY p1, XY p2);
	void discretizeSegment(std::vector<XY>::iterator iter_begin, std::vector<XY> myvec, int n_even_segments);
	int cardinalDirection(XY dx_dy);
}

