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

typedef std::vector<double> matrix1d;
typedef std::vector<std::vector<double> > matrix2d;
typedef std::vector<std::vector<std::vector<double> > > matrix3d;

// Applies a given function to every comparison between container_1 and container_2
	// Note, ContainerOut should be a 2d version of ContainerIn (should be indexable, like vector), 2d list currently unsupported
	template <class ContainerIn, class ContainerOut, class StaticFunction>
		 void forEachPairing(ContainerIn container_1, ContainerIn container_2, ContainerOut &output, StaticFunction func){
			 output = ContainerOut(container_1.size());
			 int ind = 0;
			for (ContainerIn::iterator c1=container_1.begin(); c1!=container_1.end(); c1++){
				for (ContainerIn::iterator c2=container_2.begin(); c2!=container_2.end(); c2++){
					output[ind].push_back(func(*c1,*c2));
				}
				ind++;
			}
		}

template <class T>
std::vector<std::vector<T> > vector_init(int XDIM, int YDIM, T INITVAL){
	std::vector<std::vector<T> > retval(XDIM);
	for (int i=0; i<XDIM; i++){
		retval[i] = std::vector<T>(YDIM,INITVAL);
	}
}


template <class T>
void clear_all(T &ptrs){
	while (T.size()){
		delete ptrs.back();
		ptrs.pop_back();
	}
}

template <class T>
void clear_all(std::vector<T*> &ptrs){
	// Deletes all pointers in a container
	for (unsigned int i=0; i<ptrs.size(); i++){
		delete ptrs[i];
	}
}

template <class T>
void clear_all(std::vector<std::vector<T*> > &ptrs){
	// Deletes all pointers in a container
	for (int i=0; i<ptrs.size(); i++){
		clear_all(ptrs[i]);
	}
}

template <class T>
void clear_all(std::vector<std::vector<std::vector<T*> > > &ptrs){
	// Deletes all pointers in a container
	for (int i=0; i<ptrs.size(); i++){
		clear_all(ptrs[i]);
	}
}

matrix1d zeros(int dim);
matrix2d zeros(int dim1, int dim2);
matrix3d zeros(int dim1, int dim2, int dim3);

namespace easymath{
	class XY{
	public:
		XY(double x, double y):x(x),y(y){};
		XY(){};
		~XY(){};
		double x,y;
		XY operator-(const XY &other){
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

	double manhattanDist(XY &p1, XY &p2);

	typedef std::pair<double,int> P;
	//typedef std::vector<P> DistIDPairVector; 
	typedef std::list<P> DistIDPairList;
	
	matrix1d mean2(matrix2d myVector);
	matrix1d mean1(matrix2d myVector);
	double mean(matrix1d myVector);
	matrix1d sum(matrix2d myVector);
	double sum(matrix1d myVector);
	double scaleValue01(double val, double min, double max);
	double boundedRand(double min, double max);
	int getMaxIndex(matrix1d myvector);
	std::set<XY> getNUniquePositions(unsigned int N, double xbound, double ybound=-1);
	double distance(XY p1, XY p2);
	void discretizeSegment(std::vector<XY>::iterator iter_begin, std::vector<XY> myvec, int n_even_segments);
	int cardinalDirection(XY dx_dy);
	double crossProduct(XY U, XY V);
	bool intersects(std::pair<XY, XY> edge1, std::pair<XY,XY> edge2);
}

