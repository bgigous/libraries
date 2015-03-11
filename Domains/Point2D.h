#pragma once
#include "World2DParameters.h"
#include "../easymath.h"
#include <set>
#include <algorithm>

using namespace easymath;

class Point2D
{
public:
	Point2D(){
		printf("Point2d erroneously called.");
		exit(1);
	}
	Point2D(World2DParameters* world_params_set):
		_world_params(world_params_set),
		orientation(COIN)
	{
		static int IDset=0;
		ID = IDset++; // Sets ID based on how many calls to this function
		_world_params->setRandXY(x,y);
	}
	~Point2D(void){};
	double x, y, orientation; // orientation scaled between 0 and 1
	int ID; // identifies object
	void setPosition(easymath::XY xy_set){
		x = xy_set.x;
		y = xy_set.y;
	}
	void adjustPosition(double dx, double dy){
		x+=dx;
		y+=dy;
		_boundPosition();
	}

	void adjustAngleAndPosition(double dtheta, double dl){
		orientation+= dtheta;
		//	printf("Action 0 was %f\n",action[0]);
		if (orientation>1.0) orientation -=1.0; // adjust for turning
		else if (orientation<0.0) orientation +=1.0; // adjust for turning


		// The proposed movement amounts
		double dx = dl*cos(orientation*2.0*3.14);
		double dy = dl*sin(orientation*2.0*3.14);
	
		adjustPosition(dx,dy);
	}

	void setUniquePoint(std::set<XY> &other_points){
		// Set a point that is not in the set of other points
		while(other_points.count(XY(x,y))){
			_world_params->setRandXY(x,y);
		}
	}

	void setRandPoint(){
		// Set a point randomly
		_world_params->setRandXY(x,y);
	}

private:
	World2DParameters* _world_params;
	void _boundPosition(){
		// Check if out of bound, then adjust according to world_params
		if (x<0) x=0;
		if (x>_world_params->edge_length) x= _world_params->edge_length;
		if (y<0) y=0;
		if (y>_world_params->edge_length) y= _world_params->edge_length;
	}
};

