#pragma once

#include "../easymath.h"
#include "World2DParameters.h"
#include "Point2D.h"

class World2D
{
public:
	World2D(void);
	World2DParameters* world_params;
	std::vector<Point2D*>* all_objects; // physical presence in the world
	
	double getPointDistance(Point2D *p1, Point2D *p2){
		double dx = p1->x-p2->x;
		double dy = p1->y-p2->y;
		return sqrt(dx*dx+dy*dy);
	}

	bool colocated(Point2D *p1, Point2D *p2, double delta=0.0){
			return getPointDistance(p1,p2)<delta;
	}
	int getRelativeQuadrant(Point2D *ref, Point2D *other){
		// returns the quadrant (0-indexed)

		double dx = ref->x-other->x;
		double dy = ref->y-other->y;

		if (dy>0) return dx>0?0:1;
		else return dx>0?3:2;
	}

	bool isBlocked(Point2D *p){
		for (int i=0; i<all_objects->size(); i++){
			if (colocated(p, all_objects->at(i))) return true;
		}
		return false;
	}

	void rotateCheckCollisionsAndMove(Point2D *me, std::vector<double> &dtheta_dr, double dr_limit){
		// Moves an embodied object in the domain
		// action is: [rotation modification][distance to move] 

		// Saves position in case of being blocked by other rover
		double xsave  = me->x;
		double ysave = me->y;

		double dtheta = dtheta_dr[0]/2.0; // amount to rotate [0,0.5] (not [0,pi])
		double dr = dtheta_dr[1]*dr_limit; // amount to move

		me->adjustAngleAndPosition(dtheta,dr);

		// Check if another object exists in that spot
		if (isBlocked(me)){
			me->x=xsave; // restore original space
			me->y=ysave;
		}
	}

	void setRandUnblockedXY(Point2D* p){
		while (isBlocked(p)){
			p->setRandPoint();
		}
	}

	~World2D(void);
};

