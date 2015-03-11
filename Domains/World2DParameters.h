#pragma once
#include "../easymath.h"

class World2DParameters{
public:
	World2DParameters():
		edge_length(1000.0)
	{};
	const double edge_length;
	double getRandX(){
		return COIN_FLOOR0*edge_length;
	}
	double getRandY(){
		return COIN_FLOOR0*edge_length;
	}
	void setRandXY(double &x, double &y){
		x = getRandX();	
		y = getRandY();
	}
};

