#pragma once

#include "UAV.h"
#include "../../Math/Matrix.h"
#include "../../Planning/AStar_grid.h"
#include <memory>


using namespace Numeric_lib;

typedef std::vector<int> Demographics;
class Sector{
public:
	// An area of space that contains some fixes
	int sectorID; // the identifier for this sector
	Sector(XY xy, int sectorIDset);
	Sector(){}; // default constructor
	~Sector(){};
	XY xy; // sector center
	std::vector<std::shared_ptr<UAV> > toward; // the UAVs that are going toward the sector
	Demographics getLoad(){
		// Get demographic information about the UAVs traveling toward the sector
		Demographics load(UAV::NTYPES,0);
		for (std::shared_ptr<UAV> &u: toward){
			load[u->type_ID]++;

			total_loads[u->type_ID]++;
		}
		nCallsToGetLoad++;
		return load;
	}

	void tallyLoad(){
		// Adds in current load for reward calc
		Demographics load(UAV::NTYPES,0);
		for (std::shared_ptr<UAV> &u: toward){
			total_loads[u->type_ID]++;
		}
		nCallsToGetLoad++;
	}

	matrix1d average_load(){
		matrix1d avg(UAV::NTYPES,0.0);
		for (int i=0; i<total_loads.size(); i++){
			avg[i] = double(total_loads[i])/double(nCallsToGetLoad);
		}
		return avg;
	}
	Demographics total_loads;
	int nCallsToGetLoad;
};
