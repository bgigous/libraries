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
	Sector(XY xy, int sectorIDset, int n_agents);
	int n_agents;
	Sector(){}; // default constructor
	~Sector(){};
	XY xy; // sector center
	std::vector<std::shared_ptr<UAV> > toward; // the UAVs that are going toward the sector
	/*Demographics getLoad(){
		// Get demographic information about the UAVs traveling toward the sector
		Demographics load(UAV::NTYPES,0);
		for (std::shared_ptr<UAV> &u: toward){
			load[u->type_ID]++;

			total_loads[u->type_ID]++;
		}
		nCallsToGetLoad++;
		return load;
	}*/

	void tallyLoad(){
		// Adds in current load for reward calc
		matrix2d med_loads(n_agents,matrix1d(UAV::NTYPES,0.0));
		for (std::shared_ptr<UAV> &u: toward){
			med_loads[u->curSectorID()][u->type_ID]++;
		}
		loads_each_step.push_back(med_loads);
		for (int i=0; i<med_loads.size(); i++){
			for (int j=0; j<med_loads[i].size(); j++){
				if (med_loads[i][j]>2.0){
					total_loads[i][j] += med_loads[i][j]-2.0; // over capacity amount, HARDCODING
				}
			}
		}
		nCallsToGetLoad++;
	}

	double average_load(int a, int t){
		return total_loads[a][t]/double(nCallsToGetLoad);
	}

	matrix3d loads_each_step; //[step][originating agent][type]
	matrix2d total_loads; // [originating agent][type]
	int nCallsToGetLoad;
};
