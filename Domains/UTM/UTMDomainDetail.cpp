#include "UTMDomainDetail.h"

using namespace std;
using namespace easymath;

UTMDomainDetail::UTMDomainDetail():
	UTMDomainAbstract()
{

	if (params==NULL) params = new UTMModes(); // use all defaults
	if (filehandler==NULL) filehandler = new UTMFileNames(params);

	Matrix<int,2> * membership_map = new Matrix<int,2>(1,1);
	FileIn::loadVariable(&membership_map,"agent_map/membership_map.csv");
	

	FileIn::loadVariable(fix_locs,"agent_map/fixes.csv");
	
	// Planning
	lowGraph = new SectorGraphManager(*membership_map,highGraph->getEdges());
		
	// initialize fixes
	fixes->clear();
	for (unsigned int i=0; i<fix_locs.size(); i++){
		fixes->push_back(Fix(fix_locs[i],i,highGraph,lowGraph,fixes, params));
	}
	
	//conflict_count_map = new ID_grid(planners->obstacle_map->dim1(), planners->obstacle_map->dim2());
}

void UTMDomainDetail::loadMaps(){

}

UTMDomainDetail::~UTMDomainDetail(void)
{
	//delete conflict_count_map;
}


vector<double> UTMDomainDetail::getPerformance(){
	return matrix1d(sectors->size(),-conflict_count);
}


vector<double> UTMDomainDetail::getRewards(){
	// MAY WANT TO ADD SWITCH HERE

	// DELAY REWARD
	return matrix1d(sectors->size(), -conflict_count);
	
	// LINEAR REWARD
	//return matrix1d(sectors->size(),-conflict_count); // linear reward


	// QUADRATIC REWARD
	/*int conflict_sum = 0;
	for (int i=0; i<conflict_count_map->size(); i++){
	for (int j=0; j<conflict_count_map->at(i).size(); j++){
	int c = conflict_count_map->at(i)[j];
	conflict_sum += c*c;
	}
	}
	return matrix1d(sectors->size(),-conflict_sum);*/
}

unsigned int UTMDomainDetail::getSector(easymath::XY p){
	// tests membership for sector, given a location
	return lowGraph->getMembership(p);
}

//HACK: ONLY GET PATH PLANS OF UAVS just generated
void UTMDomainDetail::getPathPlans(){
	for (std::shared_ptr<UAV> &u : UAVs){
		u->planDetailPath(); // sets own next waypoint
	}
}

void UTMDomainDetail::getPathPlans(std::list<std::shared_ptr<UAV> > &new_UAVs){
	for (std::shared_ptr<UAV> &u : new_UAVs){
		u->planDetailPath(); // sets own next waypoint
	}
}


void UTMDomainDetail::incrementUAVPath(){
	for (std::shared_ptr<UAV> &u: UAVs)
		u->moveTowardNextWaypoint(); // moves toward next waypoint (next in low-level plan)
	// IMPORTANT! At this point in the code, agent states may have changed
}

void UTMDomainDetail::reset(){
	UAVs.clear();
	conflict_count = 0;
	//(*conflict_count_map) = ID_grid(planners->obstacle_map->dim1(), planners->obstacle_map->dim2());
}


void UTMDomainDetail::exportLog(std::string fid, double G){
	static int calls = 0;
	//PrintOut::toFileMatrix2D(*conflict_count_map,fid+to_string(calls)+".csv");
	calls++;
}

void UTMDomainDetail::detectConflicts(){
	for (list<std::shared_ptr<UAV> >::iterator u1=UAVs.begin(); u1!=UAVs.end(); u1++){
		for (list<std::shared_ptr<UAV> >::iterator u2=std::next(u1); u2!=UAVs.end(); u2++){
			
			double d = easymath::distance((*u1)->loc,(*u2)->loc);

			if (d>params->get_conflict_thresh()) continue; // No conflict!

 			conflict_count++;

			int midx = ((int)(*u1)->loc.x+(int)(*u2)->loc.x)/2;
			int midy = ((int)(*u1)->loc.y+(int)(*u2)->loc.y)/2;
			//conflict_count_map->at(midx,midy)++;

			addConflict(*u1, *u2);
		}
	}

	for (unsigned int i=0; i<sectors->size(); i++){
		sectors->at(i).steps++;
	}

	//printf("UAVs=%i\n",UAVs.size());
}