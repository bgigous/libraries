/*@file roverdomain.h

*/

#pragma once

#include "World2D.h"
#include "..\..\..\master_libraries\easyio\easyio.h"
#include "NeuralNet\NeuralNet.h"
#include "IDomainStateful.h"

using namespace std;
using namespace easymath;

class RoverDomainParameters{
public:
	RoverDomainParameters(void);
	~RoverDomainParameters(void);
	static const int n_rovers=10;
	static const int n_steps= 100;
	static const int n_POI=20;
	static const int n_state_elements=8;
	const double rover_move_scale;
	const double observation_distance;
	std::vector<int> rover_types_fixed;
};

class StepClockLogger{
public:
	// Performs clock and logging functions for all objects in system
	StepClockLogger(){};
	StepClockLogger(int max_steps_set, std::vector<Point2D*>* all_objects_set);
	void reset();
	void logStep(int step);
	void exportLog(std::string file_name, double G);
	void addGLine(double G);

private:
	int max_steps;
	std::vector<std::vector<std::vector<double> > > log; // [step][object][x=0,y=1]
	std::vector<Point2D*>* all_objects; // Pointer to physical presence of all 2D objects
};


class Rover: public Point2D{
public:
	Rover(World2DParameters* world_params, RoverDomainParameters* domain_params_set);
	RoverDomainParameters* domain_params;
	~Rover();
	void selectAction(NeuralNet* NN);
	vector<double> stateInputs;
	enum RoverType{FAST,SLOWTURN,ERRATIC,NORMAL,NUMTYPES};
	RoverType type;
	vector<double> action; // 2 outputs: dx,dy
};

class POI: public Point2D{
public:
	POI(World2DParameters* world_params);
	~POI();
};

class RoverDomain: public IDomainStateful
{
public:
	RoverDomain(void);
	~RoverDomain(void);
	void exportLog(std::string fid, double G){
		log.exportLog(fid,G);
	}

	std::vector<std::vector<double> > getStates(){
		std::vector<std::vector<double> > S = std::vector<std::vector<double> >(rovers.size());
		for (int i=0; i<rovers.size(); i++){
			// Reserve space for state elements
			std::vector<double> si = std::vector<double>(domain_params->n_state_elements);

			// Count number of rovers in each fix, as percentage of total rovers
			for (int j=0; j<rovers.size(); j++){
				if (i==j) continue;
				int Qj = world->getRelativeQuadrant(rovers[i],rovers[j]); // Quadrant of rover j, relative to i
				si[Qj] += 1.0/double(rovers.size()-1);
			}

			// Count number of POIs in each fix
			for (int j=0; j<POIs.size(); j++){
				int Qj = world->getRelativeQuadrant(rovers[i],POIs[j]); // Quadrant of POI j, relative to i
				si[Qj+4]+=1.0/double(POIs.size()); // Adding this to the state (offset by earlier state elements)
			}

			// Add element si to S
			S[i] = si;

			// Gets the nearest neighbor type
			DistIDPairList pq = getSortedRoverDistances(rovers[i]);
			if (rovers[pq.begin()->second]->ID==rovers[i]->ID) pq.erase(pq.begin());
			int neighbor_type = rovers[pq.front().second]->type; // TEMPORARY EDIT; ALL 1 TYPE

			if (type_blind){
				 neighbor_type = Rover::NORMAL;
			}
			S[i].push_back(neighbor_type); // Appends as the last element
		}
		return S;
	}
	std::vector<double> getRewards(){
		double G_sum = 0.0;
		for (int i=0; i<POIs.size(); i++){
			DistIDPairList pq = getSortedRoverDistances(POIs[i]);
			DistIDPairList::iterator it = pq.begin();
			double dij = it->first;
			it++;
			double dik = it->first;

			// BOUND CLOSENESS
			if (dij<1.0) dij = 1.0;
			if (dik<1.0) dik = 1.0;

			double Nij= 0.0;
			if (dij<domain_params->observation_distance){
				Nij = 1.0;
			}
			double Nik = 0.0;
			if (dik<domain_params->observation_distance){
				Nik = 1.0;
			}
			G_sum += 2.0*(Nij*Nik)/(dij+dik);
		}
		return vector<double>(rovers.size(),G_sum);
	}

	void simulateStep(std::vector<std::vector<double> > agent_actions){
		for (int i=0; i<rovers.size(); i++){
			// Adjust actions to match type
			if (rovers[i]->type==Rover::ERRATIC){
				if (COIN<0.2){
					// Take a random action
					agent_actions[i][0] = COIN_FLOOR0;
					agent_actions[i][1] = COIN_FLOOR0;
				}
			} else if (rovers[i]->type==Rover::FAST){
				agent_actions[i][0]*=10.0;
				agent_actions[i][1]*=10.0;
			} else if (rovers[i]->type==Rover::SLOWTURN){
				agent_actions[i][0]/=10.0;
				agent_actions[i][0]/=10.0;
			}

			rovers[i]->adjustPosition(agent_actions[i][0],agent_actions[i][1]);
		}
	}
	
	void simulateRoverEpisodeTypes(vector<vector<NeuralNet*> > NNSet, vector<double> &fitnesses, matrix2d &xi);
	
	StepClockLogger log;
	vector<Rover*> rovers;

	void logStep(int step){
		log.logStep(step);
	}
	void reset();
	

private:
	
	World2D* world;
	vector<POI*> POIs;
	RoverDomainParameters* domain_params;

	//void setRoverStateTypes(Rover *r, int neighbor_type);
	//void setRoverActions(vector<vector<NeuralNet*> > NN_set);
	//void moveRovers(); // moves many rovers at once
	//std::vector<double> getReward();
	DistIDPairList getSortedRoverDistances(Point2D *me);
	
	void addRover();
	void addPOI();
	std::vector<std::vector<double> > roverPositionSave;


};
