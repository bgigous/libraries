/*@file roverdomain.h

*/

#pragma once

#include "../../SingleAgent/NeuralNet/NeuralNet.h"
#include "../IDomainStateful.h"
#include "../../Math/easymath.h"
#include "../GridWorld.h"
#include "Rover.h"

class IRoverReward {
	// BASE CLASS FOR ROVER REWARDS

};

class RoverRewardGlobal {
	// TODO: WRITE
};

class RoverRewardDifference {
	// TODO: WRITE
};

class RoverDomainParameters : public IDomainStatefulParameters {
public:
	RoverDomainParameters(void);
	~RoverDomainParameters(void);
	std::vector<int> rover_types_fixed;

	int get_n_state_elements() { return 8; };
	int get_n_control_elements() { return 2; };
	int get_n_steps() { return 100; };
	int get_n_agents() { return 10; }; // number of rovers
	int get_n_types() { return Rover::NUMTYPES; };
	int get_n_POI() { return 20; };
	double get_rover_move_scale() { return 2.0; };
	double get_observation_distance() { return 10.0; };

	matrix1d distance_bounds;
	double get_travel_bound() { return 10.0; }; // Scaling factor for neural network
	double get_deltaO() { return 10.0; };
};

class POI : public easymath::XY {
public:
	POI(easymath::XY);
	~POI();
	double val;
};

class RoverDomain : public IDomainStateful
{
public:
	RoverDomain(void);
	~RoverDomain(void);



private:


	std::vector<easymath::XY> fixedRoverPositions;
	std::vector<easymath::XY> fixedPOIPositions;
	RoverDomainParameters* domain_params;

	
	matrix2d getStates();
	matrix3d getTypeStates();
	matrix1d getRewards();
	void simulateStep(matrix2d agent_actions);
	std::vector<Rover*> rovers;
	void reset();


	std::vector<Rover*> get_neighbors(Rover* ref, std::vector<Rover*> all_rovers, int n_neighbors);
	matrix1d rover_quadrant_count(Rover* ref);
	matrix1d POI_quadrant_count(Rover* ref);
	
	matrix1d getPerformance();
	void logStep() {};
	std::string createExperimentDirectory() { return std::string(); };
	void synch_step(int* step) {}; // note: check whether this needs to change anything in this domain

	GridWorld* world;
	std::vector<POI*> POIs;

	const double deltaO;
	matrix1d performanceVals;
	std::vector<std::vector<int> > startingPositions;
	std::vector<easymath::XY> staticRoverPositions;
	// Sensor functions
	int distance_discretization(easymath::XY &p1, easymath::XY &p2);
	matrix2d pathLog;
	
	// Rewards
	double getLocalReward(int me);
	double getGlobalReward();
	matrix1d getDifferenceReward();
	matrix1d getState(int me);
};