/*@file roverdomain.h

*/

#pragma once

#include "../../SingleAgent/NeuralNet/NeuralNet.h"
#include "../IDomainStateful.h"
#include "../../Math/easymath.h"
#include "../GridWorld.h"

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

class Rover: public easymath::XY{
public:
	int ID;
	Rover(GridWorld* world, RoverDomainParameters* domain_params_set);
	RoverDomainParameters* domain_params;
	~Rover();
	void selectAction(NeuralNet* NN);
	matrix1d stateInputs;
	enum RoverType{FAST,SLOWTURN,ERRATIC,NORMAL,NUMTYPES};
	RoverType type;
	matrix1d action; // 2 outputs: dx,dy
};

class POI: public easymath::XY{
public:
	POI(easymath::XY);
	~POI();
};

class RoverDomain: public IDomainStateful
{
public:
	RoverDomain(void);
	~RoverDomain(void);


	std::vector<easymath::XY> fixedRoverPositions;
	std::vector<easymath::XY> fixedPOIPositions;
	
	matrix2d getStates(){
		matrix2d S = matrix2d(rovers.size(),matrix1d(domain_params->n_state_elements,0.0));
		for (int i=0; i<rovers.size(); i++){

			// Count number of rovers in each fix, as percentage of total rovers
			for (int j=0; j<rovers.size(); j++){
				if (i==j) continue;
				int Qj = easymath::cardinal_direction(*rovers[i]-*rovers[j]); // Quadrant of rover j, relative to i
				S[i][Qj] += 1.0/double(rovers.size()-1);
			}

			// Count number of POIs in each fix
			for (int j=0; j<POIs.size(); j++){
				int Qj = easymath::cardinal_direction(*rovers[i]-*POIs[j]); // Quadrant of POI j, relative to i
				S[i][Qj+4]+=1.0/double(POIs.size()); // Adding this to the state (offset by earlier state elements)
			}
			
			Rover * ref = rovers[i];
			sort(rovers.begin(),rovers.end(),[ref](Rover* r1, Rover* r2){
				if (r1->ID == ref->ID) return false;
				else if (r2->ID == ref->ID) return true;
				else return (manhattan_distance(*r1, *ref) < manhattan_distance(*r2, *ref));
			});

			int neighbor_type;
			if (type_blind){
				 neighbor_type = Rover::NORMAL;
			} else {
				neighbor_type= rovers[0]->type;
			}
			S[i].push_back(neighbor_type); // Appends as the last element
		}
		return S;
	}
	
	matrix1d getRewards(){
		double G_sum = 0.0;
		for (int i=0; i<POIs.size(); i++){
			matrix1d d = matrix1d(rovers.size());
			for (size_t j = 0; j < rovers.size(); j++)
			{
				d[j] = easymath::manhattan_distance(*POIs[i], *rovers[j]);
				if (d[j] < 1.0) d[j] = 1.0;	 // bound closeness
			}
			std::sort(d.begin(),d.end());

			double Nij = d[0] < domain_params->observation_distance;
			double Nik = d[1] < domain_params->observation_distance;
			G_sum += 2.0*(Nij*Nik)/(d[0]+d[1]);
		}
		return matrix1d(rovers.size(),G_sum);
	}

	void simulateStep(matrix2d agent_actions){
		for (int i=0; i<rovers.size(); i++){
			// Adjust actions to match type
			if (rovers[i]->type==Rover::ERRATIC){
				if (easymath::rand(0,1)<0.2){
					// Take a random action
					agent_actions[i][0] = easymath::rand(0, 0.99);
					agent_actions[i][1] = easymath::rand(0, 0.99);
				}
			} else if (rovers[i]->type==Rover::FAST){
				agent_actions[i][0]*=10.0;
				agent_actions[i][1]*=10.0;
			} else if (rovers[i]->type==Rover::SLOWTURN){
				agent_actions[i][0]/=10.0;
				agent_actions[i][0]/=10.0;
			}

			(XY)*rovers[i] = *rovers[i] + XY(agent_actions[i][0], agent_actions[i][1]);
		}
	}
	
	void simulateRoverEpisodeTypes(std::vector<std::vector<NeuralNet*> > NNSet, matrix1d &fitnesses, matrix2d &xi);
	
	std::vector<Rover*> rovers;

	void reset();
	

private:
	
	GridWorld* world;
	std::vector<POI*> POIs;
	RoverDomainParameters* domain_params;
};
