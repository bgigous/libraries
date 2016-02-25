#pragma once
#include "NeuroEvo.h"
#include "../../Math/easymath.h"
#include <algorithm>
#include <functional>
#include "../IAgent.h"

class TypeNeuroEvo: public IAgent
{
public:
	TypeNeuroEvo(void);
	TypeNeuroEvo(NeuroEvoParameters* NEParams, int nTypes):
		NETypes(std::vector<NeuroEvo*>(nTypes)),
		xi(matrix1d(nTypes,0.0))
	{
		for (NeuroEvo* ne: NETypes){
			ne = new NeuroEvo(NEParams);
		}
	};
	void deepCopyNETypes(std::vector<NeuroEvo*> &NETypesSet){
		// Creates new pointer addresses for the neuro evo instances
		// Calls functions inside neuro evo to create new neural net pointers
		// deletes original pointers

		for (NeuroEvo* ne: NETypes){
			delete ne;
		}

		NETypes = std::vector<NeuroEvo*>(NETypesSet.size());
		for (uint i=0; i<NETypesSet.size(); i++){
			NETypes[i] = new NeuroEvo();
			NETypes[i]->deepCopy(*NETypesSet[i]);
			NETypes[i]->pop_member_active = NETypes[i]->population.begin();
		}

	}


	std::vector<NeuroEvo*> NETypes; // the set of neuro-evo instances for each type in the system



	virtual void generateNewMembers(){
		for (NeuroEvo* ne : NETypes){
			ne->generateNewMembers();
		}
	}
	bool selectNewMemberAll(){
		// note; only checks the last
		bool selected = false;
		for (NeuroEvo* ne: NETypes){
			selected = ne->selectNewMember();
		}
		return selected;
	}

	matrix1d getBestMemberValAll(){
		matrix1d memberVals = matrix1d(NETypes.size());
		for (uint i=0; i<NETypes.size(); i++){
			memberVals[i] = NETypes[i]->getBestMemberVal();
		}
		return memberVals;
	}

	void setNNToBestMemberAll(){
		for (NeuroEvo* ne: NETypes){
			ne->setNNToBestMember();
		}
	}
	void selectSurvivorsAll(){
		for (NeuroEvo* ne: NETypes){
			ne->selectSurvivors();
		}
	}

	matrix1d getAction(matrix1d state){
		printf("GetAction being called in multimind setting. Need neighbor_type identification, or else this will not work. Debug before continuing.");
		system("pause");
		exit(10);
		return matrix1d();
	}


	matrix1d getAction(matrix1d state, int neighbor_type){
		xi[neighbor_type]++;
		return NETypes[neighbor_type]->getAction(state);
	}


	matrix1d getAction(matrix2d state){
		// vote among all TYPES for an action
		matrix1d action_sum = getAction(state[0],0);
		for (uint j=1; j<state.size(); j++){ // starts at 1: initialized by 0
			matrix1d action_sum_temp = getAction(state[j],j); // specifies which NN to use
			for (uint k=0; k<action_sum.size(); k++){
				action_sum[k] += action_sum_temp[k];
			}
		}
		for (double &a: action_sum){
			a /= double(state.size()); // normalize (magnitude unbounded for voting)
		}
		return action_sum;
	}

	/*	OLD
	matrix1d getAction(matrix1d state, int neighbor_type){
	int neighbor_type = state.back();
	xi[neighbor_type]++;
	state.pop_back();
	return NETypes[neighbor_type]->getAction(state);
	}
	*/

	matrix1d xi; // eligibility trace: count of how many times each neural net used in run

	void updatePolicyValues(double R){
		// Add together xi values, for averaging
		double sumXi = easymath::sum(xi);
		for (uint i=0; i<NETypes.size(); i++){
			double xi_i = xi[i]/sumXi; // scaled proportional to other member values
			double V = (*NETypes[i]->pop_member_active)->evaluation; // get evaluation of active member
			V = xi_i*(R -V) + V;
			(*NETypes[i]->pop_member_active)->evaluation = V;
		}
		xi = matrix1d(NETypes.size(),0.0);
	}

	~TypeNeuroEvo(void);
};

