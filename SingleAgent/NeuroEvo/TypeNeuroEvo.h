#include "NeuroEvo\NeuroEvo.h"
#include "../easymath.h"
#include <algorithm>
#include <functional>
#include "IAgent.h"

#pragma once
class TypeNeuroEvo: public IAgent
{
public:
	TypeNeuroEvo(void);
	TypeNeuroEvo(NeuroEvoParameters* NEParams, int nTypes):
		NETypes(std::vector<NeuroEvo*>(nTypes)),
		xi(std::vector<double>(nTypes,0.0))
	{
		for (int i=0; i<NETypes.size(); i++){
			NETypes[i] = new NeuroEvo(NEParams);
		}
	};
	void deepCopyNETypes(std::vector<NeuroEvo*> &NETypesSet){
		// Creates new pointer addresses for the neuro evo instances
		// Calls functions inside neuro evo to create new neural net pointers
		// deletes original pointers
		
		for (int i=0; i<NETypes.size(); i++){
			delete NETypes[i];
		}

		NETypes = std::vector<NeuroEvo*>(NETypesSet.size());
		for (int i=0; i<NETypesSet.size(); i++){
			NETypes[i] = new NeuroEvo();
			NETypes[i]->deepCopy(*NETypesSet[i]);
			NETypes[i]->pop_member_active = NETypes[i]->population.begin();
		}

	}

	std::vector<NeuroEvo*> NETypes; // the set of neuro-evo instances for each type in the system
	void generateNewMembers(){
		for (int i=0; i<NETypes.size(); i++){
			NETypes[i]->generateNewMembers();
		}
	}
	std::vector<bool> selectNewMemberAll(){
		std::vector<bool> selected = std::vector<bool>(NETypes.size());
		for (int i=0; i<NETypes.size(); i++){
			selected[i] = NETypes[i]->selectNewMember();
		}
		return selected;
	}

	std::vector<double> getBestMemberValAll(){
		std::vector<double> memberVals = std::vector<double>(NETypes.size());
		for (int i=0; i<NETypes.size(); i++){
			memberVals[i] = NETypes[i]->getBestMemberVal();
		}
		return memberVals;
	}

	void setNNToBestMemberAll(){
		for (int i=0; i<NETypes.size(); i++){
			NETypes[i]->setNNToBestMember();
		}
	}
	void selectSurvivorsAll(){
		for (int i=0; i<NETypes.size(); i++){
			NETypes[i]->selectSurvivors();
		}
	}

	std::vector<double> getAction(std::vector<double> state){
		int neighbor_type = state.back();
		xi[neighbor_type]++;
		state.pop_back();
		return NETypes[neighbor_type]->getActiveMemberOutput(state);
	}

	std::vector<double> xi; // eligibility trace: count of how many times each neural net used in run

	void updatePolicyValues(double R){
		// Add together xi values, for averaging
		double sumXi = easymath::sum(xi);
		for (int i=0; i<NETypes.size(); i++){
			double xi_i = xi[i]/sumXi; // scaled proportional to other member values
			double V = (*NETypes[i]->pop_member_active)->evaluation; // get evaluation of active member
			V = xi_i*(R -V) + V;
			(*NETypes[i]->pop_member_active)->evaluation = V;
		}
		xi = std::vector<double>(NETypes.size(),0.0);
	}

	~TypeNeuroEvo(void);
};

