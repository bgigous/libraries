#pragma once
#include "../../Math/easymath.h"
#include "../../FileIO/FileOut.h"
#include "UTMModesAndFiles.h"
#include "UAV.h"
#include <string>
#include <memory>

class IAgentManager{
public:
	IAgentManager(UTMModes* params):
		params(params),
		metrics(std::vector<Reward_Metrics>(params->get_n_agents(),Reward_Metrics(params->get_n_types())))
	{
	}

	UTMModes* params;
	virtual matrix2d actions2weights(matrix2d agent_actions)=0;
	matrix3d agentActions;
	void logAgentActions(matrix2d agentStepActions){
		agentActions.push_back(agentStepActions);
	}
	void exportAgentActions(int fileID){
		FileOut::print_vector(agentActions,"visualization/actions"+std::to_string(fileID)+".csv");
	}

	//! Metrics relating to a reward. Each of these maps to an agent.
	struct Reward_Metrics{
		Reward_Metrics(int n_types):
			local(matrix1d(n_types,0.0)),
			G_avg(matrix1d(n_types,0.0)),
			G_minus_downstream(matrix1d(n_types,0.0)),
			G_random_realloc(matrix1d(n_types,0.0))
		{

		}

		matrix1d local;

		// counterfactuals
		matrix1d G_avg;
		matrix1d G_minus_downstream;
		matrix1d G_random_realloc;
	};
	
	int* steps; // keeps time with simulator

	double global(){
		double sum=0.0;
		for (Reward_Metrics r:metrics){
			sum += easymath::sum(r.local);
			r.local;
		}
		return sum;
	}

	void reset(){
		metrics = std::vector<Reward_Metrics>(params->get_n_agents(),Reward_Metrics(params->get_n_types()));
	}

	std::vector<Reward_Metrics> metrics;
	virtual void add_delay(const UAV_ptr &u)=0;
	virtual void detect_conflicts()=0;

	void add_average_counterfactual();
	virtual void add_downstream_delay_counterfactual(const UAV_ptr &u)=0;
	/*virtual void add_downstream_conflict_counterfactual()=0;
	virtual void add_realloc_delay_counterfactual()=0;
	virtual void add_realloc_conflict_counterfactual()=0;*/
};