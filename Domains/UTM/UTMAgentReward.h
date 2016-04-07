#pragma once
#include "../../Math/easymath.h"
#include "../../FileIO/FileOut.h"
#include "UTMModesAndFiles.h"
#include "UAV.h"
#include <string>
#include <memory>

/**
* Provides an interface for agents to interact with the simulator.
* This allows for redefinition of agents in the UTM simulation, and also
* collects information relevant to calculating difference, global, and
* local rewards. Logging of agent actions can also be performed for
* qualitative assessment of behavior.
*/

class IAgentManager {
public:
	typedef matrix1d(IAgentManager::*counterfactual_method)();
	counterfactual_method counterfactual;

	IAgentManager(UTMModes* params);	//! Constructor that sets up agent management based on UTMModes.
	~IAgentManager() {};

	bool square_reward;		//! Identifies whether squared reward is used (D = G^2-G_c^2)
	
	matrix1d global();			//! Global reward (not squared, even if square_reward set)
	matrix1d Gc_average();		//! Replace the impact of the individual with the average delay
	matrix1d Gc_downstream();	//! Remove the downstream traffic of an agent
	matrix1d Gc_realloc();		//! Reallocate an individual's traffic to other agents
	matrix1d Gc_touched();		//! Remove any traffic that touches the individual during the run
	matrix1d Gc_0();			//! Zero counterfactual (G-Gc_0=G);
	matrix1d reward();			//! This is G-counterfactual (potentially squared)
	matrix1d performance();		//! Global reward, squared if square_reward set

	virtual matrix2d actions2weights(matrix2d agent_actions) = 0;	//! Translates neural net output to link search costs
	matrix3d agentActions;		//! Stored agent actions, [*step][agent][action]
	matrix3d agentStates;		//! Stored agent states, [*step][agent][state]
	void logAgentActions(matrix2d agentStepActions);	//! Adds to agentActions
	bool last_action_different();			//! Returns true if the last action was different. Used to prompt replanning.
	void exportAgentActions(int fileID);	//! Exports list of agent actions to a numbered file.

	
	struct Reward_Metrics {
		/**
		* Metrics relating to a reward.
		* This contains all information necessary to calculate an agent's
		* reward for a run. This is to collect all information in one place,
		* so that data for this calculation is not scattered all over the
		* simulator.
		*/
		Reward_Metrics(int n_types) :
			local(zeros(n_types)),
			G_avg(zeros(n_types)),
			G_minus_downstream(zeros(n_types)),
			G_random_realloc(zeros(n_types)),
			G_touched(zeros(n_types))
		{};

		matrix1d local;					//! Local reward
		matrix1d G_avg;					//! Average counterfactual
		matrix1d G_minus_downstream;	//! Downstream counterfactual
		matrix1d G_random_realloc;		//! Random reallocation counterfactual
		matrix1d G_touched;				//! Touched counterfactual
	};

	int* steps;		//! Keeps time with simulator

	void reset();	//! Resets for the next simulation call

	std::vector<Reward_Metrics> metrics;	//! Stores metrics for each agent, used in reward calculation.
	virtual void add_delay(UAV* u) = 0;		//! Adds delay from UAV (based on agent definition)
	virtual void detect_conflicts() = 0;	//! Detects conflicts (based on agent definition)

	void add_average_counterfactual();		//! Simulates that step an agent replaced by its historical average.
	virtual void add_downstream_delay_counterfactual(UAV* u) = 0;	//! Keeps track of Gc_downstream (based on agent definition)
};