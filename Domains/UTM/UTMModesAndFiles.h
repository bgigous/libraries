#pragma once
#include <string>
#include <direct.h>


class UTMModes{
public:
	UTMModes():
		// Defaults
		_capacity_mode(0), // 0 is default parameter (variable value is not 0)
		_nagents_mode(3),
		_reward_mode(GLOBAL),
		_airspace_mode(GENERATED),
		_arrival_mode(EXACT),
		_traffic_mode(DETERMINISTIC)
	{};
	~UTMModes(){};

	// OPTION HERE FOR ONE AGENT PER LINK
	enum AgentDefinition{SECTOR,LINK};
	AgentDefinition _agent_defn_mode;

	// NUMBER OF AGENTS
	static const int NAGENTNUMBERS = 4;
	int _nagents_mode;
	int get_n_agents(){
		int agent_num_trials[] = {20,30,40,50};

		if (_agent_defn_mode==SECTOR)
			return agent_num_trials[_nagents_mode];
		else{
			// MAKE A WAY FOR THIS TO BE INPUT TO UTMMODESANDFILES
			printf("Currently in link mode! Exiting.");
			system("pause");
			exit(1);
		}
	}

	// REWARDS
	static const enum RewardMode
	{
		// LINEAR REWARDS
		GLOBAL, 
		DIFFERENCE_DOWNSTREAM,
		DIFFERENCE_TOUCHED,
		DIFFERENCE_REALLOC,
		DIFFERENCE_AVG,
		// SQUARED REWARDS
		GLOBAL_SQ,
		DIFFERENCE_DOWNSTREAM_SQ,
		DIFFERENCE_TOUCHED_SQ,
		DIFFERENCE_REALLOC_SQ,
		DIFFERENCE_AVG_SQ,
		NMODES
	};
	RewardMode _reward_mode;
	std::string getRewardModeName(){
		std::string reward_names[RewardMode::NMODES] = {
			"GLOBAL", 
			"DIFFERENCE_DOWNSTREAM",
			"DIFFERENCE_TOUCHED",
			"DIFFERENCE_REALLOC",
			"DIFFERENCE_AVG",
			"GLOBAL_SQ",
			"DIFFERENCE_DOWNSTREAM_SQ",
			"DIFFERENCE_TOUCHED_SQ",
			"DIFFERENCE_REALLOC_SQ",
			"DIFFERENCE_AVG_SQ",
		};
		return reward_names[_reward_mode];
	}


	// CAPACITIES
	static const int NCAPACITYMODES = 4;
	int _capacity_mode;
	int get_flat_capacity(){
		int capacity_trials[] = {2,4,6,8};
		return capacity_trials[_capacity_mode];
	}


	// AIRSPACE
	enum AirspaceMode{SAVED,GENERATED};
	AirspaceMode _airspace_mode;

	//SUBCLASS MODES/CONSTANTS
	enum TrafficMode{DETERMINISTIC, PROBABILISTIC};
	TrafficMode _traffic_mode;
	enum ArrivalMode{EXACT, THRESHOLD};
	ArrivalMode _arrival_mode;

	
	// UAV types
	
	const enum UAVType{SLOW, FAST, NTYPES=1};
	//const enum UAVType{SLOW,NTYPES};

	// CONSTANTS
	int get_n_state_elements(){
		if (_agent_defn_mode==SECTOR) return 4;
		else return 1;
	} // 4 state elements for sectors ( number of planes traveling in cardinal directions)
	int get_n_control_elements(){
		return get_n_state_elements()*NTYPES;
	}
	int get_n_steps(){return 100;};
	int get_n_types(){return NTYPES;};
	double get_p_gen(){return 0.5;};
	int get_gen_rate(){return 10;};
	double get_dist_thresh(){return 2.0;};
	double get_conflict_thresh(){return 2.0;};
};

class UTMFileNames{
public:
	UTMFileNames(UTMModes* modes):modes(modes){
	printf("me");
	}
	~UTMFileNames(){
		printf("dying");
	}

	UTMModes* modes;
	
	std::string createExperimentDirectory(){
		std::string EXPERIMENT_FOLDER = "Experiments/";
		// Creates a directory for the experiment and then returns that as a string
		// DIRECTORY HIERARCHY: EXPERIMENTS/NAGENTS/TRAFFIC/CAPACITY/REWARDTYPE/
		// typehandling(file name).csv assumed
		std::string AGENT_FOLDER = EXPERIMENT_FOLDER+std::to_string(modes->get_n_agents())+"_Agents/";
		std::string TRAFFIC_FOLDER;
		switch(modes->_traffic_mode){
		case UTMModes::DETERMINISTIC:
			TRAFFIC_FOLDER = AGENT_FOLDER + "Deterministic_" + std::to_string(modes->get_gen_rate()) + "_Traffic/";
			break;
		case UTMModes::PROBABILISTIC:
			TRAFFIC_FOLDER = AGENT_FOLDER + "Probabilistic_" + std::to_string(modes->get_p_gen()*100) + "_Traffic/";
			break;
		default:
			TRAFFIC_FOLDER = "UNKNOWN";
			break;
		}

		std::string CAPACITY_FOLDER = TRAFFIC_FOLDER + std::to_string(modes->get_flat_capacity()) + "_Capacity/"; // assume uniform sector capacity
		std::string REWARD_FOLDER = CAPACITY_FOLDER + modes->getRewardModeName() + "_Reward/";

		_mkdir(EXPERIMENT_FOLDER.c_str());
		_mkdir(AGENT_FOLDER.c_str());
		_mkdir(TRAFFIC_FOLDER.c_str());
		_mkdir(CAPACITY_FOLDER.c_str());
		_mkdir(REWARD_FOLDER.c_str());

		return REWARD_FOLDER; // returns the full directory path just generated
	}
private:

};