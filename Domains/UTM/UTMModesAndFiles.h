#pragma once
#include <string>
//if windows use direct.h, if linux use unistd.h
#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#endif
#include "../IDomainStateful.h"




class UTMModes: public IDomainStatefulParameters{
public:

	UTMModes():
		// Mode defaults
		_reward_mode(UTMModes::RewardMode::GLOBAL),
		_airspace_mode(UTMModes::AirspaceMode::SAVED),
		_arrival_mode(UTMModes::ArrivalMode::EXACT),
		_traffic_mode(UTMModes::TrafficMode::DETERMINISTIC),
		_agent_defn_mode(UTMModes::AgentDefinition::LINK),
		_reward_type_mode(UTMModes::RewardType::DELAY),
		_search_type_mode(UTMModes::SearchDefinition::ASTAR),
		// Constants defaults
		square_reward(false),
		n_sectors(20)
	{};
	~UTMModes(){};

	// OPTION HERE FOR ONE AGENT PER LINK
	enum class AgentDefinition{SECTOR,LINK};
	AgentDefinition _agent_defn_mode;

	enum class SearchDefinition{ASTAR,RAGS};
	SearchDefinition _search_type_mode;


	// NUMBER OF SECTORS
	int n_sectors;
	int get_n_sectors(){
		return n_sectors;
	}

	// Agents
	int get_n_agents(){
		try {
			if (_agent_defn_mode == UTMModes::AgentDefinition::SECTOR) {
				return get_n_sectors();
			}
			else if (_agent_defn_mode == UTMModes::AgentDefinition::LINK) {
				return get_n_links();
			}
			else {
				throw std::runtime_error("Bad _agent_defn_mode");
			}
		}
		catch (std::runtime_error) {
			exit(1);
		}
	}

	// This should be set after the graph is constructed!
	int n_links;
	int get_n_links(){
		return n_links;
	}

	// REWARDS
	enum class RewardMode
	{
		GLOBAL,
		DIFFERENCE_DOWNSTREAM,
		DIFFERENCE_TOUCHED,
		DIFFERENCE_REALLOC,
		DIFFERENCE_AVG,
		NMODES
	};
	bool square_reward;

	RewardMode _reward_mode;
	std::string getRewardModeName(){
		std::string reward_names[size_t(RewardMode::NMODES)] = {
			"GLOBAL",
			"DIFFERENCE_DOWNSTREAM",
			"DIFFERENCE_TOUCHED",
			"DIFFERENCE_REALLOC",
			"DIFFERENCE_AVG"
		};
		return reward_names[size_t(_reward_mode)];
	}

	enum class RewardType{	// this is which types of environment variable is counted
		CONFLICTS,
		DELAY,
		NREWARDTYPES
	};
	RewardType _reward_type_mode;


	// CAPACITIES
	int get_flat_capacity(){ return 2;};


	// AIRSPACE
	enum class AirspaceMode{SAVED,GENERATED};
	AirspaceMode _airspace_mode;

	//SUBCLASS MODES/CONSTANTS
	enum class TrafficMode{DETERMINISTIC, PROBABILISTIC};
	TrafficMode _traffic_mode;
	enum class ArrivalMode{EXACT, THRESHOLD};
	ArrivalMode _arrival_mode;


	// UAV types

	enum class UAVType{SLOW, FAST, NTYPES=4};
	//const enum UAVType{SLOW,NTYPES};

	// CONSTANTS
	int get_n_state_elements(){
		if (_agent_defn_mode==UTMModes::AgentDefinition::SECTOR) return 4;
		else return 1;
	} // 4 state elements for sectors ( number of planes traveling in cardinal directions)
	int get_n_control_elements(){
		return get_n_state_elements()*int(UAVType::NTYPES);
	}
	int get_n_steps(){return 200;};
	int get_n_types(){return int(UAVType::NTYPES);};
	double get_p_gen(){return 0.5;};
	int get_gen_rate(){return 50;};
	double get_dist_thresh(){return 2.0;};
	double get_conflict_thresh(){return 2.0;};
};

class UTMFileNames{
public:
	UTMFileNames(UTMModes* modes_set=NULL):modes(modes_set){
		if (modes_set==NULL){
			modes = new UTMModes(); // Uses the default
			kill_modes=true;
		} else {
			kill_modes = false;
		}
	}
	~UTMFileNames(){
		if (kill_modes)
			delete modes;
	}
	bool kill_modes;
	UTMModes* modes;

	std::string createDomainDirectory(){
		// Saves the map information
		std::string DOMAIN_FOLDER = "Domains/";
		std::string SECTOR_FOLDER = DOMAIN_FOLDER+std::to_string(modes->get_n_sectors())+"_Sectors/";
#ifdef _WIN32
		_mkdir(DOMAIN_FOLDER.c_str());
		_mkdir(SECTOR_FOLDER.c_str());
#else
		mkdir(DOMAIN_FOLDER.c_str(), ACCESSPERMS);
		mkdir(SECTOR_FOLDER.c_str(), ACCESSPERMS);
#endif
		return SECTOR_FOLDER;
	}

	std::string createExperimentDirectory(){
		std::string EXPERIMENT_FOLDER = "Experiments/";
		// Creates a directory for the experiment and then returns that as a string
		// DIRECTORY HIERARCHY: EXPERIMENTS/NAGENTS/TRAFFIC/CAPACITY/REWARDTYPE/
		// typehandling(file name).csv assumed
		std::string AGENTS_FOLDER;

		switch(modes->_agent_defn_mode){
		case UTMModes::AgentDefinition::LINK:
			AGENTS_FOLDER = EXPERIMENT_FOLDER+"Link_agents/";
			break;
		case UTMModes::AgentDefinition::SECTOR:
			AGENTS_FOLDER = EXPERIMENT_FOLDER+"Sector_agents/";
			break;
		default:
			AGENTS_FOLDER = EXPERIMENT_FOLDER+"Unknown/";
			break;
		}

		std::string SECTOR_FOLDER = AGENTS_FOLDER+std::to_string(modes->get_n_sectors())+"_Sectors/";
		std::string TRAFFIC_FOLDER = SECTOR_FOLDER + "Rate_" + std::to_string(modes->get_gen_rate())+"/";
		std::string STEPS_FOLDER = TRAFFIC_FOLDER + std::to_string(modes->get_n_steps())+ "_Steps/";
		std::string TYPES_FOLDER = STEPS_FOLDER + std::to_string(modes->get_n_types())+"_Types/";
		std::string REWARD_FOLDER = TYPES_FOLDER + modes->getRewardModeName() + "_Reward/";

#ifdef _WIN32
		_mkdir(EXPERIMENT_FOLDER.c_str());
		_mkdir(AGENTS_FOLDER.c_str());
		_mkdir(SECTOR_FOLDER.c_str());
		_mkdir(TRAFFIC_FOLDER.c_str());
		_mkdir(STEPS_FOLDER.c_str());
		_mkdir(TYPES_FOLDER.c_str());
		_mkdir(REWARD_FOLDER.c_str());
#else
		mkdir(EXPERIMENT_FOLDER.c_str(),ACCESSPERMS);
		mkdir(AGENTS_FOLDER.c_str(),ACCESSPERMS);
		mkdir(SECTOR_FOLDER.c_str(),ACCESSPERMS);
		mkdir(TRAFFIC_FOLDER.c_str(),ACCESSPERMS);
		mkdir(STEPS_FOLDER.c_str(),ACCESSPERMS);
		mkdir(TYPES_FOLDER.c_str(),ACCESSPERMS);
		mkdir(REWARD_FOLDER.c_str(),ACCESSPERMS);
#endif
		/*
		std::string TRAFFIC_FOLDER;
		switch(modes->_traffic_mode){
		case UTMModes::DETERMINISTIC:
		TRAFFIC_FOLDER = SECTOR_FOLDER + "Deterministic_" + std::to_string(modes->get_gen_rate()) + "_Traffic/";
		break;
		case UTMModes::PROBABILISTIC:
		TRAFFIC_FOLDER = SECTOR_FOLDER + "Probabilistic_" + std::to_string(modes->get_p_gen()*100) + "_Traffic/";
		break;
		default:
		TRAFFIC_FOLDER = "UNKNOWN/";
		break;
		}

		std::string CAPACITY_FOLDER = TRAFFIC_FOLDER + std::to_string(modes->get_flat_capacity()) + "_Capacity/"; // assume uniform sector capacity
		std::string REWARD_TYPE_FOLDER;
		switch(modes->_reward_type_mode){
		case UTMModes::RewardType::CONFLICTS:
		REWARD_TYPE_FOLDER = CAPACITY_FOLDER + "Conflict_Reward/";
		break;
		case UTMModes::RewardType::DELAY:
		REWARD_TYPE_FOLDER = CAPACITY_FOLDER + "Delay_Reward/";
		break;
		default:
		REWARD_TYPE_FOLDER = "UNKNOWN/";
		break;
		}
		std::string REWARD_FOLDER = REWARD_TYPE_FOLDER + modes->getRewardModeName() + "_Reward/";

		#ifdef _WIN32
		_mkdir(EXPERIMENT_FOLDER.c_str());
		_mkdir(AGENTS_FOLDER.c_str());
		_mkdir(SECTOR_FOLDER.c_str());
		_mkdir(TRAFFIC_FOLDER.c_str());
		_mkdir(CAPACITY_FOLDER.c_str());
		_mkdir(REWARD_TYPE_FOLDER.c_str());
		_mkdir(REWARD_FOLDER.c_str());
		#else
		mkdir(EXPERIMENT_FOLDER.c_str(), ACCESSPERMS);
		mkdir(AGENTS_FOLDER.c_str(), ACCESSPERMS);
		mkdir(SECTOR_FOLDER.c_str(), ACCESSPERMS);
		mkdir(TRAFFIC_FOLDER.c_str(), ACCESSPERMS);
		mkdir(CAPACITY_FOLDER.c_str(), ACCESSPERMS);
		mkdir(REWARD_TYPE_FOLDER.c_str(), ACCESSPERMS);
		mkdir(REWARD_FOLDER.c_str(), ACCESSPERMS);
		#endif
		*/
		return REWARD_FOLDER; // returns the full directory path just generated
	}

	/*std::string createDomainDirectory(){
	std::string DOMAIN_FOLDER = "Domains/";

	}*/
private:

};
