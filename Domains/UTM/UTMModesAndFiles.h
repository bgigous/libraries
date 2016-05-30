// Copyright 2016 Carrie Rebhuhn
#ifndef DOMAINS_UTM_UTMMODESANDFILES_H_
#define DOMAINS_UTM_UTMMODESANDFILES_H_

#include <string>

#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#include <stdexcept>
#endif

#include "../IDomainStateful.h"

class UTMModes : public IDomainStatefulParameters {
 public:
     UTMModes() :
         // Mode defaults
         _reward_mode(UTMModes::RewardMode::GLOBAL),
         _airspace_mode(UTMModes::AirspaceMode::SAVED),
         _traffic_mode(UTMModes::TrafficMode::DETERMINISTIC),
         _agent_defn_mode(UTMModes::AgentDefinition::LINK),
         _reward_type_mode(UTMModes::RewardType::CONFLICTS),
         _search_type_mode(UTMModes::SearchDefinition::ASTAR),
		 _disposal_mode(UTMModes::DisposalMode::DESTROY),
         // Constants defaults
         square_reward(false),
         n_sectors(20),
         alpha(1000.0)
    {};
    ~UTMModes() {}

    double alpha; // amount that a neural network impacts the system

    // OPTION HERE FOR ONE AGENT PER LINK
    enum class AgentDefinition { SECTOR, LINK };
    AgentDefinition _agent_defn_mode;

    enum class SearchDefinition { ASTAR, RAGS };
    SearchDefinition _search_type_mode;


    // NUMBER OF SECTORS
    int n_sectors;
    int get_n_sectors() {
        return n_sectors;
    }

    // Agents
    int get_n_agents() {
        try {
            if (_agent_defn_mode == UTMModes::AgentDefinition::SECTOR) {
                return get_n_sectors();
            } else if (_agent_defn_mode == UTMModes::AgentDefinition::LINK) {
                return get_n_links();
            } else {
                throw std::runtime_error("Bad _agent_defn_mode");
            }
        }
        catch (std::runtime_error) {
            printf("Bad agent defn!");
            exit(1);
        }
    }

    // This should be set after the graph is constructed!
    int n_links;
    int get_n_links() {
        return n_links;
    }

    // REWARDS
    enum class RewardMode {
        GLOBAL,
        DIFFERENCE_DOWNSTREAM,
        DIFFERENCE_TOUCHED,
        DIFFERENCE_REALLOC,
        DIFFERENCE_AVG,
        NMODES
    };
    bool square_reward;

    RewardMode _reward_mode;
    std::string getRewardModeName() {
        std::string reward_names[size_t(RewardMode::NMODES)] = {
            "GLOBAL",
            "DIFFERENCE_DOWNSTREAM",
            "DIFFERENCE_TOUCHED",
            "DIFFERENCE_REALLOC",
            "DIFFERENCE_AVG"
        };
        return reward_names[size_t(_reward_mode)];
    }

    // This is which types of environment variable is counted
    enum class RewardType {
        CONFLICTS,
        DELAY,
        NREWARDTYPES
    };
    RewardType _reward_type_mode;


    // CAPACITIES
    int get_flat_capacity() { return 2; }


    // AIRSPACE
    enum class AirspaceMode { SAVED, GENERATED };
    AirspaceMode _airspace_mode;

    // SUBCLASS MODES/CONSTANTS
    enum class TrafficMode { DETERMINISTIC, PROBABILISTIC };
    TrafficMode _traffic_mode;


    // UAV types

    enum class UAVType { SLOW, FAST, NTYPES = 1 };
    // const enum UAVType{SLOW,NTYPES};

	// DISPOSAL
	// KEEP means when UAVs reach their goal, they are removed from the system temporarily
	// DESTROY removes permanently UAVs that have reached their goals
	enum class DisposalMode { KEEP, DESTROY };

	DisposalMode _disposal_mode;

    // CONSTANTS
    //! Returns 4 state elements for sectors (number of planes traveling in
    //! cardinal directions). Returns 1 for links.
    int get_n_state_elements() {
        if (_agent_defn_mode == UTMModes::AgentDefinition::SECTOR)
            return 4;
        else
            return 1;
    }
    int get_n_control_elements() {
        return get_n_state_elements()*get_n_types();
    }
    int get_n_steps() { return 200; }
    int get_n_types() { return static_cast<int>(UAVType::NTYPES); }
    double get_p_gen() { return 0.5; }

    //! UAVs are generated every get_gen_rate() steps
    int get_gen_rate() { return 10; }
    double get_dist_thresh() { return 2.0; }
    double get_conflict_thresh() { return 2.0; }
};


class UTMFileNames {
 public:
    explicit UTMFileNames(UTMModes* modes_set = NULL) :
        modes(modes_set) {
        if (modes_set == NULL) {
            modes = new UTMModes();  // Uses the default
            kill_modes = true;
        } else {
            kill_modes = false;
        }
    }
    ~UTMFileNames() {
        if (kill_modes)
            delete modes;
    }
    bool kill_modes;
    UTMModes* modes;

    std::string createDomainDirectory() {
        // Saves the map information
        std::string DOMAIN_FOLDER = "Domains/";
        std::string SECTOR_FOLDER = DOMAIN_FOLDER +
            std::to_string(modes->get_n_sectors()) + "_Sectors/";
#ifdef _WIN32
        _mkdir(DOMAIN_FOLDER.c_str());
        _mkdir(SECTOR_FOLDER.c_str());
#else
        mkdir(DOMAIN_FOLDER.c_str(), ACCESSPERMS);
        mkdir(SECTOR_FOLDER.c_str(), ACCESSPERMS);
#endif
        return SECTOR_FOLDER;
    }

    std::string createExperimentDirectory() {
        std::string EXPERIMENT_FOLDER = "Experiments/";
        // Creates a directory for the experiment and returns that as a string
        // DIRECTORY HIERARCHY: EXPERIMENTS/NAGENTS/TRAFFIC/CAPACITY/REWARDTYPE/
        // typehandling(file name).csv assumed
        std::string AGENTS_FOLDER;

        switch (modes->_agent_defn_mode) {
        case UTMModes::AgentDefinition::LINK:
            AGENTS_FOLDER = EXPERIMENT_FOLDER + "Link_agents/";
            break;
        case UTMModes::AgentDefinition::SECTOR:
            AGENTS_FOLDER = EXPERIMENT_FOLDER + "Sector_agents/";
            break;
        default:
            AGENTS_FOLDER = EXPERIMENT_FOLDER + "Unknown/";
            break;
        }

        std::string SECTOR_FOLDER = AGENTS_FOLDER +
            std::to_string(modes->get_n_sectors()) + "_Sectors/";
        std::string TRAFFIC_FOLDER = SECTOR_FOLDER +
            "Rate_" + std::to_string(modes->get_gen_rate()) + "/";
        std::string STEPS_FOLDER = TRAFFIC_FOLDER +
            std::to_string(modes->get_n_steps()) + "_Steps/";
        std::string TYPES_FOLDER = STEPS_FOLDER +
            std::to_string(modes->get_n_types()) + "_Types/";
        std::string REWARD_FOLDER = TYPES_FOLDER +
            modes->getRewardModeName() + "_Reward/";
        std::string ALPHA_FOLDER = REWARD_FOLDER +
            std::to_string(static_cast<int>(modes->alpha)) + "_alpha/";
#ifdef _WIN32
        _mkdir(EXPERIMENT_FOLDER.c_str());
        _mkdir(AGENTS_FOLDER.c_str());
        _mkdir(SECTOR_FOLDER.c_str());
        _mkdir(TRAFFIC_FOLDER.c_str());
        _mkdir(STEPS_FOLDER.c_str());
        _mkdir(TYPES_FOLDER.c_str());
        _mkdir(REWARD_FOLDER.c_str());
        _mkdir(ALPHA_FOLDER.c_str());
#else
        mkdir(EXPERIMENT_FOLDER.c_str(), ACCESSPERMS);
        mkdir(AGENTS_FOLDER.c_str(), ACCESSPERMS);
        mkdir(SECTOR_FOLDER.c_str(), ACCESSPERMS);
        mkdir(TRAFFIC_FOLDER.c_str(), ACCESSPERMS);
        mkdir(STEPS_FOLDER.c_str(), ACCESSPERMS);
        mkdir(TYPES_FOLDER.c_str(), ACCESSPERMS);
        mkdir(REWARD_FOLDER.c_str(), ACCESSPERMS);
#endif
        return ALPHA_FOLDER;  // returns the full directory path just generated
    }
};
#endif  // DOMAINS_UTM_UTMMODESANDFILES_H_
