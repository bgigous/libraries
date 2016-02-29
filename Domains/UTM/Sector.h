#pragma once

#include "UTMModesAndFiles.h"
#include "Link.h"

class Sector{
public:
	// An area of space that contains some fixes
	Sector(easymath::XY xy, int sectorIDset, int n_agents, std::vector<int> connections, UTMModes* params);
	Sector():
		ID(0)
	{}; // default constructor
	~Sector(){};

	// Location properties
	const int ID; // the identifier for this sector
	const std::vector<int> connections;
	const easymath::XY xy; // sector center
};

//! Class that manages sectors as agents
class SectorAgentManager: public IAgentManager{
public:
	SectorAgentManager(std::vector<Link*> links_set, int n_types_set, std::vector<Sector*> sectors_set, UTMModes* params):
		IAgentManager(params),n_types(n_types_set)
	{
		links = links_set;
		sectors = sectors_set;

		for (Link* l:links){
			links_toward_sector[l->target].push_back(l);
		}
	};
	~SectorAgentManager(){};

	std::vector<Link*> links; // links in the entire system
	std::map<int,std::vector<Link*> > links_toward_sector;
	const int n_types;

	virtual matrix2d actions2weights(matrix2d agent_actions){
		// Converts format of agent output to format of A* weights

		matrix2d weights = easymath::zeros(n_types,links.size());;
		for (uint i=0; i<links.size(); i++){
			for (int j=0; j<n_types; j++){
				int s = links[i]->source;
				int d = j*(n_types-1) + links[i]->cardinal_dir; // type/direction combo
				weights[j][i] = agent_actions[s][d]*1000.0;	// turns into 'type', 'edge'
			}
		}
		return weights;
	}
	std::vector<Sector*> sectors;

	void add_delay(UAV* u){
		//double tt = links.at(u->next_link_ID)->predicted_traversal_time()[0];
		//printf("at link %i: %fs to wait ", u->next_link_ID,tt);
		//system("pause");
		metrics.at(u->curSectorID()).local[u->type_ID]++;
	}
	void add_downstream_delay_counterfactual(UAV* u){
				// remove the effects of the UAV for the counterfactual..
		// calculate the G that means that the UAV's impact is removed...

		if (params->_reward_mode==UTMModes::DIFFERENCE_AVG_SQ ||
			params->_reward_mode==UTMModes::DIFFERENCE_DOWNSTREAM_SQ ||
			params->_reward_mode==UTMModes::DIFFERENCE_REALLOC_SQ ||
			params->_reward_mode==UTMModes::GLOBAL_SQ){
				// squared reward... needs all of delay at that node at that time to be squared
				printf("SQUARED TODO");
				exit(1);
		}
		else{
			for (size_t i=0; i<metrics.size(); i++){
				if (u->sectors_touched.count(i)==0){
					metrics[i].G_minus_downstream[u->type_ID]++;
				} else {
					continue;
				}
			}
		}
	}
	void detect_conflicts(){
		// all links going TO the sector are considered to contribute to its conflict
		for (size_t s=0; s<sectors.size(); s++){
			std::vector<Link*> toward = links_toward_sector[s];

			// also defined by link capacities
			if (params->_reward_mode==UTMModes::DIFFERENCE_AVG_SQ ||
				params->_reward_mode==UTMModes::DIFFERENCE_DOWNSTREAM_SQ ||
				params->_reward_mode==UTMModes::DIFFERENCE_REALLOC_SQ ||
				params->_reward_mode==UTMModes::GLOBAL_SQ){
					for (size_t i=0; i<toward.size(); i++){
						for (size_t j=0; j<toward[i]->traffic.size(); j++){
							int over_capacity = toward[i]->number_over_capacity(j);
							if (over_capacity>0)
								metrics[i].local[j]+=over_capacity*over_capacity;
						}
					}
			} else {
				for (size_t i=0; i<toward.size(); i++){
					for (size_t j=0; j<toward[i]->traffic.size(); j++){
						int over_capacity = toward[i]->number_over_capacity(j);
						if (over_capacity>0)
							metrics[i].local[j]+=over_capacity;
					}
				}
			}
		}
	}
};
