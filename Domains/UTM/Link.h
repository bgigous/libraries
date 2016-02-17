#pragma once
#include "UTMAgentReward.h"

class Link{
public:
	Link(int ID, int source, easymath::XY source_loc, int target, easymath::XY target_loc, 
		int time, matrix1d capacity, int cardinal_dir):
		ID(ID),
		source(source),
		source_loc(source_loc),
		target(target),
		target_loc(target_loc),
		time(time),
		cardinal_dir(cardinal_dir),
		capacity(capacity),
		traffic(UTMModes::NTYPES,std::vector<UAV_ptr>())
	{
	}
	const int ID;
	const int source;
	const easymath::XY source_loc;
	const int target;
	const easymath::XY target_loc;
	const int time;		// Amount of time it takes to travel across link
	const int cardinal_dir;
	matrix1d capacity;
	std::vector<std::vector<UAV_ptr> > traffic;

	// Returns the predicted amount of time it would take to cross the node if the UAV got there immediately
	matrix1d predicted_traversal_time(){
		matrix1d predicted(traffic.size());
		for (int i=0; i<traffic.size(); i++){
			matrix1d waits; // the wait time for each traffic bit
			for(UAV_ptr u : traffic[i]){
				waits.push_back(u->t);
			}
			std::sort(waits.begin(),waits.end(),std::greater<double>());
			if (waits.size()-capacity[i] >= 0 )
				waits.resize(waits.size()-capacity[i]);	// drop the last [capacity[i]] elements
			predicted[i] = time + easymath::sum(waits);
		}
		return predicted;
	}


	bool at_capacity(int UAV_type){
		return capacity[UAV_type]-traffic[UAV_type].size() <= 0;
	}
};

typedef std::shared_ptr<Link> Link_ptr;

class LinkAgentManager: public IAgentManager{
public:
	// The agent that communicates with others
	LinkAgentManager(int n_edges, int n_types, std::vector<Link_ptr> links, UTMModes* params):
		n_edges(n_edges),n_types(n_types), IAgentManager(params),links(links)
	{};
	~LinkAgentManager(){};
	// weights are ntypesxnagents

	const int n_edges;
	const int n_types;

	virtual matrix2d actions2weights(matrix2d agent_actions){
		matrix2d weights = easymath::zeros(n_types,n_edges);

		for (int i=0; i<n_edges; i++){
			matrix1d predicted = links.at(i)->predicted_traversal_time();
			for (int t=0; t<n_types; t++){
				weights[t][i] = predicted[t] + agent_actions[i][t];
			}
		}
		return weights;
	}

	std::vector<Link_ptr> links;
	
	void add_delay(const UAV_ptr &u){
		metrics.at(u->cur_link_ID).local[u->type_ID]++; // adds to the local delay
	}

	void add_downstream_delay_counterfactual(const UAV_ptr &u){
		// remove the effects of the UAV for the counterfactual..
		// calculate the G that means that the UAV's impact is removed...

		if (params->_reward_mode==UTMModes::DIFFERENCE_AVG_SQ ||
			params->_reward_mode==UTMModes::DIFFERENCE_DOWNSTREAM_SQ ||
			params->_reward_mode==UTMModes::DIFFERENCE_REALLOC_SQ ||
			params->_reward_mode==UTMModes::GLOBAL_SQ){
				// squared reward... needs all of delay at that edge at that time to be squared
				printf("SQUARED TODO");
				exit(1);
		}
		else{
			for (int i=0; i<metrics.size(); i++){
				if (u->links_touched.count(i)==0){
					metrics[i].G_minus_downstream[u->type_ID]++;
				} else {
					continue;
				}
			}
		}
	} 

	void detect_conflicts(){
		if (params->_reward_mode==UTMModes::DIFFERENCE_AVG_SQ ||
			params->_reward_mode==UTMModes::DIFFERENCE_DOWNSTREAM_SQ ||
			params->_reward_mode==UTMModes::DIFFERENCE_REALLOC_SQ ||
			params->_reward_mode==UTMModes::GLOBAL_SQ){
				for (int i=0; i<links.size(); i++){
					for (int j=0; j<links[i]->traffic.size(); j++){
						int over_capacity = links[i]->traffic[j].size()-links[i]->capacity[j];
						if (over_capacity>0)
							metrics[i].local[j]+=over_capacity*over_capacity;
					}
				}
		} else {
			for (int i=0; i<links.size(); i++){
				for (int j=0; j<links[i]->traffic.size(); j++){
					int over_capacity = links[i]->traffic[j].size()-links[i]->capacity[j];
					if (over_capacity>0)
						metrics[i].local[j]+=over_capacity;
				}
			}
		}

	}
};