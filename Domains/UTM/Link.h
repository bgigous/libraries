#pragma once
#include "UTMAgentReward.h"
#include <numeric>

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
		traffic(UTMModes::NTYPES,std::list<UAV*>())
	{}

	//!
	bool at_capacity(int UAV_type){
		return number_over_capacity(UAV_type) >= 0;
	}

	int number_over_capacity(int type_ID){
		return int(traffic[type_ID].size()-capacity[type_ID]);
	}
	std::vector<std::list<UAV*> > traffic;


	//! Returns the predicted amount of time it would take to cross the node if the UAV got there immediately
	matrix1d predicted_traversal_time(){

		// Get predicted wait time for each type of UAV
		matrix1d predicted(traffic.size());
		for (uint i=0; i<traffic.size(); i++){
			
			// Collect wait times on all UAVs ON the link
			matrix1d waits;
			for (UAV* u : traffic[i]) {
				waits.push_back(u->t);
			}
			
			// Sort by wait (ascending)
			std::sort(waits.begin(),waits.end(),std::less<double>());

			// Count waits for UAVs over capacity, with one space for a future UAV
			if (double(waits.size()) - capacity[i] >= 0.0) {
				waits.resize(waits.size() - capacity[i] + 1);
			}

			// Store predicted link time.
			predicted[i] = time + easymath::sum(waits);
		}

		return predicted;
	}


	//! Grabs the UAV u from link l
	void move_from(UAV* u, Link* l){//std::shared_ptr<Link> l){
		// Add to other list (u is temporarily duplicated)
		add(u);

		// Remove from previous node (l)
		l->remove(u);
	}

	//! Also sets the time
	void add(UAV* u){
		u->t = time;
		traffic.at(size_t(u->type_ID)).push_back(u);
		u->cur_link_ID = ID;
		u->loc = source_loc;
	}

	void remove(UAV* u){
		traffic[u->type_ID].erase(std::find(traffic[u->type_ID].begin(),traffic[u->type_ID].end(),u));
	}

	const int source;
	const int target;
	const int cardinal_dir;
	void reset(){
		traffic.clear();
	}


private:

	const int ID;
	const easymath::XY source_loc;
	const easymath::XY target_loc;
	const int time;		// Amount of time it takes to travel across link

	std::vector<size_t> capacity;	// Amount of each type of UAV that are allowed on the link
};

class LinkAgentManager: public IAgentManager{
public:
	// The agent that communicates with others
	LinkAgentManager(int n_edges, int n_types, std::vector<Link*> links, UTMModes* params):
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
				//weights[t][i] = predicted[t] + agent_actions[i][t];
				weights[t][i] = agent_actions[i][t]*1000.0;
			}
		}
		return weights;
	}

	std::vector<Link*> links;

	void add_delay(UAV* u){
		//printf("#%i delayed",u->ID);
		metrics.at(u->cur_link_ID).local[u->type_ID]++; // adds to the local delay
	}

	void add_downstream_delay_counterfactual(UAV* u){
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
			for (size_t i=0; i<metrics.size(); i++){
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
				for (size_t i=0; i<links.size(); i++){
					for (size_t j=0; j<links[i]->traffic.size(); j++){
						int over_capacity = links[i]->number_over_capacity(j);
						if (over_capacity>0)
							metrics[i].local[j]+=over_capacity*over_capacity;
					}
				}
		} else {
			for (size_t i=0; i<links.size(); i++){
				for (size_t j=0; j<links[i]->traffic.size(); j++){
					int over_capacity = links[i]->number_over_capacity(j);
					if (over_capacity>0)
						metrics[i].local[j]+=over_capacity;
				}
			}
		}

	}


};
