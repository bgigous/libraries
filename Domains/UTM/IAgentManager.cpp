#include "UTMAgentReward.h"

using namespace easymath;
using namespace std;

IAgentManager::IAgentManager(UTMModes* params) :
	square_reward(params->square_reward),
	metrics(vector<Reward_Metrics>(params->get_n_agents(), Reward_Metrics(params->get_n_types())))
{
	try {
		switch (params->_reward_mode) {
		case (UTMModes::RewardMode::DIFFERENCE_AVG) :
			counterfactual = &IAgentManager::Gc_average;
			break;
		case (UTMModes::RewardMode::DIFFERENCE_DOWNSTREAM) :
			counterfactual = &IAgentManager::Gc_downstream;
			break;
		case (UTMModes::RewardMode::DIFFERENCE_REALLOC) :
			counterfactual = &IAgentManager::Gc_realloc;
			break;
		case (UTMModes::RewardMode::DIFFERENCE_TOUCHED) :
			counterfactual = &IAgentManager::Gc_touched;
			break;
		case (UTMModes::RewardMode::GLOBAL) :
			counterfactual = &IAgentManager::Gc_0;
			break;
		default:
			throw runtime_error("Bad reward mode.");
			break;
		}
	}
	catch (runtime_error) {
		exit(1);
	}

}

matrix1d IAgentManager::global() {
	double sum = 0.0;
	for (Reward_Metrics r : metrics) {
		sum += easymath::sum(r.local);
	}
	return matrix1d(metrics.size(), sum);
}

matrix1d IAgentManager::Gc_average() {
	matrix1d G_c = zeros(metrics.size());
	for (size_t i = 0; i < metrics.size(); i++)
		G_c[i] = -sum(metrics[i].G_avg);

	return G_c;
}

void IAgentManager::add_average_counterfactual(){
	size_t n_types = metrics[0].local.size();
	for (uint i=0; i<metrics.size(); i++){
		matrix1d m = zeros(n_types);
		for (uint j=0; j<metrics.size(); j++){
			if (i!=j) m = m + metrics[i].local;
			else m = m + (metrics[i].local/(*steps));
		}
		metrics[i].G_avg = m;
	}
}

matrix1d IAgentManager::Gc_downstream() {
	matrix1d G_c = zeros(metrics.size());
	for (size_t i = 0; i < metrics.size(); i++) {
		G_c[i] = -sum(metrics[i].G_minus_downstream);
	}
	return G_c;
}

matrix1d IAgentManager::Gc_realloc() {
	matrix1d G_c = zeros(metrics.size());
	for (size_t i = 0; i < metrics.size(); i++) {
		G_c[i] = -sum(metrics[i].G_random_realloc);
	}
	return G_c;
}

matrix1d IAgentManager::Gc_touched() {
	matrix1d G_c = zeros(metrics.size());
	for (size_t i = 0; i < metrics.size(); i++) {
		G_c[i] = -sum(metrics[i].G_touched);
	}
	return G_c;
}

matrix1d IAgentManager::Gc_0() {
	return easymath::zeros(metrics.size());
};

matrix1d IAgentManager::performance() {
	matrix1d G = global();
	if (square_reward)
		square(G);
	return G;
}

matrix1d IAgentManager::reward() {
	matrix1d Gc = (this->*counterfactual)();
	matrix1d G = global();

	if (square_reward) {
		square(Gc);
		square(G);
	}

	return G - Gc;
}

void IAgentManager::logAgentActions(matrix2d agentStepActions) {
	agentActions.push_back(agentStepActions);
}

bool IAgentManager::last_action_different() {
	if (agentActions.size() > 1) {
		matrix2d last_action = agentActions.back();
		matrix2d cur_action = agentActions[agentActions.size() - 2];

		return last_action != cur_action;
	}
	return true;
}

void IAgentManager::exportAgentActions(int fileID) {
	FileOut::print_vector(agentActions, "actions-" + std::to_string(fileID) + ".csv");
	FileOut::print_vector(agentStates, "states-" + std::to_string(fileID) + ".csv");
}

void IAgentManager::reset() {
	agentActions.clear();
	agentStates.clear();
	size_t n_agents = metrics.size();
	size_t n_types = metrics[0].local.size();
	metrics = std::vector<Reward_Metrics>(n_agents, Reward_Metrics(n_types));
}