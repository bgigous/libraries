#include "UTMAgentReward.h"

using namespace easymath;

void IAgentManager::add_average_counterfactual(){
	// Replace the impact of the individual with the average delay
	for (int i=0; i<metrics.size(); i++){
		matrix1d sum = zeros(params->get_n_types());
		for (int j=0; j<metrics.size(); j++){
			if (i!=j) sum = sum + metrics[i].local;
			else sum = sum + (metrics[i].local/(*steps));
		}
	}

}