#include "UTMAgentReward.h"

using namespace easymath;

void IAgentManager::add_average_counterfactual(){
	// Replace the impact of the individual with the average delay
	for (unsigned int i=0; i<metrics.size(); i++){
		matrix1d m = zeros(params->get_n_types());
		for (unsigned int j=0; j<metrics.size(); j++){
			if (i!=j) m = m + metrics[i].local;
			else m = m + (metrics[i].local/(*steps));
		}
		metrics[i].G_avg = m;
	}
}