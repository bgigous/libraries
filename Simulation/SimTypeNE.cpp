#include "SimTypeNE.h"

using namespace easymath;
using namespace std;

matrix2d SimTypeNE::getActions(){
	matrix3d S = domain->getTypeStates(); // [agent id][type id][state element
	
	/*
		int nnz = 0;
	double sum = 0.0;
	for (int i=0; i<S.size(); i++){
		for (int j=0; j<S[i].size(); j++){
			for (int k=0; k<S[i][j].size(); k++){
				if (S[i][j][k]>0.0){
					nnz++;
				}
				sum += S[i][j][k];
			}
		}
	}

	printf("%i, %f\n",nnz,sum);
	*/

	return ((MultiagentTypeNE*)MAS)->getActions(S);
}

SimTypeNE::SimTypeNE(IDomainStateful* domain, MultiagentTypeNE::TypeHandling type_mode):
	SimNE(domain), type_mode(type_mode)
{
	// NOTE: THIS PART IS NEW, THERE MAY BE A BUG HERE
	//delete MAS;
	MAS = new MultiagentTypeNE(domain->n_agents,new NeuroEvoParameters(domain->n_state_elements,domain->n_control_elements),type_mode,domain->n_types);
}

SimTypeNE::SimTypeNE(IDomainStateful *domain, MultiagentNE* MAS, MultiagentTypeNE::TypeHandling type_mode):
	SimNE(domain, MAS), type_mode(type_mode)
{

}

SimTypeNE::~SimTypeNE(void)
{
	/*
	delete sim_params;
	delete ((MultiagentTypeNE*)MAS)->NE_params;
	delete ((MultiagentTypeNE*)MAS);
*/
}