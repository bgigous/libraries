#include "SimTypeNE.h"

using namespace easymath;
using namespace std;

matrix2d SimTypeNE::getActions(){
	matrix3d S = domain->getTypeStates(); // [agent id][type id][state element
	return ((MultiagentTypeNE*)MAS)->getActions(S);
}

SimTypeNE::SimTypeNE(IDomainStateful* domain, MultiagentTypeNE::TypeHandling type_mode):
	SimNE(domain), type_mode(type_mode)
{
}

SimTypeNE::~SimTypeNE(void)
{
	delete sim_params;
	delete ((MultiagentTypeNE*)MAS)->NE_params;
	delete ((MultiagentTypeNE*)MAS);
}