#include "IDomainStateful.h"


IDomainStateful::IDomainStateful(IDomainStatefulParameters* params):
	n_control_elements(params->get_n_control_elements()),
	n_state_elements(params->get_n_state_elements()),
	n_steps(params->get_n_steps()),
	n_types(params->get_n_types())
{
}


IDomainStateful::~IDomainStateful(void)
{
}
