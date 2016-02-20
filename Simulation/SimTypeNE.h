#pragma once

#include "SimNE.h"
#include "../Multiagent/MultiagentTypeNE.h"

class SimTypeNE: public SimNE
{
public:
	SimTypeNE(IDomainStateful *domain, MultiagentTypeNE::TypeHandling type_mode);
	SimTypeNE(IDomainStateful *domain, MultiagentNE* MAS, MultiagentTypeNE::TypeHandling type_mode);
	~SimTypeNE(void);
	
	NeuroEvoParameters* NE_params;
	MultiagentTypeNE::TypeHandling type_mode;
	virtual matrix2d getActions();
};

