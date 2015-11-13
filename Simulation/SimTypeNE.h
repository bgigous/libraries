#pragma once

#include "SimNE.h"
#include "../Multiagent/MultiagentTypeNE.h"

//#include "RoverDomain.h"
/*#include "TypeNeuroEvo.h"
#include "../../../master_libraries/easyio/easyio.h"*/

class SimTypeNE: public SimNE
{
public:
	SimTypeNE(IDomainStateful *domain, MultiagentTypeNE::TypeHandling type_mode);
	SimTypeNE(IDomainStateful *domain, MultiagentNE* MAS, MultiagentTypeNE::TypeHandling type_mode);
	~SimTypeNE(void);
	
	MultiagentTypeNE::TypeHandling type_mode;
	virtual matrix2d getActions(); // OVERLOADED VERSION OF SIMNE ... do we need to explicitly call this in function? ... yes
};

