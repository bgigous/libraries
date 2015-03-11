#pragma once
/*
 */

#include "Classes\PredatorPreyDomain\PredatorPreyDomain.h"
#include "PredPreyDomainSim.h"
#include "TypeNeuroEvo.h"
#include "../../master_libraries/easyio/easyio.h"
#include "easymath.h"


class PredPreyDomainSimTypeNE
{
public:
	PredPreyDomainSimTypeNE(void);
	~PredPreyDomainSimTypeNE(void);
	void runPredatorPreyTypeNE();

private:
	PredPreyDomainSimParameters* sim_params;
	void initializeDomainWithStereotypes(PredatorPreyDomain* domain, std::vector<TypeNeuroEvo*> &TNESet);
	void generateStereoTypes();
	void simulatePredPreyTypeNERun();
	void simulatePredPreyTypeNEEpoch(std::vector<TypeNeuroEvo*> &TNESet, PredatorPreyDomain* domain, double &GLogElement);
	
	// class variables
	std::vector<std::vector<NeuroEvo*> > stereotypes; // evolved stereotype approximations  [my Type][neighbor] returns correct neuroevo set; FOR COPYING after init, not changing
	PredatorPreyDomainParameters *domainParams;
	NeuroEvoParameters* NEParams;
};

