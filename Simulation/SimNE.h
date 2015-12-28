#pragma once
#include "ISimulator.h"
#include "../Multiagent/MultiagentNE.h"
#include "../Math/easymath.h"
#include <limits>
#include <sstream>

using namespace std;
using namespace easymath;

class SimNE: public ISimulator{
public:
	SimNE(IDomainStateful* domain);
	SimNE(IDomainStateful* domain, MultiagentNE* MAS);
	~SimNE(void);

	static const int n_epochs=500; // testing parameter, changed from 500;
	static const int n_trials=1;

	virtual void runExperiment();
	virtual void epoch(int ep);
	virtual matrix2d getActions(); // Gets actions based on current state: OVERLOAD FOR TYPES
};