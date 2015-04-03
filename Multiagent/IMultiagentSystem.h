#pragma once

// STL includes
#include <vector>

// Library includes
#include "../SingleAgent/IAgent.h"


typedef std::vector<std::vector<double> > matrix2d;
typedef std::vector<double> matrix1d;

class IMultiagentSystem
{
	// Agent container

public:
	IMultiagentSystem(void);
	virtual ~IMultiagentSystem(void);

	std::vector<IAgent*> agents; // Set of agents in the system (set externally)

	matrix2d getActions(matrix2d S);
	void updatePolicyValues(matrix1d R);
};