#pragma once
#include "../FileIO/FileOut.h"
#include "../Domains/IDomainStateful.h"
#include "../Multiagent/IMultiagentSystem.h"

/*
Instructions for using ISimulator:

1) declare child class of ISimulator
2) set Domain externally

*/

class ISimulator{
public:
	ISimulator(IDomainStateful* domain, IMultiagentSystem* MAS):
		domain(domain), MAS(MAS)
	{

	}
	virtual ~ISimulator(void){};
	IDomainStateful* domain;
	IMultiagentSystem* MAS;
	virtual void runExperiment(void)=0; // run the experiment
	void outputRewardLog(std::string reward_file){
		FileOut::print_vector(reward_log, reward_file);
	}
	void outputMetricLog(std::string metric_file){
		// Prints out log of performance metric/epoch
		std::string filepath = domain->createExperimentDirectory();
		FileOut::print_vector(metric_log,filepath+metric_file);
	}

	std::vector<double> reward_log;
	std::vector<double> metric_log;
};