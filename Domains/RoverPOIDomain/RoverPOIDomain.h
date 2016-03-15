#pragma once
#include "../GridWorld.h""

typedef easymath::XY POI;

class RoverPOIDomain :
	public GridWorld
{
public:
	RoverPOIDomain(void);
	~RoverPOIDomain(void);

	int nPOIs;
	std::vector<POI> POIs;
	bool teleportation;
	void initializeRoverDomain(bool usingTypesSet, bool teleportationSet, std::string rewardSet, bool firstInitialization=true);
	void simulateRunRoverDomain();
	//void simulateEpisodeRoverDomain();
	void simulateEpochRoverDomain();

	// Rewards
	double getLocalReward(int me);
	double getGlobalReward();
	std::vector<double> getDifferenceReward();

	void resetForEpoch();

	
	//void generateStaticPOIPositions();
	//PairQueueAscending sortedPOIDists(double xref, double yref);
	matrix1d getState(int me);
	
	//void generatePOIs();
};