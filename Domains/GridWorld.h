#pragma once
#include "IDomainStateful.h"
#include "../Math/easymath.h"
#include <set>



class GridParameters;

// Moving objects in the gridworld
class GridRover : public easymath::XY {
public:
	GridRover(double xset, double yset) :
		XY(xset,yset)
	{
	}
	void randWalk() {
		x += rand() % 3 - 1; // (-1,0,1)
		y += rand() % 3 - 1;
	}
};

class GridWorld :	// NOTE: make gridworld not a domain
	public IDomainStateful
{
public:
	// Generation functions
	GridWorld();
	~GridWorld();

	static const double TRAVELBOUND; // 10.0 // amount you can travel in any direction (scaling parameter for NN)
	static const double CLOSEBOUND; // 5.0 // distance away and still close (past this medium or far)
	static const double MEDIUMBOUND; //10.0 // distance away and still medium (past this far)
	static const int n_bounds = 3;
	static const int n_directions = 4;

	// Class variables
	GridParameters* params;
	typedef int Direction;	// for use with easymath::cardinalDirection
	enum DistanceDivision { CLOSE, MEDIUM, FAR, NDISTANCES };
	int nRovers;
	const double percentFail; // failure/type enactment percentage
	const double deltaO;
	const int xsize;
	const int ysize;
	matrix1d performanceVals;
	std::vector<std::vector<int> > startingPositions;
	std::vector<easymath::XY> staticRoverPositions;

	// Initialization
	void generateRovers();
	void generateStaticRoverPositions();

	// Sensor functions
	DistanceDivision distance_discretization(easymath::XY &p1, easymath::XY &p2);
	void logRoverPositions();

	std::vector<GridRover> rovers;
	std::vector<std::vector<char> > positionData; // deprecated?


	// REWARD STRUCTURES
	virtual double getLocalReward(int me) = 0;
	virtual double getGlobalReward() = 0;
	virtual std::vector<double> getDifferenceReward() = 0;


	// Walking functions
	void roverRandomWalk();

	// IO
	void exportGLog(std::string fileName);
	void exportPositionLog(std::string fileName, int rover);
	void exportCSV2(std::vector<std::vector<double> > myvec, std::string fileName);

	// Terrain changes
	void resetPositionData();
	void reducePositionValues();

	std::vector<std::vector<double> > pathLog;
	std::vector<double> GLog;

	// Logging
	void logPosition(easymath::XY r);

	// Reset
	void randomizePositions(std::vector<XY> &r);
	void randomizeErrTypes();
	void resetStaticRovers();

};

class GridParameters : public IDomainStatefulParameters {
public:
	GridParameters() {};
	~GridParameters() {};
	int get_n_state_elements()
	{
		return GridWorld::n_bounds;
	}
	int get_n_control_elements()
	{
		return 2; // heading change, speed
	}
	int get_n_steps() {
		return 100;
	}
	int get_n_types() {
		return 1;
	}
};