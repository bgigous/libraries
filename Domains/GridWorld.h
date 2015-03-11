#pragma once
#include "../EnvironmentBounds/EnvironmentBounds.h"
#include "../State/State.h"
#include "../../Point2D.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <random>
#include "../NeuralNet/NeuralNet.h"
#include "../../easymath.h"
#include "../../World2D.h"

#define TRAVELBOUND 10.0 // amount you can travel in any direction (scaling parameter for NN)
#define CLOSEBOUND 5.0 // distance away and still close (past this medium or far)
#define MEDIUMBOUND 10.0 // distance away and still medium (past this far)

class GridRover: public Point2D{
	public:
	GridRover(World2DParameters* world_params): Point2D(world_params){

	}
	void randWalk(){
		int dx = rand()%3-1; // (-1,0,1)
		int dy = rand()%3-1;
		adjustPosition(dx,dy);
	}
};

class GridWorld{
public:
	// Generation functions
	GridWorld();
	~GridWorld();

	// Class variables
	enum Direction{Q1,Q2,Q3,Q4,NSECTORS};
	enum DistanceDivision{CLOSE,MEDIUM,FAR,NDISTANCES};
	int nRovers;
	double percentFail; // failure/type enactment percentage
	double deltaO;
	std::vector<double> performanceVals;
	std::vector<std::vector<int> > startingPositions;
	std::vector<std::vector<double> > staticRoverPositions;
	World2D* world;

	// Initialization
	void generateRovers();
	void generateStaticRoverPositions();

	// Sensor functions
	std::pair<Direction,DistanceDivision> relativePosition(double x1, double y1, double x2, double y2);
	void logRoverPositions();
	easymath::PairQueueAscending sortedRoverDists(double xref, double yref); // Rovers, sorted by distance away from reference point
	int nearestNeighbor(int roverID); // just get the nearest


	double gridDistance(double x1, double y1, double x2, double y2);
	double gridDistance(Point2D &r1, Point2D &r2);
	
	EnvironmentBounds bounds;
	std::vector<GridRover> rovers;
	std::vector<std::vector<char> > positionData; // deprecated?

	
	// REWARD STRUCTURES
	virtual double getLocalReward(int me)=0;
	virtual double getGlobalReward()=0;
	virtual std::vector<double> getDifferenceReward()=0;

	
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
	void logPosition(Point2D r);

	// Reset
	void randomizePositions(std::vector<Point2D> &r);
	void randomizeErrTypes(std::vector<Point2D> &rovers);
	void resetStaticRovers();

};

