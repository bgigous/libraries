#pragma once
//#include "../GridWorld/GridWorld.h"
#include "omp.h"
#include <time.h>
#include "../../easymath.h"
#include "../Point2D.h"
#include "../World2D.h"
#include "../NeuroEvo/NeuroEvo.h"
#include <fstream>

class PredatorPreyDomainParameters;


class Prey: public Point2D
{
public:
	Prey(World2DParameters* world_params);
	Prey(){
		printf("Prey erroneously called.");
	}
	bool is_caught;
};

class Predator:
	public Point2D
{
public:
	Predator(World2DParameters* world_params);

	// Types definition
	const enum PredatorTypes{Normal,CW, CCW, Fast, numTypes};
	//const enum stateElementNames{orientationMe,dxNearestPrey1,dyNearestPrey1,dxNearestPrey2,dyNearestPrey2,dxNearestNeighbor1,dyNearestNeighbor1,numNonTypeElements};
	//const enum stateElementNames {orientationMe,dxNearestPrey,dyNearestPrey,dxNearestNeighbor,dyNearestNeighbor,numNonTypeElements};
	static const int orientationMe=0; // INDEX OF ORIENTATION STATE ELEMENT (0)
	
	const double moveDistCap;
	PredatorTypes type;

	bool involved_in_capture;
	std::vector<double> stateInputs;
	std::vector<double> actionToTake; // output as a dx dy command

	void selectAction(NeuralNet* NN);
};


class PredatorPreyDomainParameters{
public:
	PredatorPreyDomainParameters();
	const enum RewardTypes{Global};
	static const int nPredators=20;
	static const int nPrey=8;
	static const int steps=100;
	static const int nPredatorSeen=nPredators-1; // don't look at yourself, predator!
	static const int nPreySeen=nPrey;
	static const int numNonTypeElements=1+nPreySeen*2+nPredatorSeen*2; // SET DEPENDING ON OBSERVATION OF OTHER PREY/PREDATORS

	const double preyMoveDistCap;
	const double predMoveDistCap;
	const double sightRange;
	bool usingTypes;
	RewardTypes rewardType;
	std::vector<Predator::PredatorTypes> fixedTypes;
	int nStateElementsCheckUsingTypes(){
		return numNonTypeElements+usingTypes?Predator::numTypes:0;
	}
};

class PredatorPreyDomain{ 
	// This domain is meant to be created and destroyed within one statistical run. Do not reuse!
	// Note: currently being revamped for multiple neural networks representing types. May cause deprecation in other areas

public:
	PredatorPreyDomain(PredatorPreyDomainParameters* paramsSet);
	~PredatorPreyDomain(void);

	
	std::vector<Predator*> predators;
	std::vector<Prey*> prey;
	std::vector<int> stepsToCapture; // this is also as large as the prey

	//void simulatePredPreyEpisode(std::vector<NeuralNet*> NNSet, std::vector<double> &predFitnesses, std::vector<double> &preyFitnesses);
	void simulatePredPreyEpisodeTypes(std::vector<std::vector<NeuralNet*> > NNSet, std::vector<double> &predFitnesses);
	
private:

	World2D* world;
	const enum PredatorTypes{Normal,CW, CCW, Fast, numTypes};
	PredatorPreyDomainParameters* params;
	
	std::vector<bool> captured; // as large as the prey; defines capture or not
	bool allCaptured;
	std::vector<double> stateMaxes; // PDSEUDO-CONSTANT, maximum values for each state input
	std::vector<double> stateMins; // PSEUDO-CONSTANT minimum values for each state input
	std::vector<int> predStepsToCapture;
	matrix_2d deltaPredPrey; // predator-centric
	matrix_2d deltaPreyPred; // prey-centric

	void addPredator();
	void addPrey();

	void detectZero();
	void outputSteps();
	//std::vector<std::vector<std::vector<double> > > deltaPreyPredDiff; // values for diff reward calc
	void getPredatorStateTypes (Predator *p, int &neighborType);
	//void getPredatorState (Predator *p);

	void setPredatorActions(std::vector<NeuralNet*> NNSet);
	void setPreyActions(std::vector<std::vector<double> > &preyActions);
	void movePredators();
	void movePrey(std::vector<std::vector<double> > &preyActions);
	bool checkSystemCapture(int t);

	double getAverageBestNNScore(std::vector<NeuroEvo*> &NESet);

	// Prey functions/elements
	bool checkCapture(Prey *p);
	bool checkKill(Prey *p);
	std::vector<double> getLocalPreyReward(void);
	std::vector<double> getLocalPredReward(void);
	
	// General domain functions
	void initializePredPreyEpisode();
	void setUniqueRandomPredPreyPositions(); // Puts predators and prey in different places

	easymath::DistIDPairList getSortedPredatorDists(Point2D *me);
	easymath::DistIDPairList getSortedPreyDists(Point2D *me);

	// parent functions
	double getLocalReward(int me); // TODO
	double getGlobalReward(); // TODO
	std::vector<double> getDifferenceReward(); // TODO
	void scaleStateInputs(std::vector<double> &stateInputs); // scales the inputs, based on params
	
	std::vector<double> getPreyCentricReward(void);
	std::vector<double> getPredCentricLocalReward(void);
	std::vector<double> getPredCentricGlobalReward(void);
};

// uncategorized functions
//void simulatePredPreyRun(PredatorPreyDomainParameters* PPparams, int nEpochs, int nTrials, std::vector<double> &GLog);
