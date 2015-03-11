#include "PredatorPreyDomain.h"

using namespace std;
using namespace easymath;

Prey::Prey(World2DParameters* world_params): Point2D(world_params),
	is_caught(false)
{
}

PredatorPreyDomain::PredatorPreyDomain(PredatorPreyDomainParameters* paramsSet):
	params(paramsSet),
	stateMaxes(vector<double>(params->numNonTypeElements,paramsSet->sightRange)),
	stateMins(vector<double>(params->numNonTypeElements,-params->sightRange)),
	deltaPredPrey(matrix_2d(params->nPredators)),
	deltaPreyPred(matrix_2d(params->nPrey)),
	captured(vector<bool>(params->nPrey,false)),
	stepsToCapture(vector<int>(params->nPrey,0)),
	predStepsToCapture(vector<int>(params->nPredators,0)),
	allCaptured(false),
	world(new World2D())
{	
	stateMaxes[Predator::orientationMe] = 1.0;
	stateMins[Predator::orientationMe] = 0.0; // ALREADY SCALED

	for (int i=0; i<params->nPredators; i++){
		addPredator();
	}
	for (int i=0; i<params->nPrey; i++){
		addPrey();
	}
}

void PredatorPreyDomain::addPredator(){
	Predator *newPred = new Predator(world->world_params);
	predators.back()->type = params->fixedTypes[predators.size()];
	world->setRandUnblockedXY(newPred);
	world->all_objects->push_back(newPred);
	predators.push_back(newPred);
}

void PredatorPreyDomain::addPrey(){
	Prey *newPrey = new Prey(world->world_params);
	world->setRandUnblockedXY(newPrey);
	world->all_objects->push_back(newPrey);
	prey.push_back(newPrey);
}


PredatorPreyDomain::~PredatorPreyDomain(void)
{
}

bool PredatorPreyDomain::checkKill(Prey *p){
	// Checks if any predators are co-located onto Prey object p
	for (int i=0; i<predators.size(); i++){
		return world->getPointDistance(predators[i],p)==0;
	}
	return false;
}

double PredatorPreyDomain::getAverageBestNNScore(std::vector<NeuroEvo*> &NESet){
	double NNBestSum = 0.0;
	for (int i=0; i<NESet.size(); i++){
		NNBestSum += NESet[i]->getBestMemberVal();
	}
	return NNBestSum/double(NESet.size());
}

void PredatorPreyDomain::detectZero(){
	// Go through all prey and see if any co-located with predators
	for (int i=0; i<prey.size(); i++){
		if (checkKill(prey[i])){
			printf("Zero detected.");
		}
	}
}

DistIDPairList PredatorPreyDomain::getSortedPredatorDists(Point2D *me){
	std::vector<P> dists(predators.size(),std::make_pair<double,int>(0.0,0));
	for (int i=0; i<predators.size(); i++){
		dists[i]=P(world->getPointDistance(me,predators[i]),i);
	}
	DistIDPairList q(dists.begin(),dists.end());
	return q;
}

DistIDPairList PredatorPreyDomain::getSortedPreyDists(Point2D *me){
	std::vector<P> dists(prey.size(),std::make_pair<double,int>(0.0,0));
	for (int i=0; i<prey.size(); i++){
		dists[i]=P(world->getPointDistance(me,prey[i]),i);
	}
	DistIDPairList q(dists.begin(),dists.end());
	return q;
}

std::vector<double> PredatorPreyDomain::getPreyCentricReward(void){
	double Gtotal=0.0;
	//std::vector<double> GCounterfact(predators.size());
	for (int i=0; i<prey.size(); i++){
		if (prey[i]->is_caught){
			Gtotal-=stepsToCapture[i];
		} else{
			double deltaSum=0.0;
			for (int t=0;t<deltaPreyPred[i].size(); t++){
				deltaSum += deltaPreyPred[i][t];
			}
			Gtotal-=deltaSum;
		}
	}

	return std::vector<double>(predators.size(),Gtotal);
}



vector<double> PredatorPreyDomain::getPredCentricGlobalReward(void){
	vector<double> rwdvect = getPredCentricLocalReward();
	double rwd = mean(rwdvect);
	return vector<double>(predators.size(),rwd);
}

vector<double> PredatorPreyDomain::getPredCentricLocalReward(void){
	std::vector<double> rwdvect(predators.size(),0.0);
	for (int i=0; i<predators.size(); i++){
		if (predators[i]->involved_in_capture){
			rwdvect[i] = -predStepsToCapture[i];
		} else {
			rwdvect[i] = -sum(deltaPredPrey[i]);
		}
	}

	return rwdvect;
}

std::vector<double> PredatorPreyDomain::getLocalPredReward(void){
	//getPredCentricLocalReward();
	//getPredCentricGlobalReward();

	//prey-centric
	return getPreyCentricReward();
}

bool PredatorPreyDomain::checkCapture(Prey *p){
	// Check if 4 predators within capture distance at once
	double capture_distance = 1.0;
	int capture_requirement = 4; // 4 predators required for capture
	int count = 0;
	for (int i=0; i<predators.size(); i++){
		if (world->colocated(p,predators[i],capture_distance)) count++;
	}
	return count>=capture_requirement;
}

void PredatorPreyDomain::setUniqueRandomPredPreyPositions(){
	set<XY> other_points;
	for (vector<Predator*>::iterator p = predators.begin(); p!=predators.end(); p++){
		(*p)->setUniquePoint(other_points);
		other_points.insert(XY((*p)->x,(*p)->y));
	}
	for (vector<Prey*>::iterator p = prey.begin(); p!=prey.end(); p++){
		(*p)->setUniquePoint(other_points);
		other_points.insert(XY((*p)->x,(*p)->y));
	}
}

void PredatorPreyDomain::initializePredPreyEpisode(){
	setUniqueRandomPredPreyPositions();

	deltaPredPrey = matrix_2d(predators.size());
	deltaPreyPred = matrix_2d(prey.size());
	stepsToCapture=std::vector<int>(prey.size(),0);
	predStepsToCapture=std::vector<int>(predators.size(),0);
	captured=std::vector<bool>(prey.size(),false);
	allCaptured = false;


	for (int i=0; i<predators.size(); i++){
		deltaPredPrey[i] = std::vector<double>(params->steps,0.0);
		predators[i]->involved_in_capture=false;
	}
	for (int i=0; i<prey.size(); i++){
		deltaPreyPred[i] = std::vector<double>(params->steps,0.0);
		prey[i]->is_caught=false;
	}

}

/*

void PredatorPreyDomain::simulatePredPreyEpisode(std::vector<NeuralNet*> NNSet, std::vector<double> &predFitnesses, std::vector<double> &preyFitnesses){
	initializePredPreyEpisode();

	for (int i=0; i<params->steps; i++){
		//printf("Step %i\n",i);
		
		for (int j=0; j<predators.size(); j++){
			if (!predators[j]->involved_in_capture){
				getPredatorState(predators[j]); // sets stateInputs
				predators[j]->selectAction(NNSet[j]); // uses stateInputs previously set
			}
		}

		matrix_2d preyActions(prey.size());
		setPreyActions(preyActions);
		movePredators();
		movePrey(preyActions);

		if (!checkSystemCapture(i)){
			// logging here?
		} else {
			// logging here?
			allCaptured = true;
			break;
		}
	}
	predFitnesses = getLocalPredReward();
	preyFitnesses = getLocalPreyReward();

}*/

void PredatorPreyDomain::simulatePredPreyEpisodeTypes(std::vector<std::vector<NeuralNet*> > NNSet, std::vector<double> &predFitnesses){
	initializePredPreyEpisode();

	for (int i=0; i<params->steps; i++){
		//printf("Step %i\n",i);
		
		for (int j=0; j<predators.size(); j++){
			if (!predators[j]->involved_in_capture){
				int neighborType =0; // neighbortype set inside functions
				getPredatorStateTypes(predators[j], neighborType); // sets stateInputs
				predators[j]->selectAction(NNSet[j][neighborType]); // uses stateInputs previously set
			}
		}

		std::vector<std::vector<double> > preyActions(prey.size()); // deprecated
		setPreyActions(preyActions);
		movePredators();
		movePrey(preyActions);
		if (!checkSystemCapture(i)){
			// logging?
		} else {
			allCaptured = true;
			break;
		}
	}
	predFitnesses = getLocalPredReward();
}

std::vector<double> PredatorPreyDomain::getDifferenceReward(){
	printf("difference reward placeholder for predator prey domain");
	return std::vector<double>();
}

std::vector<double> PredatorPreyDomain::getLocalPreyReward(void){
	//printf("nonlearning prey");
	return std::vector<double>(prey.size(),0.0); // RANDOM MOTION ON PREY FOR NOW
}

double PredatorPreyDomain::getLocalReward(int me){
	// placeholder
	return 0.0;
}

double PredatorPreyDomain::getGlobalReward(){
	printf("placeholder only");
	return 0.0;
}


void PredatorPreyDomain::scaleStateInputs(vector<double> &stateInputs){
	// Scales the state inputs to be between 0 and 1
	if (stateInputs.size()!=stateMaxes.size()) printf("stateInputs and stateMaxes don't match");
	if (stateInputs.size()!=stateMins.size()) printf("statInputs and stateMins don't match");

	for (int i=0; i<stateInputs.size(); i++){
		stateInputs[i] = scaleValue01(stateInputs[i],stateMins[i],stateMaxes[i]);
	}
}

void PredatorPreyDomain::getPredatorStateTypes(Predator *p,int &neighborType){
	int stateIndex = 0;
	p->stateInputs = vector<double>(params->numNonTypeElements,0.0);
	p->stateInputs[stateIndex++] = p->orientation;

	DistIDPairList pq = getSortedPreyDists(p);
	for (int i=stateIndex; i<params->nPreySeen; i++){ // alternate x and y
		if (pq.size() && pq.front().first<params->sightRange){ // if in range
			p->stateInputs[stateIndex++] = prey[pq.front().second]->x-p->x;
			p->stateInputs[stateIndex++] = prey[pq.front().second]->y-p->y;
		} else {
			p->stateInputs[stateIndex++] = params->sightRange;
			p->stateInputs[stateIndex++] = params->sightRange;
		}
		pq.pop_front();
	}

	pq = getSortedPredatorDists(p);
	bool neighborFound = false;
	for (int i=stateIndex; i<=params->nPredatorSeen; i++){ // neighbor also includes type
		if (predators[pq.front().second]->ID==p->ID) pq.pop_front(); // remove me
		if (!neighborFound){
			neighborFound = true;
			if (pq.size()) neighborType = predators[pq.front().second]->type;
		}
		if (pq.size() && pq.front().first<params->sightRange){
			p->stateInputs[stateIndex++] = predators[pq.front().second]->x-p->x;
			p->stateInputs[stateIndex++] = predators[pq.front().second]->y-p->y;
		} else {
			p->stateInputs[stateIndex++] = params->sightRange;
			p->stateInputs[stateIndex++] = params->sightRange;
		}
	}
	scaleStateInputs(p->stateInputs);
}

Predator::Predator(World2DParameters* world_params): Point2D(world_params),
	type(PredatorTypes(rand()%int(numTypes))),
	stateInputs(std::vector<double>(PredatorPreyDomainParameters::numNonTypeElements,0.0)),
	involved_in_capture(false),
	moveDistCap(2.0)
{
	// fill with other things
	static int IDset = 0;
	ID = IDset++;
};

void Predator::selectAction(NeuralNet* NN){ // given a certain stateInputs (defined prior) get an action
	// Sets the values of actionToTake
	actionToTake = NN->predictContinuous(stateInputs); // gives [rotation dtheta][movement d]
	actionToTake[0] = 2.0*actionToTake[0]-1.0; // scale between [-1,1]


	//printf("NN out = %f,%f\n",actionToTake[0],actionToTake[1]);

	// Type implementation scales the output
	if (type==CCW){
		// scale the neural network orientation command between 0-pi
		//actionToTake[0]/=4.0; // HACK: can only turn 90deg max
		actionToTake[0]/=8.0;

		// Hack #2: immobile agent
		//actionToTake[1] = 0.0;
	} else if (type==CW){	
		// unreliable agent
		//if (rand()%100<30){ // 30% of the time random
		//	actionToTake[0] = double(rand())/double(RAND_MAX)*2.0-1.0;
		//	actionToTake[1] = double(rand())/double(RAND_MAX);
		//}
		if (rand()%100<80){ // 30% of the time random
			actionToTake[0] = boundedRand(-1.0,1.0);
			actionToTake[1] = boundedRand(0.0,1.0);
		}
	} else if (type== Fast){
		//actionToTake[1] *= 2.0; // multiply output action by 2
		actionToTake[1] *= 10.0;
	}

	actionToTake[1]*=moveDistCap;
}


PredatorPreyDomainParameters::PredatorPreyDomainParameters():
	usingTypes(true),
	rewardType(Global),
	preyMoveDistCap(2.0),
	predMoveDistCap(2.0),
	sightRange(20.0)
{
	if (nPredatorSeen>nPredators){
		printf("Abort! Hallucinating predators!");
		exit(1);
	}
	if (nPreySeen>nPrey){
		printf("Abort! Hallucinating prey!");
	}
	fixedTypes= std::vector<Predator::PredatorTypes>(nPredators);
	for (int i=0; i<fixedTypes.size(); i++){
		// Creates an even distribution of predator types
		fixedTypes[i] = Predator::PredatorTypes(i%int(Predator::PredatorTypes::numTypes));
	}
};

/*void PredatorPreyDomain::setPredatorActions(std::vector<NeuralNet*> NNSet){
	for (int i=0; i<predators.size(); i++){
		if (!predators[i]->involved_in_capture){
			getPredatorState(predators[i]);
			predators[i]->selectAction(NNSet[i]); // uses stateInputs previously set
		}
	}
}*/


void PredatorPreyDomain::setPreyActions(std::vector<std::vector<double> > &preyActions){
	for (int j=0; j<prey.size(); j++){
		preyActions[j] = std::vector<double>(2);
		if (!prey[j]->is_caught){
//			preyActions[j][0] = double(rand())/double(RAND_MAX)*2.0-1.0;
//			preyActions[j][1] = double(rand())/double(RAND_MAX);
			// escaping prey

			double fx = 0.0;
			double fy = 0.0;
			for (int i=0; i<predators.size(); i++){
				double dx = predators[i]->x - prey[j]->x;
				double dy = predators[i]->y - prey[j]->y;
				double dxy = sqrt(dx*dx + dy*dy);
				double Fpred=0;
				if (dxy>0){
					Fpred = 1/(dxy*dxy); // note: could multiply by charge force; omitting because assuming constant
					fx += Fpred*(-dx/dxy); // F*xhat, also changes direction (opposite of predators)
					fy += Fpred*(-dy/dxy); // F*yhat
				}
			}
			double fDir = (atan2(fy,fx)/3.1416); //[-1,1]
			fDir *= 0.5; // rescale to bounds [0,1]
			if (fDir<0) fDir = 1.0-fDir; // rescale to bounds [0,1]

			// ABSOLUTELY SET ORIENTATION
			prey[j]->orientation = fDir;
			preyActions[j][0]=0; // already set by fdir
			preyActions[j][1]=2.0;
		}
	}
}

void PredatorPreyDomain::movePredators(){
	for (int j=0; j<predators.size(); j++){
		if (!predators[j]->involved_in_capture) world->rotateCheckCollisionsAndMove(predators[j],predators[j]->actionToTake,params->predMoveDistCap);
	}
}

void PredatorPreyDomain::movePrey(std::vector<std::vector<double> > &preyActions){
	for (int j=0; j<prey.size(); j++){
		if (!prey[j]->is_caught) world->rotateCheckCollisionsAndMove((Point2D*)prey[j],preyActions[j],params->preyMoveDistCap);
	}
}

bool PredatorPreyDomain::checkSystemCapture(int t){
	// Goal check, mixed with reward updating
	int nCaptured = 0;
	for (int j=0; j<prey.size(); j++){
		DistIDPairList pq = getSortedPredatorDists(prey[j]);
		if (!captured[j]){
			captured[j] = checkCapture(prey[j]);
			if (!captured[j]){
				stepsToCapture[j]++;
				deltaPreyPred[j][t] += pq.front().first;
				pq.pop_front();
				deltaPreyPred[j][t] += pq.front().first; // TWO are required for capture
			}
		} else {
			nCaptured++;
		}
	}
	for (int j=0; j<predators.size(); j++){
		if(!predators[j]->involved_in_capture) predStepsToCapture[j]++;
	}

	if (nCaptured==prey.size()){
		allCaptured = true;
		return allCaptured;
	} else {
		for (int j=0; j<predators.size(); j++){
			DistIDPairList pq = getSortedPreyDists(predators[j]); // get distance to prey
			while(pq.size() && prey[pq.front().second]->is_caught){
				pq.pop_front();
			}

			if (pq.size()){
				deltaPredPrey[j][t] = pq.front().first;
			}
		}
		return allCaptured;
	}
}