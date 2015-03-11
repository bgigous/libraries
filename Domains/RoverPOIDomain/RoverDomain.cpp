#include "RoverDomain.h"

	void RoverDomain::addPOI(){ // Adds a rover to the end, and to the world
		POI* newPOI = new POI(world->world_params);
		world->all_objects->push_back(newPOI); // add physical presence to world
		POIs.push_back(newPOI);
	}

void RoverDomain::addRover(){ // Adds a rover to the end, and to the world
		Rover* newRover = new Rover(world->world_params, domain_params);
		newRover->ID = rovers.size();
		newRover->type = Rover::RoverType(domain_params->rover_types_fixed[newRover->ID]); //Rover::RoverType(rovers.size()%Rover::NUMTYPES);
		world->setRandUnblockedXY(newRover);

		world->all_objects->push_back(newRover); // add physical presence to world
		rovers.push_back(newRover);
		std::vector<double> pos(2,0.0);
		pos[0] = newRover->x;
		pos[1] = newRover->y;
		roverPositionSave.push_back(pos);
	}

void StepClockLogger::addGLine(double G){
	log.push_back(std::vector<std::vector<double> >(all_objects->size())); // essentially add a 'step' full of G
	for (int i=0; i<log.back().size(); i++){
		log.back()[i] = std::vector<double>(2,G);
	}
}

void StepClockLogger::exportLog(std::string file_name, double G){
	addGLine(G);
	PrintOut::toFile3D(log,file_name);
	//PrintOut::appendToFile(file_name,G);
	log.pop_back(); // removes added line
}

void StepClockLogger::logStep(int step){
	// Increments step counter; returns false if end of run
	log[step] = std::vector<std::vector<double> >(all_objects->size());
	for (int i=0; i<all_objects->size(); i++){
		log[step][i] = std::vector<double>(2,0);
		log[step][i][0] = all_objects->at(i)->x;
		log[step][i][1] = all_objects->at(i)->y;
		//printf("(%.3f,%.3f) ",all_objects->at(i)->x,all_objects->at(i)->y);
	}
}

void StepClockLogger::reset(){
	log = std::vector<std::vector<std::vector<double> > >(max_steps);
}

StepClockLogger::StepClockLogger(int max_steps_set, std::vector<Point2D*>* all_objects_set): 
	max_steps(max_steps_set),
	log(std::vector<std::vector<std::vector<double> > >(max_steps)),
	all_objects(all_objects_set){
}

RoverDomainParameters::RoverDomainParameters(): 
	rover_move_scale(2.0),
	observation_distance(10.0),
	rover_types_fixed(vector<int>(n_rovers))
{
	for (int i=0; i<n_rovers; i++){
		//rover_types_fixed[i] = i%Rover::NUMTYPES;
		//rover_types_fixed[i] = Rover::NORMAL;
		rover_types_fixed[i] = Rover::SLOWTURN;
	}
}

RoverDomainParameters::~RoverDomainParameters(){

}

Rover::Rover(World2DParameters* world_params, RoverDomainParameters* domain_params_set):
	Point2D(world_params), domain_params(domain_params_set)
{

}

Rover::~Rover(){

}

POI::POI(World2DParameters* world_params): Point2D(world_params){

}

POI::~POI(){

}

RoverDomain::RoverDomain(void):
	world(new World2D()),
	domain_params(new RoverDomainParameters())
{
	// base class variables
	n_state_elements=domain_params->n_state_elements;
	n_control_elements=2; // dx,dy
	n_steps=domain_params->n_steps;
	n_agents=domain_params->n_rovers;
	fixed_types.assign(domain_params->rover_types_fixed.begin(), domain_params->rover_types_fixed.end());
	n_types = int(Rover::NUMTYPES);
	// end base class variables

	log = StepClockLogger(domain_params->n_steps,world->all_objects);
	for (int i=0; i<domain_params->n_rovers; i++){
		addRover();
	}
	for (int i=0; i<domain_params->n_POI; i++){
		addPOI();
	}

	// HACK, FIX POI AND ROVER POSITION
	
	static vector<vector<double> > fixedRoverPositions;
	static vector<vector<double> > fixedPOIPositions;
	if (fixedRoverPositions.size()){
		for (int i=0; i<rovers.size(); i++){
			rovers[i]->x = fixedRoverPositions[i][0];
			rovers[i]->y = fixedRoverPositions[i][1];
		}
		for (int i=0; i<POIs.size(); i++){
			POIs[i]->x = fixedPOIPositions[i][0];
			POIs[i]->y = fixedPOIPositions[i][1];
		}
	} else {
		printf("reseting positions");
		fixedRoverPositions = vector<vector<double> >(rovers.size());
		for (int i=0; i<rovers.size(); i++){
			fixedRoverPositions[i] = vector<double>(2,0.0);
			fixedRoverPositions[i][0] = rovers[i]->x;
			fixedRoverPositions[i][1] = rovers[i]->y;
		}
		fixedPOIPositions = vector<vector<double> >(POIs.size());
		for (int i=0; i<POIs.size(); i++){
			fixedPOIPositions[i] = vector<double>(2,0.0);
			fixedPOIPositions[i][0] = POIs[i]->x;
			fixedPOIPositions[i][1] = POIs[i]->y;
		}
	}
	for (int i=0; i<roverPositionSave.size(); i++){
		roverPositionSave[i] = fixedRoverPositions[i];
	}
}


RoverDomain::~RoverDomain(void)
{
	delete world;
	delete domain_params;

	for (int i=0; i<rovers.size(); i++){
		delete rovers[i];
	}

	for (int i=0; i<POIs.size(); i++){
		delete POIs[i];
	}
}

void RoverDomain::reset(){
	/// NON RANDOM POSITIONS
	//resets the position of the rovers
	for (int i=0; i<roverPositionSave.size(); i++){
		rovers[i]->x = roverPositionSave[i][0];
		rovers[i]->y = roverPositionSave[i][1];
	}
	/// RANDOM POSITIONS
	/*for (int i=0; i<POIs.size(); i++){
		world->setRandUnblockedXY(POIs[i]); // may be issues at saturation, gauranteed no overlap
	}
	
	for (int i=0; i<rovers.size(); i++){
		world->setRandUnblockedXY(rovers[i]);
	}*/
}

DistIDPairList RoverDomain::getSortedRoverDistances(Point2D *me){
	DistIDPairList dists;
	for (int i=0; i<rovers.size(); i++){
		dists.push_back(P(world->getPointDistance(me,rovers[i]),i));
	}
	dists.sort();
	return dists;
}