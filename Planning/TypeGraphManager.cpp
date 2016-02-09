#include "TypeGraphManager.h"

using namespace std;
using namespace easymath;

TypeGraphManager::TypeGraphManager(void)
{
}

TypeGraphManager::TypeGraphManager(int n_types, std::vector<edge> edges, vector<XY> agentLocs):
	n_types(n_types),edges(edges)
{
	initializeTypeLookupAndDirections(agentLocs);
}	

TypeGraphManager::TypeGraphManager(string edgesFile, string verticesFile, int n_types):
	n_types(n_types)
{
	//vector<XY> agentLocs;
	// Read in files for sector management
	vector<XY> agentLocs = FileIn::read_pairs<XY>(verticesFile);
	//FileIn::loadVariable(agentLocs, verticesFile);
	edges = FileIn::read_pairs<edge>(edgesFile); // NOTE: this leaves to the user the task of making edges bidirectional

	//FileIn::loadVariable(edges, edgesFile);

	
	for (unsigned int i=0; i<agentLocs.size(); i++){
		loc2mem[agentLocs[i]]=i; // add in reverse lookup
	}

	initializeTypeLookupAndDirections(agentLocs);
}

TypeGraphManager::TypeGraphManager(int n_vertices, int n_types, double gridSizeX, double gridSizeY):
	n_types(n_types)
{
	vector<XY> agentLocs = vector<XY>(n_vertices);
	for (XY &l : agentLocs){
		l = XY(rand()%int(gridSizeX),rand()%int(gridSizeY));
	}

	vector<pair<int,int> > candidates;
	for (unsigned int i=0; i<agentLocs.size(); i++){
		for (unsigned int j=0; j<agentLocs.size(); j++){
			if (i==j) continue;
			candidates.push_back(make_pair(i,j));
		}
	}
	random_shuffle(candidates.begin(), candidates.end());

	// Add as many edges as possible
	for (edge c: candidates){
		if (!intersectsExistingEdge(c,agentLocs)){
			edges.push_back(c);
			edges.push_back(make_pair(c.second,c.first));
		}
	}

	bool isfullyconnected = fullyConnected(agentLocs);

	for (unsigned int i=0; i<agentLocs.size(); i++){
		loc2mem[agentLocs[i]]=i; // add in reverse lookup
	}

	initializeTypeLookupAndDirections(agentLocs);
}


bool TypeGraphManager::intersectsExistingEdge(edge candidate, vector<XY> agentLocs){
	for (edge e:edges){
		if (intersects_in_center(line_segment(agentLocs[e.first],agentLocs[e.second]),
			line_segment(agentLocs[candidate.first],agentLocs[candidate.second])))
			return true;
	}
	return false;
}

bool TypeGraphManager::fullyConnected(vector<XY> agentLocs){
	LinkGraph a = LinkGraph(agentLocs,edges);

	for (unsigned int i=0; i<agentLocs.size(); i++){
		for (unsigned int j=0; j<agentLocs.size(); j++){
			if (i==j) continue;
			list<int> path = a.astar(i,j);
			if (path.size()==1) return false;
		}
	}
	return true;
}


TypeGraphManager::~TypeGraphManager(void)
{
}


void TypeGraphManager::setCostMaps(matrix2d agent_actions){
	
	for (unsigned int i=0; i<Graph_highlevel.size(); i++){
		Graph_highlevel[i]->setWeights(agent_actions[i]);
	}
}


list<int> TypeGraphManager::astar(int mem1, int mem2, int type_ID){
	return Graph_highlevel[type_ID]->astar(mem1,mem2);
}

int TypeGraphManager::getMembership(easymath::XY pt){
	if (loc2mem.find(pt)!=loc2mem.end()) {
		return loc2mem[pt];
	} else {
		printf("Point (%f,%f) not found in membership lookup. (Were you trying to use the detailed map?) \n",pt.x,pt.y);
		system("pause");
		exit(1);
	}
}

XY TypeGraphManager::getLocation(int sectorID){
	return Graph_highlevel[0]->locations[sectorID];
}

vector<TypeGraphManager::edge> TypeGraphManager::getEdges(){
	return edges;
}

int TypeGraphManager::getNVertices(){
	return Graph_highlevel[0]->locations.size();
}

void TypeGraphManager::initializeTypeLookupAndDirections(vector<XY> agentLocs)
{
	Graph_highlevel = std::vector<LinkGraph*>(n_types);
	for (int i=0; i<n_types; i++){
		Graph_highlevel[i] = new LinkGraph(agentLocs,edges);
	}
}