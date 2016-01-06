#include "TypeGraphManager.h"


TypeGraphManager::TypeGraphManager(void)
{
}

TypeGraphManager::TypeGraphManager(int n_types, std::vector<Edge> edges, vector<XY> agentLocs):
	n_types(n_types),edges(edges)
{
	initializeTypeLookupAndDirections(agentLocs);
}	

TypeGraphManager::TypeGraphManager(string edgesFile, string verticesFile, int n_types):
	n_types(n_types)
{
	vector<XY> agentLocs;
	// Read in files for sector management
	FileIn::loadVariable(agentLocs, verticesFile);
	FileIn::loadVariable(edges, edgesFile);

	
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

	for (pair<int,int> c: candidates){
		if (!intersectsExistingEdge(c,agentLocs)) edges.push_back(c);
	}

	bool isfullyconnected = fullyConnected(agentLocs);

	for (unsigned int i=0; i<agentLocs.size(); i++){
		loc2mem[agentLocs[i]]=i; // add in reverse lookup
	}

	initializeTypeLookupAndDirections(agentLocs);
}


bool TypeGraphManager::intersectsExistingEdge(pair<int, int> candidate, vector<XY> agentLocs){
	for (pair<int,int> e:edges){
		if (candidate.first==e.first 
			|| candidate.first == e.second 
			|| candidate.second == e.first 
			|| candidate.second == e.second)
			return false;
		else if (intersects(pair<XY,XY>(agentLocs[e.first],agentLocs[e.second]),pair<XY,XY>(agentLocs[candidate.first],agentLocs[candidate.second])))
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

matrix2d TypeGraphManager::sectorTypeVertex2SectorTypeDirection(matrix2d agent_actions){
	// Converts format of agent output to format of A* weights
	int n_edges = edges.size();

	matrix2d weights(n_types, matrix1d(n_edges,0.0));
	for (int i=0; i<n_edges; i++){
		for (int j=0; j<n_types; j++){
			int s = sector_dir_map[i].first;	// sector
			int d = j*(n_types-1) + sector_dir_map[i].second; // type/direction combo
			weights[j][i] = agent_actions[s][d]*1000.0;	// turns into 'type', 'edge'
		}
	}
	return weights;
}

void TypeGraphManager::setCostMaps(matrix2d agent_actions){
	// SWITCH BASED ON MODES

	matrix2d weights = sectorTypeVertex2SectorTypeDirection(agent_actions);
	
	for (unsigned int i=0; i<Graph_highlevel.size(); i++){
		Graph_highlevel[i]->setWeights(weights[i]);
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

vector<TypeGraphManager::Edge> TypeGraphManager::getEdges(){
	return edges;
}

int TypeGraphManager::getNAgents(){
	return Graph_highlevel[0]->locations.size();
}

void TypeGraphManager::initializeTypeLookupAndDirections(vector<XY> agentLocs)
{
	Graph_highlevel = std::vector<LinkGraph*>(n_types);
	for (int i=0; i<n_types; i++){
		Graph_highlevel[i] = new LinkGraph(agentLocs,edges);
	}

	// STATE SPECIFIC
	// Get the directions
	for (unsigned int i=0; i<edges.size(); i++){
		Edge e = edges[i];
		int memi = e.first; // membership of origin of edge
		int memj = e.second; // membership of connected node
		XY xyi = agentLocs[memi];
		XY xyj = agentLocs[memj];
		XY dx_dy = xyj-xyi;
		int xydir = cardinalDirection(dx_dy);
		sector_dir_map[i] = make_pair(memj,xydir); // add at new index
	}
}