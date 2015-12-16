#include "TypeAStarAbstract.h"


TypeAStarAbstract::TypeAStarAbstract(void)
{
}

TypeAStarAbstract::TypeAStarAbstract(int n_types, std::vector<Edge> edges, vector<XY> agentLocs):
	n_types(n_types),edges(edges)
{
	initializeTypeLookupAndDirections(agentLocs);
}	

TypeAStarAbstract::TypeAStarAbstract(string edgesFile, string verticesFile, int n_types):
	n_types(n_types)
{
	vector<XY> agentLocs;
	// Read in files for sector management
	Load::loadVariable(agentLocs, verticesFile);
	Load::loadVariable(edges, edgesFile);

	initializeTypeLookupAndDirections(agentLocs);
}

TypeAStarAbstract::TypeAStarAbstract(int n_vertices, int n_types, double gridSizeX, double gridSizeY):
	n_types(n_types)
{
	vector<XY> agentLocs = vector<XY>(n_vertices);
	for (XY &l : agentLocs){
		l = XY(rand()%int(gridSizeX),rand()%int(gridSizeY));
	}

	// voronoi

	//Construct a planar graph
	/*do {
		for (unsigned int i=0; i<2*agentLocs.size(); i++){ // scales with number of agents (tries to add 2x the number of agents before testing connectedness)
			// Add edge randomly
			int p1 = int(rand()%agentLocs.size());
			int p2 = int(rand()%agentLocs.size());
			pair<int,int> candidate = pair<int,int>(p1,p2);

			if (!intersectsExistingEdge(candidate, agentLocs)) edges.push_back(candidate);
		}
	} while (!fullyConnected(agentLocs));*/

	initializeTypeLookupAndDirections(agentLocs);
}

/*
bool TypeAStarAbstract::intersectsExistingEdge(pair<int, int> candidate, vector<XY> agentLocs){
	for (pair<int,int> e:edges){
		if (intersects(pair<XY,XY>(agentLocs[e.first],agentLocs[e.second]),pair<XY,XY>(agentLocs[candidate.first],agentLocs[candidate.second]))){
			return true;
		}
	}
	return false;
}

bool TypeAStarAbstract::fullyConnected(vector<XY> agentLocs){
	AStarAbstract a = AStarAbstract(agentLocs,edges);

	for (unsigned int i=0; i<agentLocs.size(); i++){
		for (unsigned int j=0; j<agentLocs.size(); j++){
			list<int> path = a.search(i,j);
			if (path.size()==1) return false;
		}
	}
	return true;
}
*/

TypeAStarAbstract::~TypeAStarAbstract(void)
{
}

matrix2d TypeAStarAbstract::sectorTypeVertex2SectorTypeDirection(matrix2d agent_actions){
	// Converts format of agent output to format of A* weights
	int n_edges = edges.size();

	matrix2d weights(n_types, matrix1d(n_edges,0.0));
	for (int i=0; i<n_edges; i++){
		for (int j=0; j<n_types; j++){
			int s = sector_dir_map[i].first;	// sector
			int d = j*(n_types-1) + sector_dir_map[i].second; // type/direction combo
			weights[j][i] = agent_actions[s][d];	// turns into 'type', 'edge'
		}
	}
	return weights;
}

void TypeAStarAbstract::setCostMaps(matrix2d agent_actions){
	matrix2d weights = sectorTypeVertex2SectorTypeDirection(agent_actions);

	for (unsigned int i=0; i<Astar_highlevel.size(); i++){
		Astar_highlevel[i]->setWeights(weights[i]);
	}
}


list<int> TypeAStarAbstract::search(int mem1, int mem2, int type_ID){
	return Astar_highlevel[type_ID]->search(mem1,mem2);
}

int TypeAStarAbstract::getMembership(easymath::XY pt){
	if (loc2mem.find(pt)!=loc2mem.end()) {
		return loc2mem[pt];
	} else {
		printf("Point not found in membership lookup. (Were you trying to use the detailed map?) \n");
		system("pause");
		exit(1);
	}
}

XY TypeAStarAbstract::getLocation(int sectorID){
	return Astar_highlevel[0]->locations[sectorID];
}

vector<TypeAStarAbstract::Edge>& TypeAStarAbstract::getEdges(){
	return edges;
}

int TypeAStarAbstract::getNAgents(){
	return sector_dir_map.size();
}

void TypeAStarAbstract::initializeTypeLookupAndDirections(vector<XY> agentLocs)
{
	Astar_highlevel = std::vector<AStarAbstract*>(n_types);
	for (int i=0; i<n_types; i++){
		Astar_highlevel[i] = new AStarAbstract(agentLocs,edges);
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