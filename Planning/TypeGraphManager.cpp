#include "TypeGraphManager.h"

using namespace std;
using namespace easymath;

TypeGraphManager::TypeGraphManager(void)
{
}

TypeGraphManager::TypeGraphManager(int n_types, vector<edge> edges, vector<XY> agentLocs):
	n_types(n_types),edges(edges), rags_map(new RAGS(agentLocs,edges))
{
	initializeTypeLookupAndDirections(agentLocs);
}

TypeGraphManager::TypeGraphManager(string edgesFile, string verticesFile, int n_types):
	n_types(n_types)
{
	// Read in files for sector management
	vector<XY> agentLocs = FileIn::read_pairs<XY>(verticesFile);
	edges = FileIn::read_pairs<edge>(edgesFile); // NOTE: this leaves to the user the task of making edges bidirectional
	rags_map = new RAGS(agentLocs, edges);

	for (uint i=0; i<agentLocs.size(); i++){
		loc2mem[agentLocs[i]]=i; // add in reverse lookup
	}

	initializeTypeLookupAndDirections(agentLocs);
}

TypeGraphManager::TypeGraphManager(int n_vertices, int n_types, double gridSizeX, double gridSizeY):
	n_types(n_types)
{
	set<XY> agent_loc_set = get_n_unique_points(0.0,gridSizeX,0.0,gridSizeY, n_vertices);
	vector<XY> agentLocs (agent_loc_set.size());
	copy(agent_loc_set.begin(),agent_loc_set.end(),agentLocs.begin());

	vector<edge > candidates;
	for (size_t i=0; i<agentLocs.size(); i++){
		for (size_t j=0; j<agentLocs.size(); j++){
			if (i==j)
				continue;
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

	rags_map = new RAGS(agentLocs, edges); // create a RAGS object which generates a graph for searching over

	bool isfullyconnected = fullyConnected(agentLocs);

	for (uint i=0; i<agentLocs.size(); i++){
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

	for (uint i=0; i<agentLocs.size(); i++){
		for (uint j=0; j<agentLocs.size(); j++){
			if (i==j) continue;
			list<int> path = a.astar(i,j);
			if (path.size()==1)
				return false;
		}
	}
	return true;
}


TypeGraphManager::~TypeGraphManager(void)
{
	delete rags_map;
	for (int i=0; i<n_types; i++){
		delete Graph_highlevel[i];
	}
}

void TypeGraphManager::setCostMaps(matrix2d agent_actions){
	for (uint i=0; i<Graph_highlevel.size(); i++){
		Graph_highlevel[i]->setWeights(agent_actions[i]);
	}
}


list<int> TypeGraphManager::astar(int mem1, int mem2, int type_ID){
	return Graph_highlevel[type_ID]->astar(mem1,mem2);
}

list<int> TypeGraphManager::rags(int mem1, int mem2, int type_ID){
	matrix1d w = Graph_highlevel[type_ID]->getWeights();
	XY start_loc = getLocation(mem1);
	XY end_loc = getLocation(mem2);
	XY next_xy = rags_map->SearchGraph(start_loc,end_loc,w);
	int next_node_ID = getMembership(next_xy);
	list<int> partial_path;
	partial_path.push_back(mem1); // Add in starting node too
	partial_path.push_back(next_node_ID);
	// NOTE TO RYAN: debug may be needed here. RAGS doesn't always seem to find a path. Please make sure I'm using it correctly.s
	return partial_path;
}

int TypeGraphManager::getMembership(easymath::XY pt){
	try{
		return loc2mem.at(pt);
	} catch (out_of_range){
		printf("Point (%f,%f) not found in membership lookup. ",pt.x,pt.y);
		printf("Were you trying to use the detailed map ?\n");
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
