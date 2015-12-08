#pragma once
#include "AStarAbstract.h"
#include "AStarGrid.h"
#include "../Math/Matrix.h"
#include <memory>


/**
* Manages AStarAbstract usage for different types in the system
*/

using namespace Numeric_lib;
using namespace std;

class TypeAStarAbstract
{
public:
	// Planar
	typedef pair<int,int> Edge;
	typedef Matrix<bool,2> barrier_grid;

	TypeAStarAbstract(void);
	~TypeAStarAbstract(void);


	TypeAStarAbstract(int n_types, std::vector<Edge> edges, vector<XY> agentLocs):
		n_types(n_types)
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

	int n_types;
	
		
	matrix2d sectorTypeVertex2SectorTypeDirection(matrix2d agent_actions){
		// TRANSLATION FUNCTION
		// Converts format of agent output to format of A* weights
		int n_edges = distance(edges(Astar_highlevel[0]->g).first,edges(Astar_highlevel[0]->g).second);

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

	void setCostMaps(matrix2d agent_actions){
		matrix2d weights = sectorTypeVertex2SectorTypeDirection(agent_actions);
		
		for (unsigned int i=0; i<Astar_highlevel.size(); i++){
			Astar_highlevel[i]->setWeights(weights[i]);
		}
	}


	list<int> search(int mem1, int mem2, int type_ID){
		return Astar_highlevel[type_ID]->search(mem1,mem2);
	}

	int getMembership(easymath::XY pt){
		if (loc2mem.find(pt)!=loc2mem.end()) {
			return loc2mem[pt];
		} else {
			printf("Point not found in membership lookup. (Were you trying to use the detailed map?) \n");
			system("pause");
			exit(1);
		}
	}

	XY getLocation(int sectorID){
		return Astar_highlevel[0]->locations[sectorID];
	}

private: 
	map<XY, int> loc2mem;
	map<int,pair<int,int> > sector_dir_map; // maps index of edge to (sector next, direction of travel)
	
	std::vector<AStarAbstract*> Astar_highlevel;
	

};

