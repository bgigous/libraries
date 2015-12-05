#pragma once
#include "AStarAbstract.h"
#include "AStarGrid.h"
#include "../Math/Matrix.h"
#include <memory>


/**
* A class to manage multiple A* instances that balances space and storage
*/

using namespace Numeric_lib;
using namespace std;

class AStarManager
{
public:
	// Planar
	typedef pair<int,int> Edge;
	typedef Matrix<bool,2> barrier_grid;
	
	map<XY, int> loc2mem;
	map<int,pair<int,int> > sector_dir_map; // maps index of edge to (sector next, direction of travel)
	
	std::vector<AStarAbstract*> Astar_highlevel;
	
	// grid
	typedef std::map<int,std::map<int,AStarGrid*> > grid_lookup;
	Matrix<int,2> * membership_map; // technically this should be an int matrix. fix later
	Matrix<bool,2> * obstacle_map; // pass these to uavs later to determine where the obstacles are
	grid_lookup m2astar;

	AStarManager(void);
	~AStarManager(void);


	AStarManager(int n_types, std::vector<Edge> edges, vector<XY> agentLocs)
	{
		initializeHighLevel(agentLocs,edges, n_types);
	}


	void initializeHighLevel(vector<XY> agentLocs, std::vector<Edge> edges, int n_types){
		// Initialize Astar object (must re-create this each time weight change
		Astar_highlevel = std::vector<AStarAbstract*>(n_types);
		for (unsigned int i=0; i<n_types; i++){
			Astar_highlevel[i] = new AStarAbstract(agentLocs,edges);
		}
		for (unsigned int i=0; i<agentLocs.size(); i++){
			loc2mem[agentLocs[i]]=i; // add in reverse lookup
		}

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


	void initializeLowLevel(Matrix<int,2>* membership_map_set, vector<Edge> edges){
		membership_map = membership_map_set;
		obstacle_map = new barrier_grid(membership_map->dim1(),membership_map->dim2());
		for (int i=0; i<membership_map->dim1(); i++){
			for (int j=0; j<membership_map->dim2(); j++){
				obstacle_map->at(i,j) = membership_map->at(i,j)<0;
			}
		}

		// Add a different A* for each connection
		for (unsigned int i=0; i<edges.size(); i++){
			Edge e = edges[i];
			m2astar[e.first][e.second] = new AStarGrid(obstacle_map, membership_map, e.first, e.second);
		}
	}
	
		
	void setCostMaps(matrix2d agent_actions, int n_types, int n_edges){
		matrix2d weights(n_types, matrix1d(n_edges,0.0));
		for (unsigned int i=0; i<n_edges; i++){
			for (unsigned int j=0; j<n_types; j++){
				int s = sector_dir_map[i].first;
				int d = j*(n_types-1) + sector_dir_map[i].second;
				weights[j][i] = agent_actions[s][d];
			}
		}
		resetGraphWeights(weights);
	}


	void resetGraphWeights(matrix2d weightset){
		for (unsigned int i=0; i<Astar_highlevel.size(); i++){
			Astar_highlevel[i]->setWeights(weightset[i]);
		}
	}
	/*
	void printMasks(){
		for (const auto& outer :m2astar){
			for (const auto& inner: outer.second){
				inner.second->m.printMap("masks/", outer.first, inner.first);
			}
		}
	}*/

	int getMembership(easymath::XY pt){
		// Returns the membership of the particular point
		if (membership_map!=NULL){
			return (int)membership_map->at((Numeric_lib::Index)pt.x,(Numeric_lib::Index)pt.y);
		} else if (loc2mem.find(pt)!=loc2mem.end()) {
			return loc2mem[pt];
		} else {
			printf("Point not found in membership lookup.\n");
			system("pause");
			exit(1);
		}
	}

};

