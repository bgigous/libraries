#pragma once
#include "AStar_easy.h"
#include "AStar_grid.h"
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

	typedef std::map<int,std::map<int,AStar_grid*> > grid_lookup;
	typedef Matrix<bool,2> barrier_grid;
	typedef Matrix<int,2> ID_grid;
	typedef pair<int,int> Edge;

	AStarManager(void);
	~AStarManager(void);
	AStarManager(int n_types, vector<Edge> edges, Matrix<int,2>* membership_map, vector<XY> agent_locs):
		membership_map(membership_map),edges(edges),agent_locs(agent_locs),n_types(n_types)
	{


		weights = matrix2d(n_types, matrix1d(edges.size(), 1.0) );

		// Initialize Astar object (must re-create this each time weight change
		Astar_highlevel = std::vector<AStar_easy*>(n_types);
		for (int i=0; i<n_types; i++){
			Astar_highlevel[i] = new AStar_easy(agent_locs,edges,weights[i]);
		}


		// Read in files for sector management
		obstacle_map = new barrier_grid(membership_map->dim1(),membership_map->dim2());
		for (int i=0; i<membership_map->dim1(); i++){
			for (int j=0; j<membership_map->dim2(); j++){
				obstacle_map->at(i,j) = membership_map->at(i,j)<0;
			}
		}

		// Add a different A* for each connection
		for (int i=0; i<edges.size(); i++){
			Edge e = edges[i];
			m2astar[e.first][e.second] = new AStar_grid(obstacle_map, membership_map, e.first, e.second);
			XY xyi = agent_locs[e.first];
			XY xyj = agent_locs[e.second];
			XY dx_dy = xyj-xyi;
			int xydir = cardinalDirection(dx_dy);
			int memj = membership_map->at(xyj); // only care about cost INTO sector
			sector_dir_map[i] = make_pair(memj,xydir); // add at new index
		}
	}

	int getMembership(easymath::XY pt){
		// Returns the membership of the particular point
		return (int)membership_map->at((Numeric_lib::Index)pt.x,(Numeric_lib::Index)pt.y);
	}


	void blockSector(int sectorID){
		saved_weights = weights;

		for (matrix1d &w: weights){
			w[sectorID] = 9999999.99;
		}

		resetGraphWeights(weights);
	}

	void unblockSector(){
		weights = saved_weights;
		resetGraphWeights(weights);
	}

	list<int> search(int type_ID, easymath::XY start_loc, easymath::XY end_loc){
		int memstart = (int)membership_map->at((Numeric_lib::Index)start_loc.x,(Numeric_lib::Index)start_loc.y);
		int memend = (int)membership_map->at((Numeric_lib::Index)end_loc.x,(Numeric_lib::Index)end_loc.y);
		list<AStar_easy::vertex> path = Astar_highlevel[type_ID]->search(memstart,memend);
		list<int> intpath;
		// NOTE; MAKE VERTICES INTS FOR HIGH LEVEL
		while (path.size()){
			intpath.push_back(path.front());
			path.pop_front();
		}
		return intpath;
	}

	void reset(){
		weights = matrix2d(n_types, matrix1d(edges.size(),1.0) );

		// re-create high level a*
		for (unsigned int i=0; i<Astar_highlevel.size(); i++){
			delete Astar_highlevel[i];
			Astar_highlevel[i] = new AStar_easy(agent_locs,edges,weights[i]);
		}
	}
	unsigned int n_types;

	void setCostMaps(vector<vector<double> > agent_actions){
		// [next sector][direction of travel] -- current
		// agent_actions  = agent, [type, dir<-- alternating]

		for (unsigned int i=0; i<weights[0].size(); i++){
			for (unsigned int j=0; j<n_types; j++){
				int s = sector_dir_map[i].first;
				//int d = j*UAV::NTYPES + sector_dir_map[i].second; // wrong

				int d = j + sector_dir_map[i].second*n_types;
				weights[j][i] = agent_actions[s][d];
			}
		}

		resetGraphWeights(weights);
	}


	void resetGraphWeights(matrix2d weightset){
		weights = weightset;

		for (unsigned int i=0; i<Astar_highlevel.size(); i++){
			delete Astar_highlevel[i];
			Astar_highlevel[i] = new AStar_easy(agent_locs,edges,weights[i]); // replace existing weights
		}
	}
	
	matrix2d saved_weights; // for blocking and unblocking sectors
	map<int,pair<int,int> > sector_dir_map; // maps index of edge to (sector next, direction of travel)
	Matrix<int,2> * membership_map; // technically this should be an int matrix. fix later
	Matrix<bool,2> * obstacle_map; // pass these to uavs later to determine where the obstacles are
	vector<XY> agent_locs;
	vector<AStar_easy::edge> edges;
	matrix2d weights; // [type][connection]
	std::vector<AStar_easy*> Astar_highlevel;
	grid_lookup m2astar;

	void printMasks(){
		for (const auto& outer :m2astar){
			for (const auto& inner: outer.second){
				inner.second->m.printMap("masks/", outer.first, inner.first);
			}
		}
	}
};

