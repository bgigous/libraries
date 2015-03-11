#include "Shortcuts.h"

using namespace std;

double bRand(double uBound, double lBound){
	// returns bounded random number, with a default lower bound of 0, default upper bound of 1
	return (double(rand())/double(RAND_MAX))*(max(lBound,uBound) - min(lBound,uBound)) + min(lBound,uBound);
}


// Generic sorting, replaced by the sorting template with bind
/*
	void insertionSortByClosest(vector<Vehicle> &A){
		vector<double> vals;
		int Asize = int(A.size());
		for (int i=0; i<Asize; i++){
			vals.push_back(distance3D(position,A[i].position));
		}
		for (int j=1; j<Asize; j++){
			Vehicle key = A[j];
			int i;
			for (i=j-1; (i>=0 && (distance3D(position, A[i].position)>distance3D(position,key.position))); i--){
				A[i+1] = A[i];
			}
			A[i+1] = key;
		}
	}

	void bucketSortByClosest(vector<Vehicle> &A){ // bucket sort assumes there is a max value... should be apparent by world size
		// courtesy of Jeremy's book :)
		int n = A.size();
		double maxDist = sqrt(3*params->worldSize*params->worldSize);
		vector<vector<Vehicle> > B (n);

		for (int i=0; i<n; i++){ // check this: not good by the book
			double valA = distance3D(position, A[i].position)/maxDist;
			if (valA==1) valA = double(n-1)/double(n);
			B[unsigned int(n*valA)].push_back(A[i]);
		}

		for (int i=0; i<n; i++){
			insertionSortByClosest(B[i]);
		}

		// CONCATENATE LISTS
		int index=0;
		for (int i=0; i<n; i++){
			int Bsize = B[i].size();
			for (int j=0; j<Bsize; j++){
				A[index++]=B[i][j];
			}
		}
	}
	*/