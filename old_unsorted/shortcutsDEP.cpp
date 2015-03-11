#pragma once
#include "shortcuts.h"

using namespace std;

void set_a(double * array_first, double n_set, int array_length){
    for (int i=0; i<array_length; i++){
        array_first[i] = n_set;
    }
}

vector<double> getDiscretized(double min, double max, double disc){
    vector<double> discretizations;
    if (min==max){
        discretizations.push_back(min);
        return discretizations;
    }
    int vsize = int( (max-min)/disc );
    discretizations.reserve(vsize+1); // Reserves size to accommodate vsize+1 elements (includes first/last elements)
    for (double d=min,i=0; d<=max; d+=disc,i++){
        discretizations[int(i)] = d;
    }
    return discretizations;
}

vector<pair<double,double> > getAllPairs(vector<double> v1, vector<double> v2){
    vector<pair<double,double> > allPairs;
    for (vector<double>::iterator outer=v1.begin(); outer!=v1.end(); ++outer){
        for (vector<double>::iterator inner=v2.begin(); inner!=v2.end(); ++inner){
            allPairs.push_back(make_pair(*outer, *inner));
        }
    }
    return allPairs;
}

bool isInteger(double myDouble){
    int myInt=(int)myDouble;
    double remainder=(double)myInt-(double)myDouble;
    if (fabs(remainder)==0.0) return true;
    else return false;
}
