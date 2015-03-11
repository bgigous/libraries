#pragma once
#include <vector>
#include <utility>
#include <cmath>

void set_a(double * array_first, double n_set, int array_length);
std::vector<double> getDiscretized(double min, double max, double disc);
std::vector<std::pair<double,double> > getAllPairs(std::vector<double> v1, std::vector<double> v2);
bool isInteger(double myDouble);

// template time!
template<class T>
void appendV(std::vector<T*>* v1, std::vector<T*>* v2){
    v1->insert(v1->end(), v2->begin(),v2->end());
}
