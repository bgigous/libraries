#pragma once
#include <stdlib.h>
#include <vector>
#include <map>
#include <algorithm>
#include <functional>

double bRand(double uBound=1.0, double lBound=0.0);

template <class Container, class ReferenceType, class Compare>
void sortWithReference(Container &elements, ReferenceType ref, Compare lessThanRef){
	// Takes a container, returns it sorted according to the comparison 'lessThanRef(element1, element2, ref)'
	typename Container::iterator start = elements.begin();
	typename Container::iterator finish = elements.end();

	auto lessThanRefBound = bind(lessThanRef, std::placeholders::_1, std::placeholders::_2, ref);

	std::sort(start,finish,lessThanRefBound);
}

template <class T>
bool vectorLessThan(T const& lhs, T const& rhs){
	// Get properties
	int lhsSize, rhsSize;
	lhsSize = lhs.size();
	rhsSize = rhs.size();

	// Compare size
	if (lhsSize<rhsSize) return true;
	if (lhsSize>rhsSize) return false;

	// Compare each element
	for (int i=0; i<lhsSize; i++){
		if (lhs[i]<rhs[i]) return true;
		if (lhs[i]>rhs[i]) return false;
	}

	// All elements are equal
	return false;
};

template <class T>
bool vectorGreaterThan(T const& lhs, T const& rhs){
	// Get properties
	int lhsSize, rhsSize;
	lhsSize = lhs.size();
	rhsSize = rhs.size();

	// Compare size
	if (lhsSize>rhsSize) return true;
	if (lhsSize<rhsSize) return false;

	// Compare each element
	for (int i=0; i<lhsSize; i++){
		if (lhs[i]>rhs[i]) return true;
		if (lhs[i]<rhs[i]) return false;
	}

	// All elements are equal
	return false;
};

template <class T>
typename T::const_iterator maxValueIterator(T const&myContainer){
	// Get types
	typedef typename T::value_type pair_type;
	typedef typename pair_type::first_type key;
	typedef typename pair_type::second_type value;

	// Set default maximums
    typename T::const_iterator best = myContainer.begin();
	value max = best->second;


	// Compare to current maximum
	for (typename T::const_iterator it = best; it!=myContainer.end(); ++it){
		if (it->second > max){
			max = it->second;
			best = it;
		} else {
			continue;
		}
	}

	// Return the winner
	return best;
}
