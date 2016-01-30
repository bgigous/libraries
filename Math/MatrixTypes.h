#pragma once
#include <vector>
#include <algorithm>


//! A file for containing matrix types.

typedef std::vector<double> matrix1d;
typedef std::vector<matrix1d> matrix2d;
typedef std::vector<matrix2d> matrix3d;

//! Also contains math functions for use with the matrices
namespace easymath{

	template<typename T>
	T sum(std::vector<T> m){
		T s=0;
		for (T i:m)
			s+=i;
		return s;
	}

	template<typename T>
	T mean(std::vector<T> m){
		return sum(m)/m.size();
	}

	template<typename T>
	std::vector<T> operator+(const std::vector<T>& a, const std::vector<T>& b){
		Assert::AreEqual(a.size(),b.size());
		std::vector<T> result;
		result.reserve(a.size());
		std::transform(a.begin(),a.end(),b.begin(),
			std::back_inserter(result),std::plus<T>());
		return result
	}

	matrix1d mean2(matrix2d myVector);
	int get_max_index(matrix1d myvector);
	double normalize(double val, double min, double max);
}