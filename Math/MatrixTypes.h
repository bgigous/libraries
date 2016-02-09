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
	T sum_if_positive(std::vector<T> m){
		T s=0;
		for (T i:m)
			if (i>0)
				s+=i;
		return s;
	}

	template<typename T>
	T mean(std::vector<T> m){
		return sum(m)/m.size();
	}

	template<typename T>
	std::vector<T> operator+(const std::vector<T>& a, const std::vector<T>& b){
		std::vector<T> result;
		result.reserve(a.size());
		std::transform(a.begin(),a.end(),b.begin(),
			std::back_inserter(result),std::plus<T>());
		return result;
	}
	
	template<typename T>
	std::vector<T> operator-(const std::vector<T>& a, const std::vector<T>& b){
		std::vector<T> result;
		result.reserve(a.size());
		std::transform(a.begin(),a.end(),b.begin(),
			std::back_inserter(result),std::minus<T>());
		return result;
	}

	template<typename T>
	std::vector<T> operator/(std::vector<T> a, const int & b){
		for (T &el: a){
			el = el/double(b);
		}
		return a;
	}

	template<typename T>
	std::vector<T> set_negative_zero(std::vector<T> &m){
		std::vector<T> r = m;
		for (T &i:r){
			if (i<0) i=0;
		}
		return r;
	}

	matrix1d mean2(matrix2d myVector);
	int get_max_index(matrix1d myvector);
	double normalize(double val, double min, double max);

	void zero(matrix2d & m);
	void zero(matrix1d & m);
	matrix1d zeros(int dim1);
	matrix2d zeros(int dim1, int dim2);
	matrix3d zeros(int dim1, int dim2, int dim3);
}