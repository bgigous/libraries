#include "MatrixTypes.h"

matrix1d easymath::mean2(matrix2d myVector){
		matrix1d myMean(myVector[0].size(),0.0);
		for (uint i=0; i<myVector.size(); i++){
			for (uint j=0; j<myVector[i].size(); j++){
				myMean[j]+=myVector[i][j]/double(myVector.size());
			}
		}
		return myMean;
	}

int easymath::get_max_index(matrix1d myvector){
	return distance(myvector.begin(),std::max_element(myvector.begin(),myvector.end()));
}

double easymath::normalize(double val, double min, double max){
		return (val-min)/(max-min);
	}

void easymath::zero(matrix2d & m){
	if (m.empty()) return;
	else m = matrix2d(m.size(), matrix1d(m[0].size(),0.0));
}

void easymath::zero(matrix1d & m){
	m = matrix1d(m.size(), 0.0);
}

matrix1d easymath::zeros(int dim1){
	return matrix1d(dim1, 0.0);
}

matrix2d easymath::zeros(int dim1, int dim2){
	return matrix2d(dim1, easymath::zeros(dim2));
}

matrix3d easymath::zeros(int dim1, int dim2, int dim3){
	return matrix3d(dim1,easymath::zeros(dim2,dim3));
}
