#include "MatrixTypes.h"

matrix1d easymath::mean2(matrix2d myVector){
		matrix1d myMean(myVector[0].size(),0.0);
		for (unsigned int i=0; i<myVector.size(); i++){
			for (unsigned int j=0; j<myVector[i].size(); j++){
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