#include "easymath.h"

namespace easymath{
	int cardinalDirection(XY dx_dy){
		if (dx_dy.y>=0){ // Going up
			if (dx_dy.x>=0) return 0; // up-right
			else return 1; // up-left
		} else {
			if (dx_dy.x>=0) return 2; // down-right
			else return 3; // down-left
		}
	}

	/*void discretizeSegment(std::vector<XY>::iterator iter_begin, std::vector<XY> &myvec, int n_even_segments){
		std::vector<XY> insertvector(n_even_segments);

		XY startpt = *iter_begin;
		iter_begin++;
		XY endpt = *iter_begin;
		iter_begin--;

		double xcomponent = endpt.x-startpt.x;
		double ycomponent = endpt.y-startpt.y;
		double h = sqrt(xcomponent*xcomponent+ycomponent*ycomponent);
		xcomponent/=h;
		ycomponent/=h;

		XY curpt = startpt;
		for (int i=0; i<n_even_segments; i++){
			curpt.x+= xcomponent*h/n_even_segments;
			curpt.y+= ycomponent*h/n_even_segments;
			insertvector.push_back(curpt);
		}

		myvec.insert(iter_begin,insertvector.begin(),insertvector.end());

	}*/
	double distance(XY p1, XY p2){
		double dx = p1.x-p2.x;
		double dy = p1.y-p2.y;
		return sqrt(dx*dx+dy*dy);
	}
	int getMaxIndex(std::vector<double> myvector){
		return distance(myvector.begin(),std::max_element(myvector.begin(),myvector.end()));
	}

	std::set<XY> getNUniquePositions(int N, double xbound, double ybound){
		if (ybound==-1) ybound=xbound;
		std::set<XY> unique_positions;
		while (unique_positions.size()<N){
			unique_positions.insert(XY(COIN_FLOOR0*xbound,COIN_FLOOR0*ybound));
		}
		return unique_positions;
	}

	std::vector<double> mean2(std::vector<std::vector<double> > myVector){
		std::vector<double> myMean(myVector[0].size(),0.0);

		for (int i=0; i<myVector.size(); i++){
			for (int j=0; j<myVector[i].size(); j++){
				myMean[j]+=myVector[i][j]/double(myVector.size());
			}
		}
		return myMean;
	}


	std::vector<double> mean1(std::vector<std::vector<double> > myVector){
		std::vector<double> myMean(myVector.size(),0.0);

		for (int i=0; i<myVector.size(); i++){
			for (int j=0; j<myVector[i].size(); j++){
				myMean[i]+=myVector[i][j];
			}
			myMean[i]/=double(myVector[i].size());
		}
		return myMean;
	}

	double mean(std::vector<double> myVector){
		return sum(myVector)/double(myVector.size());
	}

	std::vector<double> sum(std::vector<std::vector<double> > myVector){
		std::vector<double> mySum(myVector.size(),0.0);

		for (int i=0; i<myVector.size(); i++){
			for (int j=0; j<myVector[i].size(); j++){
				mySum[i]+=myVector[i][j];
			}
		}
		return mySum;
	}

	double sum(std::vector<double> myVector){
		double mySum = 0.0;
		for (int i=0; i<myVector.size(); i++){
			mySum+=myVector[i];
		}
		return mySum;
	}

	int sum(std::vector<bool> myVector){
		int mySum = 0;
		for (int i=0; i<myVector.size(); i++){
			mySum+=myVector[i]?1:0;
		}
		return mySum;
	}

	double scaleValue01(double val, double min, double max){
		return (val-min)/(max-min);
	}

	double boundedRand(double min, double max){
		if (min>max){ // switch them!
			double tmp = min;
			min = max;
			max = tmp;
		}
		return (max-min)*COIN+min;
	}

}