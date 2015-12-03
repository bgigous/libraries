#pragma once

#include "easyio.h"

using namespace std;
using namespace Numeric_lib;

void Load::loadVariable(std::vector<std::vector<bool> >* var, std::string filename, double thresh, std::string separator){
	// must be above threshold to be counted as a boolean
	string_matrix2d f = FileManip::read(filename, separator);
	*var = std::vector<std::vector<bool> >(f.size());

	for (unsigned int i=0; i<f.size(); i++){
		var->at(i) = std::vector<bool>(f[i].size());
		for (unsigned int j=0; j<f[i].size(); j++){
			if (atof(f[i][j].c_str())<=thresh){
				var->at(i)[j] = false;
			} else {
				var->at(i)[j] = true;
			}
		}
	}
}


void Load::loadVariable(Matrix<bool,2> **var, std::string filename, double thresh, std::string separator){
	// must be above threshold to be counted as a boolean
	string_matrix2d f = FileManip::read(filename, separator);
	Matrix<bool,2> * mat = new Matrix<bool,2>(f.size(),f[0].size());

	for (unsigned int i=0; i<f.size(); i++){
		for (unsigned int j=0; j<f[i].size(); j++){
			if (atof(f[i][j].c_str())<=thresh){
				mat->at(i,j) = false;
			} else {
				mat->at(i,j) = true;
			}
		}
	}
	(*var) = mat;
}

void Load::loadVariable(Matrix<int,2> **var, std::string filename, std::string separator){
	// must be above threshold to be counted as a boolean
	string_matrix2d f = FileManip::read(filename, separator);
	Matrix<int,2> *mat = new Matrix<int,2>(f.size(),f[0].size());

	for (unsigned int i=0; i<f.size(); i++){
		for (unsigned int j=0; j<f[i].size(); j++){
			mat->at(i,j) = atoi(f[i][j].c_str());
		}
	}
	(*var)=mat;
}

void Load::loadVariable(std::vector<easymath::XY> &var, std::string filename, std::string separator){
	string_matrix2d f = FileManip::read(filename, separator);
	var.clear();
	for (string_matrix1d i:f){
		if (i.size()!=2){
			printf("Error! %s does not contain xy values.",filename.c_str());
			exit(100);
		} else {
			double x = atof(i[0].c_str());
			double y = atof(i[1].c_str());
			var.push_back(easymath::XY(x,y));
		}
	}
}

void Load::loadVariable(std::vector<std::pair<int,int> > &var, std::string filename, std::string separator){
	string_matrix2d f = FileManip::read(filename, separator);
	var.clear();
	for (string_matrix1d i:f){
		if (i.size()!=2){
			printf("Error! %s does not contain pair values.",filename.c_str());
			exit(100);
		} else {
			int x = atoi(i[0].c_str());
			int y = atoi(i[1].c_str());
			var.push_back(pair<int,int>(x,y));
		}
	}
}

void Load::loadVariable(matrix2d &var, std::string filename, std::string separator){
	string_matrix2d f = FileManip::read(filename, separator);
	var.clear();
	
	for (string_matrix1d i:f){
		matrix1d m;
		for (string j:i){
			m.push_back(atof(j.c_str()));
		}
		var.push_back(m);
	}
}

matrix2d FileManip::str2double(string_matrix2d mystring){
	matrix2d mymatrix = matrix2d(mystring.size());
	for (unsigned int i=0; i<mystring.size(); i++){
		mymatrix[i] = matrix1d(mystring[i].size());
		for (unsigned int j=0; j<mystring[i].size(); j++){
			mymatrix[i][j] = stod(mystring[i][j]);
		}
	}
	return mymatrix;
}


vector<string> FileManip::divide(string myString, string separator){
	//----- Divides a string at the points of 'separator' -----//
	vector<string> divided;
	while (myString.find(separator)!=string::npos){
		// Find the separator
		size_t found = myString.find(separator);
		// Cut the string at the point of the separator
		divided.push_back(myString.substr(0,found));
		// Remove the portion of the string read to the separator
		myString.erase(myString.begin(),myString.begin()+found+1);
	}
	divided.push_back(myString);
	return divided;
}

void PrintOut::screen(vector<string> myVector, string separator){
	for (unsigned int i=0; i<myVector.size(); i++){
		printf("%s%s",myVector[i].c_str(),separator.c_str());
	}
}

vector<vector<string> > FileManip::read(string filename, string separator){
	//----- Reads in a file and converts to a vector matrix of strings -----//

	if (separator==STRING_UNINITIALIZED){
		// Determine the type of file and infer the separator
		vector<string> divided = divide(filename,".");
		if (divided.back()=="csv"){
			separator = ",";
		} else if (divided.back()=="xls"){
			separator = "\t";
		} else {
			printf("Unknown extension %s: please specify separator type. Aborting.\n",divided.back().c_str());
			system("pause");
			exit(1);
		}

	}


	ifstream file(filename.c_str());
	if (!file.is_open()) printf("Failed to open file %s.\n", filename.c_str());
	string value;
	vector<vector<string> > filematrix;

	while (file.good()){
		getline(file,value);
		istringstream iss(value);
		string word;
		vector<string> line;
		while (getline(iss,word,*separator.c_str())){
			line.push_back(word);
		}
		if (line.size()) filematrix.push_back(line);
	}
	file.close();
	return filematrix;
}

matrix2d FileManip::readDouble(std::string filename, std::string separator){
	string_matrix2d temp = read(filename, separator);
	return str2double(temp);
}

double DataManip::stringToDouble(string s){
	return atof(s.c_str());
}

matrix2d DataManip::stringToDouble(vector<vector<string> > stringVector){
	matrix2d doubleVector;
	for (unsigned int i=0; i<stringVector.size(); i++){
		doubleVector.push_back(vector<double>(stringVector[i].size(),0.0));
		for (unsigned int j=0; j<stringVector[i].size(); j++){
			doubleVector[i][j] = stringToDouble(stringVector[i][j]);
		}
	}
	return doubleVector;
}

vector<double> DataManip::getColumn(matrix2d doubleVector, unsigned int col){
	vector<double> colVector = vector<double>(doubleVector.size(),0.0);
	for (unsigned int i=0; i<doubleVector.size(); i++){
		if (doubleVector[i].size()<=col){
			printf("Cannot access column %i. Aborting",col);
			system("pause");
			exit(1);
		} else {
			colVector[i]=doubleVector[i][col];
		}
	}
	return colVector;
}

matrix2d DataManip::getColumns(matrix2d doubleVector, vector<int> cols){
	matrix2d colVector;
	for (unsigned int i=0; i<doubleVector.size(); i++){
		colVector.push_back(vector<double>(cols.size(),0.0));
		for (unsigned int j=0; j<cols.size(); j++){
			unsigned int col = cols[j];
			if (doubleVector[i].size()<=col){
				printf("Cannot access column %i. Aborting",col);
				system("pause");
				exit(1);
			} else{
				colVector[i][j]=doubleVector[i][col];
			}
		}
	}
	return colVector;
}