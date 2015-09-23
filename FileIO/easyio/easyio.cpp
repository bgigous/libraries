#pragma once

#include "easyio.h"

using namespace std;

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