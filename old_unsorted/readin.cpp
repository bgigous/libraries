#pragma once

#include "readin.h"

using namespace std;

string ftoa(double tochange){
    stringstream ss;
    ss << tochange;
    string changed;
    ss >> changed;
    return changed;
}

void stripFilePath(string* filename){
    istringstream iss(*filename);
    string token;
    while (getline(iss,token,'/')); // Returns the last element in the filepath
    *filename = token;
}

void stripFilePaths(list<string>* filenames){
    list<string>::iterator it;
    for (it=filenames->begin(); it!=filenames->end(); it++){
        stripFilePath(&(*it));
    }
}

void stripExtension(string* filename){
    size_t found;
    found = filename->find('.');
    filename->resize(found);
}

void stripExtensions(list<string>* filenames){
    list<string>::iterator it;
    for (it=filenames->begin(); it!=filenames->end(); it++){
        stripExtension(&(*it));
    }
}

void removeExcept(std::string extensionDesired, std::list<std::string> *filenames){
    list<string>::reverse_iterator rit;
    size_t pos;
    string ext;
    for (rit=filenames->rbegin(); rit!=filenames->rend(); ++rit){
        pos = rit->find('.');
        ext = rit->substr(pos);
        if (ext!=extensionDesired){
            filenames->remove(*rit);
        }
    }
}

list<string> importlist(string filename){
    /*
    | Imports a list, ignoring blank spaces.
    */

    ifstream file(filename.c_str());
    if (!file.is_open()) printf("Failed to open file %s.\n", filename.c_str());
    string value;
    list<string> listContents;

    while (file.good()){
        getline(file,value);
        if (value!=""){
            listContents.push_back(value);
        }
    }
    file.close();
    return listContents;
}

void exportlist(string filename, list<string> listContents){
    /*
    | Exports a list, delimited by newlines.
    */

    ofstream file(filename.c_str());
	for (list<string>::iterator it=listContents.begin(); it!=listContents.end(); it++){	
		file << *it;
		if (next(it)!=listContents.end()) file<<"\n";
	}
    file.close();
}

vector<vector<string> > importxls(string filename){
    ifstream file(filename.c_str());
    if (!file.is_open()) printf("Failed to open file %s.\n", filename.c_str());
    string value;
    vector<vector<string> > filematrix;

    while (file.good()){
        getline(file,value);
        istringstream iss(value);
        string word;
        vector<string> line;
        while (getline(iss,word,'\t')){
            line.push_back(word);
        }
        if (line.size()) filematrix.push_back(line);
    }
    file.close();
    return filematrix;
}

vector<vector<string> > importconfigfile(string filename){
    /*------------------------------------------------------------------------------------------*
    | Imports a list of attributes of the form [ATTRIBUTENAME] = [VALUE] [UNITS]\n              |
    | Ignores the first two lines (reserved for comments)                                       |
    | Returns the list broken up into the vector [ATTRIBUTENAME][VALUE][UNITS]                  |
    *------------------------------------------------------------------------------------------*/

    ifstream file(filename.c_str());
    if (!file.is_open()){
        printf("Failed to open file %s.\n", filename.c_str());
        wait_for_key();
    }
    string value;
    vector<vector<string> > filematrix;

    while (file.good()){
        getline(file, value);
        istringstream iss(value);
        string word;
        vector<string> line;
        while (getline(iss,word,' ')){
            if (word!="=") line.push_back(word);
        }
        if(line.size()){
            filematrix.push_back(line);
        }
    }
    file.close();

    filematrix.erase(filematrix.begin());
    filematrix.erase(filematrix.begin());
    return filematrix;
}

vector<vector<string> > importcsv(string filename, string startphrase, string endphrase){
    /*------------------------------------------------------------------------------------------*
    | Reads in a csv file with "" denoting the fields. Reads from 'startphrase' to 'endphrase', |
    | ignoring text after that in the file. If these are "" then it reads in beginning to end.  |
    *------------------------------------------------------------------------------------------*/

    ifstream file(filename.c_str());
    if (!file.is_open()) printf("Failed to open file %s.\n", filename.c_str());
    string value;
    vector<vector<string> > filematrix;

    bool isentering = false;    // Sets default to not record
    if (startphrase=="") isentering = true; // Sets to record all if no startphrase given
    while (file.good()){
        getline(file, value);
        istringstream iss(value);
        string word;
        vector<string> line;
        bool switchentering = false; // Detects turning on or off entering
        while (getline(iss,word,',')){
            if (word!=""){  // this portion should be commented out if csv file has no "" around it
                if (word.substr(0,1)=="\"" && word.substr(word.length()-1,word.length()-2)=="\"") {
                    word = word.substr(1, word.length()-2);    // chops off quotation marks
                }
            }
            if (word==startphrase && word!="") isentering = true, switchentering = true;
            if (word==endphrase && word!="") isentering = false, switchentering = true;
            if (isentering) line.push_back(word);
        }
        if(line.size() && (isentering || switchentering)){
            filematrix.push_back(line);
        }
    }
    file.close();
    return filematrix;
}

void exportcsv(vector<vector<string> > filematrix, string filename, string separator){
    ofstream file;
    file.open(filename.c_str());
    if (file.is_open()){
        for (int i=0; i<filematrix.size(); i++){
            for (int j=0; j<filematrix[i].size(); j++){
                printf("%s%s",filematrix[i][j].c_str(),separator.c_str());
                file << filematrix[i][j] << separator;
            }
            printf("\nh");
            file << "\n";
        }
        file.close();
    }
    else {
        printf("Failed to open %s.",filename.c_str());
    }
}

bool getYesNo(string question){
    while(true){
        string user_entry;
        printf("%s (y/n)\t", question.c_str());
        cin >> user_entry;
        if (user_entry=="y"){
            return true;
        } else if (user_entry=="n"){
            return false;
        } else {
            printf("Unrecognized entry. ");
        }
    }
}

bool scrapeMatrixForVariable(std::vector<std::vector<std::string> > &varMatrix, std::string varName, double &varVal){
//bool scrapeMatrixForVariable(vector<vector<string> > &varMatrix, string varName, double varVal){
	for (int i=0; i<varMatrix.size(); i++){
		for (int j=0; j<varMatrix[i].size(); j++){
			if (varMatrix[i][j]==varName){
				varVal = atof(varMatrix[i][j+1].c_str());
				return true;
			}
		}
	}
	printf("Variable %s not found!\n",varName.c_str());
	system("pause");
	return false;
}

bool scrapeMatrixForVariable(std::vector<std::vector<std::string> > &varMatrix, std::string varName, int &varVal){
//bool scrapeMatrixForVariable(vector<vector<string> > &varMatrix, string varName, double varVal){
	for (int i=0; i<varMatrix.size(); i++){
		for (int j=0; j<varMatrix[i].size(); j++){
			if (varMatrix[i][j]==varName){
				varVal = atoi(varMatrix[i][j+1].c_str());
				return true;
			}
		}
	}
	printf("Variable %s not found!\n",varName.c_str());
	system("pause");
	return false;
}

bool scrapeMatrixForVariable(std::vector<std::vector<std::string> > &varMatrix, std::string varName, std::string &varVal){
//bool scrapeMatrixForVariable(vector<vector<string> > &varMatrix, string varName, double varVal){
	for (int i=0; i<varMatrix.size(); i++){
		for (int j=0; j<varMatrix[i].size(); j++){
			if (varMatrix[i][j]==varName){
				varVal = varMatrix[i][j+1];
				return true;
			}
		}
	}
	printf("Variable %s not found!\n",varName.c_str());
	system("pause");
	return false;
}