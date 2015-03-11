#pragma once

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <sstream>
#include <fstream>

//string conversion
std::string ftos(double f);
std::string itos(int i);

// FANCY ARRAY STUFF: NOT TEMPLATED
void prints(double * array_first, int dim1, std::string graph_label="");
void prints(double * array_first, int dim1, int dim2, std::string graph_label="");

//WAITING FUNCTIONS
void wait_for_key();
void wait_for_key_silently();


template<class T>
void print1(T data,std::string label="",std::string separator=","){
    /*
    | Prints a 1D stl container, given a container and an optional label and separator.
    */

    if (label!="") std::cout << label << ":" << std::endl;

    for (auto it=data.begin(); it!=data.end(); ++it){
        std::cout << *it << separator;
    }
    std::cout << std::endl;
}

template<class T>
void print2(T data,std::string label="",std::string separator=","){
    /*
    | Prints a 2D stl container, given a container and an optional label and separator.
    */
    if (label!="") std::cout << label << ":" << std::endl;

    for (auto outer=data.begin(); outer!=data.end(); ++outer){
        for (auto inner=outer->begin(); inner!=outer->end(); ++inner){
            std::cout << *inner << separator;
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

template<class T>
void print2_ptr(T data,std::string label="",std::string separator=","){
    /*
    | Prints a 2D stl container, given a container and an optional label and separator.
    */
    if (label!="") std::cout << label << ":" << std::endl;

    for (auto outer=data.begin(); outer!=data.end(); ++outer){
        for (auto inner=(*outer)->begin(); inner!=(*outer)->end(); ++inner){
            std::cout << *inner << separator;
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

template<class T>
void printpairs(T data, std::string label,std::string separator=","){
    /*
    | Print-to-file for a 1D container of pairs.
    */
    FILE* file;
    file = fopen(label.c_str(),"w");
    for (auto it=data.begin(); it!=data.end(); ++it){
        fprintf(file,"%f%s%f%s\n",it->first,separator,it->second,separator);
    }
    fclose(file);
}

template<class T>
void printfile1(T data, std::string label, std::string separator=","){
    /*
    | Prints a separated list of items in the container.
    */
    ofstream myfile;
	myfile.open(label);
    for (auto it=data.begin(); it!=data.end(); ++it){
		myfile << *it << separator;
    }
	myfile.close();
}

template <class T>
void printfile2(T data, std::string label, std::string separator=","){
    /*
    | Print-to-file for a 2D container.
    */
	ofstream myfile;
	myfile.open(label.c_str());
	if (myfile.good()){
		for (auto outer=data.begin(); outer!=data.end(); ++outer){
			for (auto inner=outer->begin(); inner!=outer->end(); ++inner){
				myfile << *inner << separator.c_str();
			}
			myfile << "\n";
		}
	} else {
		printf("File not opening.");
		system("pause");
	}


    myfile.close();
}
