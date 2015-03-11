#pragma once

#include "printing.h"

using namespace std;

string ftos(double f){
	string s;
	stringstream ss;
	ss << f;
	ss >> s;
	return s;
}

string itos(int i){
	string s;
	stringstream ss;
	ss << i;
	ss >> s;
	return s;
}


void prints(double * array_first, int dim1, int dim2, std::string label){
    /*------------------------------------------------------------------------------------------*
    |   "Print Array": Takes in address of first element of 2D array and prints out, labeled    |
    *------------------------------------------------------------------------------------------*/

    printf("Showing %s:\n\n", label.c_str());
    for (int i=0; i<dim1; i++){
        for (int j=0; j<dim2; j++){
            printf("%f, ", array_first[i*dim1+j]);
        }
        printf("\n");
    }
    printf("\n\n");
}

void prints(double * array_first, int dim1, std::string label){
    /*------------------------------------------------------------------------------------------*
    |   "Print Array": Takes in address of first element of 1D array and prints out, labeled    |
    *------------------------------------------------------------------------------------------*/

    printf("Showing %s:\n\n", label.c_str());
    for (int i=0; i<dim1; i++){
        printf("%f, ", array_first[i]);
    }
    printf("\n\n");
}


void wait_for_key(){
    printf("\nPress ENTER to continue...\n");
    cin.clear();
    cin.ignore(cin.rdbuf()->in_avail());
    cin.get();
    return;
}

void wait_for_key_silently(){
    printf("\n");
    cin.clear();
    cin.ignore(cin.rdbuf()->in_avail());
    cin.get();
    return;
}
