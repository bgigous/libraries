#pragma once

#include "printing.h"
#include <fstream>
#include <sstream>
#include <list>
#include <vector>


std::string ftoa(double tochange);
void stripFilePath(std::string *filename);
void stripFilePaths(std::list<std::string> *filenames);
void stripExtension(std::string *filename);
void stripExtensions(std::list<std::string> *filenames);
void removeExcept(std::string extensionDesired, std::list<std::string> *filenames);
std::list<std::string> importlist(std::string filename);
void exportlist(std::string filename, std::list<std::string> listContents);
std::vector<std::vector<std::string> > importxls(std::string filename);
std::vector<std::vector<std::string> > importcsv(std::string filename, std::string startphrase="", std::string endphrase="");
std::vector<std::vector<std::string> > importconfigfile(std::string filename);
void exportcsv(std::vector<std::vector<std::string> > filematrix, std::string filename="unnamed-from-program.csv",std::string separator=",");
bool getYesNo(std::string);
bool scrapeMatrixForVariable(std::vector<std::vector<std::string> > &varMatrix, std::string varName, double &varVal);
bool scrapeMatrixForVariable(std::vector<std::vector<std::string> > &varMatrix, std::string varName, int &varVal);
bool scrapeMatrixForVariable(std::vector<std::vector<std::string> > &varMatrix, std::string varName, std::string &varVal);