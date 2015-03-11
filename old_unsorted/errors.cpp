#pragma once

#include "errors.h"

using namespace std;

void fatal(int errorNum, string functionName, string hint){
	printf("FATAL%i: %s(...) %s Exiting.\n\n",errorNum,functionName.c_str(), hint.c_str());
	system("pause");
	exit(errorNum);
}
