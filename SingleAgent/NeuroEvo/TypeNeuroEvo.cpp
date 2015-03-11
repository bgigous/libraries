#include "TypeNeuroEvo.h"


TypeNeuroEvo::TypeNeuroEvo(void)
{
}


TypeNeuroEvo::~TypeNeuroEvo(void)
{
	for (int i=0; i<NETypes.size(); i++){
		delete NETypes[i];
	}
}
