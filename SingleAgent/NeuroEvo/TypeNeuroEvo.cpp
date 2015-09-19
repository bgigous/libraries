#include "TypeNeuroEvo.h"


TypeNeuroEvo::TypeNeuroEvo(void)
{
}


TypeNeuroEvo::~TypeNeuroEvo(void)
{
	for (NeuroEvo* ne: NETypes){
		delete ne;
	}
}
