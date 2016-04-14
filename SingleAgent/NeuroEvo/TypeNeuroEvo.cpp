// Copyright 2016 Carrie Rebhuhn
#include "TypeNeuroEvo.h"


TypeNeuroEvo::TypeNeuroEvo(void) {
}


TypeNeuroEvo::~TypeNeuroEvo(void) {
    for (NeuroEvo* ne : NETypes) {
        delete ne;
    }
}
