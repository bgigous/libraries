// Copyright 2016 Carrie Rebhuhn
#include "MultiagentNE.h"
#include <vector>

using std::vector;

MultiagentNE::MultiagentNE(void) {}

MultiagentNE::MultiagentNE(int n_agents, NeuroEvoParameters* NE_params) :
    NE_params(NE_params) {
    for (int i = 0; i < n_agents; i++) {
        agents.push_back(new NeuroEvo(NE_params));
    }
}

MultiagentNE::~MultiagentNE(void) {
    for (size_t i = 0; i < agents.size(); i++) {
        delete agents[i];
    }
}

void MultiagentNE::generateNewMembers() {
    // Generate new population members
    for (size_t i = 0; i < agents.size(); i++) {
        reinterpret_cast<NeuroEvo*>(agents[i])->generateNewMembers();
    }
}

void MultiagentNE::selectSurvivors() {
    // Specific to Evo: select survivors
    for (size_t i = 0; i < agents.size(); i++) {
        reinterpret_cast<NeuroEvo*>(agents[i])->selectSurvivors();
    }
}

bool MultiagentNE::setNextPopMembers() {
    // Kind of hacky; select the next member and return true if not at the end
    // Specific to Evo

    vector<bool> is_another_member(agents.size(), false);
    for (size_t i = 0; i < agents.size(); i++) {
        is_another_member[i]
            = reinterpret_cast<NeuroEvo*>(agents[i])->selectNewMember();
    }
    for (size_t i = 0; i < is_another_member.size(); i++) {
        if (!is_another_member[i]) {
            return false;
        }
    }
    return true;
}
