// Copyright 2016 Carrie Rebhuhn
#ifndef DOMAINS_UTM_LINK_H_
#define DOMAINS_UTM_LINK_H_


#include "UTMAgentReward.h"
#include <numeric>
#include <algorithm>
#include <functional>
#include <list>
#include <vector>

class Link {
 public:
    Link(int ID, int source_set, int target_set,
        int time, std::vector<size_t> capacity, int cardinal_dir) :
        ID(ID),
        source(source_set),
        target(target_set),
        time(time),
        cardinal_dir(cardinal_dir),
        capacity(capacity),
        traffic(size_t(UTMModes::UAVType::NTYPES), std::list<UAV*>())
    {}

    //!
    bool at_capacity(size_t UAV_type) {
        return number_over_capacity(UAV_type) >= 0;
    }

    int number_over_capacity(size_t type_ID) {
        return static_cast<int>(traffic[type_ID].size() - capacity[type_ID]);
    }
    std::vector<std::list<UAV*> > traffic;


    //! Returns the predicted amount of time it would take to cross the node if
    //! the UAV got there immediately
    matrix1d predicted_traversal_time() {
        // Get predicted wait time for each type of UAV
        matrix1d predicted(traffic.size());
        for (size_t i = 0; i < traffic.size(); i++) {
            // Collect wait times on all UAVs ON the link
            matrix1d waits;
            for (UAV* u : traffic[i]) {
                waits.push_back(u->t);
            }

            // Sort by wait (descending)
            std::sort(waits.begin(), waits.end(), greater<double>());

            size_t n_ok = capacity[i] - 1;  // UAVs you don't have to wait for
            size_t n_wait = waits.size() - n_ok;  // UAVs before you in line
            if (waits.size() > n_ok)
                waits.resize(n_wait);


            // Store predicted link time.
            double w = easymath::sum(waits);
            predicted[i] = time + w;
        }


        return predicted;
    }


    //! Grabs the UAV u from link l
    void move_from(UAV* u, Link* l) {
        // Add to other list (u is temporarily duplicated)
        add(u);

        // Remove from previous node (l)
        l->remove(u);
    }

    //! Also sets the time
    void add(UAV* u) {
        u->t = time;
        traffic.at(size_t(u->type_ID)).push_back(u);
        u->cur_link_ID = ID;
        u->mem = source;
    }

    void remove(UAV* u) {
        traffic[size_t(u->type_ID)].erase(
            std::find(traffic[size_t(u->type_ID)].begin(),
                traffic[size_t(u->type_ID)].end(), u));
    }

    const int source;
    const int target;
    const int cardinal_dir;
    void reset() {
        traffic = std::vector < std::list<UAV*> >
            (size_t(UTMModes::UAVType::NTYPES), std::list<UAV*>());
    }


 private:
    const int ID;
    const easymath::XY source_loc;
    const easymath::XY target_loc;
    const int time;  // Amount of time it takes to travel across link

    std::vector<size_t> capacity;  // Capacity for each UAV type [#types]
};

/**
* Provides an interface for link agents to interact with the simulator.
* This allows for redefinition of agents in the UTM simulation, and also
* collects information relevant to calculating difference, global, and
* local rewards. Logging of agent actions can also be performed for
* qualitative assessment of behavior.
*/

class LinkAgentManager : public IAgentManager {
 public:
    // The agent that communicates with others
    LinkAgentManager(int n_edges, int n_types, std::vector<Link*> links,
        UTMModes* params) :
        n_edges(n_edges), n_types(n_types), IAgentManager(params), links(links)
    {};
    ~LinkAgentManager() {}
    // weights are ntypesxnagents

    const int n_edges;
    const int n_types;

    /**
    * Translates the output of a neural network into costs applied to a link.
    * This can include the 'predicted' cost of the link, which is the
    * traversal time plus the instantaneous wait time at that link. In
    * addition, this translates neural network output, which is in the form
    * [agent #][type #] into weights on the graph, which is in the form
    * [type #][link #]. In the case of link agents, this mapping is
    * agent # = link #, but this is not the case with sector agents.
    * @param agent_actions neural network output, in the form of [agent #][type #]
    * @return the costs for each link in the graph
    */
    virtual matrix2d actions2weights(matrix2d agent_actions) {
        matrix2d weights = easymath::zeros(n_types, n_edges);
        double alpha = 0.0;

        for (int i = 0; i < n_edges; i++) {
            matrix1d predicted = links.at(i)->predicted_traversal_time();
            for (int t = 0; t < n_types; t++) {
                // note: is there a scalable metric here?
                weights[t][i] = predicted[t] + agent_actions[i][t] * alpha;
                // weights[t][i] = agent_actions[i][t]*1000.0;
            }
        }
        return weights;
    }

    std::vector<Link*> links;

    /**
    * Adds to the delay for the agent assigned to that link.
    * Agent reward metrics and link function are kept separate.
    */
    void add_delay(UAV* u) {
        metrics.at(u->cur_link_ID).local[size_t(u->type_ID)]++;
    }

    void add_downstream_delay_counterfactual(UAV* u) {
        // remove the effects of the UAV for the counterfactual..
        // calculate the G that means that the UAV's impact is removed...

        if (square_reward) {
            // Non-functional, todo
            printf("SQUARED TODO");
            exit(1);
        } else {
            for (size_t i = 0; i < metrics.size(); i++) {
                if (u->links_touched.count(i) == 0) {
                    metrics[i].G_minus_downstream[size_t(u->type_ID)]++;
                } else {
                    continue;
                }
            }
        }
    }

    void detect_conflicts() {
        for (size_t i = 0; i < links.size(); i++) {
            for (size_t j = 0; j < links[i]->traffic.size(); j++) {
                int over_capacity = links[i]->number_over_capacity(j);
                if (over_capacity <= 0)
                    continue;  // no congestion
                else if (square_reward)
                    metrics[i].local[j] += over_capacity*over_capacity;
                else
                    metrics[i].local[j] += over_capacity;
            }
        }
    }
};
#endif  // DOMAINS_UTM_LINK_H_
