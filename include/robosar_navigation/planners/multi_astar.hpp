// Created by Charvi on 25/03/22
#ifndef MULTI_ASTAR_HPP
#define MULTI_ASTAR_HPP

#include <vector>
#include <ros/ros.h>
#include "astar.hpp"
using namespace std;

class MultiAStar {

public:
    MultiAStar(Graph* graph, vector<double*> currPos, vector<double*> targetPos, vector<string> agents):pg(graph), nh_("~"){
        this->numAgents = agents.size();
        for(int i=0;i<numAgents;i++)
            agentId.push_back(i);
        this->curr_Pos = currPos;
        this->tar_Pos = targetPos;
        agentsNames = agents;
    }

    ~MultiAStar(){
    }

    bool run_multi_astar(){
        assignPriority(agentId);
        for(int i=0;i<numAgents;i++){
            int index = agentId[i];
            double* start= curr_Pos[index];
            double* goal = tar_Pos[index];
            AStar planner(agentsNames[index],pg,goal,start,&nh_);
            ros::Duration(1.0).sleep();
            bool planning_success = planner.run_planner(pg->getNumNodes());

        }
        return false;
    }

private:
    ros::NodeHandle nh_;
    int numAgents;
    Graph* pg;
    vector<string> agentsNames;
    vector<int> agentId;
    vector<double*> curr_Pos;
    vector<double*> tar_Pos;
    vector<int> assignPriority(vector<int> agentId){
        return agentId;
    }
};

#endif