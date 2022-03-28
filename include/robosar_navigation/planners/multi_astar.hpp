// Created by Charvi on 25/03/22
#ifndef MULTI_ASTAR_HPP
#define MULTI_ASTAR_HPP

#include <vector>
#include <ros/ros.h>
using namespace std;

class MultiAStar {

public:
    MultiAStar(Graph* graph, vector<pair<double,double>> currPos, vector<pair<double,double>> targetPos, int numAgents):pg(graph){
        this->numAgents = numAgents;
        for(int i=0;i<numAgents;i++)
            agentId.push_back(i);
        this->curr_Pos = currPos;
        this->tar_Pos = targetPos;
        //curr_Pos.push_back(make_pair(0,0));
        //tar_Pos.push_back(make_pair(100,100));
    }

    ~MultiAStar(){
    }

    bool run_multi_astar(){
        assignPriority(agentId);
        for(int i=0;i<numAgents;i++){
            int index = agentId[i];
            double start[2]={curr_Pos[index].first,curr_Pos[index].second};
            double goal[2] = {tar_Pos[index].first,tar_Pos[index].second};
            AStar planner(pg,goal,start);
            bool path_for_agent = planner.run_planner(100); //TODO check what format path is returned in
        }
        return false;
    }

private:
    int numAgents;
    Graph* pg;
    vector<int> agentId;
    vector<pair<double,double>> curr_Pos;
    vector<pair<double,double>> tar_Pos;
    vector<int> assignPriority(vector<int> agentId){
        return agentId;
    }
};

#endif