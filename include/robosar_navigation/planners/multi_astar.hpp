#ifndef MULTI_ASTAR_HPP
#define MULTI_ASTAR_HPP

#include <vector>
#include <ros/ros.h>
using namespace std;


class MultiAStar {

public:
    MultiAStar(Graph* graph):pg(graph){
        numAgents = 3;
        for(int i=0;i<numAgents;i++)
            agentId.push_back(i);
        
        curr_Pos.push_back(make_pair(0,0));
        tar_Pos.push_back(make_pair(100,100));
    }

    ~MultiAStar(){
    }

    bool run_multi_astar(){
        assignPriority(agentId);
        for(int i=0;i<numAgents;i++){
            int start[2]={curr_Pos[i].first,curr_Pos[i].second};
            int goal[2] = {tar_Pos[i].first,tar_Pos[i].second};
            AStar planner(pg,start,goal);
            bool path_for_agent = planner.run_planner(100); //TODO check what format path is returned in
        }
        return false;
    }

private:
    int numAgents;
    Graph* pg;
    vector<int> agentId;
    vector<pair<int,int>> curr_Pos;
    vector<pair<int,int>> tar_Pos;
    vector<int> assignPriority(vector<int> agentId){
        return agentId;
    }
    tf::TransformListener listener;
};

#endif