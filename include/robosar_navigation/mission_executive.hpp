// Created by Indraneel and Charvi on 5/04/21

#ifndef MISSION_EXECUTIVE_HPP
#define MISSION_EXECUTIVE_HPP

#include "graph_3d_grid.hpp"
#include "multi_astar.hpp"
#include <ros/console.h>
#include <std_msgs/Bool.h>
#include "robosar_messages/agent_status.h"
#include "lg_action_server.hpp"

#include "robosar_messages/task_allocation.h"
class MissionExecutive {

public:
    MissionExecutive() : nh_("") {

        ROS_INFO("Starting a new RoboSAR Nav Mission! Get ready for a show!");

        status_subscriber_ = nh_.subscribe("/robosar_agent_bringup/status", 1, &MissionExecutive::statusCallback, this);
        task_allocation_subscriber = nh_.subscribe("task_allocation", 1, &MissionExecutive::taskAllocationCallback,this);
        // Get latest fleet info from agent bringup
        status_client = nh_.serviceClient<robosar_messages::agent_status>("fleet_status");

        fleet_info = getFleetStatusInfo();

        // Create controllers for these agents
        createControllerActionServers(fleet_info);
        
        fleet_status_outdated = false;
        
        run_mission();
    }

    ~MissionExecutive() {

    }


private:

    void run_mission() {



    }

    bool areControllersIdle() {
        return true;
    }

    std::set<string> getFleetStatusInfo() {

        robosar_messages::agent_status srv;

        if (status_client.call(srv)) {
            std::vector<string> agentsVec = srv.response.agents_active;
            std::set<string> agentsSet;

            for(auto agent:agentsVec)
                agentsSet.insert(agent);

            return agentsSet;
        }
        else
        {
            ROS_ERROR("Failed to call fleet info service");
            return fleet_info;
        }
    }

    bool createControllerActionServers(std::set<std::string> new_agents) {
        
        for(auto agent:new_agents){
            // Create new controller server
            LGControllerAction controller(agent);
            // save it in the map
            controller_map[agent] = &controller;
        }
        return true;
    }

    void statusCallback(const std_msgs::Bool &status_msg) {
        fleet_status_outdated = true;
    }

    void taskAllocationCallback(robosar_messages::task_allocation ta_msg){
    
        for(int i=0;i<ta_msg.id.size();i++){
            double goal[] = {ta_msg.goalx[i],ta_msg.goaly[i],0.0};
            double start[] = {ta_msg.startx[i],ta_msg.starty[i],0.0};
            agents.push_back(ta_msg.id[i]);
            currPos.push_back(start);
            targetPos.push_back(goal);
        }
    }

    std::map<std::string,LGControllerAction*> controller_map;
    std::set<std::string> fleet_info;
    ros::ServiceClient status_client; 

    std::vector<std::string> agents;
    std::vector<double*> currPos;
    std::vector<double*> targetPos;
    Graph3DGrid gridmap;
    ros::Subscriber status_subscriber_,task_allocation_subscriber;
    ros::NodeHandle nh_;
    bool fleet_status_outdated;
};

#endif