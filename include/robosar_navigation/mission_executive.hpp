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
#include "actionlib_msgs/GoalStatus.h"
#include <actionlib/client/simple_action_client.h>
#include <thread>


class MissionExecutive {

public:
    MissionExecutive() : nh_("") {

        ROS_INFO("Starting a new RoboSAR Nav Mission! Get ready for a show!");

        status_subscriber_ = nh_.subscribe("/robosar_agent_bringup/status", 1, &MissionExecutive::statusCallback, this);
        task_allocation_subscriber = nh_.subscribe("task_allocation", 10, &MissionExecutive::taskAllocationCallback,this);
        // Get latest fleet info from agent bringup
        status_client = nh_.serviceClient<robosar_messages::agent_status>("/robosar_agent_bringup_node/agent_status");

        fleet_info = getFleetStatusInfo();
        ROS_INFO(" [MISSION_EXEC] Active fleet size %ld",fleet_info.size());
        // Create controllers for these agents
        createControllerActionServers(fleet_info);
        
        fleet_status_outdated = false;
        
        // Running our executive
        mission_thread_ = std::thread(&MissionExecutive::run_mission, this);
    }

    ~MissionExecutive() {

        // destroy controller servers
        for(std::map<std::string,LGControllerAction*>::iterator it=controller_map.begin();it!= controller_map.end();it++ )
            delete it->second;

        // free the heap
        for(auto startHeap : start_vec)
            delete [] startHeap;

        for(auto goalHeap : goal_vec)
            delete [] goalHeap; 
        
        mission_thread_.join();
    }


private:

    void run_mission() {

        ros::Rate loop_rate(10);

        while (ros::ok()) {
            
            if(areControllersIdle() && !agents.empty())
            {
                ROS_INFO("Processing Tasks %ld",agents.size());
                // Process tasks from task allocator
                gridmap.clearTrajCache();
                MultiAStar multi_astar(&gridmap,currPos,targetPos,agents);

                ros::Duration(2.0).sleep();
                bool status = multi_astar.run_multi_astar();

                for(auto agent:agents){
                    
                    // Check if planning was successful
                    if(multi_astar.trajectory_map.find(agent)!=multi_astar.trajectory_map.end())  {

                        actionlib::SimpleActionClient<robosar_controller::RobosarControllerAction> ac(agent, true);
                        ROS_INFO("Waiting for action server to start.");
                        // wait for the action server to start
                        //ac.waitForServer(); //will wait for infinite time

                        ROS_INFO("Action server started, sending goal.");
                        robosar_controller::RobosarControllerGoal goal;
                        
                        // Get the trajectory from the map
                        std::vector<geometry_msgs::PoseStamped> traj_agent =  multi_astar.trajectory_map[agent];
                        for(auto pose:traj_agent)
                            goal.path.poses.push_back(pose);
  
                        // Send the goal
                         ac.sendGoal(goal);
                    }
                }

                // Clear out old tasks
                agents.clear();
                currPos.clear();
                targetPos.clear();

            }
            
            loop_rate.sleep();
        }

    }

    bool areControllersIdle() {
        std::map<std::string,LGControllerAction*>::iterator it;
        for (it = controller_map.begin(); it != controller_map.end(); it++){
            
            if(it->second->as_.isActive())
                return false;
        }
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
            LGControllerAction *controller = new LGControllerAction(agent);
            // save it in the map
            controller_map[agent] = controller;
        }
        return true;
    }

    void statusCallback(const std_msgs::Bool &status_msg) {
        fleet_status_outdated = true;
    }

    void taskAllocationCallback(robosar_messages::task_allocation ta_msg) {
    
        for(int i=0;i<ta_msg.id.size();i++){
            ROS_INFO("Start x:%f,y:%f, Goal x:%f,y:%f",ta_msg.startx[i],ta_msg.starty[i],ta_msg.goalx[i],ta_msg.goaly[i]);

            double* goalHeap = new double[3];
            goalHeap[0] = ta_msg.goalx[i]; 
            goalHeap[1] = ta_msg.goaly[i]; 
            goalHeap[2] = 0.0; 

            double* startHeap = new double[3];
            startHeap[0] = ta_msg.startx[i]; 
            startHeap[1] = ta_msg.starty[i]; 
            startHeap[2] = 0.0; 
        
            agents.push_back(ta_msg.id[i]);
            currPos.push_back(startHeap);
            targetPos.push_back(goalHeap);
            
            // Save them on the heap so that you can free them later
            goal_vec.push_back(goalHeap);
            start_vec.push_back(startHeap);
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
    std::thread mission_thread_;

    // Since we are sending a pointer we need to keep these from going out of scope until search is complete
    std::vector<double*> goal_vec;
    std::vector<double*> start_vec;
};

#endif