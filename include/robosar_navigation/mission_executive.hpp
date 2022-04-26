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
#include <algorithm>
#include <mutex>


class MissionExecutive {

public:
    MissionExecutive() : nh_("") {

        ROS_INFO("Starting a new RoboSAR Nav Mission! Get ready for a show!");

        status_subscriber_ = nh_.subscribe("/robosar_agent_bringup_node/status", 1, &MissionExecutive::statusCallback, this);
        task_allocation_subscriber = nh_.subscribe("task_allocation", 10, &MissionExecutive::taskAllocationCallback,this);
        // Get latest fleet info from agent bringup
        status_client = nh_.serviceClient<robosar_messages::agent_status>("/robosar_agent_bringup_node/agent_status");

        fleet_info = getFleetStatusInfo();
        ROS_INFO(" [MISSION_EXEC] Active fleet size %ld",fleet_info.size());
        // Create controllers for these agents
        createControllerActionServers(fleet_info);
        // Create clients for these controllers
        createControllerActionClients(fleet_info);
        
        fleet_status_outdated = false;
        
        // Running our executive
        mission_thread_ = std::thread(&MissionExecutive::run_mission, this);
    }

    ~MissionExecutive() {

        // destroy controller servers
        for(std::map<std::string,LGControllerAction*>::iterator it=controller_map.begin();it!= controller_map.end();it++ )
            delete it->second;

        // destroy controller clients
        for(std::map<std::string,actionlib::SimpleActionClient<robosar_controller::RobosarControllerAction>*>::iterator it=client_map.begin();
                                                                            it!= client_map.end();it++ )
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
            
            if(!agentsCB.empty())
            {
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    // Copy over callback data
                    agents = agentsCB;
                    currPos = currPosCB;
                    targetPos = targetPosCB;

                    // clear callback data
                    agentsCB.clear();
                    currPosCB.clear();
                    targetPosCB.clear();
                }

                ROS_INFO("[MISSION_EXEC] Processing Tasks %ld",agents.size());
                // Process tasks from task allocator
                gridmap.clearTrajCache();
                gridmap.addGoalCache(targetPos,agents);
                MultiAStar multi_astar(&gridmap,currPos,targetPos,agents);

                ros::Duration(2.0).sleep();
                bool status = multi_astar.run_multi_astar();

                for(auto agent:agents){
                    
                    // Check if planning was successful
                    if(multi_astar.trajectory_map.find(agent)!=multi_astar.trajectory_map.end())  {

                        actionlib::SimpleActionClient<robosar_controller::RobosarControllerAction> *ac 
                                    = client_map[agent];
                        ROS_INFO("[MISSION_EXEC] Waiting for action server to start.");
                        // wait for the action server to start
                        //ac.waitForServer(); //will wait for infinite time

                        ROS_INFO("[MISSION_EXEC] Action server started, sending goal.");
                        robosar_controller::RobosarControllerGoal goal;
                        
                        // Get the trajectory from the map
                        std::vector<geometry_msgs::PoseStamped> traj_agent =  multi_astar.trajectory_map[agent];
                        for(auto pose:traj_agent)
                            goal.path.poses.push_back(pose);
  
                        // Send the goal
                         ac->sendGoal(goal);
                    }
                }

                // Clear out old tasks
                agents.clear();
                currPos.clear();
                targetPos.clear();

            }
            else if(fleet_status_outdated) {
                ROS_WARN("[MISSION_EXEC] New fleet info received!!");
                processNewAgentStatus(getFleetStatusInfo());
                fleet_status_outdated = false;
            }
            
            loop_rate.sleep();
        }

    }

    void processNewAgentStatus(std::set<string> new_fleet_info) {

        // Get newly added agents
        std::set<string> additions;
        std::set<string> subtractions;
        std::set_difference(new_fleet_info.begin(), new_fleet_info.end(),
                                fleet_info.begin(), fleet_info.end(), std::inserter(additions, additions.begin()));

        std::set_difference(fleet_info.begin(), fleet_info.end(),
                                new_fleet_info.begin(), new_fleet_info.end(), std::inserter(subtractions, subtractions.begin()));

        if(!additions.empty()) {
            createControllerActionServers(additions);
            createControllerActionClients(additions);

            // Add them to our fleet info!
            fleet_info.insert(additions.begin(),additions.end());
        }

        if(!subtractions.empty()) {
            // Dont destroy the action server for now
            // Just stop it from executing
            for(std::set<string>::iterator it=subtractions.begin();it!=subtractions.end();it++) {
                if(controller_map[*it]->as_.isActive()) {
                    ROS_WARN("[MISSION_EXEC] Preempting controller action execution for %s",&(*it)[0]);
                    client_map[*it]->cancelGoal();
                }
            }
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
            ROS_ERROR("[MISSION_EXEC] Failed to call fleet info service");
            return fleet_info;
        }
    }

    bool createControllerActionServers(std::set<std::string> new_agents) {
        
        for(auto agent:new_agents){
            // Create new controller server
            LGControllerAction *controller = new LGControllerAction(agent);
            ROS_DEBUG("[MISSION_EXEC] Created controller for %s",&agent[0]);
            // save it in the map
            controller_map[agent] = controller;
        }
        return true;
    }

     bool createControllerActionClients(std::set<std::string> new_agents) {
        
        for(auto agent:new_agents){
            // Create new controller client
            actionlib::SimpleActionClient<robosar_controller::RobosarControllerAction>* ac = 
                                    new actionlib::SimpleActionClient<robosar_controller::RobosarControllerAction>(agent, true);
            ROS_DEBUG("[MISSION_EXEC] Created controller client for %s",&agent[0]);
            // save it in the map
            client_map[agent] = ac;
        }
        return true;
    }

    

    void statusCallback(const std_msgs::Bool &status_msg) {
        fleet_status_outdated = true;
    }

    void taskAllocationCallback(robosar_messages::task_allocation ta_msg) {
        
        std::lock_guard<std::mutex> lock(mutex);
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
        
            agentsCB.push_back(ta_msg.id[i]);
            currPosCB.push_back(startHeap);
            targetPosCB.push_back(goalHeap);
            
            // Save them on the heap so that you can free them later
            goal_vec.push_back(goalHeap);
            start_vec.push_back(startHeap);
        }
    }

    std::map<std::string,LGControllerAction*> controller_map;
    std::map<std::string,actionlib::SimpleActionClient<robosar_controller::RobosarControllerAction>*> client_map;
    std::set<std::string> fleet_info;
    ros::ServiceClient status_client; 

    std::vector<std::string> agentsCB;
    std::vector<double*> currPosCB;
    std::vector<double*> targetPosCB;

    std::vector<std::string> agents;
    std::vector<double*> currPos;
    std::vector<double*> targetPos;
    Graph gridmap;
    ros::Subscriber status_subscriber_,task_allocation_subscriber;
    ros::NodeHandle nh_;
    bool fleet_status_outdated;
    std::thread mission_thread_;

    // Since we are sending a pointer we need to keep these from going out of scope until search is complete
    std::vector<double*> goal_vec;
    std::vector<double*> start_vec;
    std::mutex mutex;
};

#endif