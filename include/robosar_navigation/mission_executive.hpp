// Created by Indraneel and Charvi on 5/04/21

#ifndef MISSION_EXECUTIVE_HPP
#define MISSION_EXECUTIVE_HPP

#include "graph_3d_grid.hpp"
#include "multi_astar.hpp"
#include <ros/console.h>
#include <std_msgs/Bool.h>
#include "robosar_messages/agent_status.h"
#include "lg_action_server.hpp"
#include "robosar_messages/mission_command.h"
#include <robosar_messages/robosar_controller.h>
#include "robosar_messages/task_allocation.h"
#include "actionlib_msgs/GoalStatus.h"
#include <actionlib/client/simple_action_client.h>
#include <thread>
#include <algorithm>
#include <mutex>


class MissionExecutive {

public:
    MissionExecutive() : nh_(""), arbitraryPoint{-2000000.0,-20000000.0}  {

        ROS_INFO("[MISSION_EXEC] Starting a new RoboSAR Nav Mission! Get ready for a show!");

        task_allocation_subscriber = nh_.subscribe("task_allocation", 10, &MissionExecutive::taskAllocationCallback,this);

        // Running our executive
        gui_subscriber_ = nh_.subscribe("/system_mission_command", 1, &MissionExecutive::eStopCallback, this);
        mission_thread_ = std::thread(&MissionExecutive::run_mission, this);
    }

    ~MissionExecutive() {
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
            
            if(eStopCommanded) {
                ROS_INFO("[MISSION_EXEC] ESTOP recived! Shutting down rbosar_nav!");
                processEStop();
            }
            else if(!agentsCB.empty())
            {
                {
                    std::lock_guard<std::mutex> lock(mutex);
                    // Copy over callback data
                    agents = agentsCB;
                    currPos = currPosCB;
                    targetPos = targetPosCB;
                    goalType = goalTypeCB;

                    // clear callback data
                    agentsCB.clear();
                    currPosCB.clear();
                    targetPosCB.clear();
                    goalTypeCB.clear();
                }

                ROS_INFO("[MISSION_EXEC] Processing Tasks %ld",agents.size());
                // Process tasks from task allocator
                gridmap.clearTrajCache();
                MultiAStar multi_astar(&gridmap,currPos,targetPos,agents);

                ros::Duration(2.0).sleep();
                bool status = multi_astar.run_multi_astar();

                // Create service request
                ros::ServiceClient controller_client = nh_.serviceClient<robosar_messages::robosar_controller>("robosar_controller/lazy_traffic_controller");
                ROS_INFO("[MISSION_EXEC] Waiting for controller service to start.");
                // wait for the action server to start
                controller_client.waitForExistence(); //will wait for infinite time
                ROS_INFO("[MISSION_EXEC] Controller server started, sending goal.");

                robosar_messages::robosar_controller srv;
                for(int i=0;i<agents.size();i++) {
                    
                    // Check if planning was successful
                    if(multi_astar.trajectory_map.find(agents[i])!=multi_astar.trajectory_map.end())  {

                        // // Get the trajectory from the map
                        std::vector<geometry_msgs::PoseStamped> traj_agent =  multi_astar.trajectory_map[agents[i]];
                        nav_msgs::Path path_agent;
                        for(auto pose:traj_agent)
                            path_agent.poses.push_back(pose);
                        // Add agent path to the service request
                        srv.request.paths.push_back(path_agent);
                        srv.request.agent_names.push_back(agents[i]);
                        srv.request.goal_type.push_back(goalType[i]);
                    }
                }
                // Call the controller service
                if (controller_client.call(srv))
                {
                    ROS_INFO("[MISSION_EXEC] Controller service called successfully");
                }
                else
                {
                    ROS_ERROR("[MISSION_EXEC] Failed to call controller service");
                }

                // Clear out old tasks
                agents.clear();
                currPos.clear();
                targetPos.clear();
                goalType.clear();

            }
            
            loop_rate.sleep();
        }

    }

    void eStopCallback(robosar_messages::mission_command gui_msg) {
        if(gui_msg.data==2) {
            eStopCommanded = true;
        }
        
    }

    void processEStop() {

        ros::ServiceClient controller_client = nh_.serviceClient<robosar_messages::robosar_controller>("robosar_controller/lazy_traffic_controller");
        ROS_INFO("[MISSION_EXEC] Waiting for controller service to start. ");
        // wait for the action server to start
        controller_client.waitForExistence(); //will wait for infinite time

        robosar_messages::robosar_controller srv;
        if(eStopCommanded) {
            srv.request.stop_controller = true;
            
        }
        // Call the controller service
        if (controller_client.call(srv))
        {
            ROS_INFO("[MISSION_EXEC] Controller service called successfully");
            eStopCommanded = false;
        }
        else
        {
            ROS_ERROR("[MISSION_EXEC] Failed to call controller service");
        }
    }

    void taskAllocationCallback(robosar_messages::task_allocation ta_msg) {
        
        std::lock_guard<std::mutex> lock(mutex);
        for(int i=0;i<ta_msg.id.size();i++){
            ROS_INFO("Start x:%f,y:%f, Goal x:%f,y:%f, Type: %d",ta_msg.startx[i],ta_msg.starty[i],ta_msg.goalx[i],ta_msg.goaly[i],ta_msg.goal_type[i]);

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
            goalTypeCB.push_back(ta_msg.goal_type[i]);
            
            // Save them on the heap so that you can free them later
            goal_vec.push_back(goalHeap);
            start_vec.push_back(startHeap);
        }
    }

    std::set<std::string> fleet_info;
    ros::ServiceClient status_client; 

    std::vector<std::string> agentsCB;
    std::vector<double*> currPosCB;
    std::vector<double*> targetPosCB;
    std::vector<int> goalTypeCB;

    std::vector<std::string> agents;
    std::vector<double*> currPos;
    std::vector<double*> targetPos;
    std::vector<int> goalType;
    Graph gridmap;
    ros::Subscriber status_subscriber_,task_allocation_subscriber, gui_subscriber_;
    ros::NodeHandle nh_;
    bool fleet_status_outdated;
    std::thread mission_thread_;
    bool eStopCommanded = false;

    // Since we are sending a pointer we need to keep these from going out of scope until search is complete
    std::vector<double*> goal_vec;
    std::vector<double*> start_vec;
    std::mutex mutex;
    double arbitraryPoint[2];
};

#endif