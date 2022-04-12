// Created by Charvi on 25/03/22
#ifndef MULTI_ASTAR_HPP
#define MULTI_ASTAR_HPP

#include <vector>
#include <ros/ros.h>
#include "astar.hpp"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

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
        traj_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("multi_astar_vis", 1);

        // Add current agent positions to the trajectory cache in the graph
        for(int i=0;i<agents.size();i++) {
            
            double* agentPos = currPos[i];

            // Create stationary trajectory
            std::map<double,std::pair<double,double>> traj_map;
            traj_map[-1.0] = std::make_pair(agentPos[0],agentPos[1]);
            graph->addTrajCache(agents[i], traj_map);
        }
    }

    ~MultiAStar(){
    }

    bool run_multi_astar(){
        vector<AStar> planner_vec;
        assignPriority(agentId);
        for(int i=0;i<numAgents;i++){
            int index = agentId[i];
            double* start= curr_Pos[index];
            double* goal = tar_Pos[index];
            AStar planner(agentsNames[index],pg,goal,start,&nh_);
            ros::Duration(1.0).sleep();
            bool planning_success = planner.run_planner(10*pg->getNumNodes());

            std::vector<geometry_msgs::PoseStamped> traj;
            if(planning_success) {
                planner_vec.push_back(planner);

                // Create the trajectory
                for(auto poses:planner.trajectory) {

                    geometry_msgs::PoseStamped pose;
                    pose.header.seq=0;
                    pose.header.stamp = ros::Time::now();
                    pose.header.frame_id = "map";
                    pose.pose.position.x = poses[0];
                    pose.pose.position.y = poses[1];
                    pose.pose.position.z = poses[2];
                    pose.pose.orientation.w = 1.0;
                    traj.push_back(pose);
                }
                trajectory_map[agentsNames[index]] = traj;
            }
        }

        //visualise_paths(planner_vec);
        return false;
    }

    void visualise_paths(vector<AStar> planner_vec)
    {
        bool vis_active = false;
        int id = 0;
        int it = 0;

        visualization_msgs::Marker marker;
        visualization_msgs::MarkerArray marker_arr;

        // common info
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = pg->getFrame();
        marker.type = 2; //sphere
        marker.lifetime = ros::Duration();
        marker.action = 0; // add
        marker.pose.orientation.w = 0;

        marker.scale.x = 0.1f;
        marker.scale.y = 0.1f;
        marker.scale.z = 0.1f;
        std_msgs::ColorRGBA color = std_msgs::ColorRGBA();
        color.r = 254.0f;
        color.g = 254.0f;
        color.b = 254.0f;
        color.a = 1;
        marker.color = color;
        marker.mesh_use_embedded_materials = false;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;

        ROS_INFO("Visualising trajectory for %ld planners\n",planner_vec.size());
        do {
            vis_active = false;
            for(auto planner:planner_vec)
            {
                if(it<planner.trajectory.size())
                {
                    // Add point to marker array
                    marker.id = id++;
                    marker.pose.position.x = planner.trajectory[it][0];
                    marker.pose.position.y = planner.trajectory[it][1];
                    marker.pose.position.z = 0.0;
                    marker_arr.markers.push_back(marker);
                    vis_active = true;
                }
            }
            // Publish marker array
            traj_pub_.publish(marker_arr);
            ros::Duration(0.1).sleep();
            it++;
        } while (vis_active && ros::ok());
    }

    std::map<string,std::vector<geometry_msgs::PoseStamped>> trajectory_map;
    
private:

    ros::Publisher traj_pub_;
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