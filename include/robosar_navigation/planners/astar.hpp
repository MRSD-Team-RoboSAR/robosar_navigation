// Created by Indraneel on 17/03/21

#ifndef ASTAR_HPP
#define ASTAR_HPP

#include "graph_2d_grid.hpp"
#include "graph_3d_grid.hpp"
#include <queue>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <unordered_map>

// Underlying graph datatype can be changed here
#undef Graph
#define Graph Graph3DGrid

#define POT_HIGH 1.0e10		// unassigned cell potential
class AStar {

public:
    AStar(std::string ns_, Graph* graph,double* g, double* s, ros::NodeHandle *nh) : planner_initialised(false),pg(graph), heuristic_weight(10.0f),
                                                    goalNode(-1,-1,0.0), startNode(-1,-1, 0.0) {

        goal = g;
        start = s;

        goalNode = graph->getNode(goal);
        startNode = graph->getNode(start);

        // Check if goal and start are collision free
        if(graph->collisionCheck(goalNode))
        {
            ROS_WARN("[AStar] Goal in collision! %f,%f\n", goal[0], goal[1]);
            return;
        }
        else if(graph->collisionCheck(startNode))
        {
           ROS_WARN("[AStar] Start in collision! %f,%f\n", start[0], start[1]); 
           return;
        }

        ROS_INFO("[AStar] Setting goal to %f,%f\n", goal[0], goal[1]);
        ROS_INFO("[AStar] Setting start to %f,%f\n",start[0], start[1]);

        // Add start to the priority queue
        std::pair<float,Graph::Node> sn = std::make_pair(0.0f,startNode);
        queue.push(sn);

        // traj publisher for rviz
        traj_pub_ = nh->advertise<nav_msgs::Path>(ns_+"/plan", 1);
        // start and goal publisher for rviz
        endpoint_pub_ = nh->advertise<visualization_msgs::MarkerArray>(ns_+"/endpoints", 1);

        planner_initialised = true;
    }

    ~AStar(){
    }

    bool run_planner(int cycles) {

        int cycle = 0;
        bool path_found = false;

        if(!planner_initialised)
        return false;

        publishEndpoints();

        // do main cycle
        for (; cycle < cycles && !queue.empty(); cycle++) // go for this many cycles or if queue is over
        {
            ROS_INFO("On cycle %d / %d with queue size %ld \n",cycle,cycles,queue.size());
            std::pair<float,Graph::Node> qn = queue.top();
            queue.pop();
            // Mark as explored
            if(pending.find(qn.second)!=pending.end())
                ROS_WARN("Already explored!!");
            pending.insert(qn.second);

            // Check if reached goal
            if(qn.second == goalNode)
            {
                goalNode = qn.second;
                path_found = true;
                break;
            }

            std::vector<Graph::Node> neighbours = pg->getNeighbours(qn.second);
            ROS_INFO("%ld",neighbours.size());
            // Graph has already done collision checking and stuff
            for(auto neighbour:neighbours) {
                
                // Check if not already visited
                if(pending.find(neighbour)==pending.end()) {
                    
                    // cost = cost_so_far + cost_cell + heuristic value
                    float new_pot = qn.first + (float)(pg->lookUpCost(neighbour)) + heuristic_weight*(pg->getDistanceBwNodes(neighbour,goalNode));

                    // Check if potential improved
                    if(potarr.find(neighbour)==potarr.end() || new_pot<potarr[neighbour])
                    {
                        potarr[neighbour] = new_pot;
                        std::pair<float,Graph::Node> neighbour_node = std::make_pair(new_pot,neighbour);
                        queue.push(neighbour_node);
                        cameFrom[neighbour] = qn.second;
                    }
                    else
                        ROS_WARN("Potential did not improve!!");

                    ROS_INFO("%d %d %f %f\n",neighbour.x,neighbour.y,new_pot,potarr[neighbour]);

                }
            }

        }

        if(path_found)
        {   
            // Traj map object for graph collision checking
            std::map<double,std::pair<double,double>> traj_map;

            // Retrace path
            std::vector<double> point = pg->toNodeInfo(goalNode);
            trajectory.push_back(point);
            traj_map[point[2]] = std::make_pair(point[0],point[1]);
            ROS_INFO("%ld : %f %f %f",trajectory.size(),point[0],point[1],point[2]);
            Graph::Node parentNode = goalNode;
            do
            {
                parentNode = cameFrom[parentNode];
                point = pg->toNodeInfo(parentNode);
                trajectory.push_back(point);
                traj_map[point[2]] = std::make_pair(point[0],point[1]);
                ROS_INFO("%ld : %f %f %f",trajectory.size(),point[0],point[1],point[2]);
            } while (!(parentNode==startNode));
            publishPlan();
            pg->addTrajCache(traj_map);
        }
        else
            ROS_WARN("Path not found!!");

        return path_found;
    }

    void publishPlan() {

        //create a message for the plan 
        nav_msgs::Path gui_path;
        gui_path.poses.resize(trajectory.size());

        gui_path.header.frame_id = pg->getFrame();
        gui_path.header.stamp = ros::Time::now();

        for(int i=0;i<trajectory.size();i++)
        {
            gui_path.poses[i].pose.position.x = trajectory[i][0];
            gui_path.poses[i].pose.position.y = trajectory[i][1];
        }

        traj_pub_.publish(gui_path);
    }

    void publishEndpoints() {

        visualization_msgs::Marker marker;
        visualization_msgs::MarkerArray marker_arr;

        // common info
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = pg->getFrame();
        marker.type = 2; //sphere
        marker.lifetime = ros::Duration();
        marker.action = 0; // add
        marker.pose.orientation.w = 0;

        marker.scale.x = 1.0f;
        marker.scale.y = 1.0f;
        marker.scale.z = 1.0f;
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


        // Add start point
        marker.id = 0;
        marker.pose.position.x = start[0];
        marker.pose.position.y = start[1];
        marker.pose.position.z = 0.0;
        marker_arr.markers.push_back(marker);

        // Add end point 
        marker.id = 1;
        marker.pose.position.x = goal[0];
        marker.pose.position.y = goal[1];
        marker.pose.position.z = 0.0;
        marker_arr.markers.push_back(marker);
        endpoint_pub_.publish(marker_arr);
    }

    /** @brief : Compare class for the priority queue */
  class CompareClass {
  public:
    bool operator()(std::pair<float, Graph::Node> a,
                    std::pair<float, Graph::Node> b) {
      if (a.first > b.first)
        return true;
      return false;
    }
  };

    std::vector<std::vector<double>> trajectory;
private:
    std::map<Graph::Node,float> potarr;
    std::map<Graph::Node,Graph::Node> cameFrom;
    std::set<Graph::Node> pending;
    Graph* pg;
    Graph::Node goalNode;
    Graph::Node startNode;
    double* goal;
    double* start;
    bool planner_initialised;
    int nx, ny, ns;		/**< size of grid, in pixels */
    float heuristic_weight;
    std::priority_queue<std::pair<float, Graph::Node>,
                      std::vector<std::pair<float, Graph::Node>>,
                      CompareClass> queue;
    ros::Publisher traj_pub_;
    ros::Publisher endpoint_pub_;
};

#endif //ASTAR_HPP