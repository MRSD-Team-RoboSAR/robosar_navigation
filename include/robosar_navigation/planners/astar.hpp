// Created by Indraneel on 17/03/21

#ifndef ASTAR_HPP
#define ASTAR_HPP

#include "graph_2d_grid.hpp"
#include "graph_3d_grid.hpp"
#include <queue>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>
// Underlying graph datatype can be changed here
#undef Graph
#define Graph Graph2DGrid

#define POT_HIGH 1.0e10		// unassigned cell potential
class AStar {

public:
    AStar(std::string ns_, Graph* graph,double* g, double* s, ros::NodeHandle *nh) : planner_initialised(false),pg(graph), heuristic_weight(10.0f),
                                                    goalNode(-1,-1,0.0), startNode(-1,-1, 0.0), planner_name(ns_) {

        goal = g;
        start = s;

        goalNode = graph->getNode(goal);

        startNode = graph->getNode(start);
        startNode.isStart = true;

        // Check if goal and start are collision free
        if(graph->collisionCheck(goalNode,planner_name))
        {
            ROS_WARN("[AStar] Goal in collision! %f,%f\n", goal[0], goal[1]);
            goalNode = graph->getClosestFreeNode(goalNode, planner_name);
            std::vector<double> goalInfo = graph->toNodeInfo(goalNode);
            goal[0] = goalInfo[0];
            goal[1] = goalInfo[1];
        }
        
        if(graph->collisionCheck(startNode,planner_name))
        {
            ROS_WARN("[AStar] Start in collision! %f,%f\n", start[0], start[1]); 
            startNode = graph->getClosestFreeNode(startNode, planner_name);
            std::vector<double> startInfo = graph->toNodeInfo(startNode);
            start[0] = startInfo[0];
            start[1] = startInfo[1];
        }

        ROS_INFO("[AStar] Setting goal to %f,%f\n", goal[0], goal[1]);
        ROS_INFO("[AStar] Setting start to %f,%f\n",start[0], start[1]);

        // Add start to the priority queue
        std::pair<float,Graph::Node> sn = std::make_pair(0.0f,startNode);
        potarr[startNode] = 0.0f;
        queue.push(sn);

        // traj publisher for rviz
        traj_pub_ = nh->advertise<nav_msgs::Path>(ns_+"/plan", 1);
        // start and goal publisher for rviz
        endpoint_pub_ = nh->advertise<visualization_msgs::MarkerArray>(ns_+"/endpoints", 1);
        // expansion visualisation rviz
        expansion_pub_ = nh->advertise<visualization_msgs::Marker>(ns_+"/expansion", 1);
        
        initialiseExpansionMarker();
        planner_initialised = true;

        // default parent for goal is start
        cameFrom[goalNode] = startNode;

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
            if(pg->getDistanceBwNodes(qn.second,goalNode)==0) {
                goalNode = qn.second;
                path_found = true;
                break;
            }

            std::vector<Graph::Node> neighbours = pg->getNeighbours(qn.second,planner_name);
            ROS_DEBUG("%d %d",qn.second.x,qn.second.y);
            // Graph has already done collision checking and stuff
            for(auto neighbour:neighbours) {
                
                // Check if not already visited
                if(pending.find(neighbour)==pending.end()) {
                    
                    float new_pot;
                    if(neighbour.x !=qn.second.x || neighbour.y!=qn.second.y)
                        new_pot = potarr[qn.second] + (float)(pg->lookUpCost(neighbour));
                    else
                        new_pot =  potarr[qn.second]  + (float)(pg->lookUpCost(neighbour))/5.0f;
    
                    ROS_DEBUG("Plan : %f %f %f %d", qn.first,
                                                    potarr[qn.second],
                                                    heuristic_weight*((float)(pg->getDistanceBwNodes(neighbour,goalNode))),
                                                    pg->getDistanceBwNodes(neighbour,goalNode));

                    // Check if potential improved
                    if(potarr.find(neighbour)==potarr.end() || new_pot<potarr[neighbour])
                    {
                        potarr[neighbour] = new_pot;
                        // cost = cost_so_far + cost_cell + heuristic value
                        float node_cost = new_pot + heuristic_weight*((float)(pg->getDistanceBwNodes(neighbour,goalNode)));
                        std::pair<float,Graph::Node> neighbour_node = std::make_pair(node_cost,neighbour);
                        queue.push(neighbour_node);
                        //publishExpansion(NODE_FRONTIER,neighbour);
                        cameFrom[neighbour] = qn.second;
                    }
                    else
                        ROS_WARN("Potential did not improve!!");

                    //ROS_INFO("%d %d %f %f\n",neighbour.x,neighbour.y,new_pot,potarr[neighbour]);

                }
            }

        }

        if(path_found)
        {   
            // Traj map object for graph collision checking
            std::map<double,std::pair<double,double>> traj_map;

            // Retrace path
            std::vector<double> point = pg->toNodeInfo(goalNode);
            trajectory.insert(trajectory.begin(),point);
            traj_map[point[2]] = std::make_pair(point[0],point[1]);
            ROS_INFO("%ld : %f %f %f",trajectory.size(),point[0],point[1],point[2]);
            Graph::Node parentNode = goalNode;
            do
            {
                parentNode = cameFrom[parentNode];
                point = pg->toNodeInfo(parentNode);
                trajectory.insert(trajectory.begin(),point);
                traj_map[point[2]] = std::make_pair(point[0],point[1]);
                ROS_INFO("%ld : %f %f %f",trajectory.size(),point[0],point[1],point[2]);
            } while (parentNode.t>0.001);

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

    typedef enum expNodeType {
        
        NODE_FRONTIER,
        NODE_CLOSED

    } expNodeType_e;

    void initialiseExpansionMarker() {

        // common info
        exp_marker.header.stamp = ros::Time::now();
        exp_marker.header.frame_id = pg->getFrame();
        exp_marker.type = 6; //cube list
        exp_marker.lifetime = ros::Duration();
        exp_marker.action = 0; // add
        
        exp_marker.pose.orientation.x = 0;
        exp_marker.pose.orientation.y = 0;
        exp_marker.pose.orientation.z = 0;
        exp_marker.pose.orientation.w = 1;
        exp_marker.scale.x = 0.1f;
        exp_marker.scale.y = 0.1f;
        exp_marker.scale.z = 0.1f;
        exp_marker.id = 1;
        exp_marker.ns = "astar_expand";
        exp_marker.points.clear();
        exp_marker.colors.clear();
        
    }


     void publishExpansion(expNodeType_e node_type, Graph::Node n) {
        
        static int marker_id = 0;

        std_msgs::ColorRGBA color = std_msgs::ColorRGBA();
        if(node_type == NODE_CLOSED) {
            color.r = 0.0f;
            color.g = 254.0f;
            color.b = 0.0f;
        }
        else if(node_type == NODE_FRONTIER) {
            color.r = 0.0f;
            color.g = 0.0f;
            color.b = 0.0f;
        }
        color.r = 0.0f;
        color.g = 250.0f;
        color.b = 0.0f;
        color.a = 1.0;
        exp_marker.colors.push_back(color);

         // Add node
        geometry_msgs::Point new_cube = geometry_msgs::Point(); 
        std::vector<double> nodeInfo = pg->toNodeInfo(n);
        new_cube.x = nodeInfo[0];
        new_cube.y = nodeInfo[1];
        new_cube.z = 0.0;
        exp_marker.points.push_back(new_cube);
        expansion_pub_.publish(exp_marker);
        ros::Duration(0.01).sleep();
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
    std::unordered_map<Graph::Node,float,Graph::Node::hashFunction> potarr;
    std::unordered_map<Graph::Node,Graph::Node,Graph::Node::hashFunction> cameFrom;
    std::unordered_set<Graph::Node,Graph::Node::hashFunction> pending;
    Graph* pg;
    Graph::Node goalNode;
    Graph::Node startNode;
    double* goal;
    double* start;
    std::string planner_name;
    bool planner_initialised;
    int nx, ny, ns;		/**< size of grid, in pixels */
    float heuristic_weight;
    std::priority_queue<std::pair<float, Graph::Node>,
                      std::vector<std::pair<float, Graph::Node>>,
                      CompareClass> queue;
    visualization_msgs::Marker exp_marker;
    ros::Publisher traj_pub_;
    ros::Publisher endpoint_pub_;
    ros::Publisher expansion_pub_;
};

#endif //ASTAR_HPP