// Created by Indraneel on 17/03/21

#ifndef ASTAR_HPP
#define ASTAR_HPP

#include "graph_2d_grid.hpp"
#include <queue>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

// Underlying graph datatype can be changed here
#undef Graph
#define Graph Graph2DGrid

#define POT_HIGH 1.0e10		// unassigned cell potential
class AStar {

public:
    AStar(Graph* graph,double g[2], double s[2]) : planner_initialised(false),pg(graph), heuristic_weight(10.0f), nh_("") {

        goal[0] = g[0];
        goal[1] = g[1];

        start[0] = s[0];
        start[1] = s[1];

        goalID = graph->toNodeID(goal);
        startID = graph->toNodeID(start);

        // Check if goal and start are collision free
        if(graph->collisionCheck(goalID))
        {
            ROS_WARN("[AStar] Goal in collision! %f,%f\n", goal[0], goal[1]);
            return;
        }
        else if(graph->collisionCheck(startID))
        {
           ROS_WARN("[AStar] Start in collision! %f,%f\n", start[0], start[1]); 
           return;
        }

        ROS_INFO("[AStar] Setting goal to %f,%f\n", goal[0], goal[1]);
        ROS_INFO("[AStar] Setting start to %f,%f\n",start[0], start[1]);

        // Initialise potential array
        int ns = graph->getNumNodes();
        potarr = new float[ns];	// navigation potential array
        for (int i=0; i<ns; i++)
        {
            potarr[i] = POT_HIGH;
        }

        pending = new bool[ns];
        cameFrom = new int[ns];
        memset(pending, 0, ns*sizeof(bool));
        memset(cameFrom, -1, ns*sizeof(int));


        // Add start to the priority queue
        std::pair<float,int> start_node = std::make_pair(0.0f,startID);
        queue.push(start_node);
        cameFrom[startID] = -1;

        // traj publisher for rviz
        traj_pub_ = nh_.advertise<nav_msgs::Path>("plan", 1);
        // start and goal publisher for rviz
        endpoint_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("endpoints", 1);
        publishEndpoints();

        planner_initialised = true;
    }

    ~AStar(){
        if(potarr)
            delete[] potarr;
        if(pending)
            delete[] pending;
        if(cameFrom)
            delete[] cameFrom;
    }

    bool run_planner(int cycles) {

        int cycle = 0;
        bool path_found = false;

        publishEndpoints();

        if(!planner_initialised)
        return false;

        // do main cycle
        for (; cycle < cycles && !queue.empty(); cycle++) // go for this many cycles or if queue is over
        {
            ROS_INFO("On cycle %d / %d with queue size %ld \n",cycle,cycles,queue.size());
            std::pair<float,int> node = queue.top();
            queue.pop();
            // Mark as explored
            if(pending[node.second])
                ROS_WARN("Already explored!!");
            pending[node.second] = 1;

            // Check if reached goal
            if(node.second == goalID)
            {
                path_found = true;
                break;
            }

            std::vector<int> neighbours = pg->getNeighbours(node.second);
            ROS_INFO("%ld",neighbours.size());
            // Graph has already done collision checking and stuff
            for(auto neighbour:neighbours) {
                
                // Check if not already visited
                if(!pending[neighbour]) {
                    
                    // cost = cost_so_far + cost_cell + heuristic value
                    float new_pot = node.first + (float)(pg->lookUpCost(neighbour)) + heuristic_weight*(pg->getDistanceBwNodes(neighbour,goalID));

                    // Check if potential improved
                    if(new_pot<potarr[neighbour])
                    {
                        potarr[neighbour] = new_pot;
                        std::pair<float,int> neighbour_node = std::make_pair(new_pot,neighbour);
                        queue.push(neighbour_node);
                        cameFrom[neighbour] = node.second;
                    }
                    else
                        ROS_WARN("Potential did not improve!!");

                    ROS_INFO("%d %f %f\n",neighbour,new_pot,potarr[neighbour]);
                }
            }

        }

        if(path_found)
        {
            // Retrace path
            std::vector<double> point = pg->toNodeInfo(goalID);
            trajectory.push_back(point);
            int parentID = cameFrom[goalID];

            while(parentID!=-1)
            {
                point = pg->toNodeInfo(parentID);
                trajectory.push_back(point);
                parentID = cameFrom[parentID];
            }
            publishPlan();
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
    bool operator()(std::pair<float, int> a,
                    std::pair<float, int> b) {
      if (a.first > b.first)
        return true;
      return false;
    }
  };

    std::vector<std::vector<double>> trajectory;
private:
    int *cameFrom;  
    float   *potarr;		/**< potential array, navigation function potential */
    bool    *pending;		/**< pending cells during propagation */
    Graph* pg;
    int goalID;
    int startID;
    double goal[2];
    double start[2];
    bool planner_initialised;
    int nx, ny, ns;		/**< size of grid, in pixels */
    float heuristic_weight;
    std::priority_queue<std::pair<float, int>,
                      std::vector<std::pair<float, int>>,
                      CompareClass> queue;
    ros::Publisher traj_pub_;
    ros::Publisher endpoint_pub_;
    ros::NodeHandle nh_;
};

#endif //ASTAR_HPP