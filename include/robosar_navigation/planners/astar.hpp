// Created by Indraneel on 17/03/21

#ifndef ASTAR_HPP
#define ASTAR_HPP

#include "graph_2d_grid.hpp"
#include <queue>

// Underlying graph datatype can be changed here
#undef Graph
#define Graph Graph2DGrid

#define POT_HIGH 1.0e10		// unassigned cell potential
class AStar {

public:
    AStar(Graph* graph,int g[2], int s[2]) : planner_initialised(false),pg(graph), heuristic_weight(10.0f) {

        goal[0] = g[0];
        goal[1] = g[1];

        start[0] = s[0];
        start[1] = s[1];

        goalID = graph->toNodeID(goal);
        startID = graph->toNodeID(start);

        // Check if goal and start are collision free
        if(graph->collisionCheck(goalID))
        {
            ROS_WARN("[AStar] Goal in collision! %d,%d\n", goal[0], goal[1]);
            return;
        }
        else if(graph->collisionCheck(startID))
        {
           ROS_WARN("[AStar] Start in collision! %d,%d\n", start[0], start[1]); 
           return;
        }

        ROS_INFO("[AStar] Setting goal to %d,%d\n", goal[0], goal[1]);
        ROS_INFO("[AStar] Setting start to %d,%d\n",start[0], start[1]);

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

        if(!planner_initialised)
        return false;

        // do main cycle
        for (; cycle < cycles && !queue.empty(); cycle++) // go for this many cycles or if queue is over
        {
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
        }

        return path_found;
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
    int goal[2];
    int start[2];
    bool planner_initialised;
    int nx, ny, ns;		/**< size of grid, in pixels */
    float heuristic_weight;
    std::priority_queue<std::pair<float, int>,
                      std::vector<std::pair<float, int>>,
                      CompareClass> queue;
};

#endif //ASTAR_HPP