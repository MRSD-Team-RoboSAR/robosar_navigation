// Created by Indraneel on 17/03/21

#ifndef ASTAR_HPP
#define ASTAR_HPP

#include "graph_2d_grid.hpp"

// Underlying graph datatype can be changed here
#undef Graph
#define Graph Graph2DGrid

#define POT_HIGH 1.0e10		// unassigned cell potential

class AStar {

public:
    AStar(Graph* graph,int g[2], int s[2]) : planner_initialised(false),pg(graph) {

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
            gradx[i] = grady[i] = 0.0;
        }

        pending = new bool[ns];
        memset(pending, 0, ns*sizeof(bool));
        gradx = new float[ns];
        grady = new float[ns];
        planner_initialised = true;
    }

    ~AStar(){
        if(potarr)
            delete[] potarr;
        if(pending)
            delete[] pending;
        if(gradx)
            delete[] gradx;
        if(grady)
            delete[] grady;
    }

    bool run_planner(int cycles) {

        if(!planner_initialised)
        return false;

        int cycle = 0;

        // do main cycle
      for (; cycle < cycles; cycle++) // go for this many cycles, unless interrupted
      {

      }


    }

private:
    float *gradx, *grady;		/**< gradient arrays, size of potential array */
    float   *potarr;		/**< potential array, navigation function potential */
    bool    *pending;		/**< pending cells during propagation */
    Graph* pg;
    int goalID;
    int startID;
    int goal[2];
    int start[2];
    bool planner_initialised;
    int nx, ny, ns;		/**< size of grid, in pixels */
};

#endif //ASTAR_HPP