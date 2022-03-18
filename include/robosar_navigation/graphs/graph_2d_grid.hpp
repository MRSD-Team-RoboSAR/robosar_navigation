// Created by Indraneel on 10/03/22

#ifndef GRAPH_2D_GRID
#define GRAPH_2D_GRID

#include <ros/ros.h>
#include "costmap_2d.hpp"

// cost defs
#define COST_UNKNOWN_ROS 255		// 255 is unknown cost
#define COST_OBS 254		// 254 for forbidden regions
#define COST_OBS_ROS 253	// ROS values of 253 are obstacles

// graph cost values are set to
// COST_NEUTRAL + COST_FACTOR * costmap_cost_value.

#define COST_NEUTRAL 50		// Set this to "open space" value
#define COST_FACTOR 0.8		// Used for translating costs in NavFn::setCostmap()

class Graph2DGrid : Costmap2D {

public:
    Graph2DGrid();
    ~Graph2DGrid();

    bool collisionCheck(int node_id);
    int toNodeID(int *point);

private:
    void scaleCostMap();
    bool allow_unknown;

};

#endif