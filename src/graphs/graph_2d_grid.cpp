// Created by Indraneel on 10/03/22

#include "graph_2d_grid.hpp"
#include <ros/console.h>


Graph2DGrid::Graph2DGrid(): allow_unknown(false) {


    // @ TODO Indraneel This is called only once 
    // but if the map is changing then this need to be called whenever underlying costmap gets updated
    scaleCostMap();
}

Graph2DGrid::~Graph2DGrid() {


}

// Scales cost values from the range of 0-252 to COST_NEURAL-COST_OBS_ROS 
void Graph2DGrid::scaleCostMap() 
{
    for (unsigned int i = 0; i < size_width*size_height; ++i)
    {
            unsigned char value = costmap_[i];
            costmap_[i] = COST_OBS;

            if(value < COST_OBS_ROS)
            {
                int new_value = COST_NEUTRAL+COST_FACTOR*value;
                if(new_value >= COST_OBS)
                    new_value = COST_OBS-1;
                costmap_[i] = new_value;
            }
            else if(value == COST_UNKNOWN_ROS && allow_unknown)
            {
                costmap_[i] = COST_OBS-1;
            }
    }

}

bool Graph2DGrid::collisionCheck(int node_id) {

    // TODO
    return false;
}

int Graph2DGrid::toNodeID(int *point) {

    // TODO
    return 0;
}

int Graph2DGrid::getNumNodes() {
    // TODO
    return 0;
}