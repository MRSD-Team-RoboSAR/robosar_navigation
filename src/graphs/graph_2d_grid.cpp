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

            // Border cells are obstacles
            if(i<size_width || i>= (size_height-1)*size_width || i%size_width==0 || (i+1)%size_width==0) 
                continue;

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

float Graph2DGrid::getDistanceBwNodes(int node1, int node2) {
    // TODO
    return 0;
}

std::vector<int> Graph2DGrid::getNeighbours(int node_id) {
    // TODO
    std::vector<int> neighbours;
    return neighbours;
}

int Graph2DGrid::lookUpCost(int node) {

    // TODO
    return (int)costmap_[node];
}
