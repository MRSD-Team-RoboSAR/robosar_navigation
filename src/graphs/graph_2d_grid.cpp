// Created by Indraneel on 10/03/22

#include "graph_2d_grid.hpp"
#include <ros/console.h>


Graph2DGrid::Graph2DGrid(): allow_unknown(false) {


    // @ TODO Indraneel This is called only once 
    // but if the map is changing then this need to be called whenever underlying costmap gets updated
    scaleCostMap();
}

Graph2DGrid::~Graph2DGrid() {

    propogation_model = {{1,0}, {0,1}, {-1,0}, {0,-1}};
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
    if(costmap_[node_id]>=COST_OBS_ROS)
        return true;

    return false;
}

int Graph2DGrid::toNodeID(double *point) {

    // TODO
    if(point[0]<origin_x || point[1]<origin_y)
        return -1;
    
    int mx = (int)((point[0] - origin_x) / resolution);
    int my = (int)((point[1] - origin_y) / resolution);

    if(mx>= size_width || my>=size_height)
        return -1;

    return my*size_width + mx;
}

std::vector<double> Graph2DGrid::toNodeInfo(int node_id) {
    // TODO
    std::vector<double> nodeInfo;

    int my = node_id/size_width;
    int mx = node_id%size_width;

    nodeInfo.push_back(origin_x+resolution*(mx+0.5));
    nodeInfo.push_back(origin_y+resolution*(my+0.5));

    return nodeInfo;
}

int Graph2DGrid::getNumNodes() {
    // TODO
    return size_width*size_height;
}

float Graph2DGrid::getDistanceBwNodes(int node1, int node2) {
    // TODO
    std::vector<double> point1 = toNodeInfo(node1);
    std::vector<double> point2 = toNodeInfo(node2);

    // Manhattan distance
    return std::fabs(point2[0]-point1[0]) + std::fabs(point2[1]-point1[1]);
}

std::vector<int> Graph2DGrid::getNeighbours(int node_id) {
    // TODO
    std::vector<int> neighbours;

    int my = node_id/size_width;
    int mx = node_id%size_width;

    for(int i=0;i<propogation_model.size();i++)
    {
        int nx = mx + propogation_model[i][0];
        int ny = my + propogation_model[i][1];

        // Check if within boundary
        if(nx >=0 && nx <size_width && ny>=0 && ny<size_height)
        {
            // Check if collision free
            if(!collisionCheck(ny*size_width + nx)) {
                neighbours.push_back(ny*size_width + nx);
            }
        }
    }
    return neighbours;
}

int Graph2DGrid::lookUpCost(int node) {

    // TODO
    return (int)costmap_[node];
}

std::string Graph2DGrid::getFrame(void) {
    return map_frame_;
}