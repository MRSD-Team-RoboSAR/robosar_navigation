// Created by Indraneel on 10/03/22

#include "graph_2d_grid.hpp"
#include <ros/console.h>


Graph2DGrid::Graph2DGrid(): allow_unknown(false) {


    // @ TODO Indraneel This is called only once 
    // but if the map is changing then this need to be called whenever underlying costmap gets updated
    scaleCostMap();
    //propogation_model = {{1,0}, {0,1}, {-1,0}, {0,-1}};
    propogation_model = {{1,0}, {0,1}, {-1,0}, {0,-1}, {1,1},{-1,1},{1,-1},{-1,-1}};
    propogation_speed = 0.2; // m/s
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



bool Graph2DGrid::collisionCheck(Node n, std::string whoami) {

    // TODO
    if(costmap_[toNodeID(n)]>=COST_OBS_ROS)
        return true;

    return false;
}

int Graph2DGrid::toNodeID(Node n) {

    // TODO
    //if(point[0]<origin_x || point[1]<origin_y)
    //    return -1;
    
    //int mx = (int)((point[0] - origin_x) / resolution);
    //int my = (int)((point[1] - origin_y) / resolution);

    if(n.x>= size_width || n.y>=size_height || n.x< 0 || n.y<0)
    {
        ROS_WARN("Invalid id request");
        return -1;
    }

    return n.y*size_width + n.x;
}

Graph2DGrid::Node Graph2DGrid::getNode(double point[2]) {

    if(point[0]<origin_x || point[1]<origin_y || point[0]>origin_x+size_width*resolution || point[1]>origin_y+size_height*resolution)
    {
        ROS_WARN("Requested node off the graph!");
        return Node(-1,-1,0.0);
    }
    
    int mx = (int)((point[0] - origin_x) / resolution);
    int my = (int)((point[1] - origin_y) / resolution);

    return Node(mx,my, 0.0);
}

std::vector<double> Graph2DGrid::toNodeInfo(Node n) {
    // TODO
    std::vector<double> nodeInfo;

    //int my = node_id/size_width;
    //int mx = node_id%size_width;

    nodeInfo.push_back(origin_x+resolution*(n.x+0.5));
    nodeInfo.push_back(origin_y+resolution*(n.y+0.5));
    nodeInfo.push_back(n.t);

    return nodeInfo;
}

int Graph2DGrid::getNumNodes() {
    // TODO
    return size_width*size_height;
}

int Graph2DGrid::getDistanceBwNodes(Node node1, Node node2) {
    // TODO

    // Manhattan distance
    return std::fabs(node1.x-node2.x) + std::fabs(node1.y-node2.y);
}

std::vector<Graph2DGrid::Node> Graph2DGrid::getNeighbours(Node n, std::string whoami) {
    // TODO
    std::vector<Node> neighbours;

    //int my = node_id/size_width;
    //int mx = node_id%size_width;
    for(int i=0;i<propogation_model.size();i++)
    {
        int nx = n.x + propogation_model[i][0];
        int ny = n.y + propogation_model[i][1];

        // Check if within boundary
        if(nx >=0 && nx <size_width && ny>=0 && ny<size_height)
        {   
            Node neighbour(nx,ny,n.t + (resolution/propogation_speed)); 
            // Check if collision free
            if(!collisionCheck(neighbour,whoami)) {
                neighbours.push_back(neighbour);
            }
        }
    }
    return neighbours;
}

int Graph2DGrid::lookUpCost(Node n) {

    // TODO
    return (int)costmap_[toNodeID(n)];
}

std::string Graph2DGrid::getFrame(void) {
    return map_frame_;
}

void Graph2DGrid::addTrajCache(std::map<double,std::pair<double,double>> trajectory) {
    // for forward compatibility
}
    
    
void Graph2DGrid::addGoalCache(std::vector<double*> goal_positions, std::vector<std::string> planner_names) {
    // for forward compatibility
}

void Graph2DGrid::clearTrajCache(void) {
    // for forward compatibility
}