// Created by Indraneel on 27/03/22

#include "graph_motion_primitives.hpp"
#include <ros/console.h>


GraphMotionPrimitives::GraphMotionPrimitives(): allow_unknown(false) {


    // @ TODO Indraneel This is called only once 
    // but if the map is changing then this need to be called whenever underlying costmap gets updated
    scaleCostMap();
    //propogation_model = {{1,0}, {0,1}, {-1,0}, {0,-1},{0,0}};
    //propogation_model = {{1,0}, {0,1}, {-1,0}, {0,-1}, {1,1},{-1,1},{1,-1},{-1,-1},{0,0}};
    //propogation_speed = 0.2; // m/s
}

GraphMotionPrimitives::~GraphMotionPrimitives() {

}

// Scales cost values from the range of 0-252 to COST_NEURAL-COST_OBS_ROS 
void GraphMotionPrimitives::scaleCostMap() 
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

bool GraphMotionPrimitives::collisionCheck(Node n) {

    if(costmap_[toNodeID(n)]>=COST_OBS_ROS)
            return true;

    return false;
}

bool GraphMotionPrimitives::collisionCheck(Node n1, Node n2) {

    // TODO
    
    if(n1 == n2) {
        // Collision check only this node
        if(collisionCheck(n1))
            return true;
    }
    else
    {
        // Interpolate and collision check 
        Node temp(n1.x,n1.y,n1.yaw,n1.t);
        double step = hypot(n2.x-n1.x,n2.y-n1.y)/10.0;
        double angle = atan2(n2.y - n1.y,
                                        n2.x - n1.x);
        while(!(temp.AreSame(n2.x,n2.y))) {
            
            temp.x +=  step*cos(angle);
            temp.y += step*sin(angle);
            if(collisionCheck(temp))
                return true;
        }

    }
    /*
    // if the robot exists it exists
    if(n.isStart)
        return false;
    
    std::vector<double> nodeMapFrame = toNodeInfo(n);
    for(auto traj_map:traj_cache) {

        // Lookup each trajectory using time lookup
        if(traj_map.find(n.t)!=traj_map.end())
        {
            std::pair<double,double> closest_point = traj_map[n.t];
            if(hypot(closest_point.first-nodeMapFrame[0],closest_point.second-nodeMapFrame[1])<COLLISION_THRESHOLD)
                return true;
        }
        // Check with last point of the traj
        // because robots donot magically disappear
        else if(traj_map.size()>0)
        {
            std::pair<double,double> closest_point = traj_map.rbegin()->second;
            //ROS_INFO("Collision checking with %f %f",closest_point.first,closest_point.second);
            if(hypot(closest_point.first-nodeMapFrame[0],closest_point.second-nodeMapFrame[1])<COLLISION_THRESHOLD)
                return true;
        }
    }
    */

    return false;
}

int GraphMotionPrimitives::toNodeID(Node n) {

    // TODO
    int mx = (int)((n.x - origin_x) / resolution);
    int my = (int)((n.y - origin_y) / resolution);

    if(mx>= size_width || my>=size_height || mx< 0 || my<0)
    {
        ROS_WARN("Invalid id request");
        return -1;
    }

    return my*size_width + mx;
}

GraphMotionPrimitives::Node GraphMotionPrimitives::getNode(double point[4]) {

    if(point[0]<origin_x || point[1]<origin_y || point[0]>origin_x+size_width*resolution || point[1]>origin_y+size_height*resolution)
    {
        ROS_WARN("Requested node off the graph!");
        return Node(0.0,0.0, 0.0,0.0);
    }
    
    //int mx = (int)((point[0] - origin_x) / resolution);
    //int my = (int)((point[1] - origin_y) / resolution);

    return Node(point[0],point[1],point[2],point[3]);
}

std::vector<double> GraphMotionPrimitives::toNodeInfo(Node n) {
    // TODO
    std::vector<double> nodeInfo;

    //int my = node_id/size_width;
    //int mx = node_id%size_width;

    nodeInfo.push_back(n.x);
    nodeInfo.push_back(n.y);
    nodeInfo.push_back(n.yaw); //radians
    nodeInfo.push_back(n.t);

    return nodeInfo;
}

int GraphMotionPrimitives::getNumNodes() {
    // TODO
    return size_width*size_height;
}

// This function should be resolution invariant
int GraphMotionPrimitives::getDistanceBwNodes(Node node1, Node node2) {
    // TODO
    int mx = (int)((node1.x - origin_x) / resolution);
    int my = (int)((node1.y - origin_y) / resolution);

    int nx = (int)((node2.x - origin_x) / resolution);
    int ny = (int)((node2.y - origin_y) / resolution);
    
    // Manhattan distance
    return std::abs(mx-nx) + std::abs(my-ny);

    // Euclidean distance
    //return hypot(point2[0]-point1[0],point2[1]-point1[1]);
}

std::vector<GraphMotionPrimitives::Node> GraphMotionPrimitives::getNeighbours(Node n) {
    // TODO

    std::vector<double> nodeMapFrame = {n.x,n.y,n.yaw,n.t};

    std::vector<std::vector<double>> neighboursMapVec = motion_primitives.propogate_model(nodeMapFrame);


    std::vector<Node> neighbours;
    //int my = node_id/size_width;
    //int mx = node_id%size_width;
    for(int i=0;i<neighboursMapVec.size();i++)
    {

        std::vector<double> point = neighboursMapVec[i];
        // Check if within boundary
        if(point[0]>origin_x && point[1]>origin_y && point[0]<origin_x+size_width*resolution && point[1]<origin_y+size_height*resolution)
        {   
            Node neighbour(point[0],point[1],point[2],point[3]); 

            // Check if collision free
            if(!collisionCheck(n,neighbour)) {
                neighbours.push_back(neighbour);
            }
        }
    }
    return neighbours;
}

int GraphMotionPrimitives::lookUpCost(Node n) {

    // TODO
    return (int)costmap_[toNodeID(n)];
}

std::string GraphMotionPrimitives::getFrame(void) {
    return map_frame_;
}

void GraphMotionPrimitives::addTrajCache(std::map<double,std::pair<double,double>> trajectory) {

    traj_cache.push_back(trajectory);
}

void GraphMotionPrimitives::clearTrajCache(void) {

    traj_cache.clear();
}