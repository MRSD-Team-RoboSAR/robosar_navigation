// Created by Indraneel on 27/03/22

#ifndef GRAPH_3D_GRID
#define GRAPH_3D_GRID

#include <ros/ros.h>
#include "costmap_2d.hpp"
#include <vector>

// cost defs
#define COST_UNKNOWN_ROS 255		// 255 is unknown cost
#define COST_OBS 254		// 254 for forbidden regions
#define COST_OBS_ROS 253	// ROS values of 253 are obstacles

// graph cost values are set to
// COST_NEUTRAL + COST_FACTOR * costmap_cost_value.

#define COST_NEUTRAL 5		// Set this to "open space" value
#define COST_FACTOR 0.8		// Used for translating costs in NavFn::setCostmap()

#define COLLISION_THRESHOLD 0.14 // Twice the radius of khepera robot

class Graph3DGrid : Costmap2D {

public:
    Graph3DGrid();
    ~Graph3DGrid();

    class Node {
        public:
            Node(int x, int y, double t) : x(x),y(y),t(t) {}
            Node(void)  {
                // Uninitialised
                x = -1;
                y = -1;
                t = 0.0;
            }

            bool operator==(const Node &n) const
            {
                return ((n.x==x) && (n.y==y));
            }

            bool operator<(const Node &n) const
            {
                if(x!=n.x)
                    return x<n.x;
                else if(y!=n.y)
                    return y<n.y;
                else 
                    return t<n.t;
            }
            
            int x; 
            int y;
            double t;
    };

    bool collisionCheck(Node n);
    int toNodeID(Node n);
    std::vector<double> toNodeInfo(Node n);
    int getNumNodes();
    Node getNode(double point[2]); 
    float getDistanceBwNodes(Node node1, Node node2);
    std::vector<Node> getNeighbours(Node node);
    int lookUpCost(Node node);
    std::string getFrame(void);
    void addTrajCache(std::map<double,std::pair<double,double>> trajectory);

private:
    void scaleCostMap();
    bool allow_unknown;
    std::vector<std::vector<int>> propogation_model;
    double propogation_speed;
    // Cached trajectories for collision checking
    std::vector<std::map<double,std::pair<double,double>>> traj_cache;

};

#endif