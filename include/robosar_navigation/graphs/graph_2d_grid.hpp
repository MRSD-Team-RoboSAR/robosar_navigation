// Created by Indraneel on 10/03/22

#ifndef GRAPH_2D_GRID
#define GRAPH_2D_GRID

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

class Graph2DGrid : Costmap2D {

public:
    Graph2DGrid();
    ~Graph2DGrid();

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

            // Hash function used by boost data structures
            struct hashFunction {
                std::size_t operator()(const Node&n) const {
                    std::size_t seed = 0;
                    boost::hash_combine(seed,n.x);
                    boost::hash_combine(seed,n.y);

                    return seed;
                };
            };
            
            int x; 
            int y;
            double t;
            bool isStart;
    };

    bool collisionCheck(Node n, std::string whoami);
    int toNodeID(Node n);
    std::vector<double> toNodeInfo(Node n);
    int getNumNodes();
    Node getNode(double point[2]); 
    int getDistanceBwNodes(Node node1, Node node2);
    std::vector<Node> getNeighbours(Node node,std::string whoami);
    int lookUpCost(Node node);
    std::string getFrame(void);
    void addTrajCache(std::map<double,std::pair<double,double>> trajectory);
    void addGoalCache(std::vector<double*> goal_positions, std::vector<std::string> planner_names);
    void clearTrajCache(void);

private:
    void scaleCostMap();
    bool allow_unknown;
    std::vector<std::vector<int>> propogation_model;

};

#endif