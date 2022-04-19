// Created by Indraneel on 27/03/22

#ifndef GRAPH_MOTION_PRIMITIVES
#define GRAPH_MOTION_PRIMITIVES

#include <ros/ros.h>
#include "costmap_2d.hpp"
#include <vector>
#include "mp_diff_drive.hpp"

// cost defs
#define COST_UNKNOWN_ROS 255		// 255 is unknown cost
#define COST_OBS 254		// 254 for forbidden regions
#define COST_OBS_ROS 253	// ROS values of 253 are obstacles

// graph cost values are set to
// COST_NEUTRAL + COST_FACTOR * costmap_cost_value.

#define COST_NEUTRAL 5		// Set this to "open space" value
#define COST_FACTOR 0.8		// Used for translating costs in NavFn::setCostmap()

#define COLLISION_THRESHOLD 0.4 // Twice the radius of khepera robot

#define EPSILON 0.001

class GraphMotionPrimitives : Costmap2D {

public:
    GraphMotionPrimitives();
    ~GraphMotionPrimitives();

    class Node {
        public:
            Node(double x, double y, double yaw, double t) : x(x),y(y),t(t),yaw(yaw), isStart(false) {}
            Node(void)  {
                // Uninitialised
                x = 0.0;
                y = 0.0;
                t = 0.0;
                yaw = 0.0;
            }

            bool AreEqual(double a, double b) const
            {
                return fabs(a - b) < EPSILON;
            }

            bool AreSame(double check_x, double check_y) 
            {
                return AreEqual(check_x,x) && AreEqual(check_y,y);
            }

            bool operator==(const Node &n) const
            {
                return (AreEqual(n.x,x) && AreEqual(n.y,y) && AreEqual(n.t,t) && AreEqual(n.yaw,yaw));
            }

            // Hash function used by boost data structures
            struct hashFunction {
                std::size_t operator()(const Node&n) const {
                    std::size_t seed = 0;
                    boost::hash_combine(seed,n.x);
                    boost::hash_combine(seed,n.y);
                    boost::hash_combine(seed,n.t);
                    boost::hash_combine(seed,n.yaw);

                    return seed;
                };
            };
            
            double x; 
            double y;
            double yaw;
            double t;
            bool isStart;
    };

    bool collisionCheck(Node n1, Node n2);
    bool collisionCheck(Node n);
    int toNodeID(Node n);
    std::vector<double> toNodeInfo(Node n);
    int getNumNodes();
    Node getNode(double point[2]); 
    int getDistanceBwNodes(Node node1, Node node2);
    std::vector<Node> getNeighbours(Node node);
    int lookUpCost(Node node);
    std::string getFrame(void);
    void addTrajCache(std::map<double,std::pair<double,double>> trajectory);
    void clearTrajCache(void);

private:
    void scaleCostMap();
    bool allow_unknown;
    std::vector<std::vector<int>> propogation_model;
    double propogation_speed;
    // Cached trajectories for collision checking
    std::vector<std::map<double,std::pair<double,double>>> traj_cache;
    MPDiffDrive motion_primitives;

};

#endif