// Created by Indraneel and Charvi on 5/04/21

#ifndef MISSION_EXECUTIVE_HPP
#define MISSION_EXECUTIVE_HPP

#include "graph_3d_grid.hpp"
#include "multi_astar.hpp"
#include <ros/console.h>
#include <std_msgs/Bool.h>

class MissionExecutive {

public:
    MissionExecutive() : nh_("") {

        ROS_INFO("Starting a new RoboSAR Nav Mission! Get ready for a show!");

        status_subscriber_ = nh_.subscribe("cmd_vel", 1, &MissionExecutive::statusCallback, this);
        // Get latest fleet info from agent bringup

    }

    ~MissionExecutive() {

    }
        

private:

    void statusCallback(const std_msgs::Bool &status_msg) {

    }
    Graph3DGrid gridmap;
    ros::Subscriber status_subscriber_;
    ros::NodeHandle nh_;
};

#endif