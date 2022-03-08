// Created by Indraneel on 7/03/22

#ifndef COSTMAP_2D_HPP
#define COSTMAP_2D_HPP

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <mutex>
#include "cost_inflator.hpp"

class Costmap2D
{
public:
    Costmap2D();
    ~Costmap2D();


private:
    void incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map);
    unsigned char interpretValue(unsigned char value);
    void onNewSubscription(const ros::SingleSubscriberPublisher& pub);
    void prepareGrid();
    void create_translation_table();
    unsigned int cellDistance(double world_dist);

    unsigned int size_width;
    unsigned int size_height;
    double origin_x;
    double origin_y;
    double resolution;
    std::string map_frame_;  /// @brief frame that map is located in

    double inscribed_radius;
    double inflation_radius;
    double inflation_cost_scaling_factor;
    CostInflator inflator_;
    static char* cost_translation_table_;  ///< Translate from 0-255 values in costmap to -1 to 100 values in message.
    std::mutex mtx;
    bool track_unknown_space;
    bool trinary_costmap;
    unsigned char* costmap_;
    bool map_received_;
    bool has_updated_data_;
    nav_msgs::OccupancyGrid grid_;
    ros::NodeHandle nh_;
    // TODO update_sub
    ros::Subscriber map_sub_, map_update_sub_;
    ros::Publisher costmap_pub_;
};


#endif