// Created by Indraneel on 7/03/21

#include "costmap_2d.hpp"
#include <ros/console.h>

Costmap2D::Costmap2D() : nh_(""), size_width(0), size_height(0), track_unknown_space(false), trinary_costmap(true),costmap_(NULL) {

    ROS_INFO("Requesting the map...");
    map_sub_ = nh_.subscribe("map", 1, &Costmap2D::incomingMap, this);
    map_received_ = false;
    has_updated_data_ = false;

    ros::Rate r(10);
    while (!map_received_ && nh_.ok())
    {
      ros::spinOnce();
      r.sleep();
    }

    ROS_INFO("Map Received!");

}

Costmap2D::~Costmap2D() {
    
    // Do we need a lock here?
    delete[] costmap_;
    costmap_ = NULL;
}

void Costmap2D::incomingMap(const nav_msgs::OccupancyGridConstPtr& new_map)
{
  unsigned int size_x = new_map->info.width, size_y = new_map->info.height;

  ROS_INFO("Received a %d X %d map at %f m/pix with origin %f %f", size_x, size_y, 
                                             new_map->info.resolution,new_map->info.origin.position.x, new_map->info.origin.position.y);
  
  // Recreate map if physical dimensions change
  if(size_width != size_x || size_height !=size_y)
  {
    // Clearing previous allocation
    if(costmap_!=NULL)
    {
        delete[] costmap_;
        costmap_ = NULL;
    }
    costmap_ = new unsigned char[size_x * size_y];
  }

  unsigned int index = 0;

  // initialize the costmap with static data
  for (unsigned int i = 0; i < size_y; ++i)
  {
    for (unsigned int j = 0; j < size_x; ++j)
    {
      unsigned char value = new_map->data[index];
      costmap_[index] = interpretValue(value);
      ++index;
    }
  }
  map_frame_ = new_map->header.frame_id;

  size_width = size_x;
  size_height = size_y;
  map_received_ = true;
  has_updated_data_ = true;

 
}

unsigned char Costmap2D::interpretValue(unsigned char value)
{
  unsigned char unknown_cost_value_ = -1;
  unsigned char lethal_threshold_ = 100;

  // check if the static value is above the unknown or lethal thresholds
  if (track_unknown_space && value == unknown_cost_value_)
    return NO_INFORMATION;
  else if (!track_unknown_space && value == unknown_cost_value_)
    return FREE_SPACE;
  else if (value >= lethal_threshold_)
    return LETHAL_OBSTACLE;
  else if (trinary_costmap)
    return FREE_SPACE;

  double scale = (double) value / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}
