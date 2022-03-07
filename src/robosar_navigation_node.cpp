// Created by Indraneel on 7/03/22

#include <ros/ros.h>
#include "costmap_2d.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robosar_navigation_node");

  //move_base::MoveBase move_base( buffer );
  Costmap2D costmap;

  //ros::MultiThreadedSpinner s;
  ros::spin();

  return (0);
}