// Created by Indraneel and Chravi on 5/04/22

#include "mission_executive.hpp"

#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robosar_mission_executive_node");

  //move_base::MoveBase move_base( buffer );

  MissionExecutive new_mission;

  //ros::MultiThreadedSpinner s;
  ros::spin();

  return (0);
}