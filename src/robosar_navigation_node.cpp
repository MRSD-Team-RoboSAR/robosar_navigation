// Created by Indraneel on 7/03/22

#include <ros/ros.h>
#include "graph_2d_grid.hpp"
#include "astar.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robosar_navigation_node");

  //move_base::MoveBase move_base( buffer );
  Graph2DGrid gridmap;

  double goal[] = {19.0,20.0};
  double start[] = {18.0,25.0};
  AStar planner(&gridmap,goal,start);

  ros::Duration(1.0).sleep();

  planner.run_planner(gridmap.getNumNodes());

  //ros::MultiThreadedSpinner s;
  ros::spin();

  return (0);
}