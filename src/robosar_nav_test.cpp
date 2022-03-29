// Created by Indraneel on 7/03/22

#include <ros/ros.h>
#include "graph_2d_grid.hpp"
#include "graph_3d_grid.hpp"

#include "multi_astar.hpp"
#include "string.h"
#include <tf/transform_listener.h>
#include <tf2/buffer_core.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "robosar_nav_test_node");
  //move_base::MoveBase move_base( buffer );
  Graph3DGrid gridmap;

  // Create endpoints
  std::vector<double*> currPos;
  std::vector<double*> targetPos;

  double goal1[] = {19.0,20.0,0.0};
  double start1[] = {18.0,25.0,0.0};
  currPos.push_back(start1);
  targetPos.push_back(goal1);

  double goal2[] = {19.0,20.0,0.0};
  double start2[] = {18.0,25.0,0.0};
  currPos.push_back(start2);
  targetPos.push_back(goal2);
  
  std::vector<std::string> agents = {"agent1","agent2"}; 


  MultiAStar multi_astar(&gridmap,currPos,targetPos,agents);

  ros::Duration(2.0).sleep();
  bool status = multi_astar.run_multi_astar();

  //ros::MultiThreadedSpinner s;
  ros::spin();

  return (0);
}