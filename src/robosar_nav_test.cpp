// Created by Indraneel on 7/03/22

#include <ros/ros.h>
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
  double start1[] = {25.0,9.0,0.0};

  //double goal1[] = {0.2,0.2,0.0};
  //double start1[] = {-0.2,-0.2,0.0};
  currPos.push_back(start1);
  targetPos.push_back(goal1);

  double goal2[] = {17.0,17.0,0.0};
  double start2[] = {28.0,21.0,0.0};
  //double goal2[] = {-0.2,0.2,0.0};
  //double start2[] = {0.2,-0.2,0.0};
  currPos.push_back(start2);
  targetPos.push_back(goal2);

  double goal3[] = {17.0,14.0,0.0};
  double start3[] = {21.0,25.0,0.0};
  //double goal2[] = {-0.2,0.2,0.0};
  //double start2[] = {0.2,-0.2,0.0};
  currPos.push_back(start3);
  targetPos.push_back(goal3);

  
  std::vector<std::string> agents = {"agent1","agent2","agent3"}; 


  MultiAStar multi_astar(&gridmap,currPos,targetPos,agents);

  ros::Duration(2.0).sleep();
  bool status = multi_astar.run_multi_astar();

  //ros::MultiThreadedSpinner s;
  ros::spin();

  return (0);
}