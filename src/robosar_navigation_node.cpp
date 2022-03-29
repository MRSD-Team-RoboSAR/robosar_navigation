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
  ros::init(argc, argv, "robosar_navigation_node");
  //move_base::MoveBase move_base( buffer );
  Graph3DGrid gridmap;

  //double goal[] = {19.0,20.0,0.0};
  //double start[] = {18.0,25.0,0.0};
  //AStar planner(&gridmap,goal,start);
  
  int numAgents=3;
  double goal[] = {0.0,0.0};
  double start[] = {0.0,0.0};
  tf2::BufferCore buffer_core;
  std::vector<std::pair<double,double>> currPos;
  std::vector<std::pair<double,double>> targetPos;
  tf::TransformListener listener;
  ros::Duration(5.0).sleep();
  tf::StampedTransform transform[numAgents];
  try{
    for(int i=0;i<numAgents;i++){
      std::string target_base = "robot_"+std::to_string(i)+"/base_link";
      listener.lookupTransform("map", target_base,
                             ros::Time(0), transform[i]);
      currPos.push_back(std::make_pair(transform[i].getOrigin().x(),transform[i].getOrigin().y()));
    }
    ROS_INFO("Transform0 x: %f y:%f",transform[0].getOrigin().x(),transform[0].getOrigin().y());
    ROS_INFO("Transform1 x: %f y:%f",transform[1].getOrigin().x(),transform[1].getOrigin().y());
    ROS_INFO("Transform2 x: %f y:%f",transform[2].getOrigin().x(),transform[2].getOrigin().y());
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
    ros::Duration(1.0).sleep();
  }
  targetPos.push_back(std::make_pair(10,10));
  targetPos.push_back(std::make_pair(10,10));
  targetPos.push_back(std::make_pair(10,10));
  MultiAStar multi_astar(&gridmap,currPos,targetPos,numAgents);
  bool status = multi_astar.run_multi_astar();
  //AStar planner(&gridmap,goal,start);

  ros::Duration(1.0).sleep();

  planner.run_planner(gridmap.getNumNodes());

  //ros::MultiThreadedSpinner s;
  ros::spin();

  return (0);
}