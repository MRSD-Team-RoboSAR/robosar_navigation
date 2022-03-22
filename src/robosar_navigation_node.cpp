// Created by Indraneel on 7/03/22

#include <ros/ros.h>
#include "graph_2d_grid.hpp"
#include "astar.hpp"
#include "string.h"
#include <tf/transform_listener.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "robosar_navigation_node");
  ros::NodeHandle node;
  //move_base::MoveBase move_base( buffer );
  Graph2DGrid gridmap;
  int numAgents=3;
  double goal[] = {0.0,0.0};
  double start[] = {0.0,0.0};
  tf::TransformListener listener;
  while(node.ok()){
    tf::StampedTransform transform[3];
    try{
      for(int i=0;i<numAgents;i++){
        std::string target_base = "robot_"+std::to_string(i)+"/base_link";
        listener.lookupTransform("map", target_base,  
                               ros::Time(0), transform[i]);
      }
      ROS_INFO("Transform0 x: %f y:%f",transform[0].getOrigin().x(),transform[0].getOrigin().y());
      ROS_INFO("Transform1 x: %f y:%f",transform[1].getOrigin().x(),transform[1].getOrigin().y());
      ROS_INFO("Transform2 x: %f y:%f",transform[2].getOrigin().x(),transform[2].getOrigin().y());
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
  }
  AStar planner(&gridmap,goal,start);

  //ros::MultiThreadedSpinner s;
  ros::spin();

  return (0);
}