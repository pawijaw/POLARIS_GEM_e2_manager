#!/bin/bash

source /opt/ros/noetic/setup.bash
source devel/setup.sh
roslaunch gem_gazebo gem_gazebo_rviz.launch velodyne_points:="true" & 
sleep 5
roslaunch robot_manager robot_manager.launch
