#!/bin/bash

# build the workspace
source environment.bash
cd $(pwd)/../..; catkin_make

source devel/setup.bash
#source /opt/ros/kinetic/setup.bash


xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch" &    
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo gmapping_demo.launch" & 
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" & 
sleep 5
xterm  -e  " roslaunch turtlebot_teleop keyboard_teleop.launch"   