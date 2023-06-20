#!/bin/bash

# build the workspace
source environment.bash
cd $(pwd)/../..; catkin_make

source devel/setup.bash
#source /opt/ros/kinetic/setup.bash
export TURTLEBOT_GAZEBO_WORLD_FILE="$(pwd)/src/worlds/new_building.world"
export ROBOT_INITIAL_POSE="-x -2.49 -y -4.28 -z 0"

xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch" &    
sleep 5
xterm  -e  " roslaunch turtlebot_gazebo gmapping_demo.launch" & 
sleep 5
xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" & 
sleep 5
xterm  -e  " roslaunch turtlebot_teleop keyboard_teleop.launch"   