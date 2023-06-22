#!/bin/bash

xterm  -e "cd $(pwd)/../..;
source devel/setup.bash;
catkin_make;
source devel/setup.bash;
export ROBOT_INITIAL_POSE='-x -2.49 -y -4.28 -z 0 -R 0 -P 0 -Y 0';
roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/src/worlds/new_building.world" &

sleep 5

xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_gazebo amcl_demo.launch map_file:=$(pwd)../../src/map/map.yaml " &

sleep 5

# launch view_navigation for rviz
xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_rviz_launchers view_navigation.launch" &

# # build the workspace
# cd $(pwd)/../..; catkin_make

# source devel/setup.bash
# #source /opt/ros/kinetic/setup.bash
# export TURTLEBOT_GAZEBO_WORLD_FILE="$(pwd)/src/worlds/new_building.world"
# #export TURTLEBOT_GAZEBO_MAP_FILE="$(pwd)/src/map/map.yaml"

# xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch" &    
# sleep 5
# xterm  -e  " roslaunch turtlebot_gazebo amcl_demo.launch" & 
# sleep 5
# xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch"