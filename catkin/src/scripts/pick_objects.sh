#!/bin/bash

xterm  -e "cd $(pwd)/../..;
source devel/setup.bash;
catkin_make;
source devel/setup.bash;
export ROBOT_INITIAL_POSE='-x -2.49 -y -4.28 -z 0 -R 0 -P 0 -Y 0';
export TURTLEBOT_GAZEBO_WORLD_FILE='$(pwd)/../../src/worlds/new_building.world';
roslaunch turtlebot_gazebo turtlebot_world.launch" &

sleep 5

xterm -e "export TURTLEBOT_GAZEBO_MAP_FILE='/home/workspace/udacity_homeservicerobot/catkin/src/map/map2.yaml';
roslaunch turtlebot_gazebo amcl_demo.launch" &

sleep 5

xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_rviz_launchers view_navigation.launch" &

sleep 10
xterm  -e  "cd $(pwd)/../..;
source devel/setup.bash;
rosparam load $(pwd)/../config/marker.yaml;
rosrun pick_objects pick_objects" &

