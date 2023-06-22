#!/bin/bash

xterm  -e "cd $(pwd)/../..;
source devel/setup.bash;
catkin_make;
source devel/setup.bash;
export ROBOT_INITIAL_POSE='-x -2.49 -y -4.28 -z 0 -R 0 -P 0 -Y 0';
export TURTLEBOT_GAZEBO_WORLD_FILE='$(pwd)/src/worlds/new_building.world';
roslaunch turtlebot_gazebo turtlebot_world.launch" &
# roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/src/worlds/new_building.world" &

sleep 5

xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
rosparam set /slam_gmapping/iterations 10;
rosparam set /slam_gmapping/linearUpdate 0.05;
rosparam set /slam_gmapping/angularUpdate 0.1;
rosparam set /slam_gmapping/map_update_interval 0.25;
rosparam set /slam_gmapping/srr 0.01;
rosparam set /slam_gmapping/srt 0.01;
rosparam set /slam_gmapping/str 0.02;
rosparam set /slam_gmapping/stt 0.02;
roslaunch turtlebot_gazebo gmapping_demo.launch" &

sleep 5

xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_rviz_launchers view_navigation.launch" &

sleep 5

xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_rviz_launchers view_navigation.launch" &

sleep 5

xterm -e "cd $(pwd)/../..;
source devel/setup.bash;
roslaunch turtlebot_teleop keyboard_teleop.launch" &

# cd $(pwd)/../..; catkin_make

# source devel/setup.bash
# #source /opt/ros/kinetic/setup.bash
# export ROBOT_INITIAL_POSE="-x -2.49 -y -4.28 -z 0 -R 0 -P 0 -Y 0"


# xterm  -e cd $(pwd)/../..
# source devel/setup.bash

# xterm  -e  " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(pwd)/src/worlds/new_building.world" &    
# sleep 5
# xterm  -e  " roslaunch turtlebot_gazebo gmapping_demo.launch" & 
# sleep 5
# xterm  -e  " roslaunch turtlebot_rviz_launchers view_navigation.launch" & 
# sleep 5
# xterm  -e  " roslaunch turtlebot_teleop keyboard_teleop.launch"   