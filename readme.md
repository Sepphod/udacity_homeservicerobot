# Home Service Robot

Final project for Udacity's Robotics Software Engineer Nanodegree Program

## Project goals

* Design an environment with the Building Editor in Gazebo to operate a turtlebot.
* Manually test SLAM by teleoperating the turtlebot.
* Use the ROS navigation stack
* Implement a pick_objects ROS node in C++ to control the robot to move to dedicated pickup and drop off positions.
* Implement an add_markers ROS node in C++ to subscribe the robot odometry and publish the pickup and drop off positions via markers in the visualization,
* Integrate the two ROS nodes to complete a home service robot

### Directory Tree and contents

```
.
├── readme.md
├── catkin
├── CMakeLists.txt
├── src
	├── add_markers
	│   └── src
	│       └── add_markers.cpp
	│   ├──  ... ...
	├── config
	│   └── marker.yaml
	├── map
	│   ├── map.pgm
	│   └── _map.yaml
	├── pick_objects
	│   └── src
	│       └── pick_objects.cpp
	│   ├──  ... ...
	├── rvizConfig
	│   └── home_service_robot.rviz
	├── scripts
	│   ├── add_markers.sh
	│   ├── environment.bash
	│   ├── add_markers.sh
	│   ├── launch.sh
	│   ├── pick_objects.sh
	│   ├── test_navigation.sh
	│   └── test_slam.sh
	├── slam_gmapping
	├── turtlebot
	├── turtlebot_interactions
	|── turtlebot_simulator

```

This directory represents the main project's `src` folder structure with following contents

* **add_markers** - add marker C++ node
* **config** - configuration of dedicated pick-up and drop-off positions
* **map**
* **pick_objects** - pick-objects C++ node
* **rvizConfig**
* **scripts**
  * `add_marker.sh` - script to test marker
  * `home_service_robot.sh` - main script for the home service robot
  * `pick_objects.sh` - script to testing pick_objects
  * `test_navigation.sh` - script to test navigation
  * `test_slam.sh` - script for performing SLAM and preparing map
* **slam_gmapping** - official ROS package with `gmapping_demo.launch` file
* **turtlebot** - official ROS package with `keyboard_teleop.launch` file
* **turtlebot_interactions** - official ROS package with `view_navigation.launch` file
* **turtlebot_simulator** - official ROS package with `turtlebot_world.launch` file

### Home-Service-Robot package

The goal was a home service robot which is capable of

* navigating to a given pick-up position to pick up virtual objects
* deliver the virtual objects to a given drop off position.

The communication between the responsible ROS nodes was established via the "/goal_to_reach" topic. The package can be launched using:

```
cd ~/catkin_ws/src/scripts
./home_service.sh
```
