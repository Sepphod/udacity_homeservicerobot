#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt8.h>
#include "tf/tf.h"


/* main function */
int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");
  //tell the action client that we want to spin a thread by default
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client{"move_base", true};
  // Wait 5 sec for move_base action server to come up
  while(!action_client.waitForServer(ros::Duration{5.0})){
    ROS_INFO("Waiting");
  }

  move_base_msgs::MoveBaseGoal goal{};


  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a list of positions and orientations for the robot to reach
  float waypoints[2][3] = { 
                            { 2.5, 2,  1.57}, 
                            {-7,   3, -3.14}  
                          };

  int num_points = 2;

  for (int i=0; i < num_points; i++){

      goal.target_pose.pose.position.x = waypoints[i][0];
      goal.target_pose.pose.position.y = waypoints[i][1];
      goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(waypoints[i][2]);

       // Send the goal position and orientation for the robot to reach
      ROS_INFO("Sending goal");
      action_client.sendGoal(goal);

      // Wait an infinite time for the results
      action_client.waitForResult();

      // Check if the robot reached its goal
      if(action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Goal reached!");
      else
        ROS_INFO("Failed!");
      ros::Duration{5.0}.sleep();
  }

  return 0;
}
