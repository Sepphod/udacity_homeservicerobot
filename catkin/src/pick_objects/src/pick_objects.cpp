#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt8.h>

/* main function */
int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle node_handle{};
  //tell the action client that we want to spin a thread by default
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client{"move_base", true};
  //set up publisher to broadcast if robot is at goal marker
  ros::Publisher goal_reach_pub = node_handle.advertise<std_msgs::UInt8>("/goal_reached", 1);
  // Wait 5 sec for move_base action server to come up
  while(!action_client.waitForServer(ros::Duration{5.0})){
    ROS_INFO("Waiting");
  }

  move_base_msgs::MoveBaseGoal goal{};
  std_msgs::UInt8 status_msg{};  

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  ROS_INFO("publishing pick-up goal");
  node_handle.getParam("/pick_up_loc/tx", goal.target_pose.pose.position.x);
  node_handle.getParam("/pick_up_loc/ty", goal.target_pose.pose.position.y);
  node_handle.getParam("/pick_up_loc/tz", goal.target_pose.pose.position.z);
  node_handle.getParam("/pick_up_loc/qx", goal.target_pose.pose.orientation.x);
  node_handle.getParam("/pick_up_loc/qy", goal.target_pose.pose.orientation.y);
  node_handle.getParam("/pick_up_loc/qz", goal.target_pose.pose.orientation.z);
  node_handle.getParam("/pick_up_loc/qw", goal.target_pose.pose.orientation.w);
  action_client.sendGoal(goal);

  action_client.waitForResult();

  // Check if the robot reached its goal
  if(action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Robot reached PICK-UP location");
    status_msg.data = 1;
    goal_reach_pub.publish(status_msg);
  }
  else {
    ROS_INFO("The turtlebot failed!");
    return 0;
  }


  // robot reached pick-up location, send drop-off location
  ROS_INFO("Sending goal for drop-off");
  // wait a bit before next message
  ros::Duration{3.0}.sleep();

  // Define a position and orientation for the robot to reach
  node_handle.getParam("/drop_off_loc/tx", goal.target_pose.pose.position.x);
  node_handle.getParam("/drop_off_loc/ty", goal.target_pose.pose.position.y);
  node_handle.getParam("/drop_off_loc/tz", goal.target_pose.pose.position.z);
  node_handle.getParam("/drop_off_loc/qx", goal.target_pose.pose.orientation.x);
  node_handle.getParam("/drop_off_loc/qy", goal.target_pose.pose.orientation.y);
  node_handle.getParam("/drop_off_loc/qz", goal.target_pose.pose.orientation.z);
  node_handle.getParam("/drop_off_loc/qw", goal.target_pose.pose.orientation.w);
  action_client.sendGoal(goal);

  ROS_INFO("Heading to Drop-off site");
  // Wait an infinite time for the results
  action_client.waitForResult();

  // Check if the robot reached its drop goal
  if(action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    //ROS_INFO("Robot has reached DROP-OFF location");
    sleep(2);
    ROS_INFO("Dropping Package");
    // publish goal-reach status
    status_msg.data = 3;
    goal_reach_pub.publish(status_msg);
  }
  else {
    ROS_INFO("The robot failed to reach drop-off location");
  }

  // wait a bit before next message
  ros::Duration{3.0}.sleep();

  return 0;
}
