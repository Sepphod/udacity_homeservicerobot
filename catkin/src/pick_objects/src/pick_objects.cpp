#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt8.h>

int main(int argc, char** argv){

  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle node_handle{};

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client{"move_base", true};
  
  ros::Publisher reachedGoalPublisher = node_handle.advertise<std_msgs::UInt8>("/goal_to_reach", 1);

  while(!action_client.waitForServer(ros::Duration{5.0})){
    ROS_INFO("Waiting");
  }

  move_base_msgs::MoveBaseGoal goal{};	
  std_msgs::UInt8 status_msg{};  

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  ROS_INFO("publishing pick-up goal");
  node_handle.getParam("/pick_up_position/tx", goal.target_pose.pose.position.x);
  node_handle.getParam("/pick_up_position/ty", goal.target_pose.pose.position.y);
  node_handle.getParam("/pick_up_position/tz", goal.target_pose.pose.position.z);
  node_handle.getParam("/pick_up_position/quat_x", goal.target_pose.pose.orientation.x);
  node_handle.getParam("/pick_up_position/quat_y", goal.target_pose.pose.orientation.y);
  node_handle.getParam("/pick_up_position/quat_z", goal.target_pose.pose.orientation.z);
  node_handle.getParam("/pick_up_position/quat_w", goal.target_pose.pose.orientation.w);
  action_client.sendGoal(goal);

  action_client.waitForResult();

  if(action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("turtlebot has reached pick-up position");
    status_msg.data = 1;
    reachedGoalPublisher.publish(status_msg);
  }
  else {
    ROS_INFO("turtlebot has failed to reach pick-up position");
    return 0;
  }

  ROS_INFO("drop-off goal");
  ros::Duration(3.0).sleep();

  node_handle.getParam("/drop_off_position/tx", goal.target_pose.pose.position.x);
  node_handle.getParam("/drop_off_position/ty", goal.target_pose.pose.position.y);
  node_handle.getParam("/drop_off_position/tz", goal.target_pose.pose.position.z);
  node_handle.getParam("/drop_off_position/quat_x", goal.target_pose.pose.orientation.x);
  node_handle.getParam("/drop_off_position/quat_y", goal.target_pose.pose.orientation.y);
  node_handle.getParam("/drop_off_position/quat_z", goal.target_pose.pose.orientation.z);
  node_handle.getParam("/drop_off_position/quat_w", goal.target_pose.pose.orientation.w);
  action_client.sendGoal(goal);

  ROS_INFO("navigating to drop-off position");
  action_client.waitForResult();

  if(action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("turtlebot has reached drop-off position");
    ros::Duration(2.0).sleep();
    status_msg.data = 3;
    reachedGoalPublisher.publish(status_msg);
  }
  else {
    ROS_INFO("turtlebot has failed to reach drop-off position");
  }

  ros::Duration(5.0).sleep();

  return 0;
}
