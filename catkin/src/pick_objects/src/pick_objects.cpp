#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt8.h>


/* main function */
int main(int argc, char** argv){

  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle node_handle{};

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client{"move_base", true};
  
  while(!action_client.waitForServer(ros::Duration{5.0})){
    ROS_INFO("Waiting");
  }

  move_base_msgs::MoveBaseGoal goal{};	
  std_msgs::UInt8 status_msg{};  

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Send the goal position and orientation for the robot for pick-up location
  ROS_INFO("publishing pick-up goal");
  // Define a position and orientation for the robot to reach
  node_handle.getParam("/pick_up_loc/tx", goal.target_pose.pose.position.x);
  node_handle.getParam("/pick_up_loc/ty", goal.target_pose.pose.position.y);
  node_handle.getParam("/pick_up_loc/tz", goal.target_pose.pose.position.z);
  node_handle.getParam("/pick_up_loc/qx", goal.target_pose.pose.orientation.x);
  node_handle.getParam("/pick_up_loc/qy", goal.target_pose.pose.orientation.y);
  node_handle.getParam("/pick_up_loc/qz", goal.target_pose.pose.orientation.z);
  node_handle.getParam("/pick_up_loc/qw", goal.target_pose.pose.orientation.w);
  action_client.sendGoal(goal);

  action_client.waitForResult();

  if(action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("turtlebot has reached pick-up position");
    status_msg.data = 1;
    goal_reach_pub.publish(status_msg);
  }
  else {
    ROS_INFO("turtlebot has failed to reach pick-up position");
    return 0;
  }

  ROS_INFO("drop-off goal");
  ros::Duration(3.0).sleep();

  // Define a position and orientation for the robot to reach
  node_handle.getParam("/drop_off_position/tx", goal.target_pose.pose.position.x);
  node_handle.getParam("/drop_off_position/ty", goal.target_pose.pose.position.y);
  node_handle.getParam("/drop_off_position/tz", goal.target_pose.pose.position.z);
  node_handle.getParam("/drop_off_position/qx", goal.target_pose.pose.orientation.x);
  node_handle.getParam("/drop_off_position/qy", goal.target_pose.pose.orientation.y);
  node_handle.getParam("/drop_off_position/qz", goal.target_pose.pose.orientation.z);
  node_handle.getParam("/drop_off_position/qw", goal.target_pose.pose.orientation.w);
  action_client.sendGoal(goal);

  ROS_INFO("navigating to drop-off position");
  action_client.waitForResult();

  if(action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("turtlebot has reached drop-off position");
    ros::Duration(2.0).sleep();
    status_msg.data = 3;
    goal_reach_pub.publish(status_msg);
  }
  else {
    ROS_INFO("turtlebot has failed to reach drop-off position");
  }

  ros::Duration(5.0).sleep();

  return 0;
}

// #include <vector>
// #include <ros/ros.h>
// #include <move_base_msgs/MoveBaseAction.h>
// #include <actionlib/client/simple_action_client.h>
// #include <std_msgs/UInt8.h>
// #include "tf/tf.h"


// /* main function */
// int main(int argc, char** argv){
//   // Initialize the simple_navigation_goals node
//   ros::init(argc, argv, "pick_objects");
//   //tell the action client that we want to spin a thread by default
//   actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> action_client{"move_base", true};
//   // Wait 5 sec for move_base action server to come up
//   while(!action_client.waitForServer(ros::Duration{5.0})){
//     ROS_INFO("Waiting");
//   }

//   move_base_msgs::MoveBaseGoal goal{};


//   goal.target_pose.header.frame_id = "map";
//   goal.target_pose.header.stamp = ros::Time::now();

//   struct WayPoint {
//     float x;
//     float y;
//     float orientation;
//   };

//   std::vector<WayPoint> all_waypoints{};

//   WayPoint pickupGoal {2,2.5,1.57};
//   all_waypoints.push_back(pickupGoal);
//   WayPoint dropOffGoal {-7,3,-3.14};
//   all_waypoints.push_back(dropOffGoal);

//   for (auto it = all_waypoints.cbegin(); it != all_waypoints.cend(); ++it) {
//     goal.target_pose.pose.position.x = it->x;
//     goal.target_pose.pose.position.y = it->y;
//     goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(it->orientation);

//     // first iteration is the pickup_goal
//     // second iteration is the goal to drop-off
//     ROS_INFO("Sending goal");
//     action_client.sendGoal(goal);

//     action_client.waitForResult();

//     if(action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
//       ROS_INFO("Goal reached!");
//     } else {
//       ROS_INFO("Failed!");
//     }

//     ros::Duration{5.0}.sleep();
//   }

//   return 0;
// }
