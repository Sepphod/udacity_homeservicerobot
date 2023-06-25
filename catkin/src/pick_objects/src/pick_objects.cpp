#include <vector>
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
  struct WayPoint {
    float x;
    float y;
    float orientation;
  };

  std::vector<WayPoint> all_waypoints{};

  WayPoint pickupGoal {2,2.5,1.57};
  all_waypoints.push_back(pickupGoal);
  WayPoint dropOffGoal {-7,3,-3.14};
  all_waypoints.push_back(dropOffGoal);

  for (auto it = all_waypoints.cbegin(); it != all_waypoints.cend(); ++it) {
    goal.target_pose.pose.position.x = it->x;
    goal.target_pose.pose.position.y = it->y;
    goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(it->orientation);

    // first iteration is the pickup_goal
    // second iteration is the goal to drop-off
    ROS_INFO("Sending goal");
    action_client.sendGoal(goal);

    action_client.waitForResult();

    if(action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      ROS_INFO("Goal reached!");
    } else {
      ROS_INFO("Failed!");
    }

    ros::Duration{5.0}.sleep();
  }

  return 0;
}
