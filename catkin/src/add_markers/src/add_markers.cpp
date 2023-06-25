#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include <std_msgs/UInt8.h>
#include <cmath>
#include <cstdint>

std::uint8_t goal_state{0};

void goalStateCallback(const std_msgs::UInt8::ConstPtr& msg) {
   goal_state = msg->data;
   return;
}

int main( int argc, char** argv ) {
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle node_handle{};
  ros::Rate rate(5);
  ros::Publisher marker_publisher = node_handle.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odom_subscriber = node_handle.subscribe<visualization_msgs::Marker>("/reached_goal", 1, goalStateCallback);

  ROS_INFO("Subscribed to required goal");

  bool isDropOffReached{false};

  while (ros::ok()) {
    //Do this every cycle to ensure the subscriber receives the message
    ros::spinOnce();
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;    

    switch (goal_state)
    {
      case 0: {
        ROS_INFO("publish marker to pick up");

        marker.action = visualization_msgs::Marker::ADD;
        node_handle.getParam("/pick_up_position/tx", marker.pose.position.x);
        node_handle.getParam("/pick_up_position/ty", marker.pose.position.y);
        node_handle.getParam("/pick_up_position/tz", marker.pose.position.z);
        node_handle.getParam("/pick_up_position/qx", marker.pose.orientation.x);
        node_handle.getParam("/pick_up_position/qy", marker.pose.orientation.y);
        node_handle.getParam("/pick_up_position/qz", marker.pose.orientation.z);
        node_handle.getParam("/pick_up_position/qw", marker.pose.orientation.w);
        break;
      } 

      case 1: {
        ROS_INFO("bot reached pickup site let's hide pickup marker");
        marker.action = visualization_msgs::Marker::DELETE;
        break;
      } 

      case 2: {
        ROS_INFO("Let's wait until bot reaches drop-off site");
        marker.action = visualization_msgs::Marker::DELETE;
        break;
      }

      case 3: {
        ROS_INFO("Let's publish the drop-off marker");
        ros::Duration{5.0}.sleep();

        isDropOffReached = true;

        marker.action = visualization_msgs::Marker::ADD;
        node_handle.getParam("/drop_off_position/tx", marker.pose.position.x);
        node_handle.getParam("/drop_off_position/ty", marker.pose.position.y);
        node_handle.getParam("/drop_off_position/tz", marker.pose.position.z);
        node_handle.getParam("/drop_off_position/qx", marker.pose.orientation.x);
        node_handle.getParam("/drop_off_position/qy", marker.pose.orientation.y);
        node_handle.getParam("/drop_off_position/qz", marker.pose.orientation.z);
        node_handle.getParam("/drop_off_position/qw", marker.pose.orientation.w);
        break;
      }
      default: {
        ROS_INFO("Unknown state - lets stop");
        break;
      }
    }

    while (marker_publisher.getNumSubscribers() < 1)
    {
      if (!ros::ok()) {
        return 0;
      }
      ROS_WARN_ONCE("subscriber is missing");
      ros::Duration{1.0}.sleep();
    }

    //publish the marker
    marker_publisher.publish(marker);

    // if last marker published and noted as done exit
    if (isDropOffReached) {
      ROS_INFO("Destination reached");
      ros::Duration{7.0}.sleep();
      return 0;
    }

    rate.sleep();
  }


  return 0;
}

