#include <ros/ros.h>

#include "tiago_iaslab_simulation/corridor_server.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "corridor_node");

  auto nh_ptr = std::make_shared<ros::NodeHandle>();

  // Using the overloaded constructor with default topic names and corridor width
  CorridorServer corridorServer(nh_ptr);

  // Or, use the full constructor with custom topic names and corridor width
  // CorridorServer corridorServer(nh_ptr, 1.5, "custom_goal_topic", "custom_robot_pose_topic", 
  //    "custom_scan_topic", "custom_cmd_vel_topic", "custom_pause_navigation_topic", "custom_feedback_topic");

  ros::spin();

  return 0;
}
