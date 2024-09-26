#include <ros/ros.h>

#include "tiago_iaslab_simulation/move_server.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_node");

  auto nh_ptr = std::make_shared<ros::NodeHandle>();

  // Using the overloaded constructor with default topic names
  MoveServer moveServer(nh_ptr);

  // Or, use the full constructor with custom topic names
  // MoveServer moveServer(nh_ptr, "custom_move_server", "custom_move_base", "custom_scan_obstacles", "custom_feedback");

  ros::spin();

  return 0;
}
