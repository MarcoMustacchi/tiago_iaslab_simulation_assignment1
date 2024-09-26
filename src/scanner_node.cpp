#include <ros/ros.h>

#include "tiago_iaslab_simulation/scanner_server.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "scanner_node");

  auto nh_ptr = std::make_shared<ros::NodeHandle>();

  // Using the overloaded constructor with default topic names and parameters
  ScannerServer scannerServer(nh_ptr);

  // Or, use the full constructor with custom parameters
  // ScannerServer scannerServer(nh_ptr, "custom_scan_obstacles", "custom_scan", 0.6, 15, 0.2, 0.3);

  ros::spin();

  return 0;
}
