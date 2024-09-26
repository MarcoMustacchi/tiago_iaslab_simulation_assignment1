#include <ros/ros.h>

#include "tiago_iaslab_simulation/client.h"
#include "tiago_iaslab_simulation/map2d.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "client_node");

  Map2D map("map");

  float x;
  float y;
  float yaw;
  std::string frameId;

  if (argc == 4) {
    x = atof(argv[1]);
    y = atof(argv[2]);
    yaw = atof(argv[3]);
  } else {
    do {
      printf("Enter your position goal x: ");
      scanf("%f", &x);
      printf("Enter your position goal y: ");
      scanf("%f", &y);
      printf("Enter your orientation goal yaw: ");
      scanf("%f", &yaw);

      if (!map.isValidPoint(x, y)) {
        printf("Invalid point");
      }
    } while (!map.isValidPoint(x, y));
  }

  if (!map.isValidPoint(x, y)) {
    return EXIT_FAILURE;
  }

  auto nh_ptr = std::make_shared<ros::NodeHandle>();

  // Using the overloaded constructor with default topic names
  Client client(nh_ptr);

  // Or, use the full constructor with custom topic names
  // Client client(nh_ptr, "custom_move_server", "custom_visualization_marker");

  client.sendPose(x, y, yaw);

  ros::spin();

  return EXIT_SUCCESS;
}
