#include "tiago_iaslab_simulation/utils.h"

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

geometry_msgs::Pose iaslab::createPose(geometry_msgs::Point point, float yaw) {
  tf2::Quaternion orientationTF2;
  orientationTF2.setRPY(0, 0, yaw);
  orientationTF2.normalize();

  geometry_msgs::Quaternion orientation;
  orientation.x = orientationTF2.getX();
  orientation.y = orientationTF2.getY();
  orientation.z = orientationTF2.getZ();
  orientation.w = orientationTF2.getW();

  geometry_msgs::Pose temp;
  temp.position = point;
  temp.orientation = orientation;

  return temp;
}

geometry_msgs::Pose iaslab::createPose(float x, float y, float yaw) {
  geometry_msgs::Point point;

  point.x = x;
  point.y = y;
  point.z = 0;

  return iaslab::createPose(point, yaw);
}
