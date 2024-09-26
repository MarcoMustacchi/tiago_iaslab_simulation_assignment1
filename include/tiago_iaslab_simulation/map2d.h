#ifndef TIAGO_IASLAB_SIMULATION_MAP2D_H
#define TIAGO_IASLAB_SIMULATION_MAP2D_H

#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/node_handle.h>
#include <string.h>

class Map2D {
 private:
  ros::NodeHandle nodeHandle_;
  std::string topic_;
  costmap_2d::Costmap2D map_;

  void generateMap(nav_msgs::OccupancyGrid grid);

 public:
  Map2D(std::string topic);
  void update();

  bool isValidPoint(float x, float y) const;
  bool isValidPoint(geometry_msgs::Point point) const;
};

#endif  // TIAGO_IASLAB_SIMULATION_MAP2D_H
