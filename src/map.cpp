#include <costmap_2d/cost_values.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/node_handle.h>
#include <ros/topic.h>
#include <string.h>

#include "tiago_iaslab_simulation/map2d.h"

Map2D::Map2D(std::string topic) : topic_(topic) {
  update();
}

void Map2D::generateMap(nav_msgs::OccupancyGrid grid) {
  map_ = costmap_2d::Costmap2D(grid.info.width,
                              grid.info.height,
                              grid.info.resolution,
                              grid.info.origin.position.x,
                              grid.info.origin.position.y);

  for (size_t i = 0; i < grid.data.size(); i++) {
    u_int mx;
    u_int my;

    map_.indexToCells(i, mx, my);

    map_.setCost(mx, my, grid.data[i]);
  }
}

void Map2D::update() {
  boost::shared_ptr<const nav_msgs::OccupancyGrid> map_msg = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(topic_, nodeHandle_);

  generateMap(*map_msg);
}

bool Map2D::isValidPoint(float x, float y) const {
  u_int mapX;
  u_int mapY;

  if (map_.worldToMap(x, y, mapX, mapY)) {
    unsigned char cost = map_.getCost(mapX, mapY);

    if (cost == costmap_2d::FREE_SPACE) {
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

bool Map2D::isValidPoint(geometry_msgs::Point point) const {
  return isValidPoint(point.x, point.y);
}
