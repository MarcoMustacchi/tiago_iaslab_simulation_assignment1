#ifndef TIAGO_IASLAB_SIMULATION_CORRIDOR_SERVER_H
#define TIAGO_IASLAB_SIMULATION_CORRIDOR_SERVER_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/LaserScan.h>
#include <string.h>

#include "tiago_iaslab_simulation/point.h"

class CorridorServer {
 private:
  std::shared_ptr<ros::NodeHandle> nodeHandle_;

  std::string scanTopic_;

  ros::Subscriber getGoal_;
  ros::Subscriber getRobotPose_;
  ros::Subscriber scanner_;
  ros::Publisher cmdVel_;
  ros::Publisher navigation_;
  ros::Publisher feedback_;

  float maxCorridorWidth_;
  geometry_msgs::PoseStamped target_;

  bool pauseNavigation_;

  /// @brief Function to start the detection of a corridor.
  void start();
  /// @brief Function to stop the detection of a corridor.
  void stop();

  /// @brief Callback to get the target goal.
  /// @param goal
  void getGoalCallback(const move_base_msgs::MoveBaseActionGoalConstPtr& goal);
  /// @brief Callback to process the robot pose.
  /// @param pose
  void getRobotPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose);
  /// @brief Callback to process the laser scan data.
  /// @param msg
  void scannerCallback(const sensor_msgs::LaserScanConstPtr& msg);

  /// @brief Publish \c pauseNavigation_ to pause or resume the \c move_base navigation.
  /// @param pauseNavigation
  void togglePauseNavigation(bool pauseNavigation);

  /// @brief Publish the velocities to keep the robot in the middle.
  /// @param distanceRight
  /// @param distanceLeft
  void move(float distanceRight, float distanceLeft);

  /// @brief Calculate if the robot is near the target.
  /// @param point
  /// @return if \c point is near the target
  bool isNearTarget(Point point);

 public:
  /// @brief It's a service that detect circular obstacles usign the laser range sensor.
  /// @param nodeHandle
  /// @param maxCorridorWidth maximum width to consider the robot in a corridor
  /// @param goalTopic topic to get the goal from \c move_base
  /// @param robotPoseTopic topic to get the robot pose
  /// @param scanTopic topic to get the laser scan data
  /// @param cmdVelTopic topic to control the robot velocities
  /// @param pauseNavigationTopic topic to pause the \c move_base navigation
  /// @param feedbackTopic topic to publish the feedback
  // Constructor with all parameters
  CorridorServer(std::shared_ptr<ros::NodeHandle> nodeHandle,
                 float maxCorridorWidth,
                 std::string goalTopic,
                 std::string robotPoseTopic,
                 std::string scanTopic,
                 std::string cmdVelTopic,
                 std::string pauseNavigationTopic,
                 std::string feedbackTopic);
                 
  // Overloaded constructor with default topic names and width
  CorridorServer(std::shared_ptr<ros::NodeHandle> nodeHandle);
};

#endif  // TIAGO_IASLAB_SIMULATION_CORRIDOR_SERVER_H
