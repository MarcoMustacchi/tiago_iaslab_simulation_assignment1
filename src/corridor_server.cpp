#include "tiago_iaslab_simulation/corridor_server.h"

#include <geometry_msgs/Twist.h>
#include <ros/node_handle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>

#include "tiago_iaslab_simulation/point.h"
#include "tiago_iaslab_simulation/status_constant.h"

// Full constructor with all parameters
CorridorServer::CorridorServer(std::shared_ptr<ros::NodeHandle> nodeHandle,
                               float maxCorridorWidth,
                               std::string goalTopic,
                               std::string robotPoseTopic,
                               std::string scanTopic,
                               std::string cmdVelTopic,
                               std::string pauseNavigationTopic,
                               std::string feedbackTopic) 
    : nodeHandle_(nodeHandle),
      maxCorridorWidth_(maxCorridorWidth),
      scanTopic_(scanTopic),
      pauseNavigation_(false) {
  getGoal_ = nodeHandle_->subscribe(goalTopic, 1000, &CorridorServer::getGoalCallback, this);
  getRobotPose_ = nodeHandle_->subscribe(robotPoseTopic, 1000, &CorridorServer::getRobotPoseCallback, this);
  cmdVel_ = nodeHandle_->advertise<geometry_msgs::Twist>(cmdVelTopic, 1);
  navigation_ = nodeHandle_->advertise<std_msgs::Bool>(pauseNavigationTopic, 1);
  feedback_ = nodeHandle_->advertise<std_msgs::UInt8>(feedbackTopic, 1);
}

// Overloaded constructor with default topic names and corridor width
CorridorServer::CorridorServer(std::shared_ptr<ros::NodeHandle> nodeHandle)
    : CorridorServer(nodeHandle, 
                     1.1, 
                     "move_base/goal", 
                     "robot_pose", 
                     "scan", 
                     "mobile_base_controller/cmd_vel", 
                     "pause_navigation", 
                     "move_server_feedback") {}


void CorridorServer::getGoalCallback(const move_base_msgs::MoveBaseActionGoalConstPtr& goal) {
  target_ = goal->goal.target_pose;

  start();
}

void CorridorServer::getRobotPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pose) {
  Point robotPosition(*pose);

  if (isNearTarget(robotPosition)) {
    stop();
    togglePauseNavigation(false);
  }
}

void CorridorServer::start() {
  scanner_ = nodeHandle_->subscribe(scanTopic_, 1000, &CorridorServer::scannerCallback, this);
}

void CorridorServer::stop() {
  scanner_.shutdown();
}

void CorridorServer::scannerCallback(const sensor_msgs::LaserScanConstPtr& msg) {
  float distanceRight = msg->ranges[66];
  float distanceLeft = msg->ranges[msg->ranges.size() - 66];

  if (distanceRight + distanceLeft < maxCorridorWidth_) {
    togglePauseNavigation(true);
    move(distanceRight, distanceLeft);
  } else {
    togglePauseNavigation(false);
  }
}

void CorridorServer::togglePauseNavigation(bool pauseNavigation) {
  if (pauseNavigation_ != pauseNavigation) {
    pauseNavigation_ = pauseNavigation;

    std_msgs::Bool msg;
    msg.data = pauseNavigation_;
    navigation_.publish(msg);

    std_msgs::UInt8 feedback;
    if (pauseNavigation_) {
      feedback.data = status::MOVING_CORRIDOR;
    } else {
      feedback.data = status::MOVING;
    }
    feedback_.publish(feedback);
  }
}

void CorridorServer::move(float distanceRight, float distanceLeft) {
  geometry_msgs::Twist vel;
  vel.linear.x = 0.5;
  vel.angular.z = 0.5 * -(distanceRight - distanceLeft);

  cmdVel_.publish(vel);
}

bool CorridorServer::isNearTarget(Point point) {
  Point target(target_);

  return point.distance(target) < maxCorridorWidth_ / 2;
}
