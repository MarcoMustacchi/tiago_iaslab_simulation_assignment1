#include "tiago_iaslab_simulation/move_server.h"

#include <actionlib/client/simple_action_client.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/subscriber.h>
#include <std_msgs/UInt8.h>
#include <tf2_ros/transform_listener.h>

#include "tiago_iaslab_simulation/scanObstacles.h"
#include "tiago_iaslab_simulation/status_constant.h"

// Full constructor with all parameters
MoveServer::MoveServer(std::shared_ptr<ros::NodeHandle> nodeHandle,
                       std::string moveServerTopic,
                       std::string moveBaseTopic,
                       std::string scannerTopic,
                       std::string feedbackTopic)
    : nodeHandle_(nodeHandle),
      actionServer_(*nodeHandle_, moveServerTopic, boost::bind(&MoveServer::move, this, _1), false),
      moveActionClient_(moveBaseTopic) {
  
  scannerClient_ = nodeHandle_->serviceClient<tiago_iaslab_simulation::scanObstacles>(scannerTopic);
  feedbackRelay_ = nodeHandle_->subscribe(feedbackTopic, 1000, &MoveServer::feedbackRelayCallback, this);

  actionServer_.start();
}

// Overloaded constructor with default topic names
MoveServer::MoveServer(std::shared_ptr<ros::NodeHandle> nodeHandle)
    : MoveServer(nodeHandle, 
                 "move_server", 
                 "move_base", 
                 "scan_obstacles", 
                 "move_server_feedback") {}


void MoveServer::move(const tiago_iaslab_simulation::moveScanGoalConstPtr& goal) {
  publishFeedback(status::READY);

  moveActionClient_.waitForServer();

  move_base_msgs::MoveBaseGoal moveGoal;
  moveGoal.target_pose = goal->pose;

  moveActionClient_.sendGoal(moveGoal);
  publishFeedback(status::MOVING);

  bool timeout = moveActionClient_.waitForResult(ros::Duration(300));

  if (!timeout) {
    publishFeedback(status::FAILED);
    actionServer_.setAborted();

  } else {
    if (moveActionClient_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
      publishFeedback(status::ARRIVED);

      if (scannerClient_.exists()) {
        tiago_iaslab_simulation::scanObstacles service;

        publishFeedback(status::SCANNING);

        if (scannerClient_.call(service)) {
          publishFeedback(status::DONE);

          tiago_iaslab_simulation::moveScanResult result;
          result.obstacles = service.response.obstacles;

          actionServer_.setSucceeded(result);

        } else {
          publishFeedback(status::FAILED);
          actionServer_.setAborted();
        }

      } else {
        publishFeedback(status::FAILED);
        actionServer_.setAborted();
      }

    } else {
      publishFeedback(status::NOT_ARRIVED);
      publishFeedback(status::FAILED);
      actionServer_.setAborted();
    }
  }
}

void MoveServer::publishFeedback(const uint status) {
  tiago_iaslab_simulation::moveScanFeedback feedback;
  feedback.current_status = status;

  actionServer_.publishFeedback(feedback);
}

void MoveServer::feedbackRelayCallback(const std_msgs::UInt8ConstPtr& feedback) {
  publishFeedback(feedback->data);
}
