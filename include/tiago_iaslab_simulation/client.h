#ifndef CLIENT_H
#define CLIENT_H

#include <actionlib/client/simple_action_client.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>

#include "tiago_iaslab_simulation/moveScanAction.h"

class Client {
 private:
  std::shared_ptr<ros::NodeHandle> nodeHandle_;

  actionlib::SimpleActionClient<tiago_iaslab_simulation::moveScanAction> actionClient_;

  ros::Publisher visualizer_;

  /// @brief Callback to process the result from the #MoveServer action.
  /// @param state
  /// @param result
  void doneCallback(const actionlib::SimpleClientGoalState& state,
                    const tiago_iaslab_simulation::moveScanResultConstPtr& result);
  /// @brief Callback for disaply the feedback from the #MoveServer action.
  /// @param feedback
  void feedbackCallback(const tiago_iaslab_simulation::moveScanFeedbackConstPtr& feedback);

 public:
  /// @brief
  /// @param nodeHandle
  /// @param moveServerTopic topic for send the goal to #MoveServer action
  /// @param visualizerTopic topic for publish the marker of centers
  // Constructor that takes all parameters
  Client(std::shared_ptr<ros::NodeHandle> nodeHandle,
         std::string moveServerTopic,
         std::string visualizerTopic);

  // Constructor with default topic names
  Client(std::shared_ptr<ros::NodeHandle> nodeHandle);

  void sendPose(float x, float y, float yaw);
};

#endif  // CLIENT_H
