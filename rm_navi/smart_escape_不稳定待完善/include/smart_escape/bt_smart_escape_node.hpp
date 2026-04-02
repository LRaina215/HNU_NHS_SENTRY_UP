#ifndef BT_SMART_ESCAPE_NODE_HPP_
#define BT_SMART_ESCAPE_NODE_HPP_

#include <string>
#include <memory>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "smart_escape/action/smart_escape.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_behavior_tree
{

class BtSmartEscapeNode : public BT::StatefulActionNode
{
public:
  BtSmartEscapeNode(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);
  ~BtSmartEscapeNode() override;

  // 【核心修复】必须显式定义端口，否则 getInput 会导致未定义行为
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("server_name", "smart_escape", "Action server name"),
      BT::InputPort<double>("server_timeout", 5.0, "Server timeout in seconds"), // 统一用 double 秒
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("escape_pose", "Computed escape pose"),
      BT::OutputPort<std::string>("result_message", "Result message")
    };
  }

protected:
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  using SmartEscape = smart_escape::action::SmartEscape;
  using ClientGoalHandle = rclcpp_action::ClientGoalHandle<SmartEscape>;

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<SmartEscape>::SharedPtr action_client_;
  
  std::string server_name_;
  double server_timeout_; // 统一用 double
  
  bool goal_sent_;
  bool result_received_;
  bool goal_accepted_;
  SmartEscape::Result::SharedPtr result_;

  void goalResponseCallback(const ClientGoalHandle::SharedPtr & goal_handle);
  void resultCallback(const ClientGoalHandle::WrappedResult & result);
  void feedbackCallback(ClientGoalHandle::SharedPtr, const std::shared_ptr<const SmartEscape::Feedback> feedback);
  void cleanup();
};

} // namespace nav2_behavior_tree

#endif