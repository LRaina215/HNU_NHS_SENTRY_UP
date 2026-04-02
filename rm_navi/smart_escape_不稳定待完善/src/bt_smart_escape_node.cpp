#include "smart_escape/bt_smart_escape_node.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp_action/rclcpp_action.hpp"

namespace nav2_behavior_tree
{

BtSmartEscapeNode::BtSmartEscapeNode(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(xml_tag_name, conf),
  goal_sent_(false), result_received_(false), goal_accepted_(false)
{
  auto blackboard = config().blackboard;
  node_ = blackboard->get<rclcpp::Node::SharedPtr>("node");

  getInput("server_name", server_name_);
  getInput("server_timeout", server_timeout_);

  action_client_ = rclcpp_action::create_client<SmartEscape>(node_, server_name_);
}

BtSmartEscapeNode::~BtSmartEscapeNode() {}

BT::NodeStatus BtSmartEscapeNode::onStart()
{
  if (!action_client_->action_server_is_ready()) {
    RCLCPP_ERROR(node_->get_logger(), "SmartEscape Action Server not ready");
    return BT::NodeStatus::FAILURE;
  }

  goal_sent_ = false;
  result_received_ = false;
  
  auto goal_msg = SmartEscape::Goal();
  auto send_goal_options = rclcpp_action::Client<SmartEscape>::SendGoalOptions();
  
  send_goal_options.goal_response_callback =
    [this](const ClientGoalHandle::SharedPtr & goal_handle) {
      if (!goal_handle) {
        result_received_ = true;
      }
    };

  send_goal_options.result_callback =
    [this](const ClientGoalHandle::WrappedResult & wrapped_result) {
      result_received_ = true;
      if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        result_ = wrapped_result.result;
      }
    };

  action_client_->async_send_goal(goal_msg, send_goal_options);
  goal_sent_ = true;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus BtSmartEscapeNode::onRunning()
{
  if (result_received_) {
    if (result_ && result_->success) {
      setOutput("escape_pose", result_->escape_pose);
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void BtSmartEscapeNode::onHalted()
{
  if (action_client_ && goal_sent_ && !result_received_) {
    action_client_->async_cancel_all_goals();
  }
}

} // namespace nav2_behavior_tree

// -------------------------------------------------------------------
// 核心修复点：必须放在 namespace 之外，且必须符合 Nav2 插件加载规范
// -------------------------------------------------------------------

#include "behaviortree_cpp_v3/bt_factory.h"

// ROS 2 Nav2 插件导出必须包含这个特定的 C 函数入口
extern "C" __attribute__((visibility("default"))) 
void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory& factory)
{
  factory.registerNodeType<nav2_behavior_tree::BtSmartEscapeNode>("SmartEscape");
}