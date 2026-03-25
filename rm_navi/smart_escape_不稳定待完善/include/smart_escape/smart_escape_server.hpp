#ifndef SMART_ESCAPE_SERVER_HPP_
#define SMART_ESCAPE_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "smart_escape/action/smart_escape.hpp"
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mutex>
// 【新增】调试用 Marker 头文件
#include <visualization_msgs/msg/marker.hpp> 

namespace smart_escape
{
using SmartEscape = smart_escape::action::SmartEscape;
using GoalHandleSmartEscape = rclcpp_action::ServerGoalHandle<SmartEscape>;

class SmartEscapeServer : public rclcpp::Node
{
public:
  explicit SmartEscapeServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Action Server
  rclcpp_action::Server<SmartEscape>::SharedPtr action_server_;
  
  // Costmap 订阅
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr current_costmap_;
  std::mutex costmap_mutex_;

  // 速度发布者
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

  // 【新增】Marker 发布者
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  
  // TF 工具
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // 参数变量
  std::string global_frame_;
  std::string robot_base_frame_;
  double robot_radius_;
  double escape_distance_;
  
  // 核心函数声明
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const SmartEscape::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleSmartEscape> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleSmartEscape> goal_handle);

  void execute_escape(const std::shared_ptr<GoalHandleSmartEscape> goal_handle);

  void calculate_escape_velocity(double &vx, double &vy);

  // 【新增】Marker 辅助函数
  void publishMarker(const geometry_msgs::msg::Pose & robot_pose, double angle_global);

  bool getRobotPose(geometry_msgs::msg::Pose & robot_pose);
};

}  // namespace smart_escape

#endif  // SMART_ESCAPE_SERVER_HPP_
