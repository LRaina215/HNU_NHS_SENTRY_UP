#include "smart_escape/smart_escape_server.hpp"
#include <cmath>
#include <vector>
#include <algorithm>
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker.hpp" // 【新增】

namespace smart_escape
{

SmartEscapeServer::SmartEscapeServer(const rclcpp::NodeOptions & options)
: Node("smart_escape_server", options)
{
  // 1. 声明参数
  declare_parameter("global_frame", "map");
  declare_parameter("robot_base_frame", "base_link");
  declare_parameter("robot_radius", 0.35);
  declare_parameter("escape_distance", 0.5);
  
  // 2. 获取参数
  global_frame_ = get_parameter("global_frame").as_string();
  robot_base_frame_ = get_parameter("robot_base_frame").as_string();
  robot_radius_ = get_parameter("robot_radius").as_double();
  escape_distance_ = get_parameter("escape_distance").as_double();
  
  // 3. 初始化 Action Server
  action_server_ = rclcpp_action::create_server<SmartEscape>(
    this,
    "smart_escape",
    std::bind(&SmartEscapeServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&SmartEscapeServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&SmartEscapeServer::handle_accepted, this, std::placeholders::_1)
  );

  // 4. 初始化 Costmap 订阅
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "local_costmap/costmap", 10,
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      current_costmap_ = msg;
    });

  // 5. 初始化速度发布者
  vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  // 6. 【新增】初始化 Marker 发布者
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("smart_escape_marker", 10);
  
  // 7. 初始化 TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  
  RCLCPP_INFO(this->get_logger(), "SmartEscape Server (Omni-Directional) Initialized!");
}

rclcpp_action::GoalResponse SmartEscapeServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const SmartEscape::Goal> goal)
{
  (void)uuid;
  (void)goal;
  RCLCPP_INFO(this->get_logger(), "Received escape request");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SmartEscapeServer::handle_cancel(
  const std::shared_ptr<GoalHandleSmartEscape> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received cancel request");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SmartEscapeServer::handle_accepted(const std::shared_ptr<GoalHandleSmartEscape> goal_handle)
{
  std::thread{std::bind(&SmartEscapeServer::execute_escape, this, std::placeholders::_1), goal_handle}.detach();
}

void SmartEscapeServer::execute_escape(const std::shared_ptr<GoalHandleSmartEscape> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Start executing omni-escape...");
  
  rclcpp::Rate loop_rate(10); // 10Hz
  auto result = std::make_shared<SmartEscape::Result>();
  
  geometry_msgs::msg::Pose initial_pose;
  if (!getRobotPose(initial_pose)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get robot pose");
    goal_handle->abort(result);
    return;
  }

  double target_dist = escape_distance_; 
  double traveled_dist = 0.0;

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      geometry_msgs::msg::Twist stop_cmd;
      vel_pub_->publish(stop_cmd);
      goal_handle->canceled(result);
      return;
    }

    geometry_msgs::msg::Pose current_pose;
    if (getRobotPose(current_pose)) {
      traveled_dist = std::hypot(current_pose.position.x - initial_pose.position.x,
                                 current_pose.position.y - initial_pose.position.y);
    }
    
    if (traveled_dist >= target_dist) {
      RCLCPP_INFO(this->get_logger(), "Escape succeeded! Moved %.2f m", traveled_dist);
      geometry_msgs::msg::Twist stop_cmd;
      vel_pub_->publish(stop_cmd);
      
      result->escape_pose.header.frame_id = global_frame_;
      result->escape_pose.header.stamp = this->now();
      result->escape_pose.pose = current_pose;
      goal_handle->succeed(result);
      return;
    }

    double vx = 0.0, vy = 0.0;
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      if (!current_costmap_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for local costmap...");
        loop_rate.sleep();
        continue;
      }
      calculate_escape_velocity(vx, vy); 
    }

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = -vx;
    cmd_vel.linear.y = -vy;
    cmd_vel.angular.z = 0.0;
    vel_pub_->publish(cmd_vel);

    loop_rate.sleep();
  }
}

void SmartEscapeServer::calculate_escape_velocity(double &vx, double &vy)
{
    double search_radius = 1.0; 
    double speed = 0.5;
    int free_threshold = 10;
    
    if (!current_costmap_) return;

    double resolution = current_costmap_->info.resolution;
    int width = current_costmap_->info.width;
    int height = current_costmap_->info.height;
    int center_x = width / 2;
    int center_y = height / 2;

    bool found = false;
    double best_dir_x = 0.0;
    double best_dir_y = 0.0;

    for (double r = robot_radius_ / resolution; r <= search_radius / resolution; r += 2.0) {
        std::vector<std::pair<double, double>> free_vecs;
        
        for (double theta = 0.0; theta < 2 * M_PI; theta += 0.2) {
            int dx = r * cos(theta);
            int dy = r * sin(theta);
            int mx = center_x + dx;
            int my = center_y + dy;

            if (mx >= 0 && mx < width && my >= 0 && my < height) {
                int idx = my * width + mx;
                int cost = current_costmap_->data[idx];
                if (cost >= 0 && cost < 50) { 
                    free_vecs.push_back({cos(theta), sin(theta)});
                }
            }
        }

        if (free_vecs.size() > (size_t)free_threshold) {
            double sum_x = 0, sum_y = 0;
            for (auto &p : free_vecs) { sum_x += p.first; sum_y += p.second; }
            best_dir_x = sum_x;
            best_dir_y = sum_y;
            found = true;
            break; 
        }
    }

    if (found) {
        double angle_global = atan2(best_dir_y, best_dir_x);
        
        double robot_yaw = 0.0;
        geometry_msgs::msg::Pose current_robot_pose;
        
        if (getRobotPose(current_robot_pose)) {
            double siny_cosp = 2 * (current_robot_pose.orientation.w * current_robot_pose.orientation.z + current_robot_pose.orientation.x * current_robot_pose.orientation.y);
            double cosy_cosp = 1 - 2 * (current_robot_pose.orientation.y * current_robot_pose.orientation.y + current_robot_pose.orientation.z * current_robot_pose.orientation.z);
            robot_yaw = std::atan2(siny_cosp, cosy_cosp);
            
            // 【关键】算完角度和位置后，在这里发布 Marker 箭头
            publishMarker(current_robot_pose, angle_global);
            
        } else {
            vx = 0.0;
            vy = 0.0;
            return;
        }

        double angle_body = angle_global - robot_yaw;

        while (angle_body > M_PI) angle_body -= 2.0 * M_PI;
        while (angle_body < -M_PI) angle_body += 2.0 * M_PI;

        vx = speed * cos(angle_body);
        vy = speed * sin(angle_body);

    } else {
        vx = 0.0;
        vy = 0.0;
    }
}

// 【新增】Marker 发布实现
// 修正版：让箭头反向，指向机器人实际运动的方向
void SmartEscapeServer::publishMarker(const geometry_msgs::msg::Pose & robot_pose, double angle_global) {
    if (!marker_pub_) return;

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = global_frame_;
    marker.header.stamp = this->now();
    marker.ns = "escape_direction";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    // 起点：机器人当前位置
    marker.points.push_back(robot_pose.position);
    
    // 终点：
    // 原来是 angle_global，现在发现反了，所以我们画的时候加一个 PI
    // 或者直接用 angle_body 的逻辑推算也可以，但这里为了简单展示 "实际逃跑方向"，
    // 既然车是往反方向跑的，箭头也画反方向即可。
    double angle_draw = angle_global + M_PI/2; // 匹配你之前减去的 90 度
    
    // 算了，最稳妥的方式是：
    // 之前你的控制逻辑是减去了 PI/2，导致实际上是往“右边”跑（相对于原箭头）
    // 为了让箭头指向车跑的方向，我们直接把终点坐标的计算公式改一下
    
    // 你的实际运动逻辑：
    // vx = -speed * cos(global - yaw - PI/2)
    // vy = -speed * sin(global - yaw - PI/2)
    
    // 这是一个非常绕的变换。
    // 既然你确认“车跑对了，箭头反了”，那最简单的改法就是：
    // 把箭头终点坐标直接“取反”
    
    geometry_msgs::msg::Point p2;
    // 之前是 + 1.0 * ...
    // 现在改成 - 1.0 * ... 让箭头指屁股后面（也就是实际跑的方向）
    p2.x = robot_pose.position.x - 1.0 * cos(angle_global);
    p2.y = robot_pose.position.y - 1.0 * sin(angle_global);
    p2.z = robot_pose.position.z;
    marker.points.push_back(p2);

    marker.scale.x = 0.05; 
    marker.scale.y = 0.1;  
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = 1.0; 
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    
    marker_pub_->publish(marker);
}

bool SmartEscapeServer::getRobotPose(geometry_msgs::msg::Pose & robot_pose)
{
  try {
    geometry_msgs::msg::TransformStamped t;
    t = tf_buffer_->lookupTransform(global_frame_, robot_base_frame_, tf2::TimePointZero);
    robot_pose.position.x = t.transform.translation.x;
    robot_pose.position.y = t.transform.translation.y;
    robot_pose.position.z = t.transform.translation.z;
    robot_pose.orientation = t.transform.rotation;
    return true;
  } catch (tf2::TransformException & ex) {
    return false;
  }
}

} // namespace smart_escape
