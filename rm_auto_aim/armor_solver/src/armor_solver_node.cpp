// Copyright Chen Jun 2023. Licensed under the MIT License.
//
// Additional modifications and features by Chengfu Zou, Labor. Licensed under Apache License 2.0.
//
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "armor_solver/armor_solver_node.hpp"

// std
#include <memory>
#include <vector>
#include <mutex>

// project
#include "armor_solver/motion_model.hpp"
#include "rm_utils/common.hpp"
#include "rm_utils/heartbeat.hpp"

namespace fyt::auto_aim {
ArmorSolverNode::ArmorSolverNode(const rclcpp::NodeOptions &options)
: Node("armor_solver", options),
  it_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){})),
  solver_(nullptr){ 
  // Register logger
  FYT_REGISTER_LOGGER("armor_solver", "~/fyt2024-log", INFO);
  FYT_INFO("armor_solver", "Starting ArmorSolverNode!");

  debug_mode_ = this->declare_parameter("debug", true);

  // Tracker
  double max_match_distance = this->declare_parameter("tracker.max_match_distance", 0.2);
  double max_match_yaw_diff = this->declare_parameter("tracker.max_match_yaw_diff", 1.0);
  tracker_ = std::make_unique<Tracker>(max_match_distance, max_match_yaw_diff);
  tracker_->tracking_thres = this->declare_parameter("tracker.tracking_thres", 5);
  lost_time_thres_ = this->declare_parameter("tracker.lost_time_thres", 0.3);
  

  predicted_position_pub_ = this->create_publisher<geometry_msgs::msg::Point>("armor_solver/predicted_position", 10);

  //7.16---
  result_image_sub_ = it_.subscribe("armor_detector/result_img",1,std::bind(&ArmorSolverNode::PreImageCallback, this, std::placeholders::_1));
  vis_predict_image_pub_ =  
                image_transport::create_publisher(this, "armor_solver/pre_aim_img");
  // 12.21
  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "camera_info", rclcpp::SensorDataQoS(),
      std::bind(&ArmorSolverNode::cameraInfoCallback, this, std::placeholders::_1));

  camera_matrix_ = cv::Mat();
  dist_coeffs_ = cv::Mat();

  pre_target_point_pub_ = this->create_publisher<geometry_msgs::msg::Point>("armor_solver/pre_target_debug_point", 10);
  //7.16---

  // EKF
  // xa = x_armor, xc = x_robot_center
  // state: xc, v_xc, yc, v_yc, zc, v_zc, yaw, v_yaw, r, d_zc
  // measurement: p, y, d, yaw
  // f - Process function
  auto f = Predict(0.005);
  // h - Observation function
  auto h = Measure();
  // update_Q - process noise covariance matrix
  s2qx_ = declare_parameter("ekf.sigma2_q_x", 20.0);
  s2qy_ = declare_parameter("ekf.sigma2_q_y", 20.0);
  s2qz_ = declare_parameter("ekf.sigma2_q_z", 20.0);
  s2qyaw_ = declare_parameter("ekf.sigma2_q_yaw", 150.0);
  s2qr_ = declare_parameter("ekf.sigma2_q_r", 800.0);
  s2qd_zc_ = declare_parameter("ekf.sigma2_q_d_zc", 800.0);

  auto u_q = [this]() {
    Eigen::Matrix<double, X_N, X_N> q;
    double t = dt_, x = s2qx_, y = s2qy_, z = s2qz_, yaw = s2qyaw_, r = s2qr_, d_zc=s2qd_zc_;
    (void)x; (void)y; (void)z; (void)yaw; (void)r; (void)d_zc; // 加上这一行消除警告
    double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
    double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * y, q_vy_vy = pow(t, 2) * y;
    // double q_z_z = pow(t, 4) / 4 * x, q_z_vz = pow(t, 3) / 2 * x, q_vz_vz = pow(t, 2) * z;
    double q_z_z = pow(t, 4) / 4 * z, q_z_vz = pow(t, 3) / 2 * z, q_vz_vz = pow(t, 2) * z; 
    
    double q_yaw_yaw = pow(t, 4) / 4 * yaw, q_yaw_vyaw = pow(t, 3) / 2 * x,
           q_vyaw_vyaw = pow(t, 2) * yaw;
    double q_r = pow(t, 4) / 4 * r;
    double q_d_zc = pow(t, 4) / 4 * d_zc;
    // clang-format off
    //    xc      v_xc    yc      v_yc    zc      v_zc    yaw         v_yaw       r       d_za
    q <<  q_x_x,  q_x_vx, 0,      0,      0,      0,      0,          0,          0,      0,
          q_x_vx, q_vx_vx,0,      0,      0,      0,      0,          0,          0,      0,
          0,      0,      q_y_y,  q_y_vy, 0,      0,      0,          0,          0,      0,
          0,      0,      q_y_vy, q_vy_vy,0,      0,      0,          0,          0,      0,
          0,      0,      0,      0,      q_z_z,  q_z_vz, 0,          0,          0,      0,
          0,      0,      0,      0,      q_z_vz, q_vz_vz,0,          0,          0,      0,
          0,      0,      0,      0,      0,      0,      q_yaw_yaw,  q_yaw_vyaw, 0,      0,
          0,      0,      0,      0,      0,      0,      q_yaw_vyaw, q_vyaw_vyaw,0,      0,
          0,      0,      0,      0,      0,      0,      0,          0,          q_r,    0,
          0,      0,      0,      0,      0,      0,      0,          0,          0,      q_d_zc;

    // clang-format on
    return q;
  };
  // update_R - measurement noise covariance matrix
  r_x_ = declare_parameter("ekf.r_x", 0.05);
  r_y_ = declare_parameter("ekf.r_y", 0.05);
  r_z_ = declare_parameter("ekf.r_z", 0.05);
  r_yaw_ = declare_parameter("ekf.r_yaw", 0.02);
  auto u_r = [this](const Eigen::Matrix<double, Z_N, 1> &z) {
    Eigen::Matrix<double, Z_N, Z_N> r;
    // clang-format off
    r << r_x_ * std::abs(z[0]), 0, 0, 0,
         0, r_y_ * std::abs(z[1]), 0, 0,
         0, 0, r_z_ * std::abs(z[2]), 0,
         0, 0, 0, r_yaw_;
    // clang-format on
    return r;
  };
  // P - error estimate covariance matrix
  Eigen::DiagonalMatrix<double, X_N> p0;
  p0.setIdentity();
  tracker_->ekf = std::make_unique<RobotStateEKF>(f, h, u_q, u_r, p0);

  // Subscriber with tf2 message_filter
  // tf2 relevant
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  // Create the timer interface before call to waitForTransform,
  // to avoid a tf2_ros::CreateTimerInterfaceException exception
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(), this->get_node_timers_interface());
  tf2_buffer_->setCreateTimerInterface(timer_interface);
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);
  // subscriber and filter
  armors_sub_.subscribe(this, "armor_detector/armors", rmw_qos_profile_sensor_data);
  target_frame_ = this->declare_parameter("target_frame", "odom");
  tf2_filter_ = std::make_shared<tf2_filter>(armors_sub_,
                                             *tf2_buffer_,
                                             target_frame_,
                                             10,
                                             this->get_node_logging_interface(),
                                             this->get_node_clock_interface(),
                                             std::chrono::duration<int>(1));
  // Register a callback with tf2_ros::MessageFilter to be called when
  // transforms are available
  tf2_filter_->registerCallback(&ArmorSolverNode::armorsCallback, this);

  // Measurement publisher (for debug usage)
  measure_pub_ = this->create_publisher<rm_interfaces::msg::Measurement>("armor_solver/measurement",
                                                                         rclcpp::SensorDataQoS());

  // Publisher
  target_pub_ = this->create_publisher<rm_interfaces::msg::Target>("armor_solver/target",
                                                                   rclcpp::SensorDataQoS());
  gimbal_pub_ = this->create_publisher<rm_interfaces::msg::GimbalCmd>("armor_solver/cmd_gimbal",
                                                                      20);
  // Timer 250 Hz
  pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(4),
                                       std::bind(&ArmorSolverNode::timerCallback, this));
  armor_target_.header.frame_id = "";
  
  // Enable/Disable Armor Solver
  enable_ = true;
  set_mode_srv_ = this->create_service<rm_interfaces::srv::SetMode>(
    "armor_solver/set_mode",
    std::bind(
      &ArmorSolverNode::setModeCallback, this, std::placeholders::_1, std::placeholders::_2));

  if (debug_mode_) {
    initMarkers();
  }

  // Heartbeat
  heartbeat_ = HeartBeatPublisher::create(this);
}

void ArmorSolverNode::timerCallback() {
  if (solver_ == nullptr) {
    return;
  }

  if (!enable_) {
    return;
  }

  // 1. 获取 EKF 的位置不确定度 (仅提取 xc, yc, zc 的方差)
  double pos_uncertainty = 100.0; // 默认给个大值(不自信)
  if (tracker_->tracker_state == Tracker::TRACKING && tracker_->ekf != nullptr) {
      // 状态向量索引: 0=xc, 2=yc, 4=zc
      pos_uncertainty = tracker_->ekf->P_post(0, 0) + 
                        tracker_->ekf->P_post(2, 2) + 
                        tracker_->ekf->P_post(4, 4);

      // FYT_INFO("armor_solver", "Current pos_uncertainty: {:.4f}", pos_uncertainty);
  }

  // Init messagepredicted_position_
  rm_interfaces::msg::GimbalCmd control_msg;

  // If target never detected
  if (armor_target_.header.frame_id.empty()) {
    control_msg.yaw_diff = 0;
    control_msg.pitch_diff = 0;
    control_msg.distance = -1;
    control_msg.pitch = 0;
    control_msg.yaw = 0;
    control_msg.fire_advice = false;
    gimbal_pub_->publish(control_msg);
    return;
  }

  if (armor_target_.tracking) {
    try {
      control_msg = solver_->solve(armor_target_, this->now(), tf2_buffer_, pos_uncertainty);
      //////////////////////////////////////////
      geometry_msgs::msg::Point predicted_position = solver_->getPredictedPosition();
      predicted_position_pub_->publish(predicted_position);
      //7.16---
      {
        // 加锁更新点坐标
        std::lock_guard<std::mutex> lock(point_mutex_);
        camera_plane_point_ = PointConvert(predicted_position);
      }
      //7.16---

    } catch (...) {
        
      
      FYT_ERROR("armor_solver", "Something went wrong in solver!");
      control_msg.yaw_diff = 0;
      control_msg.pitch_diff = 0;
      control_msg.distance = -1;
      control_msg.fire_advice = false;
    }
    
  } else {
    control_msg.yaw_diff = 0;
    control_msg.pitch_diff = 0;
    control_msg.distance = -1;
    // control_msg.fire_advice = false;
    // 目标丢失时重置点坐标
    std::lock_guard<std::mutex> lock(point_mutex_);
    camera_plane_point_ = cv::Point2f(-1, -1);
  }
  // std::cout << "yaw: " << control_msg.yaw << std::endl;
  gimbal_pub_->publish(control_msg);

  if (debug_mode_) {
    publishMarkers(armor_target_, control_msg);
  }
}

// 12.21
void ArmorSolverNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
  // 加锁，防止和预测线程冲突
  std::lock_guard<std::mutex> lock(point_mutex_);

  // 将 ROS 的 array 转换为 OpenCV 的 Mat
  camera_matrix_ = cv::Mat(3, 3, CV_64F, const_cast<double*>(msg->k.data())).clone();
  dist_coeffs_ = cv::Mat(1, 5, CV_64F, const_cast<double*>(msg->d.data())).clone();

  // 如果只需要读取一次，可以在这里销毁订阅者释放资源 (可选)
  cam_info_sub_.reset(); 
  
  // FYT_INFO("armor_solver", "收到相机内参，已自动更新！");
}

void ArmorSolverNode::initMarkers() noexcept {
  // Visualization Marker Publisher
  // See http://wiki.ros.org/rviz/DisplayTypes/Marker
  position_marker_.ns = "position";
  position_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  position_marker_.scale.x = position_marker_.scale.y = position_marker_.scale.z = 0.1;
  position_marker_.color.a = 1.0;
  position_marker_.color.g = 1.0;
  linear_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  linear_v_marker_.ns = "linear_v";
  linear_v_marker_.scale.x = 0.03;
  linear_v_marker_.scale.y = 0.05;
  linear_v_marker_.color.a = 1.0;
  linear_v_marker_.color.r = 1.0;
  linear_v_marker_.color.g = 1.0;
  angular_v_marker_.type = visualization_msgs::msg::Marker::ARROW;
  angular_v_marker_.ns = "angular_v";
  angular_v_marker_.scale.x = 0.03;
  angular_v_marker_.scale.y = 0.05;
  angular_v_marker_.color.a = 1.0;
  angular_v_marker_.color.b = 1.0;
  angular_v_marker_.color.g = 1.0;
  armors_marker_.ns = "filtered_armors";
  armors_marker_.type = visualization_msgs::msg::Marker::CUBE;
  armors_marker_.scale.x = 0.03;
  armors_marker_.scale.z = 0.125;
  armors_marker_.color.a = 1.0;
  armors_marker_.color.b = 1.0;
  selection_marker_.ns = "selection";
  selection_marker_.type = visualization_msgs::msg::Marker::SPHERE;
  selection_marker_.scale.x = selection_marker_.scale.y = selection_marker_.scale.z = 0.1;
  selection_marker_.color.a = 1.0;
  selection_marker_.color.g = 1.0;
  selection_marker_.color.r = 1.0;
  trajectory_marker_.ns = "trajectory";
  trajectory_marker_.type = visualization_msgs::msg::Marker::POINTS;
  trajectory_marker_.scale.x = 0.01;
  trajectory_marker_.scale.y = 0.01;
  trajectory_marker_.color.a = 1.0;
  trajectory_marker_.color.r = 1.0;
  trajectory_marker_.color.g = 0.75;
  trajectory_marker_.color.b = 0.79;
  trajectory_marker_.points.clear();

  marker_pub_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("armor_solver/marker", 10);
}
void ArmorSolverNode::armorsCallback(const rm_interfaces::msg::Armors::SharedPtr armors_msg) {
  // Lazy initialize solver owing to weak_from_this() can't be called in constructor
  if (solver_ == nullptr) {
    solver_ = std::make_unique<Solver>(weak_from_this());
  }

  // Tranform armor position from image frame to world coordinate
  for (auto &armor : armors_msg->armors) {

    // 12.21
    if (std::isnan(armor.pose.position.x) || std::isnan(armor.pose.position.y) || std::isnan(armor.pose.position.z)) {
        FYT_WARN("armor_solver", "Received NaN armor pose from detector! Skipping.");
        continue; 
    }

    geometry_msgs::msg::PoseStamped ps;
    ps.header = armors_msg->header;
    ps.pose = armor.pose;

    // 【核心修复 1】: 强制修改时间戳为 0，防止 TF Extrapolation Exception
    ps.header.stamp = rclcpp::Time(0);

    try {
      armor.pose = tf2_buffer_->transform(ps, target_frame_).pose;
    } catch (const tf2::TransformException &ex) {
      FYT_ERROR("armor_solver", "Transform error: {}", ex.what());
      return;
    }
  }

  // Filter abnormal armors
  armors_msg->armors.erase(std::remove_if(armors_msg->armors.begin(),
                                          armors_msg->armors.end(),
                                          [](const rm_interfaces::msg::Armor &armor) {
                                            return abs(armor.pose.position.z) > 2;
                                          }),
                           armors_msg->armors.end());

  // Init message
  rm_interfaces::msg::Measurement measure_msg;
  rm_interfaces::msg::Target target_msg;
  rclcpp::Time time = armors_msg->header.stamp;
  target_msg.header.stamp = time;
  target_msg.header.frame_id = target_frame_;


  // Update tracker
  if (tracker_->tracker_state == Tracker::LOST) {
    tracker_->init(armors_msg);
    target_msg.tracking = false;
  } else {
    dt_ = (time - last_time_).seconds();
    tracker_->lost_thres = std::abs(static_cast<int>(lost_time_thres_ / dt_));
    if (tracker_->tracked_id == "outpost") {
      tracker_->ekf->setPredictFunc(Predict{dt_, MotionModel::CONSTANT_ROTATION});
    } else {
      tracker_->ekf->setPredictFunc(Predict{dt_, MotionModel::CONSTANT_VEL_ROT});
    }
    tracker_->update(armors_msg);
    // Publish measurement
    measure_msg.x = tracker_->measurement(0);
    measure_msg.y = tracker_->measurement(1);
    measure_msg.z = tracker_->measurement(2);
    measure_msg.yaw = tracker_->measurement(3);
    measure_pub_->publish(measure_msg);

    if (tracker_->tracker_state == Tracker::DETECTING) {
      target_msg.tracking = false;
    } else if (tracker_->tracker_state == Tracker::TRACKING ||
               tracker_->tracker_state == Tracker::TEMP_LOST) {
      target_msg.tracking = true;
      // Fill target message
      const auto &state = tracker_->target_state;
      target_msg.id = tracker_->tracked_id;
      target_msg.armors_num = static_cast<int>(tracker_->tracked_armors_num);
      target_msg.position.x = state(0);
      target_msg.velocity.x = state(1);
      target_msg.position.y = state(2);
      target_msg.velocity.y = state(3);
      target_msg.position.z = state(4);
      target_msg.velocity.z = state(5);
      target_msg.yaw = state(6);
      target_msg.v_yaw = state(7);
      target_msg.radius_1 = state(8);
      target_msg.radius_2 = tracker_->another_r;
      target_msg.d_zc = state(9);
      target_msg.d_za = tracker_->d_za;
      

    }
  }

  // Store and Publish the target_msg
  armor_target_ = target_msg;
  target_pub_->publish(target_msg);

  last_time_ = time;
}

void ArmorSolverNode::publishMarkers(const rm_interfaces::msg::Target &target_msg,
                                     const rm_interfaces::msg::GimbalCmd &gimbal_cmd) noexcept {
  position_marker_.header = target_msg.header;
  linear_v_marker_.header = target_msg.header;
  angular_v_marker_.header = target_msg.header;
  armors_marker_.header = target_msg.header;
  selection_marker_.header = target_msg.header;
  trajectory_marker_.header = target_msg.header;

  visualization_msgs::msg::MarkerArray marker_array;

  if (target_msg.tracking) {
    double yaw = target_msg.yaw, r1 = target_msg.radius_1, r2 = target_msg.radius_2;
    double xc = target_msg.position.x, yc = target_msg.position.y, zc = target_msg.position.z;
    double vx = target_msg.velocity.x, vy = target_msg.velocity.y, vz = target_msg.velocity.z;
    double d_za = target_msg.d_za, d_zc = target_msg.d_zc;
    position_marker_.action = visualization_msgs::msg::Marker::ADD;
    position_marker_.pose.position.x = xc;
    position_marker_.pose.position.y = yc;
    position_marker_.pose.position.z = zc;

    linear_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    linear_v_marker_.points.clear();
    linear_v_marker_.points.emplace_back(position_marker_.pose.position);
    geometry_msgs::msg::Point arrow_end = position_marker_.pose.position;
    arrow_end.x += vx;
    arrow_end.y += vy;
    arrow_end.z += vz;
    linear_v_marker_.points.emplace_back(arrow_end);

    angular_v_marker_.action = visualization_msgs::msg::Marker::ADD;
    angular_v_marker_.points.clear();
    angular_v_marker_.points.emplace_back(position_marker_.pose.position);
    arrow_end = position_marker_.pose.position;
    arrow_end.z += target_msg.v_yaw / M_PI;
    angular_v_marker_.points.emplace_back(arrow_end);

    armors_marker_.action = visualization_msgs::msg::Marker::ADD;
    armors_marker_.scale.y = tracker_->tracked_armor.type == "small" ? 0.135 : 0.23;
    // Draw armors
    bool is_current_pair = true;
    size_t a_n = target_msg.armors_num;
    geometry_msgs::msg::Point p_a;
    double r = 0;
    for (size_t i = 0; i < a_n; i++) {
      double tmp_yaw = yaw + i * (2 * M_PI / a_n);
      // Only 4 armors has 2 radius and height
      if (a_n == 4) {
        r = is_current_pair ? r1 : r2;
        p_a.z = zc + d_zc +  (is_current_pair ? 0 : d_za);
        is_current_pair = !is_current_pair;
      } else {
        r = r1;
        p_a.z = zc;
      }
      p_a.x = xc - r * cos(tmp_yaw);
      p_a.y = yc - r * sin(tmp_yaw);

      armors_marker_.id = i;
      armors_marker_.pose.position = p_a;
      tf2::Quaternion q;
      q.setRPY(0, target_msg.id == "outpost" ? -0.2618 : 0.2618, tmp_yaw);
      armors_marker_.pose.orientation = tf2::toMsg(q);
      marker_array.markers.emplace_back(armors_marker_);
    }

    selection_marker_.action = visualization_msgs::msg::Marker::ADD;
    selection_marker_.points.clear();
    selection_marker_.pose.position.y = gimbal_cmd.distance * sin(gimbal_cmd.yaw * M_PI / 180);
    selection_marker_.pose.position.x = gimbal_cmd.distance * cos(gimbal_cmd.yaw * M_PI / 180);
    selection_marker_.pose.position.z = gimbal_cmd.distance * sin(gimbal_cmd.pitch * M_PI / 180);

    trajectory_marker_.action = visualization_msgs::msg::Marker::ADD;
    trajectory_marker_.points.clear();
    trajectory_marker_.header.frame_id = "gimbal_link";
    for (const auto &point : solver_->getTrajectory()) {
      geometry_msgs::msg::Point p;
      p.x = point.first;
      p.z = point.second;
      trajectory_marker_.points.emplace_back(p);
    }
    if (gimbal_cmd.fire_advice) {
      trajectory_marker_.color.r = 0;
      trajectory_marker_.color.g = 1;
      trajectory_marker_.color.b = 0;
    } else {
      trajectory_marker_.color.r = 1;
      trajectory_marker_.color.g = 1;
      trajectory_marker_.color.b = 1;
    }

  } else {
    position_marker_.action = visualization_msgs::msg::Marker::DELETE;
    linear_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
    angular_v_marker_.action = visualization_msgs::msg::Marker::DELETE;
    armors_marker_.action = visualization_msgs::msg::Marker::DELETE;
    trajectory_marker_.action = visualization_msgs::msg::Marker::DELETE;
    selection_marker_.action = visualization_msgs::msg::Marker::DELETE;
  }

  marker_array.markers.emplace_back(position_marker_);
  marker_array.markers.emplace_back(trajectory_marker_);
  marker_array.markers.emplace_back(linear_v_marker_);
  marker_array.markers.emplace_back(angular_v_marker_);
  marker_array.markers.emplace_back(armors_marker_);
  marker_array.markers.emplace_back(selection_marker_);
  marker_pub_->publish(marker_array);
}

void ArmorSolverNode::setModeCallback(
  const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
  std::shared_ptr<rm_interfaces::srv::SetMode::Response> response) {
  response->success = true;

  VisionMode mode = static_cast<VisionMode>(request->mode);
  std::string mode_name = visionModeToString(mode);
  if (mode_name == "UNKNOWN") {
    FYT_ERROR("armor_solver", "Invalid mode: {}", request->mode);
    return;
  }

  switch (mode) {
    case VisionMode::AUTO_AIM_RED:
    case VisionMode::AUTO_AIM_BLUE: {
      enable_ = true;
      break;
    }
    default: {
      enable_ = false;
      break;
    }
  }

  FYT_WARN("armor_solver", "Set Mode to {}", visionModeToString(mode));
}

//7.16---
cv::Point2f ArmorSolverNode::PointConvert (geometry_msgs::msg::Point odom_3d_point)
{
  // [新增] 安全检查：如果内参还没收到，直接返回无效点
  if (camera_matrix_.empty() || dist_coeffs_.empty()) {
    // 可以在这里打印一次警告，用 static bool 控制频率
    static bool warned = false;
    if (!warned) {
      FYT_WARN("armor_solver", "Waitting for camera_info ... cannot reflect pre-points.");
      warned = true;
    }
    return cv::Point2f(-1, -1);
  }

  try {
  //convert to camera
  geometry_msgs::msg::PointStamped odom_point_stamped;
  odom_point_stamped.header.frame_id = target_frame_; // target_frame_ 应该是 "odom"
  
  // 【核心修复 2】: 同样强制使用 Time(0) 以避免 visualization 时的 TF 报错
  odom_point_stamped.header.stamp = rclcpp::Time(0); 
  
  odom_point_stamped.point = odom_3d_point;

  geometry_msgs::msg::PointStamped camera_3d_point_stamped;

        // 等待并执行从 target_frame_ (odom) 到 "camera_link" 的变换
        // "camera_link" 是相机物理位置的坐标系，OpenCV 投影函数需要相机坐标系下的点
        camera_3d_point_stamped = tf2_buffer_->transform(
            odom_point_stamped,
            "camera_link", // 目标帧：相机坐标系
            tf2::durationFromSec(0.1) // 等待变换的超时时间
        );
        
  // Calculate target distance (camera frame x-axis is depth)
  double distance = camera_3d_point_stamped.point.x;
  
  // Dynamic adjustment based on distance (linear function: adjustment = a * distance + b)
  double a = -0.07; // Slope coefficient
  double b = -0.01; // Intercept
  double dynamic_adjustment = a * distance + b;
  
  std::vector<cv::Point3d> object_points;
  object_points.push_back(cv::Point3d(-camera_3d_point_stamped.point.y,
                                         -camera_3d_point_stamped.point.z + dynamic_adjustment,
                                         camera_3d_point_stamped.point.x)); //NOTE!!! 相机坐标系与OpenCV坐标系不同 

  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F); // 旋转向量 (0,0,0)
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F); // 平移向量 (0,0,0)

  cv::projectPoints(object_points, rvec, tvec, camera_matrix_, dist_coeffs_, image_points);

  //Use for debugging
  geometry_msgs::msg::Point point_msg;
  point_msg.x = image_points[0].x;
  point_msg.y = image_points[0].y;
  point_msg.z = 0;
  pre_target_point_pub_->publish(point_msg);

  return cv::Point2f(static_cast<float>(image_points[0].x),static_cast<float>(image_points[0].y));

  // return cv::Point(image_points[0].x,image_points[0].y);
  }
  catch (const tf2::TransformException &ex) 
  {
    FYT_ERROR("armor_solver", "TF转换失败: {}", ex.what());
    return cv::Point2f(-1, -1);
  } 
  catch (const cv::Exception &ex) 
  {
    FYT_ERROR("armor_solver", "OpenCV错误: {}", ex.what());
    return cv::Point2f(-1, -1);
  }
}

void ArmorSolverNode::PreImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &image)
{
  try
  {
  // Convert to OpenCV Picture
  cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      FYT_ERROR("armor_solver", "cv_bridge异常: %s", e.what());
      return;
    }
    
  cv::Mat image_aim = cv_ptr->image;
  
  // Draw Point
  // 安全获取点坐标
  cv::Point2f point_to_draw;
    {
      std::lock_guard<std::mutex> lock(point_mutex_);
      point_to_draw = camera_plane_point_;
    }
  // 只绘制有效点
  if (point_to_draw.x >= 0 && point_to_draw.y >= 0)
   {
      cv::Point pre_aim_pixel_point(point_to_draw.x,point_to_draw.y);
      // 确保点在图像范围内
      cv::Rect image_rect(0, 0, image_aim.cols, image_aim.rows);
      cv::circle(image_aim,pre_aim_pixel_point,10, cv::Scalar(0, 255, 0), -1);

  if (image_rect.contains(pre_aim_pixel_point))
  {
  }
  else {
        FYT_WARN("armor_solver", "预瞄点超出图像范围: [%f, %f]", 
                 point_to_draw.x, point_to_draw.y);  
        }

    }
    // Publish image
    auto msg = cv_bridge::CvImage(image->header, "bgr8", image_aim).toImageMsg();
    vis_predict_image_pub_.publish(msg);
    //FYT_INFO("armor_solver", "Pre-aim point: [%f, %f]", point_to_draw.x, point_to_draw.y);
  }
  catch (const std::exception &e) 
  {
    FYT_ERROR("armor_solver", "图像处理异常: %s", e.what());
  }  
}
//7.16---

}// namespace fyt::auto_aim

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable
// when its library is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::auto_aim::ArmorSolverNode)