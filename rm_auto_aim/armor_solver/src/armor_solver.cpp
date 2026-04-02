// Created by Chengfu Zou
// Maintained by Chengfu Zou, Labor
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

#include "armor_solver/armor_solver.hpp"
// std
#include <cmath>
#include <cstddef>
#include <stdexcept>
// project
#include "armor_solver/armor_solver_node.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/math/utils.hpp"

const double MANUAL_Z_CONST = std::tan((1.0/45.0) * M_PI); // 约 0.06992

namespace fyt::auto_aim {


// 线性插值函数
inline double linearInterpolation(const double x, const std::vector<std::pair<double, double>>& points) {
  if (points.empty()) {
    return 0.0;
  }
  
  // 找到x所在的区间
  size_t i = 0;
  while (i < points.size() - 1 && points[i + 1].first < x) {
    ++i;
  }
  
  // 处理边界情况
  if (i == 0) {
    return points[0].second;
  }
  if (i == points.size() - 1) {
    return points.back().second;
  }
  
  // 线性插值
  const auto& p1 = points[i];
  const auto& p2 = points[i + 1];
  double t = (x - p1.first) / (p2.first - p1.first);
  return p1.second * (1 - t) + p2.second * t;
}
Solver::Solver(std::weak_ptr<rclcpp::Node> n) : node_(n) {
  auto node = node_.lock();
  
  shooting_range_w_ = node->declare_parameter("solver.shooting_range_width", 0.335);
  shooting_range_h_ = node->declare_parameter("solver.shooting_range_height", 5.935);
  max_tracking_v_yaw_ = node->declare_parameter("solver.max_tracking_v_yaw", 6.0);
  prediction_delay_ = node->declare_parameter("solver.prediction_delay", 0.009);
  controller_delay_ = node->declare_parameter("solver.controller_delay", 0.002);
  side_angle_ = node->declare_parameter("solver.side_angle", 15.0);
  min_switching_v_yaw_ = node->declare_parameter("solver.min_switching_v_yaw", 1.0);

  anti_spin_bias_ = node->declare_parameter("solver.anti_spin_bias", 0.3);
  spin_judge_low_thres_ = node->declare_parameter("solver.spin_judge_low_thres", 1.0);
  fire_permit_uncertainty_ = node->declare_parameter("solver.fire_permit_uncertainty", 0.75);

  std::string compenstator_type = node->declare_parameter("solver.compensator_type", "ideal");
  trajectory_compensator_ = CompensatorFactory::createCompensator(compenstator_type);
  trajectory_compensator_->iteration_times = node->declare_parameter("solver.iteration_times", 20);
  trajectory_compensator_->velocity = node->declare_parameter("solver.bullet_speed", 20.0);
  trajectory_compensator_->gravity = node->declare_parameter("solver.gravity", 9.8);
  trajectory_compensator_->resistance = node->declare_parameter("solver.resistance", 0.001);

  // 插值法pitch补偿参数初始化
  // 定义距离-角度补偿点对（基于发射器高度0.45米，靶子高度0.15米，实际测量）
  // 整体补偿上升0.10米，每米一个点
  pitch_compensation_points_ = {
    {0.0, -0.06},    // 0米
    {1.0, -0.06},    // 1米
    {2.0, -0.05},    // 2米
    {3.0, -0.05},    // 3米
    {4.0, -0.06},    // 4米
    {4.5, -0.01},  
    {5.0, -0.01},    // 5米
    {5.5, 0.00},
    {6.0, -0.010},
    {6.5, 0.060},    // 6米
    {7.0, 0.055},    // 7米
    {8.0, 0.040},    // 8米
    {9.0, 0.035},    // 9米
    {10.0, 0.040},   // 10米
    {11.0, -0.04},   // 11米
    {12.0, -0.05},   // 12米

  };

  manual_compensator_ = std::make_unique<ManualCompensator>();
  auto angle_offset = node->declare_parameter("solver.angle_offset", std::vector<std::string>{});
  if(!manual_compensator_->updateMapFlow(angle_offset)) {
    FYT_WARN("armor_solver", "Manual compensator update failed!");
  }

  state = State::TRACKING_ARMOR;
  overflow_count_ = 0;
  transfer_thresh_ = 5;

  node.reset();
}

// 0202 TJU
// rm_interfaces::msg::GimbalCmd Solver::solve(const rm_interfaces::msg::Target &target,
//                                          const rclcpp::Time &current_time,
//                                          std::shared_ptr<tf2_ros::Buffer> tf2_buffer_, 
//                                          double pos_uncertainty) {
//   // --- [第 1 步: 获取参数和云台当前状态] ---
//   try {
//     auto node = node_.lock();
//     max_tracking_v_yaw_ = node->get_parameter("solver.max_tracking_v_yaw").as_double();
//     prediction_delay_ = node->get_parameter("solver.prediction_delay").as_double();
//     controller_delay_ = node->get_parameter("solver.controller_delay").as_double();
//     side_angle_ = node->get_parameter("solver.side_angle").as_double();
//     min_switching_v_yaw_ = node->get_parameter("solver.min_switching_v_yaw").as_double();
//     // 插值法pitch补偿参数无需实时更新
//     node.reset();
//   } catch (const std::runtime_error &e) {
//     FYT_ERROR("armor_solver", "{}", e.what());
//   }

//   // 获取云台的 TF 变换 (从 odom 到 gimbal_link)
//   geometry_msgs::msg::TransformStamped gimbal_tf;
//   try {
//     gimbal_tf =
//       tf2_buffer_->lookupTransform(target.header.frame_id, "gimbal_link", tf2::TimePointZero);
//   } catch (tf2::TransformException &ex) {
//     FYT_ERROR("armor_solver", "{}", ex.what());
//     throw ex;
//   }
  
//   // --- [第 2 步: 关键修正 - 提取云台在 odom 中的位置] ---
//   auto gimbal_translation = gimbal_tf.transform.translation;
//   Eigen::Vector3d gimbal_pos_odom(
//     gimbal_translation.x,
//     gimbal_translation.y,
//     gimbal_translation.z
//   );

//   // 从 TF 中获取云台当前的 RPY (Roll, Pitch, Yaw)
//   auto msg_q = gimbal_tf.transform.rotation;
//   tf2::Quaternion tf_q;
//   tf2::fromMsg(msg_q, tf_q);
//   tf2::Matrix3x3(tf_q).getRPY(rpy_[0], rpy_[1], rpy_[2]);
//   rpy_[1] = -rpy_[1]; // 你的自定义约定


//   // --- [第 3 步: 迭代求解 1 - 主迭代循环] ---
  
//   // a. 获取 EKF 状态 (在 odom 坐标系)
//   Eigen::Vector3d base_center_pos(target.position.x, target.position.y, target.position.z);
//   Eigen::Vector3d base_center_vel(target.velocity.x, target.velocity.y, target.velocity.z);
//   double base_center_yaw = target.yaw;
//   double base_center_v_yaw = target.v_yaw;

//   // b. 计算基础延迟
//   double base_dt =
//     (current_time - rclcpp::Time(target.header.stamp)).seconds() + prediction_delay_;

//   // c. 迭代求解
//   int num_iterations = 5;
//   double flying_time_guess = 0.0;
//   Eigen::Vector3d chosen_armor_position; // 我们要求的最终目标点 (odom 系)
//   Eigen::Vector3d predicted_center_odom; // 迭代后的中心点 (odom 系)
  
//   for (int i = 0; i < num_iterations; ++i) {
//     double total_dt = base_dt + flying_time_guess;

//     predicted_center_odom = base_center_pos + total_dt * base_center_vel;
//     double predicted_yaw_odom = base_center_yaw + total_dt * base_center_v_yaw;

//     // `selectBestArmor` 需要车辆中心相对于【云台】的向量
//     Eigen::Vector3d relative_center_vec = predicted_center_odom - gimbal_pos_odom;

//     std::vector<Eigen::Vector3d> armor_positions_odom = getArmorPositions(
//         predicted_center_odom, predicted_yaw_odom, target.radius_1, target.radius_2, 
//         target.d_zc, target.d_za, target.armors_num);
        
//     int idx = selectBestArmor(
//         armor_positions_odom, relative_center_vec, predicted_yaw_odom,
//         target.v_yaw, target.armors_num);
        
//     chosen_armor_position = armor_positions_odom.at(idx);

//     // 计算从【云台】到【未来目标】的向量
//     Eigen::Vector3d target_vec = chosen_armor_position - gimbal_pos_odom;

//     // 检查到【云台】的距离
//     if (target_vec.norm() < 0.1) {
//        throw std::runtime_error("No valid armor to shoot (in iteration)");
//     }

//     // [核心] 用这个向量计算新的、更准的飞行时间
//     flying_time_guess = trajectory_compensator_->getFlyingTime(target_vec);
//   }
  
//   // --- [第 4 步: 计算 Yaw/Pitch] ---
//   Eigen::Vector3d target_vec_odom = chosen_armor_position - gimbal_pos_odom;
//   double yaw, pitch;
//   calcYawAndPitch(target_vec_odom, rpy_, yaw, pitch);
//   double distance = target_vec_odom.norm();

//   // 更新调试用的预测点
//   predicted_position_.x = chosen_armor_position.x();
//   predicted_position_.y = chosen_armor_position.y();
//   predicted_position_.z = chosen_armor_position.z();
  
//   // --- [第 5 步: 填充指令和状态机] ---
//   rm_interfaces::msg::GimbalCmd gimbal_cmd;
//   double pos_uncertainty = tracker_->ekf->P_post.block<3,3>(0,0).trace();
//   gimbal_cmd.header = target.header;
//   gimbal_cmd.distance = distance;
//   // gimbal_cmd.fire_advice = isOnTarget(rpy_[2], rpy_[1], yaw, pitch, distance);
//   gimbal_cmd.fire_advice = isOnTarget(rpy_[2], rpy_[1], yaw, pitch, distance, pos_uncertainty);


//   switch (state) {
//     case TRACKING_ARMOR: {
//       if (std::abs(target.v_yaw) > max_tracking_v_yaw_) {
//         overflow_count_++;
//       } else {
//         overflow_count_ = 0;
//       }

//       if (overflow_count_ > transfer_thresh_) {
//         state = TRACKING_CENTER;
//       }

//       // --- [第 6 步: 你的 `controller_delay_` 逻辑 (非迭代)] ---
//       //
//       // 遵从你的要求，这里不使用迭代，
//       // 而是使用“单次预测”逻辑，但应用了坐标系修正
//       //
//       if (controller_delay_ != 0) {
        
//         // a. 使用主迭代的 flying_time_guess 作为“最好的猜测”
//         double total_dt_delayed = base_dt + controller_delay_ + flying_time_guess;

//         // b. 执行【一次】预测
//         Eigen::Vector3d predicted_center_delayed = base_center_pos + total_dt_delayed * base_center_vel;
//         double predicted_yaw_delayed = base_center_yaw + total_dt_delayed * base_center_v_yaw;

//         // c. [修正] 传递相对向量给 selectBestArmor
//         Eigen::Vector3d relative_center_delayed = predicted_center_delayed - gimbal_pos_odom;

//         std::vector<Eigen::Vector3d> armor_positions_delayed = getArmorPositions(
//             predicted_center_delayed, predicted_yaw_delayed, target.radius_1, target.radius_2, 
//             target.d_zc, target.d_za, target.armors_num);
        
//         int idx_delayed = selectBestArmor(
//             armor_positions_delayed, relative_center_delayed, predicted_yaw_delayed, 
//             target.v_yaw, target.armors_num);
            
//         Eigen::Vector3d chosen_armor_pos_delayed = armor_positions_delayed.at(idx_delayed);

//         // d. [覆盖] 覆盖 yaw, pitch, distance
//         Eigen::Vector3d delayed_target_vec = chosen_armor_pos_delayed - gimbal_pos_odom;
//         gimbal_cmd.distance = delayed_target_vec.norm(); // 覆盖
//         calcYawAndPitch(delayed_target_vec, rpy_, yaw, pitch); // 覆盖 yaw 和 pitch
        
//         // e. [覆盖] 覆盖调试用的预测点
//         predicted_position_.x = chosen_armor_pos_delayed.x();
//         predicted_position_.y = chosen_armor_pos_delayed.y();
//         predicted_position_.z = chosen_armor_pos_delayed.z();
        
//         // f. [覆盖] 覆盖最终补偿要用的 `chosen_armor_position`
//         chosen_armor_position = chosen_armor_pos_delayed;
//       }
//       break;
//     }
//     case TRACKING_CENTER: {
//       if (std::abs(target.v_yaw) < max_tracking_v_yaw_) {
//          overflow_count_++;
//       } else {
//         overflow_count_ = 0;
//       }

//       if (overflow_count_ > transfer_thresh_) {
//         state = TRACKING_ARMOR;
//         overflow_count_ = 0;
//       }
//       gimbal_cmd.fire_advice = true;
      
//       // [修正] 瞄准中心时，也必须使用相对向量
//       Eigen::Vector3d center_vec_odom = predicted_center_odom - gimbal_pos_odom;
//       calcYawAndPitch(center_vec_odom, rpy_, yaw, pitch);
//       break;
//     }
//   }

//   // --- [第 7 步: 最终补偿和发送] ---  
//   // [修正] `angleHardCorrect` 也应该使用相对距离
//   double final_target_dist_xy = (chosen_armor_position - gimbal_pos_odom).head(2).norm();
//   double final_target_dist_z = (chosen_armor_position - gimbal_pos_odom).z();
//   double final_target_dist = (chosen_armor_position - gimbal_pos_odom).norm();
  
//   auto angle_offset = manual_compensator_->angleHardCorrect(final_target_dist_xy, final_target_dist_z);
//   double pitch_offset = angle_offset[0] * M_PI / 180;
//   double yaw_offset = angle_offset[1] * M_PI / 180;
  
//   // 基于距离的动态pitch调整
//   // 使用线性插值法
//   double dynamic_pitch_adjust = linearInterpolation(final_target_dist, pitch_compensation_points_);
  
//   double cmd_pitch = pitch + pitch_offset + dynamic_pitch_adjust;
//   double cmd_yaw = angles::normalize_angle(yaw + yaw_offset);
//   // 0820 (已修正)  
//   predicted_position_.y += -std::tan(yaw_offset) * predicted_position_.x;
//   predicted_position_.z -= (std::tan(pitch_offset) + MANUAL_Z_CONST) * final_target_dist_xy;
  
//   gimbal_cmd.yaw = cmd_yaw * 180 / M_PI;
//   gimbal_cmd.pitch = cmd_pitch * 180 / M_PI - 4.0;
//   gimbal_cmd.yaw_diff = (cmd_yaw - rpy_[2]) * 180 / M_PI;
//   gimbal_cmd.pitch_diff = (cmd_pitch - rpy_[1]) * 180 / M_PI;
  
//  if (gimbal_cmd.fire_advice) {
//   FYT_DEBUG("armor_solver", "You Need Fire!");
//  }
//  return gimbal_cmd;
// }

rm_interfaces::msg::GimbalCmd Solver::solve(const rm_interfaces::msg::Target &target,
                                         const rclcpp::Time &current_time,
                                         std::shared_ptr<tf2_ros::Buffer> tf2_buffer_, 
                                         double pos_uncertainty) {
  // 1. 参数更新 & TF 获取 (保持原样)
  // --- [第 1 步: 获取参数和云台当前状态] ---
  try {
    auto node = node_.lock();
    max_tracking_v_yaw_ = node->get_parameter("solver.max_tracking_v_yaw").as_double();
    prediction_delay_ = node->get_parameter("solver.prediction_delay").as_double();
    controller_delay_ = node->get_parameter("solver.controller_delay").as_double();
    side_angle_ = node->get_parameter("solver.side_angle").as_double();
    min_switching_v_yaw_ = node->get_parameter("solver.min_switching_v_yaw").as_double();
    anti_spin_bias_ = node->get_parameter("solver.anti_spin_bias").as_double();
    spin_judge_low_thres_ = node->get_parameter("solver.spin_judge_low_thres").as_double();
    fire_permit_uncertainty_ = node->get_parameter("solver.fire_permit_uncertainty").as_double();
    trajectory_compensator_->resistance = node->get_parameter("solver.resistance").as_double();
    // 插值法pitch补偿参数无需实时更新
    node.reset();
  } catch (const std::runtime_error &e) {
    FYT_ERROR("armor_solver", "{}", e.what());
  }
  
  // 获取云台 TF (保持原样)
  geometry_msgs::msg::TransformStamped gimbal_tf;
  try {
    gimbal_tf = tf2_buffer_->lookupTransform(target.header.frame_id, "gimbal_link", tf2::TimePointZero);
  } catch (tf2::TransformException &ex) {
    FYT_ERROR("armor_solver", "{}", ex.what());
    throw ex;
  }
  
  Eigen::Vector3d gimbal_pos_odom(gimbal_tf.transform.translation.x,
                                  gimbal_tf.transform.translation.y,
                                  gimbal_tf.transform.translation.z);
  
  auto msg_q = gimbal_tf.transform.rotation;
  tf2::Quaternion tf_q;
  tf2::fromMsg(msg_q, tf_q);
  tf2::Matrix3x3(tf_q).getRPY(rpy_[0], rpy_[1], rpy_[2]);
  rpy_[1] = -rpy_[1]; 

  // --- [优化核心] 迭代求解 ---

  // A. 准备数据
  Eigen::Vector3d base_pos(target.position.x, target.position.y, target.position.z);
  Eigen::Vector3d base_vel(target.velocity.x, target.velocity.y, target.velocity.z);
  double base_yaw = target.yaw;
  double v_yaw = target.v_yaw;

  // 0314
  // --- 【修改点1】状态机前置与双阈值防抖 ---
  // 建议在你的 yaml 参数文件里，把 max_tracking_v_yaw 设为 3.0，min_switching_v_yaw 设为 1.5
  if (state == TRACKING_ARMOR) {
      if (std::abs(v_yaw) > max_tracking_v_yaw_) overflow_count_++;
      else overflow_count_ = 0;
      
      if (overflow_count_ > transfer_thresh_) {
          state = TRACKING_CENTER;
          overflow_count_ = 0;
          // 加上这句高亮打印，确认是否真的触发了反陀螺！
          FYT_INFO("armor_solver", ">>>>> TRIGGER ANTI-SPIN (TRACKING_CENTER) <<<<<");
      }
  } else { // TRACKING_CENTER
      // 退出反陀螺用较小的 min_switching_v_yaw_ 阈值
      if (std::abs(v_yaw) < min_switching_v_yaw_) overflow_count_++;
      else overflow_count_ = 0;
      
      if (overflow_count_ > transfer_thresh_) {
          state = TRACKING_ARMOR;
          overflow_count_ = 0;
          FYT_INFO("armor_solver", "<<<<< EXIT ANTI-SPIN (TRACKING_ARMOR) >>>>>");
      }
  }

  // B. 合并所有固定延迟 (预测延迟 + 控制延迟)
  // 这样迭代算出来的就是最终包含控制延迟的精确解，不需要后面再修补
  double total_fixed_delay = 
      (current_time - rclcpp::Time(target.header.stamp)).seconds() + prediction_delay_ + controller_delay_;

  // C. 迭代变量
  double fly_time = 0.0;
  Eigen::Vector3d final_aim_pos_odom;
  // 0314
  // --- 【新增】把变量声明提到循环外，让后面的代码也能用 ---
  Eigen::Vector3d pred_center; 
  double pred_yaw = 0.0;
  
  // D. 开始迭代 (增加收敛退出)
  for (int i = 0; i < 8; ++i) { // 最多8次，通常2-3次就收敛
    double total_time = total_fixed_delay + fly_time;

    // 预测状态
    // Eigen::Vector3d pred_center = base_pos + base_vel * total_time;
    // double pred_yaw = base_yaw + v_yaw * total_time;
    // 0314
    // --- 【修改】这里去掉类型声明，直接赋值 ---
    pred_center = base_pos + base_vel * total_time;
    pred_yaw = base_yaw + v_yaw * total_time;


    // 获取装甲板位置列表 (使用预测后的中心和Yaw)
    std::vector<Eigen::Vector3d> armor_pos_list = getArmorPositions(
        pred_center, pred_yaw, target.radius_1, target.radius_2, 
        target.d_zc, target.d_za, target.armors_num);
    
    // 选板 (注意：现在 selectBestArmor 是纯选择器，直接传 pred_yaw 即可)
    // 我们还需要算出相对于云台的 yaw，因为 selectBestArmor 里的归一化是假设云台 yaw=0
    // 但其实只要 pred_yaw 是 Odom 下的，我们在 selectBestArmor 里减去 rpy_[2] 就可以对齐
    // 或者简单点：selectBestArmor 里计算的是 diff，只要找到离“当前朝向”最近的就行
    // 这里我们传入 pred_yaw，函数内部会找最接近 0 (即 x 轴) 的角度
    // *重要*：为了让 selectBestArmor 正常工作，我们需要把 pred_yaw 转到云台系吗？
    // 你的原代码逻辑似乎隐含了 target.yaw 是在 Odom 系。
    // 既然 selectBestArmor 里没有减去 gimbal_yaw，那它其实是在选“世界坐标系下 Yaw 接近 0 的板”。
    // 这可能是不对的！应该选“世界坐标系下 Yaw 接近 云台Yaw 的板”。
    
    // 修正选板逻辑传入参数：传入 (pred_yaw - rpy_[2]) 作为 target_yaw
    // 这样 0 度就代表枪口方向
    // int idx = selectBestArmor(armor_pos_list, pred_yaw - rpy_[2], v_yaw, target.armors_num);
    
    // final_aim_pos_odom = armor_pos_list[idx];

    // // 状态机强制修正 (Anti-Spin)
    // if (state == TRACKING_CENTER) {
    //     // 如果是打中心模式，强行瞄准中心
    //     // 0314
    //     final_aim_pos_odom = pred_center; 
    // }
    // 0314
    // 选板 (注意：现在 selectBestArmor 是纯选择器...)
    int idx = selectBestArmor(armor_pos_list, pred_yaw - rpy_[2], v_yaw, target.armors_num);
    final_aim_pos_odom = armor_pos_list[idx];

    // 状态机强制修正 (Anti-Spin)
    if (state == TRACKING_CENTER) {
        // --- 【修改点2】更严谨的空间“守株待兔” ---
        // 只在 XY 平面上计算指向云台的向量，避免 Z 轴俯仰带来的距离误差
        Eigen::Vector2d center_vec_xy(pred_center.x() - gimbal_pos_odom.x(), 
                                      pred_center.y() - gimbal_pos_odom.y());
        Eigen::Vector2d center_dir_xy = center_vec_xy.normalized();
        
        // double r = target.radius_1; 
        
        // // 瞄准点：XY往云台方向拉回半径r，Z轴加上装甲板的固有高度 d_zc
        // final_aim_pos_odom.x() = pred_center.x() - center_dir_xy.x() * r; 
        // final_aim_pos_odom.y() = pred_center.y() - center_dir_xy.y() * r;
        // final_aim_pos_odom.z() = pred_center.z() + target.d_zc; 

        // 【改进点 6】增加一个“切向偏移”补偿
        // 如果目标在快速移动，云台需要稍微往移动方向的“前方”偏一点点
        double r = target.radius_1; 
        final_aim_pos_odom.x() = pred_center.x() - center_dir_xy.x() * r; 
        final_aim_pos_odom.y() = pred_center.y() - center_dir_xy.y() * r;
        final_aim_pos_odom.z() = pred_center.z() + target.d_zc; 
      }

    // 计算飞行时间
    Eigen::Vector3d target_vec = final_aim_pos_odom - gimbal_pos_odom;
    double new_fly_time = trajectory_compensator_->getFlyingTime(target_vec);

    // 收敛检查
    if (std::abs(new_fly_time - fly_time) < 1e-4) {
        fly_time = new_fly_time;
        break;
    }
    fly_time = new_fly_time;
  }

  // --- [后续处理] ---
  
  // 1. 转态机逻辑 (判定是否切中心)
  if (state == TRACKING_ARMOR) {
      if (std::abs(v_yaw) > max_tracking_v_yaw_) overflow_count_++;
      else overflow_count_ = 0;
      if (overflow_count_ > transfer_thresh_) state = TRACKING_CENTER;
  } else { // TRACKING_CENTER
      if (std::abs(v_yaw) < max_tracking_v_yaw_) overflow_count_++;
      else overflow_count_ = 0;
      if (overflow_count_ > transfer_thresh_) state = TRACKING_ARMOR;
  }

  // 2. 解算最终 Yaw/Pitch
  Eigen::Vector3d aim_vec = final_aim_pos_odom - gimbal_pos_odom;
  double yaw, pitch;
  calcYawAndPitch(aim_vec, rpy_, yaw, pitch);
  
  // 3. 手动补偿 (Angle Offset)
  double dist_xy = aim_vec.head(2).norm();
  double dist_z = aim_vec.z();
  auto offset = manual_compensator_->angleHardCorrect(dist_xy, dist_z);
  double pitch_offset = offset[0] * M_PI / 180.0;
  double yaw_offset = offset[1] * M_PI / 180.0;
  
  // 4. 距离动态补偿
  double dist = aim_vec.norm();
  // double dynamic_pitch = linearInterpolation(dist, pitch_compensation_points_);
  double dynamic_pitch = 0;

  // 5. 填充指令
  rm_interfaces::msg::GimbalCmd gimbal_cmd;
  gimbal_cmd.header = target.header;
  gimbal_cmd.distance = dist;
  
  double cmd_pitch = pitch + pitch_offset + dynamic_pitch;
  double cmd_yaw = angles::normalize_angle(yaw + yaw_offset);

  gimbal_cmd.yaw = cmd_yaw * 180 / M_PI - 0.8; // +0.3
  gimbal_cmd.pitch = cmd_pitch * 180 / M_PI + 0.1; // -3.7
  gimbal_cmd.yaw_diff = (cmd_yaw - rpy_[2]) * 180 / M_PI;
  gimbal_cmd.pitch_diff = (cmd_pitch - rpy_[1]) * 180 / M_PI;
  
  // 0314
  // // --- 【修改点】计算最终下发给云台的真实弧度目标 ---
  double final_target_yaw_rad = cmd_yaw + (0.3 * M_PI / 180.0);
  double final_target_pitch_rad = cmd_pitch - (0.5 * M_PI / 180.0);

  // // 6. 开火许可 (使用上一轮讨论的 pos_uncertainty)
  gimbal_cmd.fire_advice = isOnTarget(rpy_[2], rpy_[1], yaw, pitch, dist, pos_uncertainty);
  
  // // 强制开火 (打中心时)
  // if (state == TRACKING_CENTER) gimbal_cmd.fire_advice = true;
  // // 0314
  // // 6. 开火许可 (包含相位校验)
  // --- 【修改点】将补偿后的 final_target 传入开火判定，替换掉原来的 raw yaw 和 raw pitch ---
  bool base_fire_advice = isOnTarget(rpy_[2], rpy_[1], final_target_yaw_rad, final_target_pitch_rad, dist, pos_uncertainty);
  // bool base_fire_advice = true;

  bool phase_matched = true;
  if (state == TRACKING_CENTER) {
      // --- 方案3 核心2：时间上的相位约束 ---
      // 1. 计算云台视线的反方向（也就是装甲板法线需要对齐的理想方向）
      double line_of_sight_yaw = std::atan2(gimbal_pos_odom.y() - pred_center.y(), 
                                            gimbal_pos_odom.x() - pred_center.x());
      
      // 2. 检查预测命中时 (pred_yaw)，离视线最近的装甲板夹角
      double min_angle_diff = M_PI;
      for (int i = 0; i < target.armors_num; ++i) {
          // pred_yaw 已经叠加了 v_yaw * fly_time，代表子弹到达时的真实朝向
          double armor_yaw = pred_yaw + i * (2 * M_PI / target.armors_num);
          double diff = std::abs(angles::shortest_angular_distance(armor_yaw, line_of_sight_yaw));
          if (diff < min_angle_diff) {
              min_angle_diff = diff;
          }
      }
      
      // 3. 设定相位开火窗口：命中时，装甲板法线与视线夹角小于 15度 (0.26弧度) 才允许开火
      // 如果转速极快，可以通过调大这个阈值来放宽开火条件
      double phase_thresh = 22 * M_PI / 180.0;
      phase_matched = (min_angle_diff < phase_thresh);
  }

  // 必须同时满足准星对齐、EKF自信、且装甲板转到位，才能开火
  gimbal_cmd.fire_advice = base_fire_advice && phase_matched;

  // 7. 更新调试信息
  predicted_position_.x = final_aim_pos_odom.x();
  predicted_position_.y = final_aim_pos_odom.y();
  predicted_position_.z = final_aim_pos_odom.z();
  // 简单的可视化修正
  predicted_position_.y += -std::tan(yaw_offset) * predicted_position_.x;
  predicted_position_.z -= (std::tan(pitch_offset) + MANUAL_Z_CONST) * dist_xy;

  return gimbal_cmd;
}

// 0202 TJU
// bool Solver::isOnTarget(const double cur_yaw,
//                         const double cur_pitch,
//                         const double target_yaw,
//                         const double target_pitch,
//                         const double distance,) const noexcept {
//   // Judge whether to shoot
//   double shooting_range_yaw = std::abs(atan2(shooting_range_w_ / 2, distance));
//   double shooting_range_pitch = std::abs(atan2(shooting_range_h_ / 2, distance));
//   // Limit the shooting area to 1 degree to avoid not shooting when distance is
//   // too large
//   shooting_range_yaw = std::max(shooting_range_yaw, 1.0 * M_PI / 180);
//   shooting_range_pitch = std::max(shooting_range_pitch, 1.0 * M_PI / 180);
//   if (std::abs(cur_yaw - target_yaw) < shooting_range_yaw &&
//       std::abs(cur_pitch - target_pitch) < shooting_range_pitch) {
//     return true;
//   }

//   return false;
// }
bool Solver::isOnTarget(const double cur_yaw,
                        const double cur_pitch,
                        const double target_yaw,
                        const double target_pitch,
                        const double distance,
                        double pos_uncertainty) const noexcept
{
  // 1. 计算原本的射击范围 (保留你原有的逻辑)
  double shooting_range_yaw = std::abs(atan2(shooting_range_w_ / 2, distance));
  double shooting_range_pitch = std::abs(atan2(shooting_range_h_ / 2, distance));

  // 限制最小射击范围 (保留原逻辑)
  shooting_range_yaw = std::max(shooting_range_yaw, 1.0 * M_PI / 180);
  shooting_range_pitch = std::max(shooting_range_pitch, 1.0 * M_PI / 180);

  // 2. 判断准星是否对齐 (Aim Ready?)
  bool is_aimed = (std::abs(cur_yaw - target_yaw) < shooting_range_yaw) &&
                  (std::abs(cur_pitch - target_pitch) < shooting_range_pitch);
  // is_aimed = true;

  // 3. [新增] 判断 EKF 是否收敛 (Confident?)
  // 阈值设定：
  // 协方差矩阵对角线之和代表位置方差 (单位 m^2)。
  // 0.20 意味着标准差约 0.44m (比较宽松)
  // 0.05 意味着标准差约 0.22m (比较严格)
  // 建议哨兵先设 0.2，如果发现乱开火再调小
  bool is_confident = pos_uncertainty < fire_permit_uncertainty_; 

  // 只有 "瞄准了" 且 "自信" 才开火
  return is_aimed && is_confident;
}

std::vector<Eigen::Vector3d> Solver::getArmorPositions(const Eigen::Vector3d &target_center,
                                                       const double target_yaw,
                                                       const double r1,
                                                       const double r2,
                                                       const double d_zc,
                                                       const double d_za,
                                                       const size_t armors_num) const noexcept {
  auto armor_positions = std::vector<Eigen::Vector3d>(armors_num, Eigen::Vector3d::Zero());
  // Calculate the position of each armor
  bool is_current_pair = true;
  double r = 0., target_dz = 0.;
  for (size_t i = 0; i < armors_num; i++) {
    double temp_yaw = target_yaw + i * (2 * M_PI / armors_num);
    if (armors_num == 4) {
      r = is_current_pair ? r1 : r2;
      target_dz = d_zc + (is_current_pair ? 0 : d_za);
      is_current_pair = !is_current_pair;
    } else {
      r = r1;
      target_dz = d_zc;
    }
    armor_positions[i] =
      target_center + Eigen::Vector3d(-r * cos(temp_yaw), -r * sin(temp_yaw), target_dz);
  }
  return armor_positions;
}

// 0202
// int Solver::selectBestArmor(const std::vector<Eigen::Vector3d> &armor_positions,
//                             const Eigen::Vector3d &target_center,
//                             const double target_yaw,
//                             const double target_v_yaw,
//                             const size_t armors_num) const noexcept {
//   // Angle between the car's center and the X-axis
//   double alpha = std::atan2(target_center.y(), target_center.x());
//   // Angle between the front of observed armor and the X-axis
//   double beta = target_yaw;

//   // clang-format off
//   Eigen::Matrix2d R_odom2center;
//   Eigen::Matrix2d R_odom2armor;
//   R_odom2center << std::cos(alpha), std::sin(alpha), 
//                   -std::sin(alpha), std::cos(alpha);
//   R_odom2armor << std::cos(beta), std::sin(beta), 
//                  -std::sin(beta), std::cos(beta);
//   // clang-format on
//   Eigen::Matrix2d R_center2armor = R_odom2center.transpose() * R_odom2armor;

//   // Equal to (alpha - beta) in most cases
//   double decision_angle = -std::asin(R_center2armor(0, 1));

//   // Angle thresh of the armor jump
//   double theta = (target_v_yaw > 0 ? side_angle_ : -side_angle_) / 180.0 * M_PI;

//   // Avoid the frequent switch between two armor
//   if (std::abs(target_v_yaw) < min_switching_v_yaw_) {
//     theta = 0;
//   }

//   double temp_angle = decision_angle + M_PI / armors_num - theta;

//   if (temp_angle < 0) {
//     temp_angle += 2 * M_PI;
//   }

//   int selected_id = static_cast<int>(temp_angle / (2 * M_PI / armors_num));
//   return selected_id;
// }

// int Solver::selectBestArmor(const std::vector<Eigen::Vector3d> &armor_positions,
//                             const double target_yaw, 
//                             const double target_v_yaw,
//                             const size_t armors_num) const noexcept
// {
//   double min_score = DBL_MAX;
//   int selected_idx = 0;

//   // 【修复点 1】定义调试字符串，防止编译报错
//   std::string debug_str = ""; 

//   for (size_t i = 0; i < armors_num; ++i) {
//     double angle_offset = i * (2 * M_PI / armors_num);
//     double armor_relative_yaw = target_yaw + angle_offset;
    
//     // 计算离中心 0 度的距离
//     double diff = angles::shortest_angular_distance(armor_relative_yaw, 0);
//     double abs_diff = std::abs(diff);

//     double direction_bias = 0.0;
    
//     // 阈值建议设为 0.1 或 0.2，保证旋转时能触发
//     if (std::abs(target_v_yaw) > spin_judge_low_thres_) {
        
//         // 【逻辑取反测试】
//         // 如果之前的现象是“死咬旧板子”，说明之前的逻辑加分加反了
//         // 我们现在强制把逻辑反过来：
        
//         if (target_v_yaw > 0) { 
//             // 之前是 if (diff > 0)，现在改成 diff < 0
//             if (diff < 0) direction_bias = -anti_spin_bias_; 
//         } 
//         else { // target_v_yaw < 0
//             // 之前是 if (diff < 0)，现在改成 diff > 0
//             if (diff > 0) direction_bias = -anti_spin_bias_;
//         }
//     }

//     double score = abs_diff + direction_bias;

//     // 【修复点 2】记录每块板的分数
//     debug_str += "ID" + std::to_string(i) + ":" + std::to_string((int)(score*10)) + " ";

//     if (score < min_score) {
//       min_score = score;
//       selected_idx = i;
//     }
//   }
  
//   // 打印调试信息：v_yaw, 各板分数, 最终选择
//   // 分数越低越好。如果看到某块板分数是负数，说明它被强力推荐了
//   // 先锁定弱指针
//   if (auto node_ptr = node_.lock()) {
//     // RCLCPP_INFO(node_ptr->get_logger(), "v_yaw:%.2f | %s | Sel:%d", target_v_yaw, debug_str.c_str(), selected_idx);
//   }  
  
//   return selected_idx;
// }

// 0204 终极修正版 selectBestArmor
// int Solver::selectBestArmor(const std::vector<Eigen::Vector3d> &armor_positions,
//                             const double target_yaw, 
//                             const double target_v_yaw,
//                             const size_t armors_num) const noexcept
// {
//   double min_score = DBL_MAX;
//   int selected_idx = 0;
  
//   // 调试字符串
//   std::string debug_str = ""; 

//   for (size_t i = 0; i < armors_num; ++i) {
//     double angle_offset = i * (2 * M_PI / armors_num);
//     double armor_relative_yaw = target_yaw + angle_offset; // 这里的 target_yaw 已经是 pred_yaw - gimbal_yaw
    
//     // 基础分：几何距离 (单位：弧度)
//     // 范围 0 ~ PI
//     double diff = angles::shortest_angular_distance(armor_relative_yaw, 0);
//     double abs_diff = std::abs(diff);

//     double score = abs_diff; // 初始分就是距离分

//     // --- [核心修改] ---
//     // 只要判断在旋转，且方向明确
//     if (std::abs(target_v_yaw) > spin_judge_low_thres_) {
        
//         // 逻辑：寻找位于“旋转前方”的板子
//         // 我们通过 diff 的正负来判断板子在左还是在右
//         // 假设：diff > 0 (左侧), diff < 0 (右侧) —— 这取决于你的坐标系，反正总有一边是对的
        
//         bool is_front_armor = false;

//         if (target_v_yaw > 0) { // 向左转 (逆时针)
//             // 下一块板子应该在右边 (diff < 0) 
//             // 且它正在向中间靠近 (也就是说它的角度是负的，比如 -45度)
//             if (diff < 0) is_front_armor = true;
//         } 
//         else { // 向右转 (顺时针)
//             // 下一块板子应该在左边 (diff > 0)
//             if (diff > 0) is_front_armor = true;
//         }

//         // 奖励机制：
//         // 如果它是“前方板子”，我们直接给它打个折，甚至给负分！
//         if (is_front_armor) {
//             // 方法A：暴力减分 (你现在用的)
//             score -= anti_spin_bias_; 
            
//             // 方法B (推荐)：距离打折
//             // 让新板子的距离看起来只有实际的 30%
//             // score = abs_diff * 0.3; 
//         } else {
//             // 如果是“后方板子”（刚转过去的），给它惩罚
//             // 这样能加速它被淘汰
//             score += anti_spin_bias_ * 0.5;
//         }
//     }

//     // 记录分数
//     debug_str += "ID" + std::to_string(i) + ":" + std::to_string((int)(score*100)) + " ";

//     if (score < min_score) {
//       min_score = score;
//       selected_idx = i;
//     }
//   }
  
//   if (auto node_ptr = node_.lock()) {
//     // 打印调试，注意这里乘了100方便看整数
//     RCLCPP_INFO(node_ptr->get_logger(), "v_yaw:%.2f | %s | Sel:%d", target_v_yaw, debug_str.c_str(), selected_idx);
//   }

//   return selected_idx;
// }

// 0205 修正版：反转左右逻辑
int Solver::selectBestArmor(const std::vector<Eigen::Vector3d> &armor_positions,
                            const double target_yaw, 
                            const double target_v_yaw,
                            const size_t armors_num) const noexcept
{
  double min_score = DBL_MAX;
  int selected_idx = 0;
  std::string debug_str = ""; 

  for (size_t i = 0; i < armors_num; ++i) {
    double angle_offset = i * (2 * M_PI / armors_num);
    double armor_relative_yaw = target_yaw + angle_offset;
    
    // 基础分：几何距离 (0 ~ PI)
    double diff = angles::shortest_angular_distance(armor_relative_yaw, 0);
    double abs_diff = std::abs(diff);

    double score = abs_diff; // 初始分

    // 阈值判定
    if (std::abs(target_v_yaw) > spin_judge_low_thres_) {
        
        bool is_front_armor = false;

        // 【关键修正】：这里把之前的 diff 判断符号全反过来了
        if (target_v_yaw > 0) { // 向左转
            // 之前是 < 0，现在改为 > 0。
            // 意思是：如果以前认为右边是前方，现在试着认为左边是前方
            if (diff > 0) is_front_armor = true; 
        } 
        else { // 向右转
            // 之前是 > 0，现在改为 < 0
            if (diff < 0) is_front_armor = true;
        }

        // 奖励新板子，惩罚旧板子
        if (is_front_armor) {
            score -= anti_spin_bias_; // 奖励
        } else {
            score += anti_spin_bias_; // 惩罚
        }
    }

    debug_str += "ID" + std::to_string(i) + ":" + std::to_string((int)(score*100)) + " ";

    if (score < min_score) {
      min_score = score;
      selected_idx = i;
    }
  }
  
  if (auto node_ptr = node_.lock()) {
    // RCLCPP_INFO(node_ptr->get_logger(), "v_yaw:%.2f | %s | Sel:%d", target_v_yaw, debug_str.c_str(), selected_idx);
  }

  return selected_idx;
}

void Solver::calcYawAndPitch(const Eigen::Vector3d &p,
                             const std::array<double, 3> rpy,
                             double &yaw,
                             double &pitch) const noexcept {
  // Calculate yaw and pitch
  yaw = atan2(p.y(), p.x());
  pitch = atan2(p.z(), p.head(2).norm());

  if (double temp_pitch = pitch; trajectory_compensator_->compensate(p, temp_pitch)) {
    pitch = temp_pitch;
  }
}

std::vector<std::pair<double, double>> Solver::getTrajectory() const noexcept {
  auto trajectory = trajectory_compensator_->getTrajectory(15, rpy_[1]);
  // Rotate
  for (auto &p : trajectory) {
    double x = p.first;
    double y = p.second;
    p.first = x * cos(rpy_[1]) + y * sin(rpy_[1]);
    p.second = -x * sin(rpy_[1]) + y * cos(rpy_[1]);
  }
  return trajectory;
}

geometry_msgs::msg::Point fyt::auto_aim::Solver::getPredictedPosition() const noexcept {
  return predicted_position_;
}

}  // namespace fyt::auto_aim
